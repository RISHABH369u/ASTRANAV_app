package com.rishabh.astranav.dvfc

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.os.Looper

import androidx.core.app.ActivityCompat

import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority

import com.rishabh.astranav.dvfc.gnss.GnssAzimuthEstimator
import com.rishabh.astranav.dvfc.math.DeviceVehicleTransform
import com.rishabh.astranav.dvfc.math.Quat
import com.rishabh.astranav.dvfc.sensor.DeviceOrientationSample
import com.rishabh.astranav.dvfc.sensor.SensorFusion
import com.rishabh.astranav.dvfc.sensor.StabilityDetector

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

import kotlin.math.sqrt


// =============================================================
// DVFC UI / DIAGNOSTIC STATE
// =============================================================

data class DvfcUiState(

    // =========================================================
    // CALIBRATION
    // =========================================================

    val status: CalibrationStatus =
        CalibrationStatus.STABILIZING,

    // =========================================================
    // ORIENTATION
    // =========================================================

    val rollDeg: Float =
        0f,

    val pitchDeg: Float =
        0f,

    val yawDeg: Float =
        0f,

    val headingOffsetDeg: Float =
        0f,

    val currentQuaternion: Quat =
        Quat.IDENTITY,

    // =========================================================
    // SENSOR AVAILABILITY
    // =========================================================

    val sensorsAvailable: Boolean =
        false,

    val gravityAvailable: Boolean =
        false,

    val accelerationAvailable: Boolean =
        false,

    val gyroscopeAvailable: Boolean =
        false,

    val rotationVectorAvailable: Boolean =
        false,

    // =========================================================
    // SENSOR ADAPTER TELEMETRY
    // =========================================================

    val sampleCount: Int =
        0,

    val timestampJitterMs: Double? =
        null,

    val averageSamplePeriodMs: Double? =
        null,

    val estimatedSampleHz: Double? =
        null,

    val dataGapCount: Int =
        0,

    val maxGapMs: Double? =
        null,

    val duplicateTimestampCount: Int =
        0,

    val resamplingActive: Boolean =
        false,

    val resamplingRateHz: Double? =
        null,

    // =========================================================
    // GRAVITY TELEMETRY
    // =========================================================

    val gravityMagnitude: Double? =
        null,

    val gravityStable: Boolean =
        false,

    val gravityLevelRollDeg: Double? =
        null,

    val gravityLevelPitchDeg: Double? =
        null,

    // =========================================================
    // ACCELERATION TELEMETRY
    // =========================================================

    val linearAccelerationMagnitude: Double? =
        null,

    // =========================================================
    // STATIONARY
    // =========================================================

    val stationarySampleCount: Int =
        0,

    val stationaryScore: Double? =
        null,

    // =========================================================
    // GYRO
    // =========================================================

    val gyroBiasX: Double? =
        null,

    val gyroBiasY: Double? =
        null,

    val gyroBiasZ: Double? =
        null,

    val gyroMagnitudeRms: Double? =
        null,

    // =========================================================
    // TRANSFORM
    // =========================================================

    val transformLocked: Boolean =
        false,

    // =========================================================
    // GNSS / AZIMUTH TELEMETRY
    // =========================================================

    val gnssAvailable: Boolean =
        false,

    val gnssAccuracyM: Double? =
        null,

    val gnssSpeedMps: Double? =
        null,

    val gnssValidSamples: Int? =
        null,

    val automaticAzimuthDeg: Double? =
        null,

    val azimuthResidualDeg: Double? =
        null,

    val azimuthConsistency: Double? =
        null,

    val automaticAzimuthAvailable: Boolean =
        false
)


// =============================================================
// DVFC CONTROLLER
// =============================================================

class DVFCController(
    private val context: Context
) {

    // =========================================================
    // CORE DVFC COMPONENTS
    // =========================================================

    private val stability =
        StabilityDetector()

    private val transform =
        DeviceVehicleTransform()

    // =========================================================
    // CALIBRATION STATE
    // =========================================================

    private var vehicleHeadingDeg =
        0f

    private var validatingSinceNs =
        0L

    // =========================================================
    // STATE
    // =========================================================

    private val _state =
        MutableStateFlow(
            DvfcUiState()
        )

    val state: StateFlow<DvfcUiState> =
        _state

    // =========================================================
    // SENSOR FUSION
    // =========================================================

    private val fusion =
        SensorFusion(context) { sample ->

            onSample(
                sample
            )
        }

    // =========================================================
    // GNSS / AZIMUTH
    // =========================================================

    private val gnssAzimuthEstimator =
        GnssAzimuthEstimator()

    private val fusedLocationClient:
            FusedLocationProviderClient =
        LocationServices
            .getFusedLocationProviderClient(
                context
            )

    private var latestYawDeg =
        0.0

    private var lastGyroTimestampNs =
        0L

    private var gnssStarted =
        false

    private var sensorStarted =
        false

    // =========================================================
    // GNSS CALLBACK
    // =========================================================

    private val locationCallback =
        object : LocationCallback() {

            override fun onLocationResult(
                result: LocationResult
            ) {

                for (
                location in result.locations
                ) {

                    handleGnssLocation(
                        location
                    )
                }
            }
        }

    // =========================================================
    // START
    // =========================================================

    fun start() {

        /*
         * IMPORTANT:
         *
         * Do not reset the entire calibration every time
         * Activity resumes.
         *
         * Quality screen can be opened while this controller
         * continues to live.
         */
        if (!sensorStarted) {

            sensorStarted =
                true

            resetDiagnostics()

            _state.value =
                _state.value.copy(

                    sensorsAvailable =
                        fusion.isAvailable,

                    status =
                        CalibrationStatus.STABILIZING,

                    transformLocked =
                        transform.lockedTransform != null
                )

            fusion.start()
        }

        startGnss()
    }

    // =========================================================
    // GNSS START
    // =========================================================

    fun refreshGnssPermission() {

        startGnss()
    }

    private fun startGnss() {

        if (gnssStarted) {
            return
        }

        val fineGranted =
            ActivityCompat.checkSelfPermission(
                context,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) ==
                    PackageManager.PERMISSION_GRANTED

        val coarseGranted =
            ActivityCompat.checkSelfPermission(
                context,
                Manifest.permission.ACCESS_COARSE_LOCATION
            ) ==
                    PackageManager.PERMISSION_GRANTED

        if (
            !fineGranted &&
            !coarseGranted
        ) {

            return
        }

        val request =
            LocationRequest.Builder(
                Priority.PRIORITY_HIGH_ACCURACY,
                1000L
            )
                .setMinUpdateIntervalMillis(
                    500L
                )
                .setWaitForAccurateLocation(
                    false
                )
                .build()

        fusedLocationClient
            .requestLocationUpdates(
                request,
                locationCallback,
                Looper.getMainLooper()
            )

        gnssStarted =
            true
    }

    // =========================================================
    // GNSS LOCATION
    // =========================================================

    private fun handleGnssLocation(
        location: Location
    ) {

        val result =
            gnssAzimuthEstimator
                .updateLocation(
                    location =
                        location,

                    deviceYawDeg =
                        latestYawDeg
                )

        val current =
            _state.value

        _state.value =
            current.copy(

                gnssAvailable =
                    result.gnssAvailable,

                gnssAccuracyM =
                    result.accuracyM,

                gnssSpeedMps =
                    result.speedMps,

                gnssValidSamples =
                    result.validSamples,

                automaticAzimuthDeg =
                    result.automaticAzimuthDeg,

                azimuthResidualDeg =
                    result.residualDeg,

                azimuthConsistency =
                    result.circularConsistency,

                automaticAzimuthAvailable =
                    result.ready
            )
    }

    // =========================================================
    // STOP
    // =========================================================

    fun stop() {

        /*
         * Stop both streams.
         *
         * Activity can call start() again and the same
         * controller will resume without losing locked
         * transform.
         */
        if (sensorStarted) {

            fusion.stop()

            sensorStarted =
                false
        }

        if (gnssStarted) {

            fusedLocationClient
                .removeLocationUpdates(
                    locationCallback
                )

            gnssStarted =
                false
        }
    }

    // =========================================================
    // SENSOR SAMPLE
    // =========================================================

    private fun onSample(
        sample: DeviceOrientationSample
    ) {

        val q =
            sample.quaternion

        val angularVelocity =
            sample.angularVelocity

        val timestampNs =
            sample.timestampNs

        if (
            angularVelocity.size < 3
        ) {
            return
        }

        // =====================================================
        // ORIENTATION
        // =====================================================

        val euler =
            q.toEulerDegrees()

        val rollDeg =
            euler.getOrElse(0) {
                0f
            }

        val pitchDeg =
            euler.getOrElse(1) {
                0f
            }

        val yawDeg =
            euler.getOrElse(2) {
                0f
            }

        latestYawDeg =
            yawDeg.toDouble()

        // =====================================================
        // GYRO
        // =====================================================

        val gyroX =
            angularVelocity
                .getOrNull(0)
                ?.toDouble()
                ?: 0.0

        val gyroY =
            angularVelocity
                .getOrNull(1)
                ?.toDouble()
                ?: 0.0

        val gyroZ =
            angularVelocity
                .getOrNull(2)
                ?.toDouble()
                ?: 0.0

        // =====================================================
        // GYRO → GNSS HEADING CONSISTENCY
        // =====================================================

        if (
            lastGyroTimestampNs != 0L
        ) {

            val dtSec =
                (
                        timestampNs -
                                lastGyroTimestampNs
                        ) /
                        1_000_000_000.0

            gnssAzimuthEstimator
                .updateGyro(
                    yawRateRadPerSec =
                        gyroZ,

                    dtSec =
                        dtSec
                )
        }

        lastGyroTimestampNs =
            timestampNs

        // =====================================================
        // GYRO MAGNITUDE
        // =====================================================

        val gyroMagnitude =
            sqrt(
                gyroX * gyroX +
                        gyroY * gyroY +
                        gyroZ * gyroZ
            )

        // =====================================================
        // VEHICLE HEADING / TRANSFORM
        // =====================================================

        val headingOffset =
            transform.headingOffsetDegrees(
                q,
                vehicleHeadingDeg
            )

        // =====================================================
        // STABILITY
        // =====================================================

        val stable =
            stability.update(
                angularVelocity
            )

        if (stable) {

            stationarySampleCount++

            gyroBiasSumX +=
                gyroX

            gyroBiasSumY +=
                gyroY

            gyroBiasSumZ +=
                gyroZ

            gyroMagnitudeSquaredSum +=
                gyroMagnitude *
                        gyroMagnitude
        }

        // =====================================================
        // CURRENT STATE
        // =====================================================

        val current =
            _state.value

        // =====================================================
        // CALIBRATION STATE MACHINE
        // =====================================================

        val nextStatus =
            when (
                current.status
            ) {

                // -------------------------------------------------
                // STABILIZING
                // -------------------------------------------------

                CalibrationStatus.STABILIZING -> {

                    if (stable) {

                        CalibrationStatus.ALIGNING

                    } else {

                        CalibrationStatus.STABILIZING
                    }
                }

                // -------------------------------------------------
                // ALIGNING
                // -------------------------------------------------

                CalibrationStatus.ALIGNING -> {

                    /*
                     * Provisional heading.
                     *
                     * This is replaced/refined by the real
                     * GNSS COG + gyro estimator separately.
                     */
                    vehicleHeadingDeg =
                        yawDeg

                    transform.calibrate(
                        q,
                        vehicleHeadingDeg
                    )

                    validatingSinceNs =
                        timestampNs

                    CalibrationStatus.VALIDATING
                }

                // -------------------------------------------------
                // VALIDATING
                // -------------------------------------------------

                CalibrationStatus.VALIDATING -> {

                    val elapsedMs =
                        (
                                timestampNs -
                                        validatingSinceNs
                                ) /
                                1_000_000L

                    when {

                        !stable -> {

                            transform.reset()

                            resetCalibrationDiagnostics()

                            CalibrationStatus.STABILIZING
                        }

                        elapsedMs >=
                                VALIDATION_HOLD_MS -> {

                            CalibrationStatus.COMPLETE
                        }

                        else -> {

                            CalibrationStatus.VALIDATING
                        }
                    }
                }

                // -------------------------------------------------
                // COMPLETE
                // -------------------------------------------------

                CalibrationStatus.COMPLETE -> {

                    CalibrationStatus.COMPLETE
                }
            }

        // =====================================================
        // LINEAR ACCELERATION
        // =====================================================

        val linearAcceleration =
            sample.linearAcceleration

        val linearAccelerationMagnitude =
            sqrt(

                (
                        linearAcceleration
                            .getOrNull(0)
                            ?.toDouble()
                            ?: 0.0
                        ) *
                        (
                                linearAcceleration
                                    .getOrNull(0)
                                    ?.toDouble()
                                    ?: 0.0
                                ) +

                        (
                                linearAcceleration
                                    .getOrNull(1)
                                    ?.toDouble()
                                    ?: 0.0
                                ) *
                        (
                                linearAcceleration
                                    .getOrNull(1)
                                    ?.toDouble()
                                    ?: 0.0
                                ) +

                        (
                                linearAcceleration
                                    .getOrNull(2)
                                    ?.toDouble()
                                    ?: 0.0
                                ) *
                        (
                                linearAcceleration
                                    .getOrNull(2)
                                    ?.toDouble()
                                    ?: 0.0
                                )
            )

        // =====================================================
        // UPDATE STATE
        // =====================================================

        _state.value =
            current.copy(

                // -------------------------------------------------
                // CALIBRATION
                // -------------------------------------------------

                status =
                    nextStatus,

                // -------------------------------------------------
                // ORIENTATION
                // -------------------------------------------------

                rollDeg =
                    rollDeg,

                pitchDeg =
                    pitchDeg,

                yawDeg =
                    yawDeg,

                headingOffsetDeg =
                    headingOffset,

                currentQuaternion =
                    q,

                // -------------------------------------------------
                // SENSOR AVAILABILITY
                // -------------------------------------------------

                sensorsAvailable =
                    fusion.isAvailable,

                gravityAvailable =
                    sample.gravityAvailable,

                accelerationAvailable =
                    sample.accelerationAvailable,

                gyroscopeAvailable =
                    sample.gyroscopeAvailable,

                rotationVectorAvailable =
                    sample.rotationVectorAvailable,

                // -------------------------------------------------
                // SENSOR ADAPTER
                // -------------------------------------------------

                sampleCount =
                    current.sampleCount + 1,

                timestampJitterMs =
                    sample.timestampJitterMs
                        .toDouble(),

                averageSamplePeriodMs =
                    if (
                        sample.estimatedSampleHz >
                        0f
                    ) {

                        1000.0 /
                                sample.estimatedSampleHz

                    } else {

                        null
                    },

                estimatedSampleHz =
                    sample.estimatedSampleHz
                        .toDouble(),

                dataGapCount =
                    sample.dataGapCount,

                maxGapMs =
                    sample.maxGapMs
                        .toDouble(),

                duplicateTimestampCount =
                    sample.duplicateTimestampCount,

                resamplingActive =
                    sample.resamplingActive,

                resamplingRateHz =
                    if (
                        sample.resamplingRateHz >
                        0f
                    ) {

                        sample.resamplingRateHz
                            .toDouble()

                    } else {

                        null
                    },

                // -------------------------------------------------
                // GRAVITY
                // -------------------------------------------------

                gravityMagnitude =
                    sample.gravityMagnitude
                        .toDouble(),

                gravityStable =
                    sample.gravityStable,

                gravityLevelRollDeg =
                    sample.gravityLevelRollDeg
                        .toDouble(),

                gravityLevelPitchDeg =
                    sample.gravityLevelPitchDeg
                        .toDouble(),

                // -------------------------------------------------
                // LINEAR ACCELERATION
                // -------------------------------------------------

                linearAccelerationMagnitude =
                    linearAccelerationMagnitude,

                // -------------------------------------------------
                // STATIONARY
                // -------------------------------------------------

                stationarySampleCount =
                    stationarySampleCount,

                stationaryScore =
                    stationaryScore(),

                // -------------------------------------------------
                // GYRO
                // -------------------------------------------------

                gyroBiasX =
                    gyroBiasX(),

                gyroBiasY =
                    gyroBiasY(),

                gyroBiasZ =
                    gyroBiasZ(),

                gyroMagnitudeRms =
                    gyroMagnitudeRms(),

                // -------------------------------------------------
                // TRANSFORM
                // -------------------------------------------------

                transformLocked =
                    transform.lockedTransform != null

                /*
                 * IMPORTANT:
                 *
                 * DO NOT write:
                 *
                 * gnssAvailable = false
                 * automaticAzimuthAvailable = false
                 *
                 * here.
                 *
                 * GNSS callback owns those values.
                 */
            )
    }

    // =========================================================
    // STATIONARY / GYRO DIAGNOSTICS
    // =========================================================

    private var stationarySampleCount =
        0

    private var gyroBiasSumX =
        0.0

    private var gyroBiasSumY =
        0.0

    private var gyroBiasSumZ =
        0.0

    private var gyroMagnitudeSquaredSum =
        0.0

    private fun stationaryScore():
            Double? {

        val total =
            _state.value.sampleCount

        if (total <= 0) {
            return null
        }

        return (
                stationarySampleCount
                    .toDouble() /
                        total.toDouble()
                ).coerceIn(
                0.0,
                1.0
            )
    }

    private fun gyroBiasX():
            Double? {

        if (
            stationarySampleCount <= 0
        ) {
            return null
        }

        return gyroBiasSumX /
                stationarySampleCount
    }

    private fun gyroBiasY():
            Double? {

        if (
            stationarySampleCount <= 0
        ) {
            return null
        }

        return gyroBiasSumY /
                stationarySampleCount
    }

    private fun gyroBiasZ():
            Double? {

        if (
            stationarySampleCount <= 0
        ) {
            return null
        }

        return gyroBiasSumZ /
                stationarySampleCount
    }

    private fun gyroMagnitudeRms():
            Double? {

        if (
            stationarySampleCount <= 0
        ) {
            return null
        }

        return sqrt(
            gyroMagnitudeSquaredSum /
                    stationarySampleCount
        )
    }

    // =========================================================
    // RESET
    // =========================================================

    private fun resetDiagnostics() {

        stationarySampleCount =
            0

        gyroBiasSumX =
            0.0

        gyroBiasSumY =
            0.0

        gyroBiasSumZ =
            0.0

        gyroMagnitudeSquaredSum =
            0.0

        lastGyroTimestampNs =
            0L

        gnssAzimuthEstimator.reset()
    }

    private fun resetCalibrationDiagnostics() {

        stationarySampleCount =
            0

        gyroBiasSumX =
            0.0

        gyroBiasSumY =
            0.0

        gyroBiasSumZ =
            0.0

        gyroMagnitudeSquaredSum =
            0.0
    }

    // =========================================================
    // TRANSFORM HAND-OFF
    // =========================================================

    fun lockedDeviceToVehicleTransform():
            Quat? {

        return transform.lockedTransform
    }

    // =========================================================
    // RECALIBRATE
    // =========================================================

    fun recalibrate() {

        transform.reset()

        stability.reset()

        vehicleHeadingDeg =
            0f

        validatingSinceNs =
            0L

        resetDiagnostics()

        _state.value =
            _state.value.copy(

                status =
                    CalibrationStatus.STABILIZING,

                transformLocked =
                    false,

                gnssAvailable =
                    false,

                gnssAccuracyM =
                    null,

                gnssSpeedMps =
                    null,

                gnssValidSamples =
                    0,

                automaticAzimuthDeg =
                    null,

                azimuthResidualDeg =
                    null,

                azimuthConsistency =
                    null,

                automaticAzimuthAvailable =
                    false,

                sampleCount =
                    0,

                stationarySampleCount =
                    0,

                stationaryScore =
                    null,

                gyroBiasX =
                    null,

                gyroBiasY =
                    null,

                gyroBiasZ =
                    null,

                gyroMagnitudeRms =
                    null
            )
    }

    // =========================================================
    // CONSTANTS
    // =========================================================

    companion object {

        private const val VALIDATION_HOLD_MS =
            2000L
    }
}