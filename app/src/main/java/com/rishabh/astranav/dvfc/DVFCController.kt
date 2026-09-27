package com.rishabh.astranav.dvfc

import android.content.Context
import com.rishabh.astranav.dvfc.math.DeviceVehicleTransform
import com.rishabh.astranav.dvfc.math.Quat
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
    // FUTURE / NOT CONNECTED YET
    // =========================================================

    val gnssAvailable: Boolean =
        false,

    val automaticAzimuthAvailable: Boolean =
        false
)


// =============================================================
// DVFC CONTROLLER
// =============================================================

/**
 * Device → Vehicle Frame Calibration controller.
 *
 * Responsibilities:
 *
 * - Run DVFC calibration state machine
 * - Maintain live orientation telemetry
 * - Monitor stationary stability
 * - Estimate stationary gyro bias
 * - Monitor sensor timestamp spacing
 * - Detect timestamp gaps
 * - Expose diagnostics to DVFCQualityActivity
 *
 * Important:
 *
 * This controller does NOT fabricate GNSS, gravity,
 * acceleration or automatic azimuth evidence.
 *
 * Those values remain unavailable until the corresponding
 * sensor/measurement pipeline is actually connected.
 */
class DVFCController(
    context: Context
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
    // TIMESTAMP DIAGNOSTICS
    // =========================================================

    private var previousTimestampNs =
        0L

    private var timestampSampleCount =
        0

    private var timestampDtSumMs =
        0.0

    private var timestampDtSquaredSumMs =
        0.0

    private var timestampMaxDtMs =
        0.0

    private var timestampGapCount =
        0

    /*
     * Current diagnostic reference cadence.
     *
     * This does NOT force Android sensors to run at 50 Hz.
     *
     * It is only used for detecting unusually large
     * timestamp gaps in the incoming sensor stream.
     */
    private val expectedPeriodMs =
        20.0


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


    // =========================================================
    // STATE FLOW
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
                sample = sample
            )
        }


    // =========================================================
    // START
    // =========================================================

    fun start() {

        resetDiagnostics()

        _state.value =
            DvfcUiState(
                sensorsAvailable =
                    fusion.isAvailable,

                status =
                    CalibrationStatus.STABILIZING,

                transformLocked =
                    transform.lockedTransform != null
            )

        fusion.start()
    }


    // =========================================================
    // STOP
    // =========================================================

    fun stop() {

        fusion.stop()
    }


    // =========================================================
    // SENSOR SAMPLE
    // =========================================================

    private fun onSample(
        sample: com.rishabh.astranav.dvfc.sensor.DeviceOrientationSample
    ) {

        val q =
            sample.quaternion

        val angularVelocity =
            sample.angularVelocity

        val timestampNs =
            sample.timestampNs

        // ---------------------------------------------------------
        // BASIC INPUT VALIDATION
        // ---------------------------------------------------------

        if (angularVelocity.size < 3) {
            return
        }

        // ---------------------------------------------------------
        // ORIENTATION
        // ---------------------------------------------------------

        val euler =
            q.toEulerDegrees()

        val headingOffset =
            transform.headingOffsetDegrees(
                q,
                vehicleHeadingDeg
            )

        // ---------------------------------------------------------
        // IMPORTANT:
        //
        // SensorAdapter already calculates:
        //
        // - sample rate
        // - timestamp jitter
        // - data gaps
        // - duplicate timestamps
        // - resampling
        // - gravity stability
        // - gravity leveling
        //
        // Therefore DVFCController must NOT recalculate
        // those values using its own 20 ms / 50 Hz assumption.
        // ---------------------------------------------------------

        // ---------------------------------------------------------
        // GYRO VALUES
        // ---------------------------------------------------------

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

        val gyroMagnitude =
            sqrt(
                gyroX * gyroX +
                        gyroY * gyroY +
                        gyroZ * gyroZ
            )

        // ---------------------------------------------------------
        // STABILITY
        // ---------------------------------------------------------

        val stable =
            stability.update(
                angularVelocity
            )

        // ---------------------------------------------------------
        // STATIONARY GYRO COLLECTION
        // ---------------------------------------------------------

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

        val current =
            _state.value

        // =========================================================
        // CALIBRATION STATE MACHINE
        // =========================================================

        val nextStatus =
            when (current.status) {

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
                     * CURRENT IMPLEMENTATION:
                     *
                     * Device yaw is used as the provisional vehicle
                     * heading.
                     *
                     * This is NOT GNSS COG refinement.
                     *
                     * Automatic mount azimuth refinement remains
                     * unavailable until the GNSS-motion pipeline
                     * is connected.
                     */

                    vehicleHeadingDeg =
                        euler.getOrElse(2) {
                            0f
                        }

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

                    val stillStable =
                        stable

                    val elapsedMs =
                        (
                                timestampNs -
                                        validatingSinceNs
                                ) / 1_000_000L

                    when {

                        !stillStable -> {

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

        // =========================================================
        // UPDATE LIVE STATE
        // =========================================================

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
                    euler.getOrElse(0) {
                        0f
                    },

                pitchDeg =
                    euler.getOrElse(1) {
                        0f
                    },

                yawDeg =
                    euler.getOrElse(2) {
                        0f
                    },

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
                // SENSOR ADAPTER TELEMETRY
                // -------------------------------------------------

                /*
                 * IMPORTANT:
                 *
                 * These values come directly from SensorAdapter.
                 *
                 * Do NOT use timestampSampleCount,
                 * timestampGapCount etc. here.
                 */

                sampleCount =
                    current.sampleCount + 1,

                timestampJitterMs =
                    sample.timestampJitterMs
                        .toDouble(),

                averageSamplePeriodMs =
                    if (
                        sample.estimatedSampleHz > 0f
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
                        sample.resamplingRateHz > 0f
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
                    sqrt(
                        (
                                sample.linearAcceleration
                                    .getOrNull(0)
                                    ?.toDouble()
                                    ?: 0.0
                                ) *
                                (
                                        sample.linearAcceleration
                                            .getOrNull(0)
                                            ?.toDouble()
                                            ?: 0.0
                                        ) +

                                (
                                        sample.linearAcceleration
                                            .getOrNull(1)
                                            ?.toDouble()
                                            ?: 0.0
                                        ) *
                                (
                                        sample.linearAcceleration
                                            .getOrNull(1)
                                            ?.toDouble()
                                            ?: 0.0
                                        ) +

                                (
                                        sample.linearAcceleration
                                            .getOrNull(2)
                                            ?.toDouble()
                                            ?: 0.0
                                        ) *
                                (
                                        sample.linearAcceleration
                                            .getOrNull(2)
                                            ?.toDouble()
                                            ?: 0.0
                                        )
                    ),

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
                    transform.lockedTransform != null,

                // -------------------------------------------------
                // NOT CONNECTED YET
                // -------------------------------------------------

                /*
                 * These remain false until real GNSS and automatic
                 * azimuth pipelines are connected.
                 */

                gnssAvailable =
                    false,

                automaticAzimuthAvailable =
                    false
            )
    }


    // =========================================================
    // TIMESTAMP DIAGNOSTICS
    // =========================================================

    private fun updateTimestampDiagnostics(
        timestampNs: Long
    ) {

        /*
         * First sample establishes the reference timestamp.
         */
        if (
            previousTimestampNs == 0L
        ) {

            previousTimestampNs =
                timestampNs

            return
        }


        val dtMs =
            (
                    timestampNs -
                            previousTimestampNs
                    ) / 1_000_000.0


        previousTimestampNs =
            timestampNs


        /*
         * Ignore invalid or backwards timestamps.
         */
        if (
            dtMs <= 0.0
        ) {
            return
        }


        timestampSampleCount++


        timestampDtSumMs +=
            dtMs


        timestampDtSquaredSumMs +=
            dtMs * dtMs


        timestampMaxDtMs =
            maxOf(
                timestampMaxDtMs,
                dtMs
            )


        /*
         * Diagnostic gap:
         *
         * > 3 × expected period.
         *
         * At 50 Hz:
         *
         * expected = 20 ms
         * gap      > 60 ms
         */
        if (
            dtMs >
            expectedPeriodMs * 3.0
        ) {

            timestampGapCount++
        }
    }


    private fun averageTimestampPeriodMs():
            Double? {

        if (
            timestampSampleCount <= 0
        ) {
            return null
        }

        return timestampDtSumMs /
                timestampSampleCount
    }


    private fun estimatedSampleHz():
            Double? {

        val period =
            averageTimestampPeriodMs()
                ?: return null

        if (
            period <= 0.0
        ) {
            return null
        }

        return 1000.0 /
                period
    }


    private fun timestampJitterMs():
            Double? {

        if (
            timestampSampleCount <= 1
        ) {
            return null
        }


        val mean =
            timestampDtSumMs /
                    timestampSampleCount


        val meanSquare =
            timestampDtSquaredSumMs /
                    timestampSampleCount


        val variance =
            (
                    meanSquare -
                            mean * mean
                    ).coerceAtLeast(
                    0.0
                )


        return sqrt(
            variance
        )
    }


    private fun timestampMaxGapMs():
            Double? {

        if (
            timestampSampleCount <= 0
        ) {
            return null
        }

        return timestampMaxDtMs
    }


    // =========================================================
    // STATIONARY DIAGNOSTICS
    // =========================================================

    private fun stationaryScore():
            Double? {

        if (
            timestampSampleCount <= 0
        ) {
            return null
        }


        val ratio =
            stationarySampleCount
                .toDouble() /
                    timestampSampleCount
                        .toDouble()


        return ratio.coerceIn(
            0.0,
            1.0
        )
    }


    // =========================================================
    // GYRO BIAS
    // =========================================================

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
    // RESET DIAGNOSTICS
    // =========================================================

    private fun resetDiagnostics() {

        previousTimestampNs =
            0L

        timestampSampleCount =
            0

        timestampDtSumMs =
            0.0

        timestampDtSquaredSumMs =
            0.0

        timestampMaxDtMs =
            0.0

        timestampGapCount =
            0

        resetCalibrationDiagnostics()
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

    /**
     * Returns the locked Device → Vehicle transform.
     *
     * Null until calibration is actually locked.
     */
    fun lockedDeviceToVehicleTransform():
            Quat? {

        return transform.lockedTransform
    }


    // =========================================================
    // RECALIBRATION
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
                    false
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