package com.rishabh.astranav.navigation

import android.content.Context
import android.util.Log

import com.rishabh.astranav.dvfc.DvfcCalibrationStore
import com.rishabh.astranav.dvfc.sensor.DeviceSensorSample
import com.rishabh.astranav.dvfc.sensor.SensorAdapter
import com.rishabh.astranav.navigation.eskf.Eskf
import com.rishabh.astranav.navigation.eskf.NavigationState
import com.rishabh.astranav.navigation.eskf.Quaternion
import com.rishabh.astranav.navigation.eskf.Vec3
import com.rishabh.astranav.navigation.zupt.ZuptDetector

import kotlin.math.atan2
import kotlin.math.sqrt


/**
 * =============================================================
 * ASTRANAV NAVIGATION RUNTIME
 * =============================================================
 *
 * Runtime V1:
 *
 * Android Sensors
 *       ↓
 * SensorAdapter
 *       ↓
 * synchronized 10 Hz DeviceSensorSample
 *       ↓
 * DVFC Device → Vehicle transform
 *       ↓
 * ASTRA-Core ESKF prediction
 *       ↓
 * ZUPT detector
 *       ├── ZUPT
 *       └── ZARU
 *       ↓
 * NHC when moving
 *       ↓
 * NavigationSolution
 *
 * IMPORTANT:
 *
 * - ESKF is the authoritative navigation state.
 * - IMU is the ONLY prediction source.
 * - ZUPT/ZARU/NHC are measurements/constraints.
 * - ML models are NOT directly modifying the ESKF here.
 * - GNSS measurement fusion is intentionally kept in
 *   GnssLocationSource/GnssRuntimeCoordinator for now.
 * - ASTRA-Speed / ASTRA-Motion / ASTRA-SPHM will be
 *   connected through measurement bridges in the next phase.
 *
 * This class does NOT replace DVFCController.
 * DVFC remains responsible for calibration/UI/diagnostics.
 */
class AstraNavigationEngine(
    private val context: Context,
    private val eskf: Eskf = Eskf(),
    private val zuptDetector: ZuptDetector = ZuptDetector()
) {

    companion object {

        private const val TAG =
            "ASTRA_NAV_ENGINE"

        /*
         * NHC should not be applied while the system is
         * stationary. It is intended for normal vehicle motion.
         */
        private const val MIN_NHC_SPEED_MPS =
            1.0

        /*
         * Avoid repeatedly applying NHC at extreme
         * low-speed transition noise.
         */
        private const val MAX_NHC_SPEED_MPS =
            60.0
    }


    // ---------------------------------------------------------
    // SENSOR ADAPTER
    // ---------------------------------------------------------

    private val sensorAdapter =
        SensorAdapter(
            context = context.applicationContext
        ) { sample ->

            onSensorSample(sample)
        }


    // ---------------------------------------------------------
    // RUNTIME STATE
    // ---------------------------------------------------------

    @Volatile
    private var running =
        false

    @Volatile
    private var latestSolution =
        NavigationSolution()

    @Volatile
    private var latestSensorSample:
            DeviceSensorSample? =
        null

    /*
     * Last timestamp actually accepted by the
     * navigation runtime.
     */
    private var lastTimestampNs =
        0L

    /*
     * Number of samples received while the
     * navigation runtime is running.
     *
     * This counter is incremented exactly once
     * inside onSensorSample().
     */
    private var processedSampleCount =
        0L

    private var stationarySampleCount =
        0

    private var movingSampleCount =
        0


    // ---------------------------------------------------------
    // DVFC TRANSFORM
    // ---------------------------------------------------------

    /*
     * The calibration screen/controller owns calibration.
     *
     * Navigation runtime only consumes the persisted
     * Device → Vehicle transform.
     *
     * We intentionally reload it when navigation starts,
     * so the navigation runtime never performs its own
     * independent calibration.
     */
    private var dvfcTransform =
        DvfcCalibrationStore.load(
            context.applicationContext
        )


    // ---------------------------------------------------------
    // START
    // ---------------------------------------------------------

    @Synchronized
    fun start() {

        if (running) {
            return
        }

        /*
         * Refresh transform at start.
         *
         * If the user completed/recalibrated DVFC before
         * entering navigation, the latest transform is used.
         */
        dvfcTransform =
            DvfcCalibrationStore.load(
                context.applicationContext
            )

        running = true

        sensorAdapter.start()

        Log.i(
            TAG,
            "ASTRANAV navigation runtime started. " +
                    "DVFC=${dvfcTransform != null}"
        )
    }


    // ---------------------------------------------------------
    // STOP
    // ---------------------------------------------------------

    @Synchronized
    fun stop() {

        if (!running) {
            return
        }

        running = false

        sensorAdapter.stop()

        Log.i(
            TAG,
            "ASTRANAV navigation runtime stopped"
        )
    }


    // ---------------------------------------------------------
    // RESET
    // ---------------------------------------------------------

    @Synchronized
    fun reset() {

        sensorAdapter.stop()

        running = false

        eskf.reset()

        /*
         * Reset runtime timing.
         */
        lastTimestampNs =
            0L

        /*
         * Reset sample counter.
         */
        processedSampleCount =
            0L

        stationarySampleCount =
            0

        movingSampleCount =
            0

        latestSensorSample =
            null

        latestSolution =
            NavigationSolution()

        /*
         * Reload latest calibration after reset.
         */
        dvfcTransform =
            DvfcCalibrationStore.load(
                context.applicationContext
            )

        Log.i(
            TAG,
            "ASTRANAV navigation runtime reset"
        )
    }


    // ---------------------------------------------------------
    // REFRESH DVFC
    // ---------------------------------------------------------

    /**
     * Reloads the currently persisted Device → Vehicle
     * transform without restarting the whole engine.
     *
     * Useful after the user completes/recalibrates DVFC.
     */
    @Synchronized
    fun refreshDvfcTransform() {

        dvfcTransform =
            DvfcCalibrationStore.load(
                context.applicationContext
            )

        Log.i(
            TAG,
            "DVFC transform refreshed. " +
                    "available=${dvfcTransform != null}"
        )
    }


    // ---------------------------------------------------------
    // SENSOR CALLBACK
    // ---------------------------------------------------------

    private fun onSensorSample(
        sample: DeviceSensorSample
    ) {

        if (!running) {
            return
        }

        /*
         * Count every sample received while the
         * navigation runtime is running.
         *
         * IMPORTANT:
         * This is the ONLY place where the counter
         * is incremented.
         */
        processedSampleCount++

        latestSensorSample =
            sample


        // -----------------------------------------------------
        // TIMESTAMP VALIDATION
        // -----------------------------------------------------

        /*
         * SensorAdapter already performs synchronization
         * and timestamp validation.
         *
         * ESKF performs its own final monotonic/dt validation.
         */

        if (sample.timestampNs <= 0L) {

            Log.w(
                TAG,
                "Ignoring invalid timestamp: " +
                        sample.timestampNs
            )

            return
        }

        if (
            lastTimestampNs != 0L &&
            sample.timestampNs <= lastTimestampNs
        ) {

            Log.w(
                TAG,
                "Ignoring non-monotonic sample: " +
                        "current=${sample.timestampNs} " +
                        "previous=$lastTimestampNs"
            )

            return
        }

        lastTimestampNs =
            sample.timestampNs


        // -----------------------------------------------------
        // DEVICE → VEHICLE FRAME
        // -----------------------------------------------------

        val deviceAcceleration =
            sample.acceleration

        val deviceGyro =
            sample.angularVelocity


        /*
         * SensorAdapter gives acceleration including gravity.
         *
         * ESKF mechanization expects specific force.
         *
         * Therefore:
         *
         *     specificForce = acceleration - gravity
         *
         * before entering ESKF.
         */
        val deviceSpecificForce =
            floatArrayOf(

                deviceAcceleration[0] -
                        sample.gravity[0],

                deviceAcceleration[1] -
                        sample.gravity[1],

                deviceAcceleration[2] -
                        sample.gravity[2]
            )


        /*
         * Apply the persisted DVFC transform when available.
         *
         * If calibration is not available yet, keep the sample
         * in device frame rather than inventing a transform.
         *
         * The navigation solution exposes dvfcActive=false so
         * the caller can prevent presenting this as calibrated
         * vehicle navigation.
         */
        val vehicleSpecificForce =
            dvfcTransform
                ?.rotate(
                    deviceSpecificForce
                )
                ?: deviceSpecificForce

        val vehicleGyro =
            dvfcTransform
                ?.rotate(
                    deviceGyro
                )
                ?: deviceGyro


        // -----------------------------------------------------
        // VALIDATE TRANSFORMED VALUES
        // -----------------------------------------------------

        if (
            !vehicleSpecificForce.all {
                it.isFinite()
            } ||
            !vehicleGyro.all {
                it.isFinite()
            }
        ) {

            Log.w(
                TAG,
                "Ignoring non-finite transformed IMU sample"
            )

            return
        }


        // -----------------------------------------------------
        // ESKF PREDICTION
        // -----------------------------------------------------

        val prediction =
            eskf.predict(

                timestampNanos =
                    sample.timestampNs,

                accelerationBody =
                    Vec3(
                        x =
                            vehicleSpecificForce[0]
                                .toDouble(),

                        y =
                            vehicleSpecificForce[1]
                                .toDouble(),

                        z =
                            vehicleSpecificForce[2]
                                .toDouble()
                    ),

                gyroBody =
                    Vec3(
                        x =
                            vehicleGyro[0]
                                .toDouble(),

                        y =
                            vehicleGyro[1]
                                .toDouble(),

                        z =
                            vehicleGyro[2]
                                .toDouble()
                    )
            )


        if (!prediction.accepted) {

            Log.w(
                TAG,
                "ESKF prediction rejected: " +
                        prediction.reason
            )

            updateSolution(
                sample = sample,
                predictionAccepted = false
            )

            return
        }


        // -----------------------------------------------------
        // CURRENT TRUSTED SPEED
        // -----------------------------------------------------

        val currentVelocity =
            eskf.getVelocity()

        val trustedSpeedMps =
            currentVelocity.norm()


        // -----------------------------------------------------
        // ZUPT DETECTION
        // -----------------------------------------------------

        val gyroX =
            vehicleGyro[0].toDouble()

        val gyroY =
            vehicleGyro[1].toDouble()

        val gyroZ =
            vehicleGyro[2].toDouble()

        val linearAcceleration =
            sample.linearAcceleration

        val zuptResult =
            zuptDetector.update(

                gyroX =
                    gyroX,

                gyroY =
                    gyroY,

                gyroZ =
                    gyroZ,

                linearAccelX =
                    linearAcceleration[0]
                        .toDouble(),

                linearAccelY =
                    linearAcceleration[1]
                        .toDouble(),

                linearAccelZ =
                    linearAcceleration[2]
                        .toDouble(),

                trustedSpeedMps =
                    trustedSpeedMps,

                /*
                 * ASTRA-Motion is not connected yet.
                 *
                 * Therefore there is currently no ML ZUPT
                 * probability to provide.
                 */
                zuptLogit =
                    null
            )


        // -----------------------------------------------------
        // STATIONARY / MOVING COUNTERS
        // -----------------------------------------------------

        if (zuptResult.active) {

            stationarySampleCount++

            movingSampleCount = 0

        } else {

            movingSampleCount++

            stationarySampleCount = 0
        }


        // -----------------------------------------------------
        // ZUPT + ZARU
        // -----------------------------------------------------

        var zuptAccepted =
            false

        var zaruAccepted =
            false


        if (zuptResult.active) {

            /*
             * Actual zero-velocity measurement update.
             */
            val zupt =
                eskf.applyZupt()

            zuptAccepted =
                zupt.accepted


            /*
             * IMPORTANT:
             *
             * ZARU receives the RAW vehicle-frame gyro
             * measurement.
             *
             * Do NOT subtract ESKF gyro bias here.
             */
            val zaru =
                eskf.applyZaru(

                    gyroBody =
                        Vec3(
                            x = gyroX,
                            y = gyroY,
                            z = gyroZ
                        )
                )

            zaruAccepted =
                zaru.accepted
        }


        // -----------------------------------------------------
        // NHC
        // -----------------------------------------------------

        var nhcAccepted =
            false

        /*
         * NHC is only meaningful during normal vehicle motion.
         *
         * We deliberately do not apply it while stationary.
         */
        val speedAfterZupt =
            eskf
                .getVelocity()
                .norm()

        val nhcEligible =
            !zuptResult.active &&
                    speedAfterZupt >=
                    MIN_NHC_SPEED_MPS &&
                    speedAfterZupt <=
                    MAX_NHC_SPEED_MPS


        if (nhcEligible) {

            val nhc =
                eskf.applyNhc()

            nhcAccepted =
                nhc.accepted
        }


        // -----------------------------------------------------
        // NAVIGATION SOLUTION
        // -----------------------------------------------------

        updateSolution(

            sample =
                sample,

            predictionAccepted =
                true,

            zuptActive =
                zuptResult.active,

            zuptAccepted =
                zuptAccepted,

            zaruAccepted =
                zaruAccepted,

            nhcAccepted =
                nhcAccepted
        )


        // -----------------------------------------------------
        // DEBUG LOG
        // -----------------------------------------------------

        /*
         * Log every 20 processed samples.
         *
         * At a 10 Hz SensorAdapter rate this is approximately
         * once every 2 seconds.
         */
        if (
            processedSampleCount % 20L == 0L
        ) {

            val state =
                eskf.getState()

            Log.d(
                TAG,
                "NAV " +
                        "speed=${"%.2f".format(
                            state.velocity.norm()
                        )}m/s " +
                        "stationary=${zuptResult.active} " +
                        "zupt=$zuptAccepted " +
                        "zaru=$zaruAccepted " +
                        "nhc=$nhcAccepted " +
                        "dvfc=${dvfcTransform != null} " +
                        "samples=$processedSampleCount"
            )
        }
    }


    // ---------------------------------------------------------
    // SOLUTION BUILDER
    // ---------------------------------------------------------

    private fun updateSolution(
        sample: DeviceSensorSample,
        predictionAccepted: Boolean,
        zuptActive: Boolean =
            false,
        zuptAccepted: Boolean =
            false,
        zaruAccepted: Boolean =
            false,
        nhcAccepted: Boolean =
            false
    ) {

        val state =
            eskf.getState()

        val velocity =
            state.velocity

        val speed =
            velocity.norm()


        /*
         * Heading from horizontal NED velocity.
         *
         * NED:
         *
         * velocity.x = North
         * velocity.y = East
         *
         * Heading:
         *
         * atan2(East, North)
         *
         * If speed is extremely low, keep the previous heading
         * instead of generating noisy atan2 output.
         */
        val horizontalVelocityMagnitude =
            sqrt(
                velocity.x * velocity.x +
                        velocity.y * velocity.y
            )

        val heading =
            if (
                horizontalVelocityMagnitude > 0.20
            ) {

                Math.toDegrees(
                    atan2(
                        velocity.y,
                        velocity.x
                    )
                ).let {
                    (it + 360.0) % 360.0
                }

            } else {

                latestSolution.headingDegrees
            }


        val horizontalSpeed =
            horizontalVelocityMagnitude


        latestSolution =
            NavigationSolution(

                timestampNanos =
                    state.timestampNanos,

                position =
                    state.position.copy(),

                velocity =
                    velocity.copy(),

                attitude =
                    state.attitude.copy(),

                gyroBias =
                    state.gyroBias.copy(),

                accelBias =
                    state.accelBias.copy(),

                speedMps =
                    speed,

                horizontalSpeedMps =
                    horizontalSpeed,

                headingDegrees =
                    heading,

                predictionAccepted =
                    predictionAccepted,

                stationary =
                    zuptActive,

                zuptAccepted =
                    zuptAccepted,

                zaruAccepted =
                    zaruAccepted,

                nhcAccepted =
                    nhcAccepted,

                dvfcActive =
                    dvfcTransform != null,

                sensorSampleCount =
                    processedSampleCount
            )
    }


    // ---------------------------------------------------------
    // REPLAY / BENCHMARK INPUT
    // ---------------------------------------------------------

    @Synchronized
    fun resetForReplay() {
        sensorAdapter.stop()
        running = true
        eskf.reset()
        lastTimestampNs = 0L
        processedSampleCount = 0L
        stationarySampleCount = 0
        movingSampleCount = 0
        latestSensorSample = null
        latestSolution = NavigationSolution()
        dvfcTransform = DvfcCalibrationStore.load(context.applicationContext)
    }

    fun processReplaySample(sample: DeviceSensorSample) {
        if (!running) resetForReplay()
        onSensorSample(sample)
    }

    // ---------------------------------------------------------
    // PUBLIC STATE
    // ---------------------------------------------------------

    fun state():
            NavigationSolution {

        return latestSolution
    }


    fun eskf():
            Eskf {

        return eskf
    }


    fun latestSensorSample():
            DeviceSensorSample? {

        return latestSensorSample
    }


    fun isRunning():
            Boolean {

        return running
    }


    fun isDvfcActive():
            Boolean {

        return dvfcTransform != null
    }


    fun processedSampleCount():
            Long {

        return processedSampleCount
    }


    fun stationarySampleCount():
            Int {

        return stationarySampleCount
    }


    fun movingSampleCount():
            Int {

        return movingSampleCount
    }
}


/**
 * =============================================================
 * ASTRANAV NAVIGATION OUTPUT
 * =============================================================
 *
 * This is the runtime-facing snapshot.
 *
 * The ESKF remains authoritative.
 */
data class NavigationSolution(

    val timestampNanos:
    Long = 0L,

    val position:
    Vec3 =
        Vec3.ZERO,

    val velocity:
    Vec3 =
        Vec3.ZERO,

    val attitude:
    Quaternion =
        Quaternion.IDENTITY,

    val gyroBias:
    Vec3 =
        Vec3.ZERO,

    val accelBias:
    Vec3 =
        Vec3.ZERO,

    val speedMps:
    Double = 0.0,

    val horizontalSpeedMps:
    Double = 0.0,

    val headingDegrees:
    Double = 0.0,

    val predictionAccepted:
    Boolean = false,

    val stationary:
    Boolean = false,

    val zuptAccepted:
    Boolean = false,

    val zaruAccepted:
    Boolean = false,

    val nhcAccepted:
    Boolean = false,

    val dvfcActive:
    Boolean = false,

    val sensorSampleCount:
    Long = 0L
)