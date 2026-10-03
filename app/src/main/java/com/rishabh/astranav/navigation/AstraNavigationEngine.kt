package com.rishabh.astranav.navigation

import android.content.Context
import android.util.Log

import com.rishabh.astranav.dvfc.DvfcCalibrationStore
import com.rishabh.astranav.dvfc.sensor.DeviceSensorSample
import com.rishabh.astranav.dvfc.sensor.SensorAdapter
import com.rishabh.astranav.navigation.eskf.Eskf
import com.rishabh.astranav.navigation.eskf.Quaternion
import com.rishabh.astranav.navigation.eskf.Vec3
import com.rishabh.astranav.navigation.zupt.ZuptDetector
import com.rishabh.astranav.replay.ReplayFrameMode


import kotlin.math.atan2
import kotlin.math.sqrt


/**
 * =============================================================
 * ASTRANAV NAVIGATION RUNTIME
 * =============================================================
 *
 * LIVE:
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
 *
 * IO-VNBD REPLAY:
 *
 * IO-VNBD S-CSV
 *       ↓
 * IoVnbdReplaySession
 *       ↓
 * DeviceSensorSample
 *       ↓
 * ReplayFrameMode.DATASET_FRAME
 *       ↓
 * NO CURRENT-PHONE DVFC
 *       ↓
 * ASTRA-Core ESKF
 *
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
    // REPLAY FRAME MODE
    // ---------------------------------------------------------

    /**
     * Current replay interpretation mode.
     *
     * null:
     *     Normal live navigation.
     *
     * DATASET_FRAME:
     *     IO-VNBD replay. Current-device DVFC is disabled.
     *
     * DVFC:
     *     Experimental replay using persisted DVFC.
     */
    @Volatile
    private var replayFrameMode:
            ReplayFrameMode? =
        null


    /**
     * Explicitly changes the replay frame mode.
     *
     * This method is useful for controlled replay experiments.
     *
     * DATASET_FRAME:
     *     Forces DVFC off.
     *
     * DVFC:
     *     Loads persisted calibration.
     *
     * null:
     *     Returns to normal live-mode calibration behavior.
     */
    @Synchronized
    fun setReplayFrameMode(
        mode: ReplayFrameMode?
    ) {

        replayFrameMode =
            mode

        dvfcTransform =
            when (mode) {

                ReplayFrameMode.DATASET_FRAME ->
                    null

                ReplayFrameMode.DVFC ->
                    DvfcCalibrationStore.load(
                        context.applicationContext
                    )

                null ->
                    DvfcCalibrationStore.load(
                        context.applicationContext
                    )
            }

        Log.i(
            TAG,
            "Replay frame mode = $mode, " +
                    "DVFC=${dvfcTransform != null}"
        )
    }


    // ---------------------------------------------------------
    // SENSOR ADAPTER
    // ---------------------------------------------------------

    private val sensorAdapter =
        SensorAdapter(
            context =
                context.applicationContext
        ) { sample ->

            onSensorSample(
                sample
            )
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
     * Last timestamp seen by the navigation runtime.
     */
    private var lastTimestampNs =
        0L

    /*
 * True after the first valid sample has established
 * the navigation timeline.
 *
 * IMPORTANT:
 * Timestamp 0 is valid for deterministic replay.
 * Therefore lastTimestampNs == 0L cannot be used
 * as the initialization sentinel.
 */
    private var hasLastTimestamp =
        false

    /*
     * Number of samples received while the
     * navigation runtime is running.
     *
     * Incremented exactly once inside onSensorSample().
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
     * In replay DATASET_FRAME mode this is explicitly null.
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
         * Starting normal live navigation exits replay mode.
         */
        replayFrameMode =
            null

        /*
         * Refresh the latest persisted calibration.
         */
        dvfcTransform =
            DvfcCalibrationStore.load(
                context.applicationContext
            )

        running =
            true

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

        running =
            false

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

        running =
            false

        eskf.reset()

        /*
         * Return to normal live-navigation mode.
         */
        replayFrameMode =
            null

        /*
         * Reset runtime timing.
         */
        lastTimestampNs =
            0L

        hasLastTimestamp =
            false
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
    // REPLAY SUPPORT
    // ---------------------------------------------------------

    /**
     * Resets the navigation runtime for offline replay.
     *
     * Unlike reset(), this immediately enables the runtime
     * without starting the Android SensorAdapter.
     *
     * DATASET_FRAME:
     *     No current-device DVFC is used.
     *
     * DVFC:
     *     Persisted Device → Vehicle calibration is used.
     */
    @Synchronized
    fun resetForReplay(
        frameMode: ReplayFrameMode =
            ReplayFrameMode.DATASET_FRAME
    ) {

        sensorAdapter.stop()

        running =
            true

        eskf.reset()

        lastTimestampNs =
            0L

        hasLastTimestamp =
            false

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

        replayFrameMode =
            frameMode

        dvfcTransform =
            when (frameMode) {

                ReplayFrameMode.DATASET_FRAME ->
                    null

                ReplayFrameMode.DVFC ->
                    DvfcCalibrationStore.load(
                        context.applicationContext
                    )
            }

        Log.i(
            TAG,
            "ASTRANAV replay runtime reset. " +
                    "frameMode=$frameMode " +
                    "DVFC=${dvfcTransform != null}"
        )
    }


    /**
     * Initializes the replay body -> navigation attitude from
     * the recorded IO-VNBD gravity vector.
     *
     * IO-VNBD gravity is expressed in the phone/device frame and
     * points in the direction of gravity. ASTRA-Core uses NED:
     *
     *     navigation +Z = Down
     *
     * We therefore construct the shortest rotation that maps
     * the measured device gravity direction onto NED +Z.
     *
     * This determines roll/pitch without inventing a yaw.
     * Yaw is intentionally left at zero because the recorded
     * IO-VNBD Euler convention has not been validated as a
     * body -> NED quaternion convention.
     */
    @Synchronized
    fun initializeReplayAttitudeFromGravity(
        gravity: FloatArray
    ) {

        if (replayFrameMode != ReplayFrameMode.DATASET_FRAME) {
            return
        }

        if (gravity.size < 3) {
            return
        }

        val gravityBody =
            Vec3(
                x = gravity[0].toDouble(),
                y = gravity[1].toDouble(),
                z = gravity[2].toDouble()
            )

        val gravityMagnitude =
            gravityBody.norm()

        if (
            !gravityMagnitude.isFinite() ||
            gravityMagnitude < 1e-6
        ) {
            Log.w(
                TAG,
                "Replay attitude init skipped: invalid gravity " +
                        "magnitude=$gravityMagnitude"
            )
            return
        }

        val from =
            gravityBody /
                    gravityMagnitude

        // ASTRA-Core NED gravity direction: +Z = Down.
        val target =
            Vec3(
                x = 0.0,
                y = 0.0,
                z = 1.0
            )

        val dot =
            from
                .dot(target)
                .coerceIn(-1.0, 1.0)

        val attitude =
            when {

                // Already aligned with NED Down.
                dot > 0.999999 -> {
                    Quaternion.IDENTITY
                }

                // Exactly opposite: choose a deterministic 180°
                // rotation around the body X axis.
                dot < -0.999999 -> {
                    Quaternion(
                        w = 0.0,
                        x = 1.0,
                        y = 0.0,
                        z = 0.0
                    )
                }

                else -> {

                    val cross =
                        from.cross(target)

                    Quaternion(
                        w = 1.0 + dot,
                        x = cross.x,
                        y = cross.y,
                        z = cross.z
                    ).normalized()
                }
            }

        eskf.setInitialState(
            position = Vec3.ZERO,
            velocity = Vec3.ZERO,
            attitude = attitude,
            gyroBias = Vec3.ZERO,
            accelBias = Vec3.ZERO,
            timestampNanos = 0L
        )

        Log.i(
            TAG,
            "Replay attitude initialized from gravity: " +
                    "g=${"%.4f".format(gravityMagnitude)} " +
                    "q=[w=${"%.6f".format(attitude.w)}, " +
                    "x=${"%.6f".format(attitude.x)}, " +
                    "y=${"%.6f".format(attitude.y)}, " +
                    "z=${"%.6f".format(attitude.z)}]"
        )
    }

    /**
     * Injects one offline replay sample through the same
     * navigation pipeline used by the live runtime.
     */
    fun processReplaySample(
        sample: DeviceSensorSample
    ) {

        if (!running) {

            resetForReplay(
                ReplayFrameMode.DATASET_FRAME
            )
        }

        /*
         * The first replay sample establishes the initial
         * body -> NED attitude before ESKF prediction.
         *
         * processedSampleCount is still zero here, because
         * onSensorSample() increments it only after entry.
         */
        if (
            replayFrameMode ==
            ReplayFrameMode.DATASET_FRAME &&
            processedSampleCount == 0L
        ) {

            initializeReplayAttitudeFromGravity(
                sample.gravity
            )
        }

        onSensorSample(
            sample
        )
    }


    // ---------------------------------------------------------
    // REFRESH DVFC
    // ---------------------------------------------------------

    /**
     * Reloads the currently persisted Device → Vehicle
     * transform.
     *
     * IMPORTANT:
     *
     * If replay is explicitly running in DATASET_FRAME mode,
     * DVFC remains disabled.
     */
    @Synchronized
    fun refreshDvfcTransform() {

        if (
            replayFrameMode ==
            ReplayFrameMode.DATASET_FRAME
        ) {

            dvfcTransform =
                null

            Log.i(
                TAG,
                "DVFC refresh ignored: " +
                        "replay is using DATASET_FRAME"
            )

            return
        }

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
 * TIMESTAMP VALIDATION
 *
 * Timestamp 0 is VALID for deterministic replay.
 *
 * Valid replay timeline:
 *
 *     0 ns
 *     100,000,000 ns
 *     200,000,000 ns
 *     300,000,000 ns
 *     ...
 *
 * Only negative timestamps are invalid.
 *
 * IMPORTANT:
 * lastTimestampNs represents the LAST ACCEPTED ESKF
 * prediction, not merely the last sample received.
 */

        if (sample.timestampNs < 0L) {

            Log.w(
                TAG,
                "Ignoring invalid negative timestamp: " +
                        sample.timestampNs
            )

            return
        }

        if (
            hasLastTimestamp &&
            sample.timestampNs <=
            lastTimestampNs
        ) {

            Log.w(
                TAG,
                "Ignoring non-monotonic sample: " +
                        "current=${sample.timestampNs} " +
                        "previous=$lastTimestampNs"
            )

            return
        }

        /*
         * DO NOT update lastTimestampNs here.
         *
         * ESKF must decide whether this timestamp becomes
         * part of the authoritative navigation timeline.
         */


        // -----------------------------------------------------
        // DEVICE → VEHICLE FRAME
        // -----------------------------------------------------

        val deviceAcceleration =
            sample.acceleration

        val deviceGyro =
            sample.angularVelocity


        /*
         * IO-VNBD / Android-style accelerometer semantics:
         *
         *     linearAcceleration = acceleration - gravity
         *
         * ASTRA-Core ESKF uses NED gravity:
         *
         *     g_n = [0, 0, +9.80665]
         *
         * and its mechanization expects TRUE inertial specific
         * force f_b, where:
         *
         *     a_n = R_nb * f_b + g_n
         *
         * Therefore, in the recorded device frame:
         *
         *     f_b = linearAcceleration - gravity
         *
         * which is:
         *
         *     f_b = acceleration - 2 * gravity
         *
         * At rest, acceleration ~= gravity, so:
         *
         *     f_b ~= -gravity
         *
         * and after rotation into NED:
         *
         *     f_n + g_n ~= 0
         *
         * This is the required gravity cancellation.
         */
        val deviceSpecificForce =
            floatArrayOf(

                sample.linearAcceleration[0] -
                        sample.gravity[0],

                sample.linearAcceleration[1] -
                        sample.gravity[1],

                sample.linearAcceleration[2] -
                        sample.gravity[2]
            )


        /*
         * Apply the persisted DVFC transform only when
         * the current runtime mode allows it.
         *
         * DATASET_FRAME:
         *     dvfcTransform is guaranteed to be null.
         *
         * LIVE:
         *     persisted DVFC is used when available.
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
                        prediction.reason +
                        " timestamp=${sample.timestampNs}ns"
            )

            /*
             * IMPORTANT:
             *
             * Do NOT advance lastTimestampNs.
             *
             * ESKF rejected this prediction, therefore the
             * authoritative navigation state has not advanced.
             */
            updateSolution(
                sample =
                    sample,

                predictionAccepted =
                    false
            )

            return
        }

        /*
         * Prediction was accepted.
         *
         * Only now commit this timestamp as the last
         * authoritative navigation timestamp.
         */
        lastTimestampNs =
            sample.timestampNs

        hasLastTimestamp =
            true


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

            movingSampleCount =
                0

        } else {

            movingSampleCount++

            stationarySampleCount =
                0
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
                            x =
                                gyroX,

                            y =
                                gyroY,

                            z =
                                gyroZ
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
         * Replay physics diagnostic.
         *
         * Log every 10 samples (~1 second at 10 Hz).
         * The important values are:
         *
         *   SF  = specific force entering ESKF
         *   NAV = navigation-frame acceleration after
         *         gravity is added by ESKF
         *   VEL = resulting speed
         *
         * For a healthy replay, NAV should NOT contain a
         * persistent +9.8 m/s² vertical component.
         */
        if (
            replayFrameMode ==
            ReplayFrameMode.DATASET_FRAME &&
            processedSampleCount % 10L == 0L
        ) {

            val state =
                eskf.getState()

            val sf =
                prediction.specificForceBody

            val navAcc =
                prediction.accelerationNavigation

            Log.d(
                TAG,
                "REPLAY_PHYSICS " +
                        "t=${sample.timestampNs / 1_000_000L}ms " +
                        "rawA=[${"%.3f".format(deviceAcceleration[0])}," +
                        "${"%.3f".format(deviceAcceleration[1])}," +
                        "${"%.3f".format(deviceAcceleration[2])}] " +
                        "g=[${"%.3f".format(sample.gravity[0])}," +
                        "${"%.3f".format(sample.gravity[1])}," +
                        "${"%.3f".format(sample.gravity[2])}] " +
                        "SF=${"%.3f".format(sf.norm())} " +
                        "NAV_A=${"%.3f".format(navAcc.norm())} " +
                        "NAV_A_Z=${"%.3f".format(navAcc.z)} " +
                        "VEL=${"%.3f".format(state.velocity.norm())}m/s " +
                        "zupt=${zuptResult.active} " +
                        "nhc=$nhcAccepted"
            )
        }
    }


    // ---------------------------------------------------------
    // SOLUTION BUILDER
    // ---------------------------------------------------------

    private fun updateSolution(
        sample:
        DeviceSensorSample,

        predictionAccepted:
        Boolean,

        zuptActive:
        Boolean =
            false,

        zuptAccepted:
        Boolean =
            false,

        zaruAccepted:
        Boolean =
            false,

        nhcAccepted:
        Boolean =
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
                velocity.x *
                        velocity.x +
                        velocity.y *
                        velocity.y
            )

        val heading =
            if (
                horizontalVelocityMagnitude >
                0.20
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