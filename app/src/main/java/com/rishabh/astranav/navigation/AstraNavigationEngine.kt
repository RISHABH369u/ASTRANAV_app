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
import kotlin.math.asin
import com.rishabh.astranav.ml.AstraMlFusionBridge
import com.rishabh.astranav.sensor.ImuSample


import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.math.cos
import kotlin.math.sin

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
 * (
 * NEW 4/10/26 5:36AMK
 *                          ┌── ASTRA-Speed ─────┐
 *                          │                    │
 * IMU → SensorAdapter → DVFC → ESKF Prediction │
 *                          │                    ↓
 *                          │              ASTRA-GUARD
 *                          │                    │
 *                          │                    ↓
 *                          │             ESKF Speed Update
 *                          │
 *                          ├── ASTRA-SPHM ──→ consistency
 *                          │
 *                          └── ASTRA-Motion
 *                                       │
 *                                       ↓
 *                                ZUPT Detector
 *                                       │
 *                               ┌───────┴───────┐
 *                               ↓               ↓
 *                             ZUPT             ZARU
 *                               │
 *                               ↓
 *                              NHC
 *)
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

        private const val REPLAY_GRAVITY =
            9.80665

        private const val REPLAY_GRAVITY_MIN =
            8.5

        private const val REPLAY_GRAVITY_MAX =
            11.0

        // -------------------------------------------------
        // INS divergence recovery / aiding
        // -------------------------------------------------

        /** Consecutive prediction rejections before re-seeding. */
        private const val REJECTION_RESEED_STREAK =
            3

        /** INS speed this far above the learned speed is suspect. */
        private const val INS_ML_DIVERGENCE_MPS =
            25.0

        /** Samples of sustained divergence before re-seeding. */
        private const val INS_ML_DIVERGENCE_SAMPLES =
            15

        /** Sane upper bound for any INS speed (m/s). */
        private const val INS_SANE_SPEED_MPS =
            70.0

        /** Std dev given to a re-seeded velocity (m/s). */
        private const val RESEED_VELOCITY_STD_MPS =
            3.0

        /** Per-sample gravity tilt-correction gain. */
        private const val GRAVITY_TILT_GAIN =
            0.003

        /**
         * Gyro-bias learning gain (1/s^2). Critically damped for
         * the 0.03 1/s proportional loop: ki = kp^2 / 4.
         */
        private const val GRAVITY_TILT_BIAS_GAIN =
            2.25e-4

        /** Only tilt-correct when turning slower than this. */
        private const val GRAVITY_TILT_MAX_GYRO_RADPS =
            0.20
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



    /**
     * Builds an initial body -> NED attitude from the
     * measured gravity vector.
     *
     * Gravity gives us roll/pitch.
     *
     * Yaw is deliberately initialized to zero because
     * gravity cannot determine yaw.
     *
     * This is much safer than starting from identity when
     * the phone is mounted at an arbitrary tilt.
     */
    private fun replayInitialAttitudeFromGravity(
        gravity: FloatArray
    ): Quaternion {

        val gx =
            gravity[0].toDouble()

        val gy =
            gravity[1].toDouble()

        val gz =
            gravity[2].toDouble()

        val magnitude =
            sqrt(
                gx * gx +
                        gy * gy +
                        gz * gz
            )

        if (
            !magnitude.isFinite() ||
            magnitude < REPLAY_GRAVITY_MIN ||
            magnitude > REPLAY_GRAVITY_MAX
        ) {
            return Quaternion.IDENTITY
        }

        /*
         * Normalize gravity.
         */
        val nx = gx / magnitude
        val ny = gy / magnitude
        val nz = gz / magnitude

        /*
         * Initial tilt estimate.
         *
         * For the NED/FRD convention used by ASTRA:
         *
         *     X = forward / north
         *     Y = right / east
         *     Z = down
         *
         * Yaw remains zero.
         */
        val roll =
            atan2(
                ny,
                nz
            )

        val pitch =
            atan2(
                -nx,
                sqrt(
                    ny * ny +
                            nz * nz
                )
            )

        /*
         * Convert small-angle roll/pitch into a quaternion.
         *
         * yaw = 0.
         */
        val cr =
            cos(roll * 0.5)

        val sr =
            sin(roll * 0.5)

        val cp =
            cos(pitch * 0.5)

        val sp =
            sin(pitch * 0.5)

        val cy =
            1.0

        val sy =
            0.0

        val w =
            cr * cp * cy +
                    sr * sp * sy

        val x =
            sr * cp * cy -
                    cr * sp * sy

        val y =
            cr * sp * cy +
                    sr * cp * sy

        val z =
            cr * cp * sy -
                    sr * sp * cy

        return Quaternion(
            w = w,
            x = x,
            y = y,
            z = z
        ).normalized()
    }

    /**
     * Computes roll/pitch from the measured gravity vector.
     *
     * Coordinate convention:
     *
     * Navigation:
     *     X = North
     *     Y = East
     *     Z = Down
     *
     * Device:
     *     gravity is measured directly in device coordinates.
     *
     * We use gravity ONLY for tilt stabilization.
     * Yaw is intentionally not inferred from gravity.
     */
    private fun gravityTiltQuaternion(
        gravity: FloatArray
    ): Quaternion {

        val gx =
            gravity[0].toDouble()

        val gy =
            gravity[1].toDouble()

        val gz =
            gravity[2].toDouble()

        val magnitude =
            sqrt(
                gx * gx +
                        gy * gy +
                        gz * gz
            )

        if (
            !magnitude.isFinite() ||
            magnitude < 1.0
        ) {
            return Quaternion.IDENTITY
        }

        val nx =
            gx / magnitude

        val ny =
            gy / magnitude

        val nz =
            gz / magnitude

        /*
         * Gravity in body/device frame:
         *
         *     gx = -sin(pitch)
         *     gy =  cos(pitch) * sin(roll)
         *     gz =  cos(pitch) * cos(roll)
         *
         * Therefore:
         */
        val pitch =
            asin(
                (-nx)
                    .coerceIn(
                        -1.0,
                        1.0
                    )
            )

        val roll =
            atan2(
                ny,
                nz
            )

        /*
         * No yaw information exists in gravity.
         *
         * Build a tilt-only quaternion.
         *
         * This is used only as a diagnostic/reference
         * orientation during replay.
         */
        val halfRoll =
            roll * 0.5

        val halfPitch =
            pitch * 0.5

        val cr =
            kotlin.math.cos(
                halfRoll
            )

        val sr =
            kotlin.math.sin(
                halfRoll
            )

        val cp =
            kotlin.math.cos(
                halfPitch
            )

        val sp =
            kotlin.math.sin(
                halfPitch
            )

        /*
         * ZYX with yaw = 0.
         */
        return Quaternion(
            w =
                cr * cp,

            x =
                sr * cp,

            y =
                cr * sp,

            z =
                -sr * sp
        ).normalized()
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
    // INS HEALTH / RECOVERY STATE
    // ---------------------------------------------------------

    /** Consecutive rejected ESKF predictions. */
    private var consecutiveRejections =
        0

    /** Consecutive samples where INS speed >> learned speed. */
    private var insDivergenceSamples =
        0

    /** Last valid learned speed (m/s), or null if none yet. */
    private var lastMlSpeedMps: Double? =
        null

    /** Number of times velocity was re-seeded (diagnostics). */
    private var velocityReseedCount =
        0

// ---------------------------------------------------------
// ASTRA ML FUSION
// ---------------------------------------------------------

    private val mlFusionBridge =
        AstraMlFusionBridge(
            context = context.applicationContext,
            eskf = eskf
        )

    @Volatile
    private var latestMlFusion:
            AstraMlFusionBridge.FusionResult =
        AstraMlFusionBridge.FusionResult()

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

        mlFusionBridge.reset()

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

        consecutiveRejections =
            0

        insDivergenceSamples =
            0

        lastMlSpeedMps =
            null

        velocityReseedCount =
            0

        mlFusionBridge.insSpeedTrusted =
            true

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

        mlFusionBridge.reset()

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

        consecutiveRejections =
            0

        insDivergenceSamples =
            0

        lastMlSpeedMps =
            null

        velocityReseedCount =
            0

        mlFusionBridge.insSpeedTrusted =
            true

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

        /*
         * The ESKF body frame is FRD (see DatasetFrameConversion).
         * Express the recorded gravity/specific-force direction in
         * that frame, so a level phone gives f = [0, 0, -g] and
         * therefore an IDENTITY initial attitude.
         */
        val gravityFrd =
            DatasetFrameConversion.specificForceToFrd(gravity)

        val gravityBody =
            Vec3(
                x = gravityFrd[0].toDouble(),
                y = gravityFrd[1].toDouble(),
                z = gravityFrd[2].toDouble()
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

        /*
         * IO-VNBD replay uses the recorded accelerometer vector as
         * accelerometer specific force.
         *
         * At rest the accelerometer is approximately +g in the
         * recorded device frame, while ASTRA-Core NED gravity is +Z.
         *
         * For the ESKF equation:
         *
         *     a_n = R_nb * f_b + g_n
         *
         * stationary motion requires:
         *
         *     R_nb * f_b ~= -g_n
         *
         * Therefore the recorded +g specific-force direction must
         * be mapped to NED -Z, NOT +Z.
         */
        val target =
            Vec3(
                x = 0.0,
                y = 0.0,
                z = -1.0
            )

        val dot =
            from
                .dot(target)
                .coerceIn(-1.0, 1.0)

        val attitude =
            when {

                // Recorded +g specific-force direction is already
                // opposite NED gravity, so it needs no tilt rotation.
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
         * -------------------------------------------------------------
         * ACCELEROMETER SEMANTICS
         * -------------------------------------------------------------
         *
         * ASTRA-Core ESKF expects TRUE SPECIFIC FORCE:
         *
         *     a_n = R_nb * f_b + g_n
         *
         * where:
         *
         *     f_b = accelerometer specific force
         *     g_n = [0, 0, +9.80665] in NED
         *
         * IMPORTANT:
         *
         * The IO-VNBD replay accelerometer is already the quantity
         * that must be supplied to this ESKF path. At rest its
         * magnitude is approximately +9.8 m/s².
         *
         * DO NOT do:
         *
         *     acceleration - gravity
         *
         * and especially DO NOT do:
         *
         *     linearAcceleration - gravity
         *
         * because that would subtract gravity twice for this replay
         * representation and destroy the ESKF's own gravity
         * cancellation.
         *
         * LIVE MODE:
         *     SensorAdapter provides Android-style acceleration and
         *     gravity separately, so preserve the existing
         *     acceleration - gravity path there.
         *
         * REPLAY DATASET_FRAME:
         *     use recorded acceleration directly as specific force.
         */
        val deviceSpecificForce =
            if (
                replayFrameMode ==
                ReplayFrameMode.DATASET_FRAME
            ) {
                deviceAcceleration.copyOf()
            } else {
                floatArrayOf(
                    deviceAcceleration[0] -
                            sample.gravity[0],

                    deviceAcceleration[1] -
                            sample.gravity[1],

                    deviceAcceleration[2] -
                            sample.gravity[2]
                )
            }


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

        val specificForceNorm =
            sqrt(
                vehicleSpecificForce[0].toDouble() *
                        vehicleSpecificForce[0].toDouble() +

                        vehicleSpecificForce[1].toDouble() *
                        vehicleSpecificForce[1].toDouble() +

                        vehicleSpecificForce[2].toDouble() *
                        vehicleSpecificForce[2].toDouble()
            )

        val rawAccelerationNorm =
            sqrt(
                deviceAcceleration[0].toDouble() *
                        deviceAcceleration[0].toDouble() +

                        deviceAcceleration[1].toDouble() *
                        deviceAcceleration[1].toDouble() +

                        deviceAcceleration[2].toDouble() *
                        deviceAcceleration[2].toDouble()
            )

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

        /*
         * ESKF body frame is FRD (X fwd, Y right, Z down).
         *
         * Live mode: DVFC already produced the vehicle frame.
         * IO-VNBD replay: convert the recorded Android-style
         * channels (Z up, gyro columns [yaw, pitch, roll]) with
         * DatasetFrameConversion. The ML models keep the original
         * arrays below; only the mechanization uses these.
         */
        val eskfSpecificForce =
            if (
                replayFrameMode ==
                ReplayFrameMode.DATASET_FRAME
            ) {
                DatasetFrameConversion
                    .specificForceToFrd(vehicleSpecificForce)
            } else {
                vehicleSpecificForce
            }

        val eskfGyro =
            if (
                replayFrameMode ==
                ReplayFrameMode.DATASET_FRAME
            ) {
                DatasetFrameConversion
                    .gyroToFrd(vehicleGyro)
            } else {
                vehicleGyro
            }

        val prediction =
            eskf.predict(

                timestampNanos =
                    sample.timestampNs,

                accelerationBody =
                    Vec3(
                        x =
                            eskfSpecificForce[0]
                                .toDouble(),

                        y =
                            eskfSpecificForce[1]
                                .toDouble(),

                        z =
                            eskfSpecificForce[2]
                                .toDouble()
                    ),

                gyroBody =
                    Vec3(
                        x =
                            eskfGyro[0]
                                .toDouble(),

                        y =
                            eskfGyro[1]
                                .toDouble(),

                        z =
                            eskfGyro[2]
                                .toDouble()
                    )
            )


        /*
 * ---------------------------------------------------------
 * REPLAY INITIAL ATTITUDE
 * ---------------------------------------------------------
 *
 * The old implementation started the IO-VNBD replay
 * from Quaternion.IDENTITY.
 *
 * That is wrong when the recorded phone is tilted.
 *
 * Gravity gives us the initial roll/pitch.
 */
        if (
            replayFrameMode ==
            ReplayFrameMode.DATASET_FRAME &&
            processedSampleCount == 1L
        ) {

            /*
             * NOTE: the initial attitude is already set by
             * initializeReplayAttitudeFromGravity() in
             * processReplaySample(). The previous code re-ran
             * an init here through Eskf.initializeReplayAttitude(),
             * which only modified a COPY of the state (getState()
             * returns a copy), so it was a no-op and has been
             * removed to avoid a second, conflicting convention.
             */
            Log.i(
                TAG,
                "REPLAY_ATTITUDE_INIT " +
                        "gravity=[" +
                        "%.3f".format(sample.gravity[0]) +
                        "," +
                        "%.3f".format(sample.gravity[1]) +
                        "," +
                        "%.3f".format(sample.gravity[2]) +
                        "]"
            )

            updateSolution(
                sample = sample,
                predictionAccepted = true
            )

            return
        }


        if (!prediction.accepted) {

            Log.w(
                TAG,
                "ESKF prediction rejected: " +
                        prediction.reason +
                        " timestamp=${sample.timestampNs}ns"
            )

            /*
             * RECOVERY:
             *
             * A "safety limit" rejection does not commit the state,
             * so if the inertial error persists, EVERY following
             * sample is rejected too and the filter is dead (no
             * ZUPT / NHC / learned-speed / GNSS update ever runs).
             *
             * After a short streak, re-seed velocity from the
             * learned speed so the filter can come back.
             */
            if (
                prediction.reason
                    ?.contains("safety limit") == true
            ) {

                consecutiveRejections++

                mlFusionBridge.insSpeedTrusted =
                    false

                if (
                    consecutiveRejections >=
                    REJECTION_RESEED_STREAK
                ) {

                    reseedVelocityFromLearnedSpeed(
                        "prediction rejected " +
                                "$consecutiveRejections times"
                    )

                    consecutiveRejections =
                        0
                }
            }

            /*
             * IMPORTANT:
             *
             * The ESKF rejected this sample, so we do not use
             * the rejected state as a navigation update.
             *
             * However, replay itself must continue on the real
             * 10 Hz timeline.
             *
             * Otherwise one rejected sample causes dt to grow:
             *
             * 0.1 -> 0.2 -> 0.3 -> 1.0 -> 10 seconds
             *
             * which makes every following prediction invalid.
             */
            lastTimestampNs =
                sample.timestampNs

            hasLastTimestamp =
                true

            updateSolution(
                sample = sample,
                predictionAccepted = false
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

        consecutiveRejections =
            0


        // -----------------------------------------------------
        // GRAVITY TILT AIDING
        // -----------------------------------------------------

        /*
         * Attitude is pure gyro integration otherwise. When the
         * vehicle is not turning hard and |f| ~ g, nudge
         * roll/pitch toward the gravity direction so small gyro
         * errors cannot accumulate into a large tilt.
         */
        val gyroMagnitude =
            sqrt(
                eskfGyro[0].toDouble() * eskfGyro[0] +
                        eskfGyro[1].toDouble() * eskfGyro[1] +
                        eskfGyro[2].toDouble() * eskfGyro[2]
            )

        val insSpeedForTilt =
            eskf.getVelocity().norm()

        if (
            gyroMagnitude < GRAVITY_TILT_MAX_GYRO_RADPS &&
            mlFusionBridge.insSpeedTrusted &&
            insSpeedForTilt.isFinite() &&
            insSpeedForTilt < INS_SANE_SPEED_MPS
        ) {

            eskf.applyGravityTilt(
                specificForceBody =
                    Vec3(
                        eskfSpecificForce[0].toDouble(),
                        eskfSpecificForce[1].toDouble(),
                        eskfSpecificForce[2].toDouble()
                    ),

                gain =
                    GRAVITY_TILT_GAIN,

                dtSeconds =
                    prediction.deltaTimeSeconds,

                biasGain =
                    GRAVITY_TILT_BIAS_GAIN,

                /*
                 * Centripetal acceleration in FRD:
                 * turning right (+wz) accelerates toward +Y.
                 */
                expectedLinearAccelBody =
                    Vec3(
                        0.0,
                        insSpeedForTilt *
                                eskfGyro[2].toDouble(),
                        0.0
                    )
            )
        }


        // -----------------------------------------------------
        // INS HEALTH (feeds the ML speed prior)
        // -----------------------------------------------------

        val insSpeedNow =
            eskf.getVelocity().norm()

        val mlSpeedRef =
            lastMlSpeedMps

        val insDiverging =
            !insSpeedNow.isFinite() ||
                    insSpeedNow > INS_SANE_SPEED_MPS ||
                    (
                            mlSpeedRef != null &&
                                    insSpeedNow - mlSpeedRef >
                                    INS_ML_DIVERGENCE_MPS
                            )

        insDivergenceSamples =
            if (insDiverging) {
                insDivergenceSamples + 1
            } else {
                0
            }

        mlFusionBridge.insSpeedTrusted =
            insDivergenceSamples == 0

        if (
            insDivergenceSamples >=
            INS_ML_DIVERGENCE_SAMPLES
        ) {

            reseedVelocityFromLearnedSpeed(
                "INS speed ${"%.1f".format(insSpeedNow)} m/s " +
                        "vs learned " +
                        "${mlSpeedRef?.let { "%.1f".format(it) } ?: "--"} m/s"
            )

            insDivergenceSamples =
                0
        }


        // -----------------------------------------------------
// ASTRA ML FUSION
// -----------------------------------------------------

        /*
         * Convert the synchronized DeviceSensorSample into the
         * common ImuSample contract used by the ML engines.
         *
         * IMPORTANT:
         *
         * ASTRA-Speed and ASTRA-SPHM intentionally receive the
         * original sensor-frame quantities because their trained
         * feature contracts were built from those quantities.
         *
         * ASTRA-Motion receives vehicle-frame quantities below.
         */
        val mlImuSample =
            ImuSample(

                timestampNanos =
                    sample.timestampNs,

                accelX =
                    sample.acceleration[0].toDouble(),

                accelY =
                    sample.acceleration[1].toDouble(),

                accelZ =
                    sample.acceleration[2].toDouble(),

                gyroX =
                    sample.angularVelocity[0].toDouble(),

                gyroY =
                    sample.angularVelocity[1].toDouble(),

                gyroZ =
                    sample.angularVelocity[2].toDouble(),

                gravityX =
                    sample.gravity[0].toDouble(),

                gravityY =
                    sample.gravity[1].toDouble(),

                gravityZ =
                    sample.gravity[2].toDouble()
            )


        /*
         * Feed all three ML models through one controlled bridge.
         *
         * ASTRA-Motion receives the DVFC/device→vehicle frame.
         *
         * ASTRA-Speed and ASTRA-SPHM preserve their trained
         * smartphone feature contract.
         */
        val mlFusion =
            mlFusionBridge.process(

                sample =
                    mlImuSample,

                vehicleSpecificForce =
                    doubleArrayOf(
                        vehicleSpecificForce[0].toDouble(),
                        vehicleSpecificForce[1].toDouble(),
                        vehicleSpecificForce[2].toDouble()
                    ),

                vehicleGyro =
                    doubleArrayOf(
                        vehicleGyro[0].toDouble(),
                        vehicleGyro[1].toDouble(),
                        vehicleGyro[2].toDouble()
                    )
            )

        latestMlFusion =
            mlFusion

        /*
         * Remember the learned speed (mean of the valid models)
         * as the fallback reference for divergence detection and
         * velocity re-seeding.
         */
        run {
            val learned =
                listOfNotNull(
                    mlFusion.speedOutput
                        ?.takeIf { it.valid }
                        ?.speedMps,

                    mlFusion.sphmOutput
                        ?.takeIf { it.valid }
                        ?.speedMps
                ).map { it.toDouble() }
                    .filter { it.isFinite() && it >= 0.0 }

            if (learned.isNotEmpty()) {
                lastMlSpeedMps =
                    learned.average()
            }
        }


// -----------------------------------------------------
// CURRENT TRUSTED SPEED
// -----------------------------------------------------

        /*
         * IMPORTANT:
         *
         * Read speed AFTER ASTRA-Speed has had a chance to
         * correct ESKF velocity magnitude.
         */
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
 * ASTRA-Motion supplies the learned stationary/motion
 * probability to the existing ZUPT detector.
 *
 * The detector still combines:
 *
 *      gyro
 *      acceleration
 *      trusted ESKF speed
 *      ASTRA-Motion ZUPT evidence
 *
 * ASTRA-Motion therefore does NOT directly trigger ZUPT.
 */
                zuptLogit =
                    mlFusion.motionZuptScore
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

                    /*
                     * ZARU observes yaw rate about the ESKF body Z
                     * axis, so it must use the ESKF-frame (FRD)
                     * gyro, not the raw dataset-ordered columns.
                     */
                    gyroBody =
                        Vec3(
                            x =
                                eskfGyro[0].toDouble(),

                            y =
                                eskfGyro[1].toDouble(),

                            z =
                                eskfGyro[2].toDouble()
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
                        "fBody=[${"%.3f".format(sf.x)}," +
                        "${"%.3f".format(sf.y)}," +
                        "${"%.3f".format(sf.z)}] " +
                        "fNav=[${"%.3f".format(prediction.specificForceNavigation.x)}," +
                        "${"%.3f".format(prediction.specificForceNavigation.y)}," +
                        "${"%.3f".format(prediction.specificForceNavigation.z)}] " +
                        "NAV_A=[${"%.3f".format(navAcc.x)}," +
                        "${"%.3f".format(navAcc.y)}," +
                        "${"%.3f".format(navAcc.z)}] " +
                        "NAV_A_NORM=${"%.3f".format(navAcc.norm())} " +
                        "VEL=${"%.3f".format(state.velocity.norm())}m/s " +
                        "zupt=${zuptResult.active} " +
                        "nhc=$nhcAccepted " +
                        "mlSpeed=${
                            mlFusion.speedOutput
                                ?.speedMps
                                ?.let { "%.3f".format(it) }
                                ?: "--"
                        } " +
                        "mlSPHM=${
                            mlFusion.sphmOutput
                                ?.speedMps
                                ?.let { "%.3f".format(it) }
                                ?: "--"
                        } " +
                        "mlZupt=${
                            mlFusion.motionZuptScore
                                ?.let { "%.3f".format(it) }
                                ?: "--"
                        } " +
                        "mlSpeedAccepted=${
                            mlFusion.speedUpdate?.accepted ?: false
                        } " +
                        "mlStd=${
                            "%.2f".format(mlFusion.speedStdMps)
                        } " +
                        "guard=${mlFusion.guardReason}"
            )
        }


    }


    // ---------------------------------------------------------
    // INS DIVERGENCE RECOVERY
    // ---------------------------------------------------------

    /**
     * Replaces the (diverged) ESKF velocity with the learned
     * speed along the current body-forward direction projected
     * on the horizontal plane, and widens the velocity
     * covariance.
     *
     * If no learned speed is available yet the velocity is
     * re-seeded to zero.
     */
    private fun reseedVelocityFromLearnedSpeed(
        reason: String
    ) {

        val speed =
            (lastMlSpeedMps ?: 0.0)
                .coerceIn(0.0, INS_SANE_SPEED_MPS)

        val forwardNav =
            eskf
                .getAttitude()
                .rotate(
                    Vec3(1.0, 0.0, 0.0)
                )

        val horizontal =
            Vec3(
                forwardNav.x,
                forwardNav.y,
                0.0
            )

        val direction =
            if (horizontal.norm() > 0.3) {
                horizontal / horizontal.norm()
            } else {
                // Body X points almost vertically: heading unknown.
                Vec3(1.0, 0.0, 0.0)
            }

        eskf.reseedVelocity(
            velocity =
                direction * speed,

            velocityStdMps =
                RESEED_VELOCITY_STD_MPS
        )

        velocityReseedCount++

        Log.w(
            TAG,
            "INS_RESEED velocity -> ${"%.2f".format(speed)} m/s " +
                    "($reason) count=$velocityReseedCount"
        )
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
    fun latestMlFusion():
            AstraMlFusionBridge.FusionResult {

        return latestMlFusion
    }

    fun mlFusion():
            AstraMlFusionBridge {

        return mlFusionBridge
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