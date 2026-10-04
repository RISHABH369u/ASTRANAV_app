package com.rishabh.astranav.ml

import android.content.Context
import android.util.Log

import com.rishabh.astranav.ml.astramotion.AstraMotionEngine
import com.rishabh.astranav.ml.astramotion.AstraMotionOutput
import com.rishabh.astranav.ml.astrasphm.AstraSphmEngine
import com.rishabh.astranav.ml.astrasphm.AstraSphmOutput
import com.rishabh.astranav.ml.gru.GruSpeedEngine
import com.rishabh.astranav.ml.gru.GruSpeedOutput

import com.rishabh.astranav.navigation.eskf.Eskf
import com.rishabh.astranav.navigation.eskf.EskfLearnedSpeedUpdate
import com.rishabh.astranav.sensor.ImuSample

import kotlin.math.abs
import kotlin.math.sqrt


/**
 * ================================================================
 * ASTRA ML FUSION BRIDGE
 * ================================================================
 *
 * Connects:
 *
 *      ASTRA-Speed
 *      ASTRA-Motion
 *      ASTRA-SPHM
 *
 * with:
 *
 *      ASTRA-Core ESKF
 *      ASTRA-GUARD integrity logic
 *
 *
 * IMPORTANT ARCHITECTURE RULE
 * ---------------------------
 *
 * None of the ML models is allowed to directly overwrite:
 *
 *      ESKF position
 *      ESKF velocity
 *      ESKF attitude
 *
 * ML models provide measurements / evidence.
 *
 * ESKF remains the authoritative navigation state.
 *
 *
 * Runtime flow:
 *
 *      IMU
 *       |
 *       +------------------> ASTRA-Speed
 *       |
 *       +------------------> ASTRA-SPHM
 *       |
 *       +--> DVFC --> ASTRA-Motion
 *                         |
 *                         v
 *                    ASTRA-GUARD
 *                         |
 *                         v
 *                    ASTRA-Core
 *
 *
 * ASTRA-Speed:
 *      primary learned speed measurement
 *
 * ASTRA-Motion:
 *      motion context + ZUPT evidence
 *
 * ASTRA-SPHM:
 *      secondary speed / heading consistency evidence
 *
 * ASTRA-GUARD:
 *      validates model outputs and controls trust
 *
 *
 *
 *
 *                           RAW IMU
 *                        │
 *           ┌────────────┼────────────┐
 *           ↓            ↓            ↓
 *      ASTRA-Speed    ASTRA-SPHM   DVFC FRAME
 *        GRU                           │
 *           │            │             ↓
 *           │            │       ASTRA-Motion
 *           │            │             │
 *           └──────┬─────┴─────────────┘
 *                  ↓
 *             ASTRA-GUARD
 *                  │
 *                  │ speed + uncertainty
 *                  ↓
 *           Eskf.applyLearnedSpeed()
 *                  │
 *                  ↓
 *              ASTRA-CORE
 *                  │
 *                  ↓
 *         AUTHORITATIVE STATE
 */
class AstraMlFusionBridge(
    context: Context,
    private val eskf: Eskf
) {

    companion object {

        private const val TAG =
            "ASTRA_ML_FUSION"

        /*
         * ASTRA-Speed normal uncertainty.
         *
         * This is deliberately conservative for the first
         * integration pass.
         */
        private const val BASE_SPEED_STD_MPS =
            1.25

        /*
         * Never allow a learned speed measurement to have
         * unrealistically small uncertainty.
         */
        private const val MIN_SPEED_STD_MPS =
            0.75

        /*
         * Maximum uncertainty supplied to ESKF.
         */
        private const val MAX_SPEED_STD_MPS =
            6.0

        /*
         * If ASTRA-Speed and ASTRA-SPHM disagree by more
         * than this amount, trust in the learned speed
         * measurement is reduced.
         */
        private const val SPEED_DISAGREEMENT_SOFT_MPS =
            2.0

        private const val SPEED_DISAGREEMENT_HARD_MPS =
            5.0

        /*
         * Heading consistency is not used as a direct ESKF
         * correction yet.
         *
         * It is only an integrity signal in this version.
         */
        private const val MAX_REASONABLE_HEADING_DELTA_RAD =
            Math.PI

        /*
         * Physical speed sanity limit.
         */
        private const val MAX_SPEED_MPS =
            50.0
    }


    // ============================================================
    // MODEL ENGINES
    // ============================================================

    private val speedEngine =
        GruSpeedEngine(
            context.applicationContext
        )

    private val motionEngine =
        AstraMotionEngine(
            context.applicationContext
        )

    private val sphmEngine =
        AstraSphmEngine(
            context.applicationContext
        )


    // ============================================================
    // RUNTIME STATE
    // ============================================================

    /**
     * Previous trusted speed supplied to ASTRA-Speed.
     *
     * IMPORTANT:
     *
     * This is NOT simply the previous raw GRU output.
     *
     * After fusion it becomes the ESKF's trusted speed.
     */
    private var previousTrustedSpeedMps =
        0.0

    private var previousYawRate =
        0.0

    private var previousTimestampNanos =
        0L

    private var initialized =
        false


    // ============================================================
    // DIAGNOSTICS
    // ============================================================

    private var speedInferenceCount =
        0L

    private var motionInferenceCount =
        0L

    private var sphmInferenceCount =
        0L

    private var speedAcceptedCount =
        0L

    private var speedRejectedCount =
        0L

    private var guardRejectedCount =
        0L

    private var lastResult =
        FusionResult()


    // ============================================================
    // INITIALIZATION
    // ============================================================

    init {

        try {

            sphmEngine.initialize()

        } catch (e: Exception) {

            Log.e(
                TAG,
                "ASTRA-SPHM initialization failed",
                e
            )
        }

        initialized = true

        Log.i(
            TAG,
            "ASTRA ML Fusion Bridge initialized"
        )
    }


    // ============================================================
    // RESET
    // ============================================================

    fun reset() {

        speedEngine.reset()

        motionEngine.reset()

        sphmEngine.clearWindow()

        previousTrustedSpeedMps =
            0.0

        previousYawRate =
            0.0

        previousTimestampNanos =
            0L

        speedInferenceCount =
            0L

        motionInferenceCount =
            0L

        sphmInferenceCount =
            0L

        speedAcceptedCount =
            0L

        speedRejectedCount =
            0L

        guardRejectedCount =
            0L

        lastResult =
            FusionResult()

        Log.i(
            TAG,
            "ASTRA ML Fusion Bridge reset"
        )
    }


    // ============================================================
    // MAIN PROCESSING ENTRY POINT
    // ============================================================

    /**
     * Process one synchronized 10 Hz IMU sample.
     *
     * vehicleSpecificForce:
     *
     *      specific force after DVFC,
     *      expressed in vehicle/body frame.
     *
     * vehicleGyro:
     *
     *      gyro after DVFC,
     *      expressed in vehicle/body frame.
     *
     * raw sample:
     *
     *      original SensorAdapter IMU representation.
     *
     * ESKF remains authoritative.
     */
    @Synchronized
    fun process(
        sample: ImuSample,
        vehicleSpecificForce: DoubleArray,
        vehicleGyro: DoubleArray
    ): FusionResult {

        if (
            vehicleSpecificForce.size < 3 ||
            vehicleGyro.size < 3
        ) {

            return rejectResult(
                "invalid vehicle-frame IMU vector"
            )
        }

        if (
            vehicleSpecificForce.any {
                !it.isFinite()
            } ||
            vehicleGyro.any {
                !it.isFinite()
            }
        ) {

            return rejectResult(
                "non-finite vehicle-frame IMU"
            )
        }


        // --------------------------------------------------------
        // TIMING
        // --------------------------------------------------------

        val timestamp =
            sample.timestampNanos

        val dt =
            if (
                previousTimestampNanos > 0L &&
                timestamp > previousTimestampNanos
            ) {

                (
                        timestamp -
                                previousTimestampNanos
                        ).toDouble() /
                        1_000_000_000.0

            } else {
                0.1
            }

        val safeDt =
            dt.coerceIn(
                0.05,
                0.25
            )


        // --------------------------------------------------------
        // CURRENT ESKF SPEED
        // --------------------------------------------------------

        val eskfSpeedBefore =
            safeEskfSpeed()


        /*
         * Use ESKF's current trusted speed as the prior for
         * the neural models.
         *
         * If this is the first usable sample, fall back to
         * the previous bridge state.
         */
        val priorSpeed =
            if (
                eskfSpeedBefore.isFinite()
            ) {
                eskfSpeedBefore
            } else {
                previousTrustedSpeedMps
            }


        // ========================================================
        // ASTRA-SPEED
        // ========================================================

        val speedOutput =
            try {

                speedEngine.add(
                    sample =
                        sample,

                    previousTrustedSpeedKmh =
                        priorSpeed * 3.6
                )

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "ASTRA-Speed failed",
                    e
                )

                null
            }

        if (
            speedOutput != null &&
            speedOutput.valid
        ) {

            speedInferenceCount++
        }


        // ========================================================
        // ASTRA-SPHM
        // ========================================================

        val sphmOutput =
            try {

                sphmEngine.addSample(
                    sample =
                        sample,

                    initialSpeedMps =
                        priorSpeed
                )

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "ASTRA-SPHM failed",
                    e
                )

                null
            }

        if (
            sphmOutput != null &&
            sphmOutput.valid
        ) {

            sphmInferenceCount++
        }


        // ========================================================
        // ASTRA-MOTION
        // ========================================================

        val aFwd =
            vehicleSpecificForce[0]

        val aLat =
            vehicleSpecificForce[1]

        val wYaw =
            vehicleGyro[2]

        val yawAccel =
            (
                    wYaw -
                            previousYawRate
                    ) / safeDt


        val motionOutput =
            try {

                motionEngine.add(
                    aFwd =
                        aFwd,

                    wYaw =
                        wYaw,

                    aLat =
                        aLat,

                    previousSpeedMps =
                        priorSpeed,

                    yawAccel =
                        yawAccel
                )

            } catch (e: Exception) {

                Log.e(
                    TAG,
                    "ASTRA-Motion failed",
                    e
                )

                null
            }

        if (
            motionOutput != null &&
            motionOutput.valid
        ) {

            motionInferenceCount++
        }


        // ========================================================
        // ASTRA-GUARD
        // ========================================================

        val guard =
            evaluateGuard(
                speed =
                    speedOutput,

                sphm =
                    sphmOutput,

                motion =
                    motionOutput,

                priorSpeedMps =
                    priorSpeed
            )


        // ========================================================
        // LEARNED SPEED → ESKF
        // ========================================================

        var eskfSpeedUpdate:
                EskfLearnedSpeedUpdate.UpdateResult? =
            null


        if (
            guard.speedMeasurementAllowed &&
            speedOutput != null &&
            speedOutput.valid
        ) {

            val learnedSpeed =
                speedOutput.speedMps


            if (
                learnedSpeed.isFinite() &&
                learnedSpeed >= 0.0 &&
                learnedSpeed <= MAX_SPEED_MPS
            ) {

                eskfSpeedUpdate =
                    try {

                        eskf.applyLearnedSpeed(
                            learnedSpeedMps =
                                learnedSpeed,

                            speedStdMps =
                                guard.speedStdMps
                        )

                    } catch (e: Exception) {

                        Log.e(
                            TAG,
                            "ESKF learned-speed update failed",
                            e
                        )

                        null
                    }


                if (
                    eskfSpeedUpdate?.accepted ==
                    true
                ) {

                    speedAcceptedCount++

                } else {

                    speedRejectedCount++
                }

            } else {

                speedRejectedCount++
            }

        } else {

            if (
                speedOutput != null &&
                speedOutput.valid
            ) {
                guardRejectedCount++
            }
        }


        // ========================================================
        // TRUSTED SPEED AFTER FUSION
        // ========================================================

        val fusedSpeed =
            safeEskfSpeed()


        previousTrustedSpeedMps =
            if (
                fusedSpeed.isFinite()
            ) {
                fusedSpeed
            } else {
                priorSpeed
            }


        // ========================================================
        // UPDATE TEMPORAL STATE
        // ========================================================

        previousYawRate =
            wYaw

        previousTimestampNanos =
            timestamp


        // ========================================================
        // FINAL RESULT
        // ========================================================

        val result =
            FusionResult(

                valid =
                    true,

                speedOutput =
                    speedOutput,

                motionOutput =
                    motionOutput,

                sphmOutput =
                    sphmOutput,

                speedUpdate =
                    eskfSpeedUpdate,

                speedStdMps =
                    guard.speedStdMps,

                speedMeasurementAllowed =
                    guard.speedMeasurementAllowed,

                speedAgreementMps =
                    guard.speedAgreementMps,

                sphmHeadingDeltaRad =
                    sphmOutput
                        ?.headingDeltaRad,

                motionZuptScore =
                    motionOutput
                        ?.zuptScore,

                guardReason =
                    guard.reason,

                fusedSpeedMps =
                    previousTrustedSpeedMps
            )


        lastResult =
            result

        return result
    }


    // ============================================================
    // GUARD
    // ============================================================

    /**
     * ASTRA-GUARD v1.
     *
     * This is deliberately conservative.
     *
     * The guard does NOT try to "fix" the neural networks.
     *
     * It decides whether a learned speed measurement is
     * sufficiently trustworthy to enter ESKF.
     */
    private fun evaluateGuard(
        speed: GruSpeedOutput?,
        sphm: AstraSphmOutput?,
        motion: AstraMotionOutput?,
        priorSpeedMps: Double
    ): GuardDecision {


        // --------------------------------------------------------
        // No ASTRA-Speed prediction yet
        // --------------------------------------------------------

        if (
            speed == null ||
            !speed.valid
        ) {

            return GuardDecision(
                speedMeasurementAllowed =
                    false,

                speedStdMps =
                    MAX_SPEED_STD_MPS,

                speedAgreementMps =
                    null,

                reason =
                    "ASTRA-Speed warming up"
            )
        }


        val speedMps =
            speed.speedMps


        // --------------------------------------------------------
        // Physical validation
        // --------------------------------------------------------

        if (
            !speedMps.isFinite() ||
            speedMps < 0.0 ||
            speedMps > MAX_SPEED_MPS
        ) {

            return GuardDecision(
                speedMeasurementAllowed =
                    false,

                speedStdMps =
                    MAX_SPEED_STD_MPS,

                speedAgreementMps =
                    null,

                reason =
                    "ASTRA-Speed physical validation failed"
            )
        }


        // --------------------------------------------------------
        // ASTRA-SPHM agreement
        // --------------------------------------------------------

        val sphmSpeed =
            sphm
                ?.takeIf {
                    it.valid
                }
                ?.speedMps
                ?.takeIf {
                    it.isFinite() &&
                            it >= 0.0 &&
                            it <= MAX_SPEED_MPS
                }


        val disagreement =
            if (
                sphmSpeed != null
            ) {

                abs(
                    speedMps -
                            sphmSpeed
                )

            } else {
                null
            }


        // --------------------------------------------------------
        // Base uncertainty
        // --------------------------------------------------------

        var std =
            BASE_SPEED_STD_MPS


        // --------------------------------------------------------
        // SPHM disagreement penalty
        // --------------------------------------------------------

        if (
            disagreement != null
        ) {

            when {

                disagreement >=
                        SPEED_DISAGREEMENT_HARD_MPS -> {

                    /*
                     * Models strongly disagree.
                     *
                     * Do not necessarily discard the result
                     * completely, because SPHM itself is only a
                     * secondary model.
                     *
                     * Instead heavily reduce trust.
                     */
                    std =
                        MAX_SPEED_STD_MPS

                }

                disagreement >=
                        SPEED_DISAGREEMENT_SOFT_MPS -> {

                    /*
                     * Moderate disagreement.
                     */
                    std =
                        2.5

                }

                else -> {

                    /*
                     * Strong agreement.
                     */
                    std =
                        1.0
                }
            }
        }


        // --------------------------------------------------------
        // Motion context
        // --------------------------------------------------------

        val motionZupt =
            motion
                ?.zuptScore


        /*
         * A high ZUPT score means the model believes the
         * vehicle is stationary.
         *
         * During a stationary interval, a positive learned
         * speed is suspicious.
         */
        if (
            motionZupt != null &&
            motionZupt.isFinite() &&
            motionZupt >= 0.80
        ) {

            if (
                speedMps >
                1.5
            ) {

                return GuardDecision(
                    speedMeasurementAllowed =
                        false,

                    speedStdMps =
                        MAX_SPEED_STD_MPS,

                    speedAgreementMps =
                        disagreement,

                    reason =
                        "Motion model indicates stationary"
                )
            }

            /*
             * For low predicted speed, let ZUPT handle the
             * actual zero-velocity correction.
             */
            std =
                maxOf(
                    std,
                    2.0
                )
        }


        // --------------------------------------------------------
        // Sudden learned-speed jump
        // --------------------------------------------------------

        val speedJump =
            abs(
                speedMps -
                        priorSpeedMps
            )


        /*
         * At 10 Hz, an enormous jump is suspicious.
         *
         * Do not use this as a hard physical acceleration
         * constraint yet because real vehicle acceleration
         * depends on braking/launch conditions.
         *
         * It only inflates uncertainty.
         */
        if (
            speedJump >
            8.0
        ) {

            std =
                maxOf(
                    std,
                    3.0
                )
        }


        // --------------------------------------------------------
        // Clamp uncertainty
        // --------------------------------------------------------

        std =
            std.coerceIn(
                MIN_SPEED_STD_MPS,
                MAX_SPEED_STD_MPS
            )


        // --------------------------------------------------------
        // Final decision
        // --------------------------------------------------------

        return GuardDecision(
            speedMeasurementAllowed =
                true,

            speedStdMps =
                std,

            speedAgreementMps =
                disagreement,

            reason =
                when {

                    disagreement == null ->
                        "ASTRA-Speed accepted; SPHM unavailable"

                    disagreement <
                            SPEED_DISAGREEMENT_SOFT_MPS ->
                        "ASTRA-Speed + SPHM agree"

                    disagreement <
                            SPEED_DISAGREEMENT_HARD_MPS ->
                        "moderate model disagreement"

                    else ->
                        "strong model disagreement; uncertainty inflated"
                }
        )
    }


    // ============================================================
    // SAFE ESKF SPEED
    // ============================================================

    private fun safeEskfSpeed(): Double {

        return try {

            val velocity =
                eskf.getVelocity()

            val speed =
                sqrt(
                    velocity.x *
                            velocity.x +
                            velocity.y *
                            velocity.y +
                            velocity.z *
                            velocity.z
                )

            if (
                speed.isFinite()
            ) {
                speed
            } else {
                Double.NaN
            }

        } catch (_: Exception) {

            Double.NaN
        }
    }


    // ============================================================
    // REJECTION
    // ============================================================

    private fun rejectResult(
        reason: String
    ): FusionResult {

        val result =
            FusionResult(
                valid =
                    false,

                guardReason =
                    reason
            )

        lastResult =
            result

        return result
    }


    // ============================================================
    // PUBLIC DIAGNOSTICS
    // ============================================================

    @Synchronized
    fun latestResult():
            FusionResult {

        return lastResult
    }


    fun isSpeedModelLoaded():
            Boolean {

        return speedEngine.isLoaded()
    }


    fun isMotionModelLoaded():
            Boolean {

        return motionEngine.isLoaded()
    }


    fun isSphmModelReady():
            Boolean {

        return sphmEngine.isReady()
    }


    fun previousTrustedSpeedMps():
            Double {

        return previousTrustedSpeedMps
    }


    fun speedInferenceCount():
            Long {

        return speedInferenceCount
    }


    fun motionInferenceCount():
            Long {

        return motionInferenceCount
    }


    fun sphmInferenceCount():
            Long {

        return sphmInferenceCount
    }


    fun speedAcceptedCount():
            Long {

        return speedAcceptedCount
    }


    fun speedRejectedCount():
            Long {

        return speedRejectedCount
    }


    fun guardRejectedCount():
            Long {

        return guardRejectedCount
    }


    // ============================================================
    // DATA CLASSES
    // ============================================================

    private data class GuardDecision(

        val speedMeasurementAllowed:
        Boolean,

        val speedStdMps:
        Double,

        val speedAgreementMps:
        Double?,

        val reason:
        String
    )


    data class FusionResult(

        val valid:
        Boolean = false,

        val speedOutput:
        GruSpeedOutput? = null,

        val motionOutput:
        AstraMotionOutput? = null,

        val sphmOutput:
        AstraSphmOutput? = null,

        val speedUpdate:
        EskfLearnedSpeedUpdate.UpdateResult? =
            null,

        val speedStdMps:
        Double =
            MAX_SPEED_STD_MPS,

        val speedMeasurementAllowed:
        Boolean =
            false,

        val speedAgreementMps:
        Double? =
            null,

        val sphmHeadingDeltaRad:
        Double? =
            null,

        val motionZuptScore:
        Double? =
            null,

        val guardReason:
        String? =
            null,

        val fusedSpeedMps:
        Double =
            0.0
    )
}