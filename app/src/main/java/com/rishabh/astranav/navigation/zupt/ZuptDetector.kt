package com.rishabh.astranav.navigation.zupt

import kotlin.math.exp
import kotlin.math.sqrt

/**
 * ASTRA-NAV ZUPT Detector
 *
 * Detects whether the vehicle/device is stationary.
 *
 * ZUPT evidence:
 *
 *  1. ASTRA-Motion ZUPT logit
 *  2. Gyroscope magnitude
 *  3. Linear acceleration magnitude
 *  4. Trusted/fused speed
 *
 * IMPORTANT:
 * This detector DOES NOT modify ESKF state.
 *
 * It only produces a stationary decision.
 *
 * ESKF zero-velocity measurement update will be
 * connected separately.
 */
class ZuptDetector {

    companion object {

        // -------------------------------------------------------------
        // Sensor thresholds
        // -------------------------------------------------------------

        /**
         * Vehicle/device considered rotationally still below this
         * angular velocity.
         *
         * rad/s
         */
        private const val GYRO_STATIONARY_THRESHOLD =
            0.08

        /**
         * Linear acceleration magnitude should be close to zero
         * when stationary.
         *
         * m/s²
         */
        private const val LINEAR_ACCEL_THRESHOLD =
            0.30

        /**
         * Trusted velocity below this value is considered
         * stationary candidate.
         *
         * m/s
         */
        private const val SPEED_THRESHOLD =
            0.50

        // -------------------------------------------------------------
        // ASTRA-Motion thresholds
        // -------------------------------------------------------------

        /**
         * Sigmoid probability above this threshold provides
         * positive ML stationary evidence.
         */
        private const val ML_ZUPT_ON_THRESHOLD =
            0.80

        /**
         * Below this probability the ML evidence is considered
         * negative.
         */
        private const val ML_ZUPT_OFF_THRESHOLD =
            0.35

        // -------------------------------------------------------------
        // Temporal hysteresis
        // -------------------------------------------------------------

        /**
         * Number of consecutive positive samples required before
         * declaring ZUPT active.
         *
         * At 10 Hz:
         *
         * 3 samples = 300 ms
         */
        private const val ON_SAMPLES =
            3

        /**
         * Number of consecutive negative samples required before
         * releasing ZUPT.
         *
         * At 10 Hz:
         *
         * 3 samples = 300 ms
         */
        private const val OFF_SAMPLES =
            3

        // -------------------------------------------------------------
        // Evidence weighting
        // -------------------------------------------------------------

        private const val WEIGHT_ML =
            0.35

        private const val WEIGHT_GYRO =
            0.25

        private const val WEIGHT_ACCEL =
            0.20

        private const val WEIGHT_SPEED =
            0.20
    }

    // -----------------------------------------------------------------
    // State
    // -----------------------------------------------------------------

    private var consecutivePositiveSamples =
        0

    private var consecutiveNegativeSamples =
        0

    private var active =
        false

    /**
     * Last calculated stationary score.
     *
     * 0.0 = definitely moving
     * 1.0 = strong stationary evidence
     */
    private var lastScore =
        0.0

    /**
     * Last ML ZUPT probability.
     */
    private var lastMlProbability =
        0.0

    /**
     * Process one synchronized IMU sample.
     *
     * @param gyroX rad/s
     * @param gyroY rad/s
     * @param gyroZ rad/s
     *
     * @param linearAccelX m/s²
     * @param linearAccelY m/s²
     * @param linearAccelZ m/s²
     *
     * @param trustedSpeedMps previous trusted/fused speed
     *
     * @param zuptLogit ASTRA-Motion raw ZUPT logit.
     *                  null if ML output is unavailable.
     */
    @Synchronized
    fun update(
        gyroX: Double,
        gyroY: Double,
        gyroZ: Double,
        linearAccelX: Double,
        linearAccelY: Double,
        linearAccelZ: Double,
        trustedSpeedMps: Double,
        zuptLogit: Double?
    ): ZuptResult {

        // -------------------------------------------------------------
        // Validate input
        // -------------------------------------------------------------

        if (
            !gyroX.isFinite() ||
            !gyroY.isFinite() ||
            !gyroZ.isFinite() ||
            !linearAccelX.isFinite() ||
            !linearAccelY.isFinite() ||
            !linearAccelZ.isFinite() ||
            !trustedSpeedMps.isFinite()
        ) {

            return ZuptResult(
                active = false,
                score = 0.0,
                mlProbability = null,
                gyroMagnitude = Double.NaN,
                linearAccelerationMagnitude = Double.NaN,
                trustedSpeedMps = trustedSpeedMps,
                positiveSamples = 0,
                negativeSamples = 0,
                reason = "INVALID_SENSOR_DATA"
            )
        }

        // -------------------------------------------------------------
        // Calculate magnitudes
        // -------------------------------------------------------------

        val gyroMagnitude =
            sqrt(
                gyroX * gyroX +
                        gyroY * gyroY +
                        gyroZ * gyroZ
            )

        val linearAccelerationMagnitude =
            sqrt(
                linearAccelX * linearAccelX +
                        linearAccelY * linearAccelY +
                        linearAccelZ * linearAccelZ
            )

        // -------------------------------------------------------------
        // ML probability
        // -------------------------------------------------------------

        val mlProbability =
            zuptLogit
                ?.takeIf { it.isFinite() }
                ?.let {
                    sigmoid(it)
                }

        lastMlProbability =
            mlProbability ?: lastMlProbability

        // -------------------------------------------------------------
        // Individual evidence scores
        // -------------------------------------------------------------

        val gyroScore =
            stationaryScore(
                value = gyroMagnitude,
                threshold = GYRO_STATIONARY_THRESHOLD
            )

        val accelScore =
            stationaryScore(
                value = linearAccelerationMagnitude,
                threshold = LINEAR_ACCEL_THRESHOLD
            )

        val speedScore =
            stationaryScore(
                value = kotlin.math.abs(
                    trustedSpeedMps
                ),
                threshold = SPEED_THRESHOLD
            )

        val mlScore =
            mlProbability?.let {
                when {
                    it >= ML_ZUPT_ON_THRESHOLD ->
                        1.0

                    it <= ML_ZUPT_OFF_THRESHOLD ->
                        0.0

                    else ->
                        (
                                it -
                                        ML_ZUPT_OFF_THRESHOLD
                                ) /
                                (
                                        ML_ZUPT_ON_THRESHOLD -
                                                ML_ZUPT_OFF_THRESHOLD
                                        )
                }
            } ?: 0.0

        // -------------------------------------------------------------
        // Weighted stationary score
        // -------------------------------------------------------------

        val score =
            if (mlProbability != null) {

                (
                        WEIGHT_ML * mlScore +
                                WEIGHT_GYRO * gyroScore +
                                WEIGHT_ACCEL * accelScore +
                                WEIGHT_SPEED * speedScore
                        )
            } else {

                /*
                 * If ML is temporarily unavailable, do not make
                 * the detector completely blind.
                 *
                 * Re-normalize the physical evidence.
                 */
                (
                        WEIGHT_GYRO * gyroScore +
                                WEIGHT_ACCEL * accelScore +
                                WEIGHT_SPEED * speedScore
                        ) /
                        (
                                WEIGHT_GYRO +
                                        WEIGHT_ACCEL +
                                        WEIGHT_SPEED
                                )
            }

        lastScore =
            score.coerceIn(0.0, 1.0)

        // -------------------------------------------------------------
        // Strong physical stationary condition
        // -------------------------------------------------------------

        val physicalStationary =
            gyroMagnitude <=
                    GYRO_STATIONARY_THRESHOLD &&
                    linearAccelerationMagnitude <=
                    LINEAR_ACCEL_THRESHOLD &&
                    kotlin.math.abs(
                        trustedSpeedMps
                    ) <=
                    SPEED_THRESHOLD

        // -------------------------------------------------------------
        // Candidate decision
        // -------------------------------------------------------------

        /*
         * ZUPT candidate requires BOTH:
         *
         * 1. Strong combined evidence
         *
         * OR
         *
         * 2. Strong physical stationary evidence
         *    with supportive ML evidence.
         */

        val mlSupportsStationary =
            mlProbability == null ||
                    mlProbability >=
                    ML_ZUPT_OFF_THRESHOLD

        val candidate =
            (
                    lastScore >= 0.72 &&
                            mlSupportsStationary
                    ) ||
                    (
                            physicalStationary &&
                                    mlSupportsStationary
                            )

        // -------------------------------------------------------------
        // Hysteresis
        // -------------------------------------------------------------

        if (candidate) {

            consecutivePositiveSamples++

            consecutiveNegativeSamples =
                0

        } else {

            consecutiveNegativeSamples++

            consecutivePositiveSamples =
                0
        }

        // -------------------------------------------------------------
        // TURN ZUPT ON
        // -------------------------------------------------------------

        if (
            !active &&
            consecutivePositiveSamples >=
            ON_SAMPLES
        ) {

            active = true

            consecutivePositiveSamples =
                ON_SAMPLES
        }

        // -------------------------------------------------------------
        // TURN ZUPT OFF
        // -------------------------------------------------------------

        if (
            active &&
            consecutiveNegativeSamples >=
            OFF_SAMPLES
        ) {

            active = false

            consecutiveNegativeSamples =
                OFF_SAMPLES
        }

        // -------------------------------------------------------------
        // Reason
        // -------------------------------------------------------------

        val reason =
            when {

                active ->
                    "STATIONARY_CONFIRMED"

                candidate ->
                    "STATIONARY_CANDIDATE"

                physicalStationary &&
                        mlProbability != null &&
                        mlProbability <
                        ML_ZUPT_OFF_THRESHOLD ->
                    "ML_REJECTED"

                gyroMagnitude >
                        GYRO_STATIONARY_THRESHOLD ->
                    "GYRO_MOTION"

                linearAccelerationMagnitude >
                        LINEAR_ACCEL_THRESHOLD ->
                    "ACCEL_MOTION"

                kotlin.math.abs(
                    trustedSpeedMps
                ) >
                        SPEED_THRESHOLD ->
                    "VELOCITY_MOTION"

                else ->
                    "INSUFFICIENT_EVIDENCE"
            }

        return ZuptResult(
            active = active,
            score = lastScore,
            mlProbability = mlProbability,
            gyroMagnitude = gyroMagnitude,
            linearAccelerationMagnitude =
                linearAccelerationMagnitude,
            trustedSpeedMps =
                trustedSpeedMps,
            positiveSamples =
                consecutivePositiveSamples,
            negativeSamples =
                consecutiveNegativeSamples,
            reason = reason
        )
    }

    /**
     * Convert logit -> probability.
     *
     * sigmoid(x) = 1 / (1 + exp(-x))
     *
     * Protected against extreme values.
     */
    private fun sigmoid(
        logit: Double
    ): Double {

        return when {

            logit >= 0.0 -> {

                val z =
                    exp(-logit)

                1.0 /
                        (1.0 + z)
            }

            else -> {

                val z =
                    exp(logit)

                z /
                        (1.0 + z)
            }
        }
    }

    /**
     * Convert physical magnitude into a smooth
     * stationary evidence score.
     *
     * value = 0 -> score 1
     * value = threshold -> score 0
     */
    private fun stationaryScore(
        value: Double,
        threshold: Double
    ): Double {

        if (
            !value.isFinite() ||
            threshold <= 0.0
        ) {
            return 0.0
        }

        if (
            value >= threshold
        ) {
            return 0.0
        }

        return (
                1.0 -
                        value / threshold
                ).coerceIn(
                0.0,
                1.0
            )
    }

    /**
     * Current ZUPT state.
     */
    fun isActive(): Boolean =
        active

    /**
     * Last combined stationary score.
     */
    fun currentScore(): Double =
        lastScore

    /**
     * Reset detector state.
     */
    @Synchronized
    fun reset() {

        consecutivePositiveSamples =
            0

        consecutiveNegativeSamples =
            0

        active =
            false

        lastScore =
            0.0

        lastMlProbability =
            0.0
    }
}

/**
 * Result produced by ZUPT detector.
 */
data class ZuptResult(

    /**
     * True only after temporal confirmation.
     */
    val active: Boolean,

    /**
     * Combined stationary evidence.
     *
     * 0..1
     */
    val score: Double,

    /**
     * ASTRA-Motion sigmoid probability.
     *
     * null when ML output is unavailable.
     */
    val mlProbability: Double?,

    /**
     * Gyroscope magnitude in rad/s.
     */
    val gyroMagnitude: Double,

    /**
     * Linear acceleration magnitude in m/s².
     */
    val linearAccelerationMagnitude: Double,

    /**
     * Trusted/fused speed in m/s.
     */
    val trustedSpeedMps: Double,

    /**
     * Number of consecutive positive candidate samples.
     */
    val positiveSamples: Int,

    /**
     * Number of consecutive negative samples.
     */
    val negativeSamples: Int,

    /**
     * Human-readable detector state.
     */
    val reason: String
)