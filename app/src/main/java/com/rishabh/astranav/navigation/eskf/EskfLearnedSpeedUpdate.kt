package com.rishabh.astranav.navigation.eskf

import kotlin.math.abs

/**
 * ASTRA-Core learned speed measurement update.
 *
 * Measurement:
 *
 *      z = learned vehicle speed magnitude [m/s]
 *
 * Predicted measurement:
 *
 *      h(x) = |v|
 *
 * where v is the ESKF velocity in local NED coordinates.
 *
 * This is intentionally a SCALAR speed measurement.
 *
 * The learned model does NOT overwrite ESKF velocity.
 * Instead, the measurement is fused through the normal
 * ESKF error-state measurement update and NIS integrity gate.
 *
 * State ordering:
 *
 *   [ dp(3),
 *     dv(3),
 *     dtheta(3),
 *     dbg(3),
 *     dba(3) ]
 */
class EskfLearnedSpeedUpdate(

    private val measurementUpdater:
    EskfMeasurementUpdate = EskfMeasurementUpdate(),

    /**
     * Default measurement standard deviation.
     *
     * This should represent the expected uncertainty of the
     * learned speed estimator, NOT the accuracy we hope it has.
     */
    private val defaultSpeedStdMps: Double = 1.25,

    /**
     * One-dimensional NIS gate.
     *
     * 9.21 is a conservative high-confidence gate for a
     * scalar measurement.
     */
    private val nisGate: Double = 9.21,

    /**
     * Prevent a learned model from claiming impossible values.
     *
     * This is only an integrity guard.
     */
    private val maxMeasurementSpeedMps: Double = 100.0
) {

    data class UpdateResult(

        val accepted: Boolean,

        /**
         * Learned speed supplied by the model.
         */
        val measurementSpeedMps: Double,

        /**
         * Speed predicted from the current ESKF velocity.
         */
        val predictedSpeedMps: Double,

        /**
         * measurement - prediction
         */
        val innovationMps: Double,

        /**
         * Absolute innovation.
         */
        val innovationAbsMps: Double,

        /**
         * NIS of this scalar measurement.
         */
        val nis: Double,

        /**
         * Measurement uncertainty actually used.
         */
        val measurementStdMps: Double,

        val reason: String? = null
    )

    /**
     * Apply a learned scalar speed measurement to ESKF.
     *
     * The ESKF state is modified only if the generic
     * measurement update accepts the measurement.
     */
    @Synchronized
    fun update(
        state: NavigationState,
        covariance: EskfCovariance,
        learnedSpeedMps: Double,
        speedStdMps: Double? = null
    ): UpdateResult {

        /*
         * ------------------------------------------------------------
         * 1. Validate learned measurement
         * ------------------------------------------------------------
         */

        if (!learnedSpeedMps.isFinite()) {

            return rejected(
                learnedSpeedMps,
                predictedSpeedMps = safeSpeed(state.velocity),
                speedStdMps = defaultSpeedStdMps,
                reason = "non-finite learned speed"
            )
        }

        if (learnedSpeedMps < 0.0) {

            return rejected(
                learnedSpeedMps,
                predictedSpeedMps = safeSpeed(state.velocity),
                speedStdMps = defaultSpeedStdMps,
                reason = "negative learned speed"
            )
        }

        if (learnedSpeedMps > maxMeasurementSpeedMps) {

            return rejected(
                learnedSpeedMps,
                predictedSpeedMps = safeSpeed(state.velocity),
                speedStdMps = defaultSpeedStdMps,
                reason = "learned speed exceeds integrity limit"
            )
        }

        /*
         * ------------------------------------------------------------
         * 2. Validate ESKF state
         * ------------------------------------------------------------
         */

        if (!state.velocity.isFinite()) {

            return rejected(
                learnedSpeedMps,
                predictedSpeedMps = Double.NaN,
                speedStdMps = defaultSpeedStdMps,
                reason = "ESKF velocity is non-finite"
            )
        }

        /*
         * ------------------------------------------------------------
         * 3. Predicted speed
         * ------------------------------------------------------------
         *
         * h(x) = |v|
         */

        val velocity =
            state.velocity

        val predictedSpeed =
            velocity.norm()

        if (
            !predictedSpeed.isFinite()
        ) {

            return rejected(
                learnedSpeedMps,
                predictedSpeedMps = predictedSpeed,
                speedStdMps = defaultSpeedStdMps,
                reason = "predicted ESKF speed is non-finite"
            )
        }

        /*
         * ------------------------------------------------------------
         * 4. Measurement uncertainty
         * ------------------------------------------------------------
         */

        val measurementStd =
            validatedStd(
                speedStdMps
            )

        /*
         * ------------------------------------------------------------
         * 5. Scalar measurement Jacobian
         * ------------------------------------------------------------
         *
         * z = |v|
         *
         * d|v|/dv =
         *
         *     [ vx/|v|, vy/|v|, vz/|v| ]
         *
         * Velocity occupies error-state indices 3..5.
         *
         * At zero velocity the derivative is undefined.
         *
         * We therefore use a zero velocity Jacobian at an
         * effectively stationary state. In that situation ZUPT
         * should be the stronger measurement anyway.
         */

        val jacobian =
            DoubleArray(15)

        if (predictedSpeed > 1.0e-6) {

            jacobian[3] =
                velocity.x /
                        predictedSpeed

            jacobian[4] =
                velocity.y /
                        predictedSpeed

            jacobian[5] =
                velocity.z /
                        predictedSpeed
        }

        /*
         * ------------------------------------------------------------
         * 6. Generic ESKF measurement
         * ------------------------------------------------------------
         */

        val measurement =
            EskfMeasurementUpdate.Measurement(

                measurement =
                    doubleArrayOf(
                        learnedSpeedMps
                    ),

                predictedMeasurement =
                    doubleArrayOf(
                        predictedSpeed
                    ),

                jacobian =
                    arrayOf(
                        jacobian
                    ),

                measurementCovariance =
                    arrayOf(
                        doubleArrayOf(
                            measurementStd *
                                    measurementStd
                        )
                    ),

                nisGate =
                    nisGate,

                name =
                    "ASTRA_LEARNED_SPEED"
            )

        /*
         * ------------------------------------------------------------
         * 7. Run standard ESKF measurement update
         * ------------------------------------------------------------
         */

        val result =
            measurementUpdater.update(
                state =
                    state,

                covariance =
                    covariance,

                measurement =
                    measurement
            )

        val innovation =
            result.innovation
                .getOrElse(0) {
                    learnedSpeedMps -
                            predictedSpeed
                }

        return UpdateResult(

            accepted =
                result.accepted,

            measurementSpeedMps =
                learnedSpeedMps,

            predictedSpeedMps =
                predictedSpeed,

            innovationMps =
                innovation,

            innovationAbsMps =
                abs(innovation),

            nis =
                result.nis,

            measurementStdMps =
                measurementStd,

            reason =
                result.reason
        )
    }

    /**
     * Validate the model's reported uncertainty.
     *
     * We deliberately prevent pathological values from making
     * the Kalman update infinitely weak or infinitely aggressive.
     */
    private fun validatedStd(
        value: Double?
    ): Double {

        if (
            value == null ||
            !value.isFinite() ||
            value <= 0.05
        ) {
            return defaultSpeedStdMps
        }

        return value.coerceIn(
            0.25,
            10.0
        )
    }

    private fun safeSpeed(
        velocity: Vec3
    ): Double {

        return if (velocity.isFinite()) {
            velocity.norm()
        } else {
            Double.NaN
        }
    }

    private fun rejected(
        measurementSpeedMps: Double,
        predictedSpeedMps: Double,
        speedStdMps: Double,
        reason: String
    ): UpdateResult {

        return UpdateResult(

            accepted = false,

            measurementSpeedMps =
                measurementSpeedMps,

            predictedSpeedMps =
                predictedSpeedMps,

            innovationMps =
                if (
                    measurementSpeedMps.isFinite() &&
                    predictedSpeedMps.isFinite()
                ) {
                    measurementSpeedMps -
                            predictedSpeedMps
                } else {
                    Double.NaN
                },

            innovationAbsMps =
                if (
                    measurementSpeedMps.isFinite() &&
                    predictedSpeedMps.isFinite()
                ) {
                    abs(
                        measurementSpeedMps -
                                predictedSpeedMps
                    )
                } else {
                    Double.NaN
                },

            nis =
                Double.NaN,

            measurementStdMps =
                speedStdMps,

            reason =
                reason
        )
    }
}