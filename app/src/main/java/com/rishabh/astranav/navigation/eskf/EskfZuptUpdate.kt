package com.rishabh.astranav.navigation.eskf

/**
 * Zero-Velocity Update adapter for ASTRA-Core ESKF.
 *
 * This class intentionally contains NO Kalman mathematics.
 *
 * It converts a zero-velocity measurement into the generic
 * EskfMeasurementUpdate format.
 *
 * Stationary detection remains the responsibility of ZuptDetector.
 *
 * Measurement:
 *
 *      z = [0, 0, 0] m/s
 *
 * Predicted measurement:
 *
 *      h(x) = current navigation velocity
 *
 * Measurement Jacobian:
 *
 *      H = [ 0 0 0 | I | 0 0 0 | 0 0 0 | 0 0 0 ]
 *
 * Measurement covariance:
 *
 *      R = sigma_zupt² I
 */
class EskfZuptUpdate(
    private val config: EskfConfig = EskfConfig()
) {

    companion object {

        private const val STATE_SIZE = 15
        private const val MEASUREMENT_SIZE = 3

        private const val VEL_INDEX = 3
    }

    /**
     * Result of one ZUPT correction.
     *
     * This keeps the old ZUPT-facing API so callers do not need
     * to know that the implementation now uses the generic
     * measurement engine.
     */
    data class UpdateResult(
        val accepted: Boolean,
        val innovation: Vec3,
        val innovationNormMps: Double,
        val nis: Double,
        val velocityCorrection: Vec3,
        val positionCorrection: Vec3,
        val attitudeCorrectionRad: Vec3,
        val gyroBiasCorrection: Vec3,
        val accelBiasCorrection: Vec3,
        val reason: String? = null
    )

    /**
     * Generic measurement update engine.
     *
     * All future measurement types should use the same engine.
     */
    private val measurementUpdate =
        EskfMeasurementUpdate(config)

    /**
     * Applies a zero-velocity measurement.
     *
     * IMPORTANT:
     *
     * This method does NOT determine whether the vehicle is
     * stationary. ZuptDetector must make that decision first.
     */
    @Synchronized
    fun update(
        state: NavigationState,
        covariance: EskfCovariance
    ): UpdateResult {

        /*
         * ------------------------------------------------------------
         * 1. Build zero-velocity measurement
         * ------------------------------------------------------------
         *
         * z = [0, 0, 0]
         */

        val measurement =
            doubleArrayOf(
                0.0,
                0.0,
                0.0
            )

        /*
         * ------------------------------------------------------------
         * 2. Predicted measurement
         * ------------------------------------------------------------
         *
         * h(x) = current velocity
         */

        val predictedMeasurement =
            doubleArrayOf(
                state.velocity.x,
                state.velocity.y,
                state.velocity.z
            )

        /*
         * ------------------------------------------------------------
         * 3. Measurement Jacobian H
         * ------------------------------------------------------------
         *
         * Error-state:
         *
         * [δp, δv, δθ, δbg, δba]
         *
         * ZUPT observes only δv.
         */

        val jacobian =
            Array(MEASUREMENT_SIZE) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until MEASUREMENT_SIZE) {

            jacobian[i][VEL_INDEX + i] = 1.0
        }

        /*
         * ------------------------------------------------------------
         * 4. Measurement covariance R
         * ------------------------------------------------------------
         */

        val variance =
            config.zuptVelocityStdMps *
                    config.zuptVelocityStdMps

        val measurementCovariance =
            Array(MEASUREMENT_SIZE) {
                    row ->
                DoubleArray(MEASUREMENT_SIZE) {
                        column ->
                    if (row == column) {
                        variance
                    } else {
                        0.0
                    }
                }
            }

        /*
         * ------------------------------------------------------------
         * 5. Build generic measurement
         * ------------------------------------------------------------
         *
         * No hard NIS gate is applied here.
         *
         * NIS is still calculated by EskfMeasurementUpdate and
         * returned for ASTRA-Guard / integrity monitoring.
         */

        val genericMeasurement =
            EskfMeasurementUpdate.Measurement(
                measurement =
                    measurement,

                predictedMeasurement =
                    predictedMeasurement,

                jacobian =
                    jacobian,

                measurementCovariance =
                    measurementCovariance,

                nisGate =
                    null,

                name =
                    "ZUPT"
            )

        /*
         * ------------------------------------------------------------
         * 6. Run generic Kalman update
         * ------------------------------------------------------------
         */

        val result =
            measurementUpdate.update(
                state = state,
                covariance = covariance,
                measurement = genericMeasurement
            )

        /*
         * ------------------------------------------------------------
         * 7. Convert generic result back to ZUPT result
         * ------------------------------------------------------------
         */

        if (!result.accepted) {

            return UpdateResult(
                accepted = false,

                innovation =
                    innovationToVec3(
                        result.innovation
                    ),

                innovationNormMps =
                    result.innovationNorm,

                nis =
                    result.nis,

                velocityCorrection =
                    Vec3.ZERO,

                positionCorrection =
                    Vec3.ZERO,

                attitudeCorrectionRad =
                    Vec3.ZERO,

                gyroBiasCorrection =
                    Vec3.ZERO,

                accelBiasCorrection =
                    Vec3.ZERO,

                reason =
                    result.reason
            )
        }

        /*
         * Generic error-state:
         *
         * 0..2   position
         * 3..5   velocity
         * 6..8   attitude
         * 9..11  gyro bias
         * 12..14 accel bias
         */

        val errorState =
            result.errorState

        val positionCorrection =
            Vec3(
                errorState[0],
                errorState[1],
                errorState[2]
            )

        val velocityCorrection =
            Vec3(
                errorState[3],
                errorState[4],
                errorState[5]
            )

        val attitudeCorrection =
            Vec3(
                errorState[6],
                errorState[7],
                errorState[8]
            )

        val gyroBiasCorrection =
            Vec3(
                errorState[9],
                errorState[10],
                errorState[11]
            )

        val accelBiasCorrection =
            Vec3(
                errorState[12],
                errorState[13],
                errorState[14]
            )

        return UpdateResult(
            accepted = true,

            innovation =
                innovationToVec3(
                    result.innovation
                ),

            innovationNormMps =
                result.innovationNorm,

            nis =
                result.nis,

            velocityCorrection =
                velocityCorrection,

            positionCorrection =
                positionCorrection,

            attitudeCorrectionRad =
                attitudeCorrection,

            gyroBiasCorrection =
                gyroBiasCorrection,

            accelBiasCorrection =
                accelBiasCorrection,

            reason = null
        )
    }

    /**
     * Converts a generic 3-element innovation to Vec3.
     */
    private fun innovationToVec3(
        innovation: DoubleArray
    ): Vec3 {

        if (innovation.size != 3) {
            return Vec3.ZERO
        }

        return Vec3(
            x = innovation[0],
            y = innovation[1],
            z = innovation[2]
        )
    }
}