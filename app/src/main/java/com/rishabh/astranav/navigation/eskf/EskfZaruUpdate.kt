package com.rishabh.astranav.navigation.eskf

/**
 * Zero Angular Rate Update (ZARU) for ASTRA-Core ESKF.
 *
 * ZARU is applied when an external stationary detector has established
 * that the vehicle is not rotating.
 *
 * Measurement:
 *
 *     z = [0, 0, 0] rad/s
 *
 * Measurement model:
 *
 *     h(x) = gyro_measurement - gyro_bias
 *
 * Therefore, when stationary:
 *
 *     gyro_measurement ~= gyro_bias
 *
 * The update primarily corrects the ESKF gyro-bias states.
 *
 * IMPORTANT:
 *
 * This class does NOT decide whether the vehicle is stationary.
 * The caller must use ZuptDetector or another validated stationary
 * detector before invoking this update.
 *
 * Error-state:
 *
 *     [δp, δv, δθ, δbg, δba]
 *
 * Gyro-bias indices:
 *
 *     9, 10, 11
 */
class EskfZaruUpdate(
    private val gyroStdRadPerSec: Double =
        DEFAULT_GYRO_STD_RAD_PER_SEC,

    private val nisGate: Double? =
        DEFAULT_NIS_GATE
) {

    companion object {

        private const val STATE_SIZE = 15

        private const val GYRO_BIAS_INDEX = 9

        /**
         * Soft ZARU measurement uncertainty.
         *
         * This is deliberately not zero because real gyroscopes
         * have noise and residual angular motion.
         */
        const val DEFAULT_GYRO_STD_RAD_PER_SEC = 0.05

        /**
         * Chi-square gate for a 3-dimensional measurement.
         *
         * Approximately corresponds to a conservative 3-DOF gate.
         */
        const val DEFAULT_NIS_GATE = 11.34
    }

    /**
     * Result exposed to ASTRA-Core.
     */
    data class UpdateResult(
        val accepted: Boolean,

        /**
         * Actual gyro measurement supplied to ZARU.
         */
        val gyroMeasurement: Vec3,

        /**
         * Current estimated gyro bias before/around update.
         */
        val gyroBias: Vec3,

        /**
         * Innovation:
         *
         *     z - h(x)
         */
        val innovation: Vec3,

        val innovationNormRadPerSec: Double,

        val nis: Double,

        /**
         * Correction applied to gyro bias.
         */
        val gyroBiasCorrection: Vec3,

        val reason: String?
    )

    init {

        require(
            gyroStdRadPerSec.isFinite() &&
                    gyroStdRadPerSec > 0.0
        ) {
            "gyroStdRadPerSec must be finite and > 0"
        }

        require(
            nisGate == null ||
                    (
                            nisGate.isFinite() &&
                                    nisGate > 0.0
                            )
        ) {
            "nisGate must be null or finite and > 0"
        }
    }

    /**
     * Applies one ZARU measurement update.
     *
     * The caller must only invoke this when a trusted stationary
     * condition has already been established.
     *
     * gyroMeasurement:
     *
     *     Raw/corrected body-frame gyro measurement in rad/s.
     *
     * The ESKF state already contains its current gyro-bias estimate.
     */
    fun update(
        state: NavigationState,
        covariance: EskfCovariance,
        gyroMeasurement: Vec3
    ): UpdateResult {

        /*
         * ------------------------------------------------------------
         * 1. Validate state
         * ------------------------------------------------------------
         */

        if (!state.position.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "State position is non-finite"
            )
        }

        if (!state.velocity.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "State velocity is non-finite"
            )
        }

        if (!state.attitude.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "State attitude is non-finite"
            )
        }

        if (!state.gyroBias.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "Gyro bias is non-finite"
            )
        }

        if (!state.accelBias.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "Accelerometer bias is non-finite"
            )
        }

        if (!gyroMeasurement.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "Gyro measurement is non-finite"
            )
        }

        if (!covariance.isFinite()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "Covariance contains non-finite values"
            )
        }

        if (!covariance.hasValidDiagonal()) {
            return rejected(
                gyroMeasurement = gyroMeasurement,
                gyroBias = state.gyroBias,
                reason = "Covariance diagonal is invalid"
            )
        }

        /*
         * ------------------------------------------------------------
         * 2. Measurement model
         * ------------------------------------------------------------
         *
         * At zero angular rate:
         *
         *     gyroMeasurement ~= gyroBias
         *
         * Therefore the predicted measurement is the current
         * nominal gyro-bias estimate.
         *
         * z:
         *
         *     actual gyro measurement
         *
         * h(x):
         *
         *     current gyro bias
         */

        val measurement =
            doubleArrayOf(
                gyroMeasurement.x,
                gyroMeasurement.y,
                gyroMeasurement.z
            )

        val predictedMeasurement =
            doubleArrayOf(
                state.gyroBias.x,
                state.gyroBias.y,
                state.gyroBias.z
            )

        /*
         * ------------------------------------------------------------
         * 3. Jacobian H
         * ------------------------------------------------------------
         *
         * Measurement:
         *
         *     h(x) = gyroBias
         *
         * Therefore:
         *
         *     dh / d(delta_bg) = I
         *
         * All other state derivatives are zero.
         */

        val jacobian =
            Array(3) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until 3) {

            jacobian[i][
                GYRO_BIAS_INDEX + i
            ] = 1.0
        }

        /*
         * ------------------------------------------------------------
         * 4. Measurement covariance R
         * ------------------------------------------------------------
         */

        val variance =
            gyroStdRadPerSec *
                    gyroStdRadPerSec

        val measurementCovariance =
            Array(3) {
                DoubleArray(3)
            }

        measurementCovariance[0][0] =
            variance

        measurementCovariance[1][1] =
            variance

        measurementCovariance[2][2] =
            variance

        /*
         * ------------------------------------------------------------
         * 5. Generic measurement update
         * ------------------------------------------------------------
         */

        val measurementModel =
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
                    nisGate,

                name =
                    "ZARU"
            )

        val measurementUpdater =
            EskfMeasurementUpdate()

        val result =
            measurementUpdater.update(
                state = state,
                covariance = covariance,
                measurement = measurementModel
            )

        /*
         * ------------------------------------------------------------
         * 6. Calculate the applied gyro-bias correction
         * ------------------------------------------------------------
         *
         * The generic update injects:
         *
         *     δbg
         *
         * into:
         *
         *     state.gyroBias
         *
         * Therefore errorState[9..11] is the actual bias
         * correction produced by the Kalman update.
         */

        val gyroBiasCorrection =
            if (result.errorState.size >=
                GYRO_BIAS_INDEX + 3
            ) {

                Vec3(
                    x =
                        result.errorState[
                            GYRO_BIAS_INDEX
                        ],

                    y =
                        result.errorState[
                            GYRO_BIAS_INDEX + 1
                        ],

                    z =
                        result.errorState[
                            GYRO_BIAS_INDEX + 2
                        ]
                )

            } else {

                Vec3.ZERO
            }

        val innovation =
            Vec3(
                x =
                    if (result.innovation.size >= 3)
                        result.innovation[0]
                    else
                        Double.NaN,

                y =
                    if (result.innovation.size >= 3)
                        result.innovation[1]
                    else
                        Double.NaN,

                z =
                    if (result.innovation.size >= 3)
                        result.innovation[2]
                    else
                        Double.NaN
            )

        return UpdateResult(

            accepted =
                result.accepted,

            gyroMeasurement =
                gyroMeasurement,

            gyroBias =
                state.gyroBias.copy(),

            innovation =
                innovation,

            innovationNormRadPerSec =
                result.innovationNorm,

            nis =
                result.nis,

            gyroBiasCorrection =
                gyroBiasCorrection,

            reason =
                result.reason
        )
    }

    /**
     * Rejected result helper.
     */
    private fun rejected(
        gyroMeasurement: Vec3,
        gyroBias: Vec3,
        reason: String
    ): UpdateResult {

        return UpdateResult(

            accepted =
                false,

            gyroMeasurement =
                gyroMeasurement,

            gyroBias =
                gyroBias.copy(),

            innovation =
                Vec3(
                    x = Double.NaN,
                    y = Double.NaN,
                    z = Double.NaN
                ),

            innovationNormRadPerSec =
                Double.NaN,

            nis =
                Double.NaN,

            gyroBiasCorrection =
                Vec3.ZERO,

            reason =
                reason
        )
    }
}