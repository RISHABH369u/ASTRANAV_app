package com.rishabh.astranav.navigation.eskf

import kotlin.math.abs
import kotlin.math.max

/**
 * ASTRA-Core Zero-Velocity Update (ZUPT).
 *
 * This class performs the actual Kalman measurement correction.
 *
 * Stationary detection is NOT performed here.
 * That responsibility belongs to ZuptDetector.
 *
 * Measurement:
 *
 *      z = [0, 0, 0] m/s
 *
 * Predicted measurement:
 *
 *      h(x) = velocity
 *
 * Innovation:
 *
 *      y = z - h(x)
 *        = -velocity
 *
 * Error-state ordering:
 *
 *      [δp, δv, δθ, δbg, δba]
 *
 *      0..2    position
 *      3..5    velocity
 *      6..8    attitude
 *      9..11   gyro bias
 *      12..14  accelerometer bias
 */
class EskfZuptUpdate(
    private val config: EskfConfig = EskfConfig()
) {

    companion object {

        private const val STATE_SIZE = 15
        private const val MEASUREMENT_SIZE = 3

        private const val POS_INDEX = 0
        private const val VEL_INDEX = 3
        private const val ATT_INDEX = 6
        private const val GYRO_BIAS_INDEX = 9
        private const val ACCEL_BIAS_INDEX = 12

        private const val MIN_S_DIAGONAL = 1.0e-12
    }

    /**
     * Result of one ZUPT correction.
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
     * Applies a zero-velocity measurement update.
     *
     * The caller must run normal ESKF prediction before this update.
     */
    @Synchronized
    fun update(
        state: NavigationState,
        covariance: EskfCovariance
    ): UpdateResult {

        /*
         * ------------------------------------------------------------
         * 1. Validate nominal state
         * ------------------------------------------------------------
         */

        if (!state.position.isFinite()) {
            return rejected(
                "Non-finite position"
            )
        }

        if (!state.velocity.isFinite()) {
            return rejected(
                "Non-finite velocity"
            )
        }

        if (!state.attitude.isFinite()) {
            return rejected(
                "Non-finite attitude"
            )
        }

        if (!state.gyroBias.isFinite()) {
            return rejected(
                "Non-finite gyro bias"
            )
        }

        if (!state.accelBias.isFinite()) {
            return rejected(
                "Non-finite accelerometer bias"
            )
        }

        /*
         * ------------------------------------------------------------
         * 2. Validate covariance
         * ------------------------------------------------------------
         */

        if (!covariance.isFinite()) {
            return rejected(
                "Covariance contains non-finite values"
            )
        }

        if (!covariance.hasValidDiagonal()) {
            return rejected(
                "Covariance diagonal is invalid"
            )
        }

        /*
         * ------------------------------------------------------------
         * 3. ZUPT measurement
         * ------------------------------------------------------------
         *
         * z = [0, 0, 0]
         *
         * h(x) = current velocity
         *
         * innovation = z - h(x)
         *            = -velocity
         */

        val innovation = Vec3(
            x = -state.velocity.x,
            y = -state.velocity.y,
            z = -state.velocity.z
        )

        if (!innovation.isFinite()) {
            return rejected(
                "Non-finite innovation"
            )
        }

        val innovationNormMps =
            innovation.norm()

        /*
         * ------------------------------------------------------------
         * 4. Measurement covariance R
         * ------------------------------------------------------------
         *
         * R = sigma² I
         */

        val measurementVariance =
            config.zuptVelocityStdMps *
                    config.zuptVelocityStdMps

        if (!measurementVariance.isFinite() ||
            measurementVariance <= 0.0
        ) {
            return rejected(
                "Invalid ZUPT measurement variance"
            )
        }

        /*
         * ------------------------------------------------------------
         * 5. Innovation covariance
         * ------------------------------------------------------------
         *
         * H observes velocity only.
         *
         * Therefore:
         *
         * S = Pvv + R
         */

        val s =
            Array(MEASUREMENT_SIZE) {
                DoubleArray(MEASUREMENT_SIZE)
            }

        for (row in 0 until MEASUREMENT_SIZE) {

            for (column in 0 until MEASUREMENT_SIZE) {

                var value =
                    covariance[
                        VEL_INDEX + row,
                        VEL_INDEX + column
                    ]

                if (row == column) {
                    value += measurementVariance
                }

                s[row][column] = value
            }
        }

        /*
         * Numerical safety for S.
         */

        for (i in 0 until MEASUREMENT_SIZE) {

            if (!s[i][i].isFinite()) {
                return rejected(
                    "Invalid innovation covariance"
                )
            }

            s[i][i] =
                max(
                    s[i][i],
                    MIN_S_DIAGONAL
                )
        }

        /*
         * ------------------------------------------------------------
         * 6. Invert S
         * ------------------------------------------------------------
         */

        val sInverse =
            invert3x3(s)
                ?: return rejected(
                    "Innovation covariance is singular"
                )

        /*
         * ------------------------------------------------------------
         * 7. Innovation vector
         * ------------------------------------------------------------
         */

        val innovationArray =
            doubleArrayOf(
                innovation.x,
                innovation.y,
                innovation.z
            )

        /*
         * ------------------------------------------------------------
         * 8. NIS
         * ------------------------------------------------------------
         *
         * NIS = yᵀ S⁻¹ y
         */

        val sInvInnovation =
            multiply3x3Vector(
                sInverse,
                innovationArray
            )

        val nis =
            dot3(
                innovationArray,
                sInvInnovation
            )

        if (!nis.isFinite() || nis < 0.0) {
            return rejected(
                "Invalid NIS"
            )
        }

        /*
         * ------------------------------------------------------------
         * 9. Kalman gain
         * ------------------------------------------------------------
         *
         * K = P Hᵀ S⁻¹
         *
         * Since H selects velocity states:
         *
         * PHᵀ = P[:,3..5]
         */

        val kalmanGain =
            Array(STATE_SIZE) {
                DoubleArray(MEASUREMENT_SIZE)
            }

        for (row in 0 until STATE_SIZE) {

            for (column in 0 until MEASUREMENT_SIZE) {

                var value = 0.0

                for (j in 0 until MEASUREMENT_SIZE) {

                    value +=
                        covariance[
                            row,
                            VEL_INDEX + j
                        ] *
                                sInverse[j] [column]
                }

                kalmanGain[row][column] =
                    value
            }
        }

        /*
         * ------------------------------------------------------------
         * 10. Error-state correction
         * ------------------------------------------------------------
         *
         * δx = K y
         */

        val errorState =
            DoubleArray(STATE_SIZE)

        for (row in 0 until STATE_SIZE) {

            var value = 0.0

            for (column in 0 until MEASUREMENT_SIZE) {

                value +=
                    kalmanGain[row][column] *
                            innovationArray[column]
            }

            errorState[row] =
                value
        }

        if (errorState.any { !it.isFinite() }) {
            return rejected(
                "Non-finite Kalman correction"
            )
        }

        /*
         * ------------------------------------------------------------
         * 11. Extract correction diagnostics
         * ------------------------------------------------------------
         */

        val positionCorrection =
            Vec3(
                errorState[POS_INDEX],
                errorState[POS_INDEX + 1],
                errorState[POS_INDEX + 2]
            )

        val velocityCorrection =
            Vec3(
                errorState[VEL_INDEX],
                errorState[VEL_INDEX + 1],
                errorState[VEL_INDEX + 2]
            )

        val attitudeCorrection =
            Vec3(
                errorState[ATT_INDEX],
                errorState[ATT_INDEX + 1],
                errorState[ATT_INDEX + 2]
            )

        val gyroBiasCorrection =
            Vec3(
                errorState[GYRO_BIAS_INDEX],
                errorState[GYRO_BIAS_INDEX + 1],
                errorState[GYRO_BIAS_INDEX + 2]
            )

        val accelBiasCorrection =
            Vec3(
                errorState[ACCEL_BIAS_INDEX],
                errorState[ACCEL_BIAS_INDEX + 1],
                errorState[ACCEL_BIAS_INDEX + 2]
            )

        /*
         * ------------------------------------------------------------
         * 12. Inject error state into nominal state
         * ------------------------------------------------------------
         */

        injectErrorState(
            state = state,
            errorState = errorState
        )

        /*
         * ------------------------------------------------------------
         * 13. Joseph-form covariance update
         * ------------------------------------------------------------
         *
         * Pnew =
         *
         *   (I-KH) P (I-KH)ᵀ
         *
         *   + K R Kᵀ
         *
         * This form is numerically safer than the simplified form.
         */

        josephUpdateInPlace(
            covariance = covariance,
            kalmanGain = kalmanGain,
            measurementVariance = measurementVariance
        )

        covariance.symmetrize()

        covariance.enforceNumericalSafety()

        /*
         * ------------------------------------------------------------
         * 14. Return successful update
         * ------------------------------------------------------------
         */

        return UpdateResult(
            accepted = true,
            innovation = innovation,
            innovationNormMps = innovationNormMps,
            nis = nis,
            velocityCorrection = velocityCorrection,
            positionCorrection = positionCorrection,
            attitudeCorrectionRad = attitudeCorrection,
            gyroBiasCorrection = gyroBiasCorrection,
            accelBiasCorrection = accelBiasCorrection,
            reason = null
        )
    }

    /**
     * Injects the 15-state error vector into the nominal state.
     */
    private fun injectErrorState(
        state: NavigationState,
        errorState: DoubleArray
    ) {

        /*
         * Position correction.
         */

        state.position =
            state.position +
                    Vec3(
                        errorState[POS_INDEX],
                        errorState[POS_INDEX + 1],
                        errorState[POS_INDEX + 2]
                    )

        /*
         * Velocity correction.
         */

        state.velocity =
            state.velocity +
                    Vec3(
                        errorState[VEL_INDEX],
                        errorState[VEL_INDEX + 1],
                        errorState[VEL_INDEX + 2]
                    )

        /*
         * Attitude correction.
         *
         * q_new = q_old ⊗ δq
         */

        val deltaTheta =
            Vec3(
                errorState[ATT_INDEX],
                errorState[ATT_INDEX + 1],
                errorState[ATT_INDEX + 2]
            )

        val deltaQuaternion =
            Quaternion.fromRotationVector(
                deltaTheta
            )

        state.attitude =
            (
                    state.attitude *
                            deltaQuaternion
                    ).normalized()

        /*
         * Gyroscope bias correction.
         */

        state.gyroBias =
            state.gyroBias +
                    Vec3(
                        errorState[GYRO_BIAS_INDEX],
                        errorState[GYRO_BIAS_INDEX + 1],
                        errorState[GYRO_BIAS_INDEX + 2]
                    )

        /*
         * Accelerometer bias correction.
         */

        state.accelBias =
            state.accelBias +
                    Vec3(
                        errorState[ACCEL_BIAS_INDEX],
                        errorState[ACCEL_BIAS_INDEX + 1],
                        errorState[ACCEL_BIAS_INDEX + 2]
                    )

        /*
         * Physical safety limits.
         */

        state.velocity =
            clampVectorMagnitude(
                state.velocity,
                config.maxVelocityMps
            )

        state.gyroBias =
            clampVectorMagnitude(
                state.gyroBias,
                config.maxGyroBiasRadPerSec
            )

        state.accelBias =
            clampVectorMagnitude(
                state.accelBias,
                config.maxAccelBiasMps2
            )

        state.position =
            clampVectorMagnitude(
                state.position,
                config.maxPositionM
            )
    }

    /**
     * Joseph covariance update performed directly on the existing
     * EskfCovariance object.
     *
     * This avoids any dependency on setFrom(Array<DoubleArray>).
     */
    private fun josephUpdateInPlace(
        covariance: EskfCovariance,
        kalmanGain: Array<DoubleArray>,
        measurementVariance: Double
    ) {

        /*
         * A = I - K H
         *
         * H contains an identity block at velocity columns 3..5.
         */

        val a =
            Array(STATE_SIZE) { row ->

                DoubleArray(STATE_SIZE) { column ->

                    var value =
                        if (row == column) {
                            1.0
                        } else {
                            0.0
                        }

                    if (
                        column >= VEL_INDEX &&
                        column < VEL_INDEX + MEASUREMENT_SIZE
                    ) {

                        val measurementColumn =
                            column - VEL_INDEX

                        value -=
                            kalmanGain[row][measurementColumn]
                    }

                    value
                }
            }

        /*
         * Save original covariance.
         *
         * We cannot update covariance while still using its old
         * values for the matrix multiplication.
         */

        val originalP =
            Array(STATE_SIZE) { row ->
                DoubleArray(STATE_SIZE) { column ->
                    covariance[row, column]
                }
            }

        /*
         * AP
         */

        val ap =
            Array(STATE_SIZE) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                var value = 0.0

                for (k in 0 until STATE_SIZE) {

                    value +=
                        a[i][k] *
                                originalP[k][j]
                }

                ap[i][j] =
                    value
            }
        }

        /*
         * APAᵀ
         */

        val updatedP =
            Array(STATE_SIZE) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                var value = 0.0

                for (k in 0 until STATE_SIZE) {

                    value +=
                        ap[i][k] *
                                a[j][k]
                }

                updatedP[i][j] =
                    value
            }
        }

        /*
         * K R Kᵀ
         *
         * R = measurementVariance * I
         */

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                var value = 0.0

                for (k in 0 until MEASUREMENT_SIZE) {

                    value +=
                        kalmanGain[i][k] *
                                measurementVariance *
                                kalmanGain[j][k]
                }

                updatedP[i][j] +=
                    value
            }
        }

        /*
         * Write matrix back through the existing
         * EskfCovariance operator fun set(row,column,value).
         */

        for (row in 0 until STATE_SIZE) {

            for (column in 0 until STATE_SIZE) {

                val value =
                    updatedP[row][column]

                if (value.isFinite()) {

                    covariance[
                        row,
                        column
                    ] = value

                } else {

                    /*
                     * If numerical instability occurred, keep the
                     * old covariance element rather than injecting
                     * NaN/Infinity into the filter.
                     */
                    covariance[
                        row,
                        column
                    ] = originalP[row][column]
                }
            }
        }
    }

    /**
     * Inverts a 3x3 matrix.
     */
    private fun invert3x3(
        matrix: Array<DoubleArray>
    ): Array<DoubleArray>? {

        val a = matrix[0][0]
        val b = matrix[0][1]
        val c = matrix[0][2]

        val d = matrix[1][0]
        val e = matrix[1][1]
        val f = matrix[1][2]

        val g = matrix[2][0]
        val h = matrix[2][1]
        val i = matrix[2][2]

        val cofactor00 =
            e * i - f * h

        val cofactor01 =
            -(d * i - f * g)

        val cofactor02 =
            d * h - e * g

        val cofactor10 =
            -(b * i - c * h)

        val cofactor11 =
            a * i - c * g

        val cofactor12 =
            -(a * h - b * g)

        val cofactor20 =
            b * f - c * e

        val cofactor21 =
            -(a * f - c * d)

        val cofactor22 =
            a * e - b * d

        val determinant =
            a * cofactor00 +
                    b * cofactor01 +
                    c * cofactor02

        if (!determinant.isFinite()) {
            return null
        }

        if (abs(determinant) < MIN_S_DIAGONAL) {
            return null
        }

        val inverseDeterminant =
            1.0 / determinant

        return arrayOf(

            doubleArrayOf(
                cofactor00 * inverseDeterminant,
                cofactor10 * inverseDeterminant,
                cofactor20 * inverseDeterminant
            ),

            doubleArrayOf(
                cofactor01 * inverseDeterminant,
                cofactor11 * inverseDeterminant,
                cofactor21 * inverseDeterminant
            ),

            doubleArrayOf(
                cofactor02 * inverseDeterminant,
                cofactor12 * inverseDeterminant,
                cofactor22 * inverseDeterminant
            )
        )
    }

    /**
     * Multiplies a 3x3 matrix by a 3-element vector.
     */
    private fun multiply3x3Vector(
        matrix: Array<DoubleArray>,
        vector: DoubleArray
    ): DoubleArray {

        return doubleArrayOf(

            matrix[0][0] * vector[0] +
                    matrix[0][1] * vector[1] +
                    matrix[0][2] * vector[2],

            matrix[1][0] * vector[0] +
                    matrix[1][1] * vector[1] +
                    matrix[1][2] * vector[2],

            matrix[2][0] * vector[0] +
                    matrix[2][1] * vector[1] +
                    matrix[2][2] * vector[2]
        )
    }

    /**
     * Dot product of two 3-element vectors.
     */
    private fun dot3(
        a: DoubleArray,
        b: DoubleArray
    ): Double {

        return a[0] * b[0] +
                a[1] * b[1] +
                a[2] * b[2]
    }

    /**
     * Creates a rejected update result.
     */
    private fun rejected(
        reason: String
    ): UpdateResult {

        return UpdateResult(
            accepted = false,
            innovation = Vec3.ZERO,
            innovationNormMps = 0.0,
            nis = Double.NaN,
            velocityCorrection = Vec3.ZERO,
            positionCorrection = Vec3.ZERO,
            attitudeCorrectionRad = Vec3.ZERO,
            gyroBiasCorrection = Vec3.ZERO,
            accelBiasCorrection = Vec3.ZERO,
            reason = reason
        )
    }

    /**
     * Limits a vector magnitude without changing its direction.
     */
    private fun clampVectorMagnitude(
        vector: Vec3,
        maximum: Double
    ): Vec3 {

        if (!vector.isFinite()) {
            return Vec3.ZERO
        }

        val magnitude =
            vector.norm()

        if (!magnitude.isFinite() ||
            magnitude <= 0.0 ||
            magnitude <= maximum
        ) {
            return vector
        }

        return vector *
                (maximum / magnitude)
    }
}