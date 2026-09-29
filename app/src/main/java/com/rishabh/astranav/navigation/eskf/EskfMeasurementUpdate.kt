package com.rishabh.astranav.navigation.eskf

import kotlin.math.abs

/**
 * Generic measurement-update engine for ASTRA-Core ESKF.
 *
 * Error-state:
 *
 *   [δp, δv, δθ, δbg, δba]
 *
 *   0..2    position
 *   3..5    velocity
 *   6..8    attitude
 *   9..11   gyro bias
 *   12..14  accel bias
 *
 * Generic measurement model:
 *
 *   z = h(x) + H * δx + noise
 *
 * Innovation:
 *
 *   y = z - h(x)
 *
 * Innovation covariance:
 *
 *   S = H P Hᵀ + R
 *
 * Kalman gain:
 *
 *   K = P Hᵀ S⁻¹
 *
 * Error-state correction:
 *
 *   δx = K y
 *
 * Covariance:
 *
 *   P = (I-KH)P(I-KH)ᵀ + K R Kᵀ
 *
 * This class does not decide whether a measurement should be trusted.
 * It performs mathematical validation and an optional NIS gate.
 */
class EskfMeasurementUpdate(
    private val config: EskfConfig = EskfConfig()
) {

    companion object {

        private const val STATE_SIZE = 15

        private const val POS_INDEX = 0
        private const val VEL_INDEX = 3
        private const val ATT_INDEX = 6
        private const val GYRO_BIAS_INDEX = 9
        private const val ACCEL_BIAS_INDEX = 12

        private const val MIN_DIAGONAL = 1.0e-12
        private const val MAX_MATRIX_VALUE = 1.0e12

        private const val MAX_MEASUREMENT_DIMENSION = 15
    }

    /**
     * Generic measurement request.
     *
     * measurement:
     *     Actual measurement z.
     *
     * predictedMeasurement:
     *     Expected measurement h(x).
     *
     * jacobian:
     *     H matrix with dimensions:
     *
     *         measurementDimension × 15
     *
     * measurementCovariance:
     *     R matrix with dimensions:
     *
     *         measurementDimension × measurementDimension
     *
     * nisGate:
     *     Optional NIS upper bound.
     *
     * If null, no NIS rejection is performed.
     */
    data class Measurement(
        val measurement: DoubleArray,
        val predictedMeasurement: DoubleArray,
        val jacobian: Array<DoubleArray>,
        val measurementCovariance: Array<DoubleArray>,
        val nisGate: Double? = null,
        val name: String = "measurement"
    )

    /**
     * Result of a generic measurement update.
     */
    data class UpdateResult(
        val accepted: Boolean,
        val measurementName: String,
        val innovation: DoubleArray,
        val innovationNorm: Double,
        val nis: Double,
        val errorState: DoubleArray,
        val reason: String? = null
    )

    /**
     * Applies one generic measurement update.
     *
     * State and covariance are modified only when the update succeeds.
     */
    @Synchronized
    fun update(
        state: NavigationState,
        covariance: EskfCovariance,
        measurement: Measurement
    ): UpdateResult {

        val dimension =
            measurement.measurement.size

        /*
         * ------------------------------------------------------------
         * 1. Validate measurement dimensions
         * ------------------------------------------------------------
         */

        if (dimension <= 0 ||
            dimension > MAX_MEASUREMENT_DIMENSION
        ) {
            return rejected(
                measurement.name,
                "Invalid measurement dimension"
            )
        }

        if (measurement.predictedMeasurement.size != dimension) {
            return rejected(
                measurement.name,
                "Predicted measurement dimension mismatch"
            )
        }

        if (measurement.jacobian.size != dimension) {
            return rejected(
                measurement.name,
                "Jacobian row count mismatch"
            )
        }

        for (row in measurement.jacobian) {

            if (row.size != STATE_SIZE) {
                return rejected(
                    measurement.name,
                    "Jacobian must have 15 columns"
                )
            }
        }

        if (
            measurement.measurementCovariance.size !=
            dimension
        ) {
            return rejected(
                measurement.name,
                "Measurement covariance dimension mismatch"
            )
        }

        for (row in measurement.measurementCovariance) {

            if (row.size != dimension) {
                return rejected(
                    measurement.name,
                    "Measurement covariance is not square"
                )
            }
        }

        /*
         * ------------------------------------------------------------
         * 2. Validate finite values
         * ------------------------------------------------------------
         */

        if (
            measurement.measurement.any { !it.isFinite() } ||
            measurement.predictedMeasurement.any { !it.isFinite() }
        ) {
            return rejected(
                measurement.name,
                "Measurement contains non-finite values"
            )
        }

        if (
            measurement.jacobian.any { row ->
                row.any { !it.isFinite() }
            }
        ) {
            return rejected(
                measurement.name,
                "Jacobian contains non-finite values"
            )
        }

        if (
            measurement.measurementCovariance.any { row ->
                row.any { !it.isFinite() }
            }
        ) {
            return rejected(
                measurement.name,
                "Measurement covariance contains non-finite values"
            )
        }

        /*
         * ------------------------------------------------------------
         * 3. Validate nominal state and covariance
         * ------------------------------------------------------------
         */

        if (!state.position.isFinite()) {
            return rejected(
                measurement.name,
                "State position is non-finite"
            )
        }

        if (!state.velocity.isFinite()) {
            return rejected(
                measurement.name,
                "State velocity is non-finite"
            )
        }

        if (!state.attitude.isFinite()) {
            return rejected(
                measurement.name,
                "State attitude is non-finite"
            )
        }

        if (!state.gyroBias.isFinite()) {
            return rejected(
                measurement.name,
                "Gyro bias is non-finite"
            )
        }

        if (!state.accelBias.isFinite()) {
            return rejected(
                measurement.name,
                "Accelerometer bias is non-finite"
            )
        }

        if (!covariance.isFinite()) {
            return rejected(
                measurement.name,
                "Covariance contains non-finite values"
            )
        }

        if (!covariance.hasValidDiagonal()) {
            return rejected(
                measurement.name,
                "Covariance diagonal is invalid"
            )
        }

        /*
         * ------------------------------------------------------------
         * 4. Innovation
         * ------------------------------------------------------------
         *
         * y = z - h(x)
         */

        val innovation =
            DoubleArray(dimension)

        for (i in 0 until dimension) {

            innovation[i] =
                measurement.measurement[i] -
                        measurement.predictedMeasurement[i]

            if (!innovation[i].isFinite()) {
                return rejected(
                    measurement.name,
                    "Innovation contains non-finite values"
                )
            }
        }

        val innovationNorm =
            vectorNorm(
                innovation
            )

        /*
         * ------------------------------------------------------------
         * 5. Calculate HP
         * ------------------------------------------------------------
         */

        val h =
            measurement.jacobian

        val p =
            copyCovariance(
                covariance
            )

        val hp =
            Array(dimension) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until dimension) {

            for (j in 0 until STATE_SIZE) {

                var value = 0.0

                for (k in 0 until STATE_SIZE) {

                    value +=
                        h[i][k] *
                                p[k][j]
                }

                hp[i][j] =
                    value
            }
        }

        /*
         * ------------------------------------------------------------
         * 6. Innovation covariance
         * ------------------------------------------------------------
         *
         * S = HPHᵀ + R
         */

        val s =
            Array(dimension) {
                DoubleArray(dimension)
            }

        for (i in 0 until dimension) {

            for (j in 0 until dimension) {

                var value = 0.0

                for (k in 0 until STATE_SIZE) {

                    value +=
                        hp[i][k] *
                                h[j][k]
                }

                value +=
                    measurement.measurementCovariance[i][j]

                s[i][j] =
                    value
            }
        }

        /*
         * Symmetrize S.
         *
         * Small floating-point asymmetry can otherwise make
         * a mathematically symmetric matrix harder to invert.
         */

        symmetrize(
            s
        )

        /*
         * Ensure positive diagonal floor.
         */

        for (i in 0 until dimension) {

            if (!s[i][i].isFinite()) {
                return rejected(
                    measurement.name,
                    "Innovation covariance is non-finite"
                )
            }

            s[i][i] =
                maxOf(
                    s[i][i],
                    MIN_DIAGONAL
                )
        }

        /*
         * ------------------------------------------------------------
         * 7. Invert S
         * ------------------------------------------------------------
         */

        val sInverse =
            invertMatrix(
                s
            ) ?: return rejected(
                measurement.name,
                "Innovation covariance is singular"
            )

        /*
         * ------------------------------------------------------------
         * 8. NIS
         * ------------------------------------------------------------
         *
         * NIS = yᵀ S⁻¹ y
         */

        val sInvInnovation =
            multiplyMatrixVector(
                sInverse,
                innovation
            )

        val nis =
            dot(
                innovation,
                sInvInnovation
            )

        if (!nis.isFinite() || nis < 0.0) {
            return rejected(
                measurement.name,
                "Invalid NIS"
            )
        }

        /*
         * Optional integrity gate.
         *
         * If a caller supplies a gate and NIS is above it,
         * reject the measurement before changing the state.
         */

        val nisGate =
            measurement.nisGate

        if (
            nisGate != null &&
            (
                    !nisGate.isFinite() ||
                            nisGate <= 0.0
                    )
        ) {
            return rejected(
                measurement.name,
                "Invalid NIS gate"
            )
        }

        if (
            nisGate != null &&
            nis > nisGate
        ) {
            return UpdateResult(
                accepted = false,
                measurementName =
                    measurement.name,
                innovation =
                    innovation.copyOf(),
                innovationNorm =
                    innovationNorm,
                nis =
                    nis,
                errorState =
                    DoubleArray(STATE_SIZE),
                reason =
                    "NIS gate rejected measurement"
            )
        }

        /*
         * ------------------------------------------------------------
         * 9. Kalman gain
         * ------------------------------------------------------------
         *
         * K = P Hᵀ S⁻¹
         *
         * First:
         *
         *     PHᵀ = (HP)ᵀ
         */

        val kalmanGain =
            Array(STATE_SIZE) {
                DoubleArray(dimension)
            }

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until dimension) {

                var value = 0.0

                for (k in 0 until dimension) {

                    value +=
                        hp[k][i] *
                                sInverse[k][j]
                }

                kalmanGain[i][j] =
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

        for (i in 0 until STATE_SIZE) {

            var value = 0.0

            for (j in 0 until dimension) {

                value +=
                    kalmanGain[i][j] *
                            innovation[j]
            }

            errorState[i] =
                value
        }

        if (
            errorState.any { !it.isFinite() }
        ) {
            return rejected(
                measurement.name,
                "Error-state correction is non-finite"
            )
        }

        /*
         * ------------------------------------------------------------
         * 11. Inject nominal-state correction
         * ------------------------------------------------------------
         */

        injectErrorState(
            state = state,
            errorState = errorState
        )

        /*
         * ------------------------------------------------------------
         * 12. Joseph covariance update
         * ------------------------------------------------------------
         *
         * A = I - KH
         *
         * Pnew =
         *
         *   A P Aᵀ + K R Kᵀ
         */

        val updatedCovariance =
            josephUpdate(
                originalCovariance = p,
                jacobian = h,
                kalmanGain = kalmanGain,
                measurementCovariance =
                    measurement.measurementCovariance
            )

        /*
         * Write the covariance back.
         */

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                val value =
                    updatedCovariance[i][j]

                if (!value.isFinite()) {

                    return rejected(
                        measurement.name,
                        "Updated covariance became non-finite"
                    )
                }

                covariance[i, j] =
                    value
            }
        }

        covariance.symmetrize()

        covariance.enforceNumericalSafety()

        /*
         * ------------------------------------------------------------
         * 13. Successful result
         * ------------------------------------------------------------
         */

        return UpdateResult(
            accepted = true,
            measurementName =
                measurement.name,
            innovation =
                innovation.copyOf(),
            innovationNorm =
                innovationNorm,
            nis =
                nis,
            errorState =
                errorState.copyOf(),
            reason = null
        )
    }

    /**
     * Injects the 15-state correction into the nominal state.
     */
    private fun injectErrorState(
        state: NavigationState,
        errorState: DoubleArray
    ) {

        /*
         * Position.
         */

        state.position =
            state.position +
                    Vec3(
                        errorState[POS_INDEX],
                        errorState[POS_INDEX + 1],
                        errorState[POS_INDEX + 2]
                    )

        /*
         * Velocity.
         */

        state.velocity =
            state.velocity +
                    Vec3(
                        errorState[VEL_INDEX],
                        errorState[VEL_INDEX + 1],
                        errorState[VEL_INDEX + 2]
                    )

        /*
         * Attitude.
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
         * Gyroscope bias.
         */

        state.gyroBias =
            state.gyroBias +
                    Vec3(
                        errorState[GYRO_BIAS_INDEX],
                        errorState[GYRO_BIAS_INDEX + 1],
                        errorState[GYRO_BIAS_INDEX + 2]
                    )

        /*
         * Accelerometer bias.
         */

        state.accelBias =
            state.accelBias +
                    Vec3(
                        errorState[ACCEL_BIAS_INDEX],
                        errorState[ACCEL_BIAS_INDEX + 1],
                        errorState[ACCEL_BIAS_INDEX + 2]
                    )

        /*
         * Physical bounds.
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
     * Joseph-form covariance update.
     */
    private fun josephUpdate(
        originalCovariance:
        Array<DoubleArray>,
        jacobian:
        Array<DoubleArray>,
        kalmanGain:
        Array<DoubleArray>,
        measurementCovariance:
        Array<DoubleArray>
    ): Array<DoubleArray> {

        val dimension =
            jacobian.size

        /*
         * A = I - KH
         */

        val a =
            Array(STATE_SIZE) {
                DoubleArray(STATE_SIZE)
            }

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                var value =
                    if (i == j) {
                        1.0
                    } else {
                        0.0
                    }

                for (k in 0 until dimension) {

                    value -=
                        kalmanGain[i][k] *
                                jacobian[k][j]
                }

                a[i][j] =
                    value
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
                                originalCovariance[k][j]
                }

                ap[i][j] =
                    value
            }
        }

        /*
         * APAᵀ
         */

        val updated =
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

                updated[i][j] =
                    value
            }
        }

        /*
         * KRKᵀ
         */

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                var value = 0.0

                for (r in 0 until dimension) {

                    for (c in 0 until dimension) {

                        value +=
                            kalmanGain[i][r] *
                                    measurementCovariance[r][c] *
                                    kalmanGain[j][c]
                    }
                }

                updated[i][j] +=
                    value
            }
        }

        /*
         * Symmetrize before returning.
         */

        symmetrize(
            updated
        )

        return updated
    }

    /**
     * Copies the custom EskfCovariance into a normal Kotlin matrix.
     */
    private fun copyCovariance(
        covariance: EskfCovariance
    ): Array<DoubleArray> {

        return Array(STATE_SIZE) { row ->
            DoubleArray(STATE_SIZE) { column ->
                covariance[row, column]
            }
        }
    }

    /**
     * Generic square-matrix inversion using Gauss-Jordan
     * elimination with partial pivoting.
     */
    private fun invertMatrix(
        input: Array<DoubleArray>
    ): Array<DoubleArray>? {

        val n =
            input.size

        if (n == 0) {
            return null
        }

        for (row in input) {
            if (row.size != n) {
                return null
            }
        }

        /*
         * Augmented matrix:
         *
         * [ A | I ]
         */

        val augmented =
            Array(n) { row ->

                DoubleArray(n * 2) { column ->

                    when {
                        column < n ->
                            input[row][column]

                        column - n == row ->
                            1.0

                        else ->
                            0.0
                    }
                }
            }

        for (column in 0 until n) {

            /*
             * Find largest pivot.
             */

            var pivotRow =
                column

            var pivotMagnitude =
                abs(
                    augmented[column][column]
                )

            for (row in column + 1 until n) {

                val magnitude =
                    abs(
                        augmented[row][column]
                    )

                if (magnitude > pivotMagnitude) {

                    pivotMagnitude =
                        magnitude

                    pivotRow =
                        row
                }
            }

            if (
                !pivotMagnitude.isFinite() ||
                pivotMagnitude < MIN_DIAGONAL
            ) {
                return null
            }

            /*
             * Swap rows.
             */

            if (pivotRow != column) {

                val temporary =
                    augmented[column]

                augmented[column] =
                    augmented[pivotRow]

                augmented[pivotRow] =
                    temporary
            }

            /*
             * Normalize pivot row.
             */

            val pivot =
                augmented[column][column]

            for (j in 0 until n * 2) {

                augmented[column][j] /=
                    pivot
            }

            /*
             * Eliminate this column from all other rows.
             */

            for (row in 0 until n) {

                if (row == column) {
                    continue
                }

                val factor =
                    augmented[row][column]

                if (factor == 0.0) {
                    continue
                }

                for (j in 0 until n * 2) {

                    augmented[row][j] -=
                        factor *
                                augmented[column][j]
                }
            }
        }

        /*
         * Extract inverse.
         */

        val inverse =
            Array(n) {
                DoubleArray(n)
            }

        for (i in 0 until n) {

            for (j in 0 until n) {

                val value =
                    augmented[i][n + j]

                if (
                    !value.isFinite() ||
                    abs(value) > MAX_MATRIX_VALUE
                ) {
                    return null
                }

                inverse[i][j] =
                    value
            }
        }

        return inverse
    }

    /**
     * Matrix × vector.
     */
    private fun multiplyMatrixVector(
        matrix: Array<DoubleArray>,
        vector: DoubleArray
    ): DoubleArray {

        val rows =
            matrix.size

        val columns =
            vector.size

        val result =
            DoubleArray(rows)

        for (i in 0 until rows) {

            var value = 0.0

            for (j in 0 until columns) {

                value +=
                    matrix[i][j] *
                            vector[j]
            }

            result[i] =
                value
        }

        return result
    }

    /**
     * Dot product.
     */
    private fun dot(
        a: DoubleArray,
        b: DoubleArray
    ): Double {

        var result = 0.0

        for (i in a.indices) {
            result +=
                a[i] *
                        b[i]
        }

        return result
    }

    /**
     * Euclidean vector norm.
     */
    private fun vectorNorm(
        vector: DoubleArray
    ): Double {

        var sum = 0.0

        for (value in vector) {

            sum +=
                value *
                        value
        }

        return kotlin.math.sqrt(sum)
    }

    /**
     * Symmetrizes a square matrix.
     */
    private fun symmetrize(
        matrix: Array<DoubleArray>
    ) {

        val n =
            matrix.size

        for (i in 0 until n) {

            for (j in i + 1 until n) {

                val average =
                    0.5 *
                            (
                                    matrix[i][j] +
                                            matrix[j][i]
                                    )

                matrix[i][j] =
                    average

                matrix[j][i] =
                    average
            }
        }
    }

    /**
     * Clamps a vector magnitude while preserving direction.
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

        if (
            !magnitude.isFinite() ||
            magnitude <= 0.0 ||
            magnitude <= maximum
        ) {
            return vector
        }

        return vector *
                (maximum / magnitude)
    }

    /**
     * Rejected update result.
     */
    private fun rejected(
        measurementName: String,
        reason: String
    ): UpdateResult {

        return UpdateResult(
            accepted = false,
            measurementName = measurementName,
            innovation = DoubleArray(0),
            innovationNorm = 0.0,
            nis = Double.NaN,
            errorState = DoubleArray(STATE_SIZE),
            reason = reason
        )
    }
}