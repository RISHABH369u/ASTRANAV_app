package com.rishabh.astranav.navigation.eskf

import kotlin.math.max

/**
 * 15x15 covariance matrix for ASTRA-Core ESKF.
 *
 * Error-state ordering:
 *
 *   0..2    position error
 *   3..5    velocity error
 *   6..8    attitude error
 *   9..11   gyro-bias error
 *   12..14  accelerometer-bias error
 *
 * The matrix is stored in row-major form.
 *
 * This class deliberately owns covariance-related operations so that
 * Eskf.kt does not become a giant matrix-management class.
 */
class EskfCovariance private constructor(
    private val matrix: Array<DoubleArray>
) {

    companion object {

        const val STATE_SIZE = 15

        /**
         * Creates covariance from the diagonal supplied by EskfConfig.
         */
        fun fromConfig(config: EskfConfig): EskfCovariance {

            val diagonal = config.initialCovarianceDiagonal()

            require(diagonal.size == STATE_SIZE) {
                "Initial covariance must contain 15 diagonal values"
            }

            val p = zeroMatrix()

            for (i in 0 until STATE_SIZE) {
                p[i][i] = diagonal[i]
            }

            return EskfCovariance(p)
        }

        private fun zeroMatrix(): Array<DoubleArray> {
            return Array(STATE_SIZE) {
                DoubleArray(STATE_SIZE)
            }
        }
    }

    /**
     * Returns a deep copy of the covariance matrix.
     *
     * This prevents callers from accidentally modifying the internal
     * covariance without going through this class.
     */
    fun copyMatrix(): Array<DoubleArray> {
        return Array(STATE_SIZE) { row ->
            matrix[row].clone()
        }
    }

    /**
     * Returns P[i,j].
     */
    operator fun get(
        row: Int,
        column: Int
    ): Double {
        checkIndex(row)
        checkIndex(column)

        return matrix[row][column]
    }

    /**
     * Sets P[i,j].
     */
    operator fun set(
        row: Int,
        column: Int,
        value: Double
    ) {
        checkIndex(row)
        checkIndex(column)

        require(value.isFinite()) {
            "Covariance value must be finite"
        }

        matrix[row][column] = value
    }

    /**
     * Returns the diagonal element P[i,i].
     */
    fun diagonal(index: Int): Double {
        checkIndex(index)
        return matrix[index][index]
    }

    /**
     * Sets the diagonal element P[i,i].
     */
    fun setDiagonal(
        index: Int,
        value: Double
    ) {
        checkIndex(index)

        require(value.isFinite()) {
            "Covariance diagonal must be finite"
        }

        require(value >= 0.0) {
            "Covariance diagonal cannot be negative"
        }

        matrix[index][index] = value
    }

    /**
     * Adds a value to P[i,i].
     */
    fun addToDiagonal(
        index: Int,
        value: Double
    ) {
        checkIndex(index)

        require(value.isFinite()) {
            "Added covariance value must be finite"
        }

        matrix[index][index] += value
    }

    /**
     * Returns the complete diagonal.
     */
    fun diagonal(): DoubleArray {
        return DoubleArray(STATE_SIZE) { i ->
            matrix[i][i]
        }
    }

    /**
     * Forces P to be exactly symmetric.
     *
     * Numerical matrix operations can introduce tiny asymmetry:
     *
     *     P[i,j] != P[j,i]
     *
     * even though mathematically P must remain symmetric.
     */
    fun symmetrize() {

        for (i in 0 until STATE_SIZE) {

            for (j in i + 1 until STATE_SIZE) {

                val average =
                    0.5 * (matrix[i][j] + matrix[j][i])

                matrix[i][j] = average
                matrix[j][i] = average
            }
        }
    }

    /**
     * Numerical safety cleanup.
     *
     * This does NOT invent covariance information.
     *
     * It only:
     *   1. checks finite values
     *   2. removes tiny negative diagonal values caused by
     *      floating-point roundoff
     *   3. prevents NaN/Infinity from silently entering ESKF
     */
    fun enforceNumericalSafety(
        minimumVariance: Double = 1e-12,
        maximumVariance: Double = 1e12
    ) {

        require(
            minimumVariance >= 0.0 &&
                    maximumVariance > minimumVariance
        ) {
            "Invalid covariance safety limits"
        }

        for (i in 0 until STATE_SIZE) {

            for (j in 0 until STATE_SIZE) {

                val value = matrix[i][j]

                require(value.isFinite()) {
                    "ESKF covariance contains NaN/Infinity at [$i,$j]"
                }

                if (i == j) {

                    matrix[i][j] = value
                        .coerceAtLeast(minimumVariance)
                        .coerceAtMost(maximumVariance)
                }
            }
        }

        symmetrize()
    }

    /**
     * Adds diagonal process noise.
     *
     * qDiagonal[i] represents variance added to P[i,i].
     */
    fun addDiagonalProcessNoise(
        qDiagonal: DoubleArray
    ) {

        require(qDiagonal.size == STATE_SIZE) {
            "Process-noise vector must have 15 elements"
        }

        for (i in 0 until STATE_SIZE) {

            val q = qDiagonal[i]

            require(q.isFinite()) {
                "Process noise must be finite"
            }

            require(q >= 0.0) {
                "Process noise variance cannot be negative"
            }

            matrix[i][i] += q
        }
    }

    /**
     * Copies values from another 15x15 covariance.
     */
    fun setFrom(
        other: EskfCovariance
    ) {

        for (i in 0 until STATE_SIZE) {
            for (j in 0 until STATE_SIZE) {
                matrix[i][j] = other.matrix[i][j]
            }
        }
    }

    /**
     * Returns the largest absolute covariance element.
     *
     * Useful for diagnostics.
     */
    fun maxAbsoluteElement(): Double {

        var maximum = 0.0

        for (i in 0 until STATE_SIZE) {
            for (j in 0 until STATE_SIZE) {

                maximum = max(
                    maximum,
                    kotlin.math.abs(matrix[i][j])
                )
            }
        }

        return maximum
    }

    /**
     * Returns true when every covariance element is finite.
     */
    fun isFinite(): Boolean {

        for (i in 0 until STATE_SIZE) {
            for (j in 0 until STATE_SIZE) {

                if (!matrix[i][j].isFinite()) {
                    return false
                }
            }
        }

        return true
    }

    /**
     * Returns true when the covariance diagonal is valid.
     */
    fun hasValidDiagonal(
        minimumVariance: Double = 0.0
    ): Boolean {

        for (i in 0 until STATE_SIZE) {

            val value = matrix[i][i]

            if (!value.isFinite()) {
                return false
            }

            if (value < minimumVariance) {
                return false
            }
        }

        return true
    }

    /**
     * Human-readable diagnostic summary.
     */
    fun diagnosticSummary(): String {

        return buildString {

            append("P15x15")
            append(" | finite=")
            append(isFinite())

            append(" | diagValid=")
            append(hasValidDiagonal())

            append(" | maxAbs=")
            append(maxAbsoluteElement())
        }
    }

    private fun checkIndex(index: Int) {

        require(
            index in 0 until STATE_SIZE
        ) {
            "Covariance index out of range: $index"
        }
    }
}