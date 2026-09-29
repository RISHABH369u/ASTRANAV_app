package com.rishabh.astranav.navigation.eskf

/**
 * Covariance propagation for ASTRA-Core ESKF.
 *
 * Error-state ordering:
 *
 *   0..2    position
 *   3..5    velocity
 *   6..8    attitude
 *   9..11   gyro bias
 *   12..14  accelerometer bias
 *
 * Total = 15 states.
 *
 * Covariance propagation:
 *
 *     P(k+1) = F P(k) F^T + Q
 */
class EskfCovariancePrediction(
    private val config: EskfConfig
) {

    companion object {

        private const val N = 15

        private const val POS = 0
        private const val VEL = 3
        private const val ATT = 6
        private const val GYRO_BIAS = 9
        private const val ACCEL_BIAS = 12
    }

    /**
     * Propagates covariance for one IMU interval.
     */
    fun propagate(
        covariance: EskfCovariance,
        state: NavigationState,
        specificForceBody: Vec3,
        gyroRateBody: Vec3,
        dtSeconds: Double
    ) {

        require(
            dtSeconds.isFinite()
        ) {
            "dtSeconds must be finite"
        }

        require(
            dtSeconds >=
                    config.minDeltaTimeSeconds &&
                    dtSeconds <=
                    config.maxDeltaTimeSeconds
        ) {
            "Invalid covariance dt: $dtSeconds"
        }

        require(
            specificForceBody.isFinite()
        ) {
            "specificForceBody contains non-finite values"
        }

        require(
            gyroRateBody.isFinite()
        ) {
            "gyroRateBody contains non-finite values"
        }

        require(
            state.attitude.isFinite()
        ) {
            "State attitude is non-finite"
        }

        val f =
            buildStateTransitionMatrix(
                state = state,
                specificForceBody =
                    specificForceBody,
                gyroRateBody =
                    gyroRateBody,
                dtSeconds =
                    dtSeconds
            )

        val q =
            buildDiscreteProcessNoise(
                dtSeconds
            )

        val p =
            covariance.copyMatrix()

        /*
         * Pnew = F * P * F^T + Q
         */
        val fp =
            multiply(
                f,
                p
            )

        val fTranspose =
            transpose(f)

        val propagated =
            multiply(
                fp,
                fTranspose
            )

        addInPlace(
            propagated,
            q
        )

        /*
         * Covariance must remain symmetric.
         */
        symmetrizeInPlace(
            propagated
        )

        /*
         * Numerical protection.
         */
        for (i in 0 until N) {

            if (
                propagated[i][i] < 0.0 &&
                propagated[i][i] > -1e-10
            ) {
                propagated[i][i] =
                    1e-12
            }
        }

        copyIntoCovariance(
            source = propagated,
            destination = covariance
        )

        covariance.enforceNumericalSafety()
    }

    /**
     * First-order discrete error-state transition matrix.
     *
     * Continuous model:
     *
     *   d(dp)/dt = dv
     *
     *   d(dv)/dt =
     *       -R[f]x dtheta
     *       -R dba
     *
     *   d(dtheta)/dt =
     *       -[w]x dtheta
     *       -dbg
     *
     *   d(dbg)/dt = 0
     *   d(dba)/dt = 0
     */
    private fun buildStateTransitionMatrix(
        state: NavigationState,
        specificForceBody: Vec3,
        gyroRateBody: Vec3,
        dtSeconds: Double
    ): Array<DoubleArray> {

        val f =
            identityMatrix(N)

        val rotation =
            state.attitude
                .normalized()
                .toRotationMatrix()

        /*
         * dp <- dv
         */
        setBlock(
            matrix = f,
            row = POS,
            column = VEL,
            block =
                identity3Scaled(
                    dtSeconds
                )
        )

        /*
         * dv <- dtheta
         *
         *     -R[f]x dt
         */
        val forceSkew =
            skew(
                specificForceBody
            )

        val velocityAttitude =
            multiply3x3(
                rotation,
                forceSkew
            )

        scale3x3(
            velocityAttitude,
            -dtSeconds
        )

        setBlock(
            matrix = f,
            row = VEL,
            column = ATT,
            block =
                velocityAttitude
        )

        /*
         * dv <- dba
         *
         *     -R dt
         */
        setBlock(
            matrix = f,
            row = VEL,
            column = ACCEL_BIAS,
            block =
                scale3x3Copy(
                    rotation,
                    -dtSeconds
                )
        )

        /*
         * dtheta <- dtheta
         *
         *     I - [omega]x dt
         */
        val gyroSkew =
            skew(
                gyroRateBody
            )

        val attitudeBlock =
            identity3()

        for (i in 0 until 3) {
            for (j in 0 until 3) {

                attitudeBlock[i][j] -=
                    gyroSkew[i][j] *
                            dtSeconds
            }
        }

        setBlock(
            matrix = f,
            row = ATT,
            column = ATT,
            block = attitudeBlock
        )

        /*
         * dtheta <- dbg
         *
         *     -I dt
         */
        setBlock(
            matrix = f,
            row = ATT,
            column = GYRO_BIAS,
            block =
                identity3Scaled(
                    -dtSeconds
                )
        )

        return f
    }

    /**
     * Discrete process-noise covariance.
     *
     * For a white-noise density:
     *
     *     variance ≈ density² * dt
     */
    private fun buildDiscreteProcessNoise(
        dtSeconds: Double
    ): Array<DoubleArray> {

        val q =
            Array(N) {
                DoubleArray(N)
            }

        val gyroVariance =
            config.gyroNoiseDensity *
                    config.gyroNoiseDensity *
                    dtSeconds

        val accelVariance =
            config.accelNoiseDensity *
                    config.accelNoiseDensity *
                    dtSeconds

        val gyroBiasVariance =
            config.gyroBiasRandomWalk *
                    config.gyroBiasRandomWalk *
                    dtSeconds

        val accelBiasVariance =
            config.accelBiasRandomWalk *
                    config.accelBiasRandomWalk *
                    dtSeconds

        /*
         * Attitude process noise.
         */
        for (i in 0 until 3) {

            q[ATT + i][ATT + i] =
                gyroVariance
        }

        /*
         * Velocity process noise.
         */
        for (i in 0 until 3) {

            q[VEL + i][VEL + i] =
                accelVariance
        }

        /*
         * Gyro-bias random walk.
         */
        for (i in 0 until 3) {

            q[GYRO_BIAS + i][GYRO_BIAS + i] =
                gyroBiasVariance
        }

        /*
         * Accelerometer-bias random walk.
         */
        for (i in 0 until 3) {

            q[ACCEL_BIAS + i][ACCEL_BIAS + i] =
                accelBiasVariance
        }

        return q
    }

    // ---------------------------------------------------------------
    // Matrix helpers
    // ---------------------------------------------------------------

    private fun identityMatrix(
        size: Int
    ): Array<DoubleArray> {

        return Array(size) { row ->
            DoubleArray(size) { column ->
                if (row == column) {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }

    private fun identity3():
            Array<DoubleArray> {

        return arrayOf(
            doubleArrayOf(
                1.0, 0.0, 0.0
            ),
            doubleArrayOf(
                0.0, 1.0, 0.0
            ),
            doubleArrayOf(
                0.0, 0.0, 1.0
            )
        )
    }

    private fun identity3Scaled(
        scale: Double
    ): Array<DoubleArray> {

        return arrayOf(
            doubleArrayOf(
                scale, 0.0, 0.0
            ),
            doubleArrayOf(
                0.0, scale, 0.0
            ),
            doubleArrayOf(
                0.0, 0.0, scale
            )
        )
    }

    /**
     * Skew matrix:
     *
     * [v]x =
     *
     *     [ 0  -vz  vy]
     *     [vz   0  -vx]
     *     [-vy vx   0 ]
     */
    private fun skew(
        vector: Vec3
    ): Array<DoubleArray> {

        return arrayOf(

            doubleArrayOf(
                0.0,
                -vector.z,
                vector.y
            ),

            doubleArrayOf(
                vector.z,
                0.0,
                -vector.x
            ),

            doubleArrayOf(
                -vector.y,
                vector.x,
                0.0
            )
        )
    }

    private fun multiply3x3(
        a: Array<DoubleArray>,
        b: Array<DoubleArray>
    ): Array<DoubleArray> {

        val result =
            Array(3) {
                DoubleArray(3)
            }

        for (i in 0 until 3) {
            for (j in 0 until 3) {

                var sum = 0.0

                for (k in 0 until 3) {
                    sum +=
                        a[i][k] *
                                b[k][j]
                }

                result[i][j] =
                    sum
            }
        }

        return result
    }

    private fun scale3x3(
        matrix: Array<DoubleArray>,
        scale: Double
    ) {

        for (i in 0 until 3) {
            for (j in 0 until 3) {

                matrix[i][j] *=
                    scale
            }
        }
    }

    private fun scale3x3Copy(
        matrix: Array<DoubleArray>,
        scale: Double
    ): Array<DoubleArray> {

        return Array(3) { i ->
            DoubleArray(3) { j ->
                matrix[i][j] *
                        scale
            }
        }
    }

    private fun setBlock(
        matrix: Array<DoubleArray>,
        row: Int,
        column: Int,
        block: Array<DoubleArray>
    ) {

        require(
            block.size == 3
        ) {
            "Block must contain 3 rows"
        }

        for (i in 0 until 3) {

            require(
                block[i].size == 3
            ) {
                "Block must contain 3 columns"
            }

            for (j in 0 until 3) {

                matrix[row + i][column + j] =
                    block[i][j]
            }
        }
    }

    private fun multiply(
        a: Array<DoubleArray>,
        b: Array<DoubleArray>
    ): Array<DoubleArray> {

        require(a.isNotEmpty()) {
            "Matrix A cannot be empty"
        }

        require(b.isNotEmpty()) {
            "Matrix B cannot be empty"
        }

        val aRows =
            a.size

        val aColumns =
            a[0].size

        val bRows =
            b.size

        val bColumns =
            b[0].size

        require(
            aColumns == bRows
        ) {
            "Matrix dimension mismatch: " +
                    "${aRows}x${aColumns} × " +
                    "${bRows}x${bColumns}"
        }

        val result =
            Array(aRows) {
                DoubleArray(bColumns)
            }

        for (i in 0 until aRows) {

            for (k in 0 until aColumns) {

                val value =
                    a[i][k]

                if (
                    kotlin.math.abs(value) <
                    1e-18
                ) {
                    continue
                }

                for (j in 0 until bColumns) {

                    result[i][j] +=
                        value *
                                b[k][j]
                }
            }
        }

        return result
    }

    private fun transpose(
        matrix: Array<DoubleArray>
    ): Array<DoubleArray> {

        val rows =
            matrix.size

        val columns =
            matrix[0].size

        return Array(columns) { column ->

            DoubleArray(rows) { row ->

                matrix[row][column]
            }
        }
    }

    private fun addInPlace(
        target: Array<DoubleArray>,
        addition: Array<DoubleArray>
    ) {

        require(
            target.size ==
                    addition.size
        ) {
            "Matrix row mismatch"
        }

        for (i in target.indices) {

            require(
                target[i].size ==
                        addition[i].size
            ) {
                "Matrix column mismatch"
            }

            for (j in target[i].indices) {

                target[i][j] +=
                    addition[i][j]
            }
        }
    }

    private fun symmetrizeInPlace(
        matrix: Array<DoubleArray>
    ) {

        for (i in matrix.indices) {

            for (j in i + 1 until matrix[i].size) {

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

    private fun copyIntoCovariance(
        source: Array<DoubleArray>,
        destination: EskfCovariance
    ) {

        require(
            source.size == N
        ) {
            "Source covariance must be 15x15"
        }

        for (i in 0 until N) {

            require(
                source[i].size == N
            ) {
                "Source covariance must be 15x15"
            }

            for (j in 0 until N) {

                destination[i, j] =
                    source[i][j]
            }
        }
    }
}