package com.rishabh.astranav.navigation.eskf

/**
 * Non-Holonomic Constraint (NHC) measurement update.
 *
 * Vehicle body frame = FRD:
 *   X = Forward
 *   Y = Right
 *   Z = Down
 *
 * For a normally moving ground vehicle:
 *
 *   lateral body velocity  ~= 0
 *   vertical body velocity ~= 0
 *
 * Forward velocity is NOT constrained.
 *
 * This class only constructs the NHC measurement model.
 * All Kalman filtering mathematics is delegated to
 * EskfMeasurementUpdate.
 */
class EskfNhcUpdate(
    private val lateralStdMps: Double = DEFAULT_LATERAL_STD_MPS,
    private val verticalStdMps: Double = DEFAULT_VERTICAL_STD_MPS,
    private val nisGate: Double? = DEFAULT_NIS_GATE
) {

    companion object {

        private const val STATE_SIZE = 15

        private const val VEL_INDEX = 3
        private const val ATT_INDEX = 6

        /**
         * Soft lateral NHC uncertainty.
         */
        const val DEFAULT_LATERAL_STD_MPS = 0.20

        /**
         * Soft vertical NHC uncertainty.
         *
         * Vertical motion can be affected by road geometry,
         * suspension and imperfect attitude estimation, so this
         * is intentionally softer than an exact zero constraint.
         */
        const val DEFAULT_VERTICAL_STD_MPS = 0.30

        /**
         * Chi-square gate for a 2-dimensional NHC measurement.
         */
        const val DEFAULT_NIS_GATE = 9.21
    }

    /**
     * Result exposed to the ESKF controller.
     */
    data class UpdateResult(
        val accepted: Boolean,
        val bodyVelocityX: Double,
        val bodyVelocityY: Double,
        val bodyVelocityZ: Double,
        val innovation: DoubleArray,
        val innovationNorm: Double,
        val nis: Double,
        val reason: String?
    )

    init {

        require(
            lateralStdMps.isFinite() &&
                    lateralStdMps > 0.0
        ) {
            "lateralStdMps must be finite and > 0"
        }

        require(
            verticalStdMps.isFinite() &&
                    verticalStdMps > 0.0
        ) {
            "verticalStdMps must be finite and > 0"
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
     * Applies one NHC measurement update.
     *
     * The measurement is:
     *
     *     z = [0, 0]
     *
     * Predicted measurement:
     *
     *     h(x) = [
     *         bodyVelocityY,
     *         bodyVelocityZ
     *     ]
     *
     * Therefore:
     *
     *     innovation =
     *         [ -bodyVelocityY,
     *           -bodyVelocityZ ]
     */
    fun update(
        state: NavigationState,
        covariance: EskfCovariance
    ): UpdateResult {

        /*
         * ------------------------------------------------------------
         * 1. Validate state
         * ------------------------------------------------------------
         */

        if (!state.position.isFinite()) {
            return rejected(
                "State position is non-finite"
            )
        }

        if (!state.velocity.isFinite()) {
            return rejected(
                "State velocity is non-finite"
            )
        }

        if (!state.attitude.isFinite()) {
            return rejected(
                "State attitude is non-finite"
            )
        }

        if (!state.gyroBias.isFinite()) {
            return rejected(
                "Gyro bias is non-finite"
            )
        }

        if (!state.accelBias.isFinite()) {
            return rejected(
                "Accelerometer bias is non-finite"
            )
        }

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
         * 2. Navigation velocity -> body velocity
         * ------------------------------------------------------------
         *
         * Navigation state stores:
         *
         *     v_n
         *
         * Quaternion stores:
         *
         *     body -> navigation
         *
         * Therefore:
         *
         *     v_b = R_bn^T * v_n
         *
         * The quaternion conjugate performs navigation -> body.
         */

        val normalizedAttitude =
            state.attitude.normalized()

        val bodyVelocity =
            normalizedAttitude
                .conjugate()
                .rotate(state.velocity)

        if (!bodyVelocity.isFinite()) {
            return rejected(
                "Body velocity is non-finite"
            )
        }

        /*
         * ------------------------------------------------------------
         * 3. NHC measurement
         * ------------------------------------------------------------
         *
         * We do NOT constrain forward velocity.
         *
         * Only:
         *
         *     body Y -> lateral
         *     body Z -> vertical
         */

        val measurement =
            doubleArrayOf(
                0.0,
                0.0
            )

        val predictedMeasurement =
            doubleArrayOf(
                bodyVelocity.y,
                bodyVelocity.z
            )

        /*
         * ------------------------------------------------------------
         * 4. Rotation matrix
         * ------------------------------------------------------------
         *
         * Quaternion.toRotationMatrix() returns:
         *
         *     R_bn
         *
         * which transforms:
         *
         *     body -> navigation
         *
         * Therefore:
         *
         *     R_nb = R_bn^T
         */

        val rotationBodyToNavigation =
            normalizedAttitude.toRotationMatrix()

        val rotationNavigationToBody =
            transpose3x3(
                rotationBodyToNavigation
            )

        /*
         * ------------------------------------------------------------
         * 5. Attitude Jacobian
         * ------------------------------------------------------------
         *
         * Body velocity:
         *
         *     v_b = R_nb v_n
         *
         * Current ESKF attitude injection is:
         *
         *     q_new = q_old ⊗ δq
         *
         * therefore the attitude error is a body-frame
         * right-multiplicative error.
         *
         * Under this convention:
         *
         *     d(v_b)/d(delta_theta)
         *         = skew(v_b)
         *
         * where:
         *
         *       [  0  -vz   vy ]
         * S(v)= [  vz   0  -vx ]
         *       [ -vy   vx   0 ]
         *
         * We retain only body-Y and body-Z rows.
         */

        val velocitySkew =
            skewSymmetric(
                bodyVelocity
            )

        /*
         * ------------------------------------------------------------
         * 6. Build H
         * ------------------------------------------------------------
         *
         * Error state:
         *
         *   [ dp(3),
         *     dv(3),
         *     dtheta(3),
         *     dbg(3),
         *     dba(3) ]
         *
         * Therefore:
         *
         *   velocity starts at 3
         *   attitude starts at 6
         */

        val jacobian =
            Array(2) {
                DoubleArray(STATE_SIZE)
            }

        /*
         * Row 0 = lateral body velocity = body Y
         */
        val lateralBodyAxis = 1

        /*
         * Velocity derivative:
         *
         *     d(v_b)/d(v_n) = R_nb
         */

        for (column in 0 until 3) {

            jacobian[0][VEL_INDEX + column] =
                rotationNavigationToBody[
                    lateralBodyAxis
                ][column]
        }

        /*
         * Attitude derivative:
         *
         *     +skew(v_b)
         */

        for (column in 0 until 3) {

            jacobian[0][ATT_INDEX + column] =
                velocitySkew[
                    lateralBodyAxis
                ][column]
        }

        /*
         * Row 1 = vertical body velocity = body Z
         */

        val verticalBodyAxis = 2

        for (column in 0 until 3) {

            jacobian[1][VEL_INDEX + column] =
                rotationNavigationToBody[
                    verticalBodyAxis
                ][column]
        }

        for (column in 0 until 3) {

            jacobian[1][ATT_INDEX + column] =
                velocitySkew[
                    verticalBodyAxis
                ][column]
        }

        /*
         * ------------------------------------------------------------
         * 7. Measurement covariance R
         * ------------------------------------------------------------
         */

        val measurementCovariance =
            Array(2) {
                DoubleArray(2)
            }

        measurementCovariance[0][0] =
            lateralStdMps * lateralStdMps

        measurementCovariance[1][1] =
            verticalStdMps * verticalStdMps

        /*
         * ------------------------------------------------------------
         * 8. Generic ESKF measurement update
         * ------------------------------------------------------------
         */

        val measurementModel =
            EskfMeasurementUpdate.Measurement(
                measurement = measurement,
                predictedMeasurement =
                    predictedMeasurement,
                jacobian = jacobian,
                measurementCovariance =
                    measurementCovariance,
                nisGate = nisGate,
                name = "NHC"
            )

        /*
         * IMPORTANT:
         *
         * EskfMeasurementUpdate is a class.
         * update() is an instance method.
         */

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
         * 9. Convert generic result to NHC result
         * ------------------------------------------------------------
         */

        return UpdateResult(
            accepted = result.accepted,

            bodyVelocityX =
                bodyVelocity.x,

            bodyVelocityY =
                bodyVelocity.y,

            bodyVelocityZ =
                bodyVelocity.z,

            innovation =
                result.innovation.copyOf(),

            innovationNorm =
                result.innovationNorm,

            nis =
                result.nis,

            reason =
                result.reason
        )
    }

    /**
     * Returns transpose of a 3x3 matrix.
     */
    private fun transpose3x3(
        matrix: Array<DoubleArray>
    ): Array<DoubleArray> {

        return Array(3) { row ->
            DoubleArray(3) { column ->
                matrix[column][row]
            }
        }
    }

    /**
     * Returns the skew-symmetric matrix of a vector.
     *
     *        [  0  -z   y ]
     * S(v) = [  z   0  -x ]
     *        [ -y   x   0 ]
     */
    private fun skewSymmetric(
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

    /**
     * Creates a rejected NHC result.
     */
    private fun rejected(
        reason: String
    ): UpdateResult {

        return UpdateResult(
            accepted = false,

            bodyVelocityX =
                Double.NaN,

            bodyVelocityY =
                Double.NaN,

            bodyVelocityZ =
                Double.NaN,

            innovation =
                DoubleArray(0),

            innovationNorm =
                0.0,

            nis =
                Double.NaN,

            reason =
                reason
        )
    }
}