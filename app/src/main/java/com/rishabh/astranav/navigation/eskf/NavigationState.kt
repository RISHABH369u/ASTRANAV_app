package com.rishabh.astranav.navigation.eskf

/**
 * Nominal navigation state used by ASTRA-Core ESKF.
 *
 * Coordinate convention:
 *
 * Navigation frame = NED
 *   X = North
 *   Y = East
 *   Z = Down
 *
 * Body frame = FRD
 *   X = Forward
 *   Y = Right
 *   Z = Down
 *
 * Nominal state:
 *
 *   position
 *   velocity
 *   attitude
 *   gyro bias
 *   accelerometer bias
 *
 * ESKF error state:
 *
 *   [ dp(3),
 *     dv(3),
 *     dtheta(3),
 *     dbg(3),
 *     dba(3) ] = 15 states
 */
data class NavigationState(

    /**
     * Position in navigation frame, meters.
     */
    var position: Vec3 = Vec3.ZERO,

    /**
     * Velocity in navigation frame, m/s.
     */
    var velocity: Vec3 = Vec3.ZERO,

    /**
     * Body -> navigation attitude quaternion.
     */
    var attitude: Quaternion = Quaternion.IDENTITY,

    /**
     * Gyroscope bias, rad/s.
     */
    var gyroBias: Vec3 = Vec3.ZERO,

    /**
     * Accelerometer bias, m/s².
     */
    var accelBias: Vec3 = Vec3.ZERO,

    /**
     * State timestamp, nanoseconds.
     */
    var timestampNanos: Long = 0L
) {

    /**
     * Creates a deep copy of this state.
     */
    fun copyState(): NavigationState {
        return NavigationState(
            position = position.copy(),
            velocity = velocity.copy(),
            attitude = attitude.copy(),
            gyroBias = gyroBias.copy(),
            accelBias = accelBias.copy(),
            timestampNanos = timestampNanos
        )
    }

    /**
     * Resets the state to the nominal initial state.
     */
    fun reset() {
        position = Vec3.ZERO
        velocity = Vec3.ZERO
        attitude = Quaternion.IDENTITY
        gyroBias = Vec3.ZERO
        accelBias = Vec3.ZERO
        timestampNanos = 0L
    }
}


/**
 * Simple 3D vector used by the navigation core.
 *
 * This class deliberately has no Android/UI dependency.
 */
data class Vec3(
    val x: Double,
    val y: Double,
    val z: Double
) {

    operator fun plus(other: Vec3): Vec3 {
        return Vec3(
            x + other.x,
            y + other.y,
            z + other.z
        )
    }

    operator fun minus(other: Vec3): Vec3 {
        return Vec3(
            x - other.x,
            y - other.y,
            z - other.z
        )
    }

    operator fun unaryMinus(): Vec3 {
        return Vec3(
            -x,
            -y,
            -z
        )
    }

    operator fun times(scale: Double): Vec3 {
        return Vec3(
            x * scale,
            y * scale,
            z * scale
        )
    }

    operator fun div(scale: Double): Vec3 {
        require(scale != 0.0) {
            "Cannot divide Vec3 by zero"
        }

        return Vec3(
            x / scale,
            y / scale,
            z / scale
        )
    }

    fun dot(other: Vec3): Double {
        return x * other.x +
                y * other.y +
                z * other.z
    }

    fun cross(other: Vec3): Vec3 {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        )
    }

    fun squaredNorm(): Double {
        return x * x +
                y * y +
                z * z
    }

    fun norm(): Double {
        return kotlin.math.sqrt(squaredNorm())
    }

    fun normalized(
        epsilon: Double = 1e-12
    ): Vec3 {

        val magnitude = norm()

        if (magnitude < epsilon) {
            return ZERO
        }

        return this / magnitude
    }

    fun isFinite(): Boolean {
        return x.isFinite() &&
                y.isFinite() &&
                z.isFinite()
    }

    fun maxAbsComponent(): Double {
        return maxOf(
            kotlin.math.abs(x),
            kotlin.math.abs(y),
            kotlin.math.abs(z)
        )
    }

    companion object {

        val ZERO = Vec3(
            x = 0.0,
            y = 0.0,
            z = 0.0
        )

        val UNIT_X = Vec3(
            x = 1.0,
            y = 0.0,
            z = 0.0
        )

        val UNIT_Y = Vec3(
            x = 0.0,
            y = 1.0,
            z = 0.0
        )

        val UNIT_Z = Vec3(
            x = 0.0,
            y = 0.0,
            z = 1.0
        )
    }
}


/**
 * Quaternion representing body -> navigation rotation.
 *
 * Convention:
 *
 *     q = w + xi + yj + zk
 *
 * Quaternion multiplication follows the Hamilton convention.
 */
data class Quaternion(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double
) {

    /**
     * Quaternion multiplication.
     */
    operator fun times(
        other: Quaternion
    ): Quaternion {

        return Quaternion(
            w = w * other.w -
                    x * other.x -
                    y * other.y -
                    z * other.z,

            x = w * other.x +
                    x * other.w +
                    y * other.z -
                    z * other.y,

            y = w * other.y -
                    x * other.z +
                    y * other.w +
                    z * other.x,

            z = w * other.z +
                    x * other.y -
                    y * other.x +
                    z * other.w
        )
    }

    /**
     * Quaternion conjugate.
     */
    fun conjugate(): Quaternion {

        return Quaternion(
            w = w,
            x = -x,
            y = -y,
            z = -z
        )
    }

    /**
     * Squared quaternion norm.
     */
    fun squaredNorm(): Double {

        return w * w +
                x * x +
                y * y +
                z * z
    }

    /**
     * Quaternion norm.
     */
    fun norm(): Double {
        return kotlin.math.sqrt(squaredNorm())
    }

    /**
     * Normalized quaternion.
     */
    fun normalized(
        epsilon: Double = 1e-12
    ): Quaternion {

        val magnitude = norm()

        if (magnitude < epsilon) {
            return IDENTITY
        }

        return Quaternion(
            w = w / magnitude,
            x = x / magnitude,
            y = y / magnitude,
            z = z / magnitude
        )
    }

    /**
     * Rotates a vector from body frame to navigation frame.
     *
     *     v_n = q * v_b * q*
     */
    fun rotate(
        vector: Vec3
    ): Vec3 {

        val normalizedQuaternion =
            normalized()

        val qVector = Quaternion(
            w = 0.0,
            x = vector.x,
            y = vector.y,
            z = vector.z
        )

        val rotated =
            normalizedQuaternion *
                    qVector *
                    normalizedQuaternion.conjugate()

        return Vec3(
            x = rotated.x,
            y = rotated.y,
            z = rotated.z
        )
    }

    /**
     * Converts quaternion into a 3x3 rotation matrix.
     *
     * Returned matrix is row-major.
     *
     * The matrix rotates body-frame vectors into
     * navigation-frame vectors.
     */
    fun toRotationMatrix(): Array<DoubleArray> {

        val q = normalized()

        val ww = q.w * q.w
        val xx = q.x * q.x
        val yy = q.y * q.y
        val zz = q.z * q.z

        val wx = q.w * q.x
        val wy = q.w * q.y
        val wz = q.w * q.z

        val xy = q.x * q.y
        val xz = q.x * q.z
        val yz = q.y * q.z

        return arrayOf(

            doubleArrayOf(
                ww + xx - yy - zz,
                2.0 * (xy - wz),
                2.0 * (xz + wy)
            ),

            doubleArrayOf(
                2.0 * (xy + wz),
                ww - xx + yy - zz,
                2.0 * (yz - wx)
            ),

            doubleArrayOf(
                2.0 * (xz - wy),
                2.0 * (yz + wx),
                ww - xx - yy + zz
            )
        )
    }

    /**
     * Checks whether all quaternion components are finite.
     */
    fun isFinite(): Boolean {

        return w.isFinite() &&
                x.isFinite() &&
                y.isFinite() &&
                z.isFinite()
    }

    companion object {

        val IDENTITY = Quaternion(
            w = 1.0,
            x = 0.0,
            y = 0.0,
            z = 0.0
        )

        /**
         * Creates a quaternion from a rotation vector.
         *
         * rotationVector = axis * angle
         *
         * angle is in radians.
         *
         * For very small angles:
         *
         *     q ≈ [1, 0.5*dtheta]
         */
        fun fromRotationVector(
            rotationVector: Vec3
        ): Quaternion {

            val angle =
                rotationVector.norm()

            if (angle < 1e-12) {

                return Quaternion(
                    w = 1.0,
                    x = 0.5 * rotationVector.x,
                    y = 0.5 * rotationVector.y,
                    z = 0.5 * rotationVector.z
                ).normalized()
            }

            val halfAngle =
                0.5 * angle

            val sinHalf =
                kotlin.math.sin(halfAngle)

            val axis =
                rotationVector / angle

            return Quaternion(
                w = kotlin.math.cos(halfAngle),
                x = axis.x * sinHalf,
                y = axis.y * sinHalf,
                z = axis.z * sinHalf
            ).normalized()
        }
    }
}