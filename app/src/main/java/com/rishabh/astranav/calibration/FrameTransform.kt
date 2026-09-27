package com.rishabh.astranav.calibration

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vec3(
    val x: Double,
    val y: Double,
    val z: Double
) {
    operator fun plus(other: Vec3) =
        Vec3(x + other.x, y + other.y, z + other.z)

    operator fun minus(other: Vec3) =
        Vec3(x - other.x, y - other.y, z - other.z)

    operator fun times(s: Double) =
        Vec3(x * s, y * s, z * s)

    fun dot(other: Vec3): Double =
        x * other.x + y * other.y + z * other.z

    fun norm(): Double =
        sqrt(x * x + y * y + z * z)

    fun normalized(): Vec3 {
        val n = norm()
        return if (n < 1e-9) {
            Vec3(0.0, 0.0, 1.0)
        } else {
            this * (1.0 / n)
        }
    }
}

data class Mat3(
    val m00: Double, val m01: Double, val m02: Double,
    val m10: Double, val m11: Double, val m12: Double,
    val m20: Double, val m21: Double, val m22: Double
) {

    operator fun times(v: Vec3): Vec3 {
        return Vec3(
            m00 * v.x + m01 * v.y + m02 * v.z,
            m10 * v.x + m11 * v.y + m12 * v.z,
            m20 * v.x + m21 * v.y + m22 * v.z
        )
    }

    operator fun times(other: Mat3): Mat3 {
        return Mat3(
            m00 * other.m00 + m01 * other.m10 + m02 * other.m20,
            m00 * other.m01 + m01 * other.m11 + m02 * other.m21,
            m00 * other.m02 + m01 * other.m12 + m02 * other.m22,

            m10 * other.m00 + m11 * other.m10 + m12 * other.m20,
            m10 * other.m01 + m11 * other.m11 + m12 * other.m21,
            m10 * other.m02 + m11 * other.m12 + m12 * other.m22,

            m20 * other.m00 + m21 * other.m10 + m22 * other.m20,
            m20 * other.m01 + m21 * other.m11 + m22 * other.m21,
            m20 * other.m02 + m21 * other.m12 + m22 * other.m22
        )
    }

    fun transpose(): Mat3 {
        return Mat3(
            m00, m10, m20,
            m01, m11, m21,
            m02, m12, m22
        )
    }

    companion object {

        fun identity() = Mat3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        )

        fun yaw(rad: Double): Mat3 {
            val c = cos(rad)
            val s = sin(rad)

            return Mat3(
                c, -s, 0.0,
                s, c, 0.0,
                0.0, 0.0, 1.0
            )
        }
    }
}

object FrameTransform {

    /**
     * Builds rotation that aligns vector `from` to vector `to`.
     *
     * Used for gravity leveling.
     */
    fun rotationBetween(from: Vec3, to: Vec3): Mat3 {

        val a = from.normalized()
        val b = to.normalized()

        val vx = a.y * b.z - a.z * b.y
        val vy = a.z * b.x - a.x * b.z
        val vz = a.x * b.y - a.y * b.x

        val s = sqrt(vx * vx + vy * vy + vz * vz)
        val c = a.dot(b)

        if (s < 1e-8) {

            // Same direction
            if (c > 0.0) {
                return Mat3.identity()
            }

            // Opposite direction.
            // 180° rotation around X.
            return Mat3(
                1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0
            )
        }

        val k = (1.0 - c) / (s * s)

        val vx2 = vx * vx
        val vy2 = vy * vy
        val vz2 = vz * vz

        return Mat3(
            1.0 + k * (-vy2 - vz2),
            -vz + k * vx * vy,
            vy + k * vx * vz,

            vz + k * vx * vy,
            1.0 + k * (-vx2 - vz2),
            -vx + k * vy * vz,

            -vy + k * vx * vz,
            vx + k * vy * vz,
            1.0 + k * (-vx2 - vy2)
        )
    }

    /**
     * Rotate vector around vehicle vertical axis.
     */
    fun yaw(v: Vec3, yawRad: Double): Vec3 {
        return Mat3.yaw(yawRad) * v
    }

    /**
     * Complete Device -> Vehicle transformation.
     */
    fun deviceToVehicle(
        deviceVector: Vec3,
        leveling: Mat3,
        mountYawRad: Double
    ): Vec3 {

        val levelled = leveling * deviceVector

        return Mat3.yaw(mountYawRad) * levelled
    }
}