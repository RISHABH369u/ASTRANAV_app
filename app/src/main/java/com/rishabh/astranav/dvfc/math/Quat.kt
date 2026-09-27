package com.rishabh.astranav.dvfc.math

import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Minimal, dependency-free quaternion (Hamilton convention, x,y,z,w).
 *
 * Deliberately NOT dev.romainguy.kotlin.math.Quaternion (SceneView/Filament's
 * math type) — the actual calibration math (spec §2, §4, §5, §8) needs to
 * stay correct and unit-testable independent of whichever SceneView version
 * is on the classpath. Only DvfcSceneRenderer converts a Quat to the
 * renderer's own quaternion type, at the very last step before handing it
 * to the 3D node.
 */
data class Quat(val x: Float, val y: Float, val z: Float, val w: Float) {

    /** Hamilton product: applying `this` then `o` (this ∘ o), matches q_a * q_b composition order used throughout this package. */
    operator fun times(o: Quat): Quat = Quat(
        x = w * o.x + x * o.w + y * o.z - z * o.y,
        y = w * o.y - x * o.z + y * o.w + z * o.x,
        z = w * o.z + x * o.y - y * o.x + z * o.w,
        w = w * o.w - x * o.x - y * o.y - z * o.z,
    )

    fun conjugate() = Quat(-x, -y, -z, w)

    fun norm() = sqrt(x * x + y * y + z * z + w * w)

    fun normalized(): Quat {
        val n = norm()
        return if (n < 1e-8f) IDENTITY else Quat(x / n, y / n, z / n, w / n)
    }

    /** For a unit quaternion, inverse == conjugate; this handles the non-unit case too. */
    fun inverse(): Quat {
        val n2 = x * x + y * y + z * z + w * w
        return if (n2 < 1e-8f) IDENTITY else Quat(-x / n2, -y / n2, -z / n2, w / n2)
    }

    /** Rotates a vector by this quaternion: v' = q * v * q⁻¹ (spec §8 — a_vehicle = R × a_device). */
    fun rotate(v: FloatArray): FloatArray {
        val qv = Quat(v[0], v[1], v[2], 0f)
        val r = (this * qv) * conjugate()
        return floatArrayOf(r.x, r.y, r.z)
    }

    /** [roll, pitch, yaw] in degrees — standard aerospace (ZYX intrinsic) convention, yaw about the vertical (Z/Up) axis. */
    fun toEulerDegrees(): FloatArray {
        val sinrCosp = 2 * (w * x + y * z)
        val cosrCosp = 1 - 2 * (x * x + y * y)
        val roll = atan2(sinrCosp, cosrCosp)

        val sinp = 2 * (w * y - z * x)
        val pitch = if (abs(sinp) >= 1f) (HALF_PI * sign(sinp)) else asin(sinp)

        val sinyCosp = 2 * (w * z + x * y)
        val cosyCosp = 1 - 2 * (y * y + z * z)
        val yaw = atan2(sinyCosp, cosyCosp)

        return floatArrayOf(
            Math.toDegrees(roll.toDouble()).toFloat(),
            Math.toDegrees(pitch.toDouble()).toFloat(),
            Math.toDegrees(yaw.toDouble()).toFloat(),
        )
    }

    companion object {
        val IDENTITY = Quat(0f, 0f, 0f, 1f)
        private const val HALF_PI = (Math.PI / 2).toFloat()

        /** Right-hand rotation of `degrees` about the given (should be unit-length) axis. */
        fun fromAxisAngleDegrees(axisX: Float, axisY: Float, axisZ: Float, degrees: Float): Quat {
            val rad = Math.toRadians(degrees.toDouble()).toFloat()
            val half = rad / 2f
            val s = sin(half)
            return Quat(axisX * s, axisY * s, axisZ * s, cos(half)).normalized()
        }
    }
}
