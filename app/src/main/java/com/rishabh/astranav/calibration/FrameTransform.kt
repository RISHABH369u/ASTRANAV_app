package com.rishabh.astranav.calibration
import kotlin.math.cos
import kotlin.math.sin
data class Vec3(val x: Double,val y: Double,val z: Double)
object FrameTransform {
    fun yaw(v: Vec3, yawRad: Double): Vec3 {
        val c=cos(yawRad); val s=sin(yawRad)
        return Vec3(c*v.x-s*v.y,s*v.x+c*v.y,v.z)
    }
}
