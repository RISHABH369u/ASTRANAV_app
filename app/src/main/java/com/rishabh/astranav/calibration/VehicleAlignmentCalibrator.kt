package com.rishabh.astranav.calibration
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
data class VehicleAlignment(val yawOffsetDeg: Double=0.0,val confidence: Double=0.0)
class VehicleAlignmentCalibrator {
    private var s=0.0; private var c=0.0; private var n=0
    fun add(phoneYawDeg: Double, courseDeg: Double, speedMps: Double, accuracyM: Double) {
        if(speedMps<2.0 || accuracyM>15.0) return
        val d=Math.toRadians(((courseDeg-phoneYawDeg+540.0)%360.0)-180.0)
        s+=sin(d); c+=cos(d); n++
    }
    fun result(): VehicleAlignment {
        if(n==0) return VehicleAlignment()
        return VehicleAlignment(Math.toDegrees(atan2(s,c)),(kotlin.math.sqrt(s*s+c*c)/n).coerceIn(0.0,1.0))
    }
}
