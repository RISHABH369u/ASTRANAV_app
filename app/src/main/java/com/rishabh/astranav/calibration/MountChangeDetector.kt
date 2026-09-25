package com.rishabh.astranav.calibration
class MountChangeDetector(private val thresholdDeg: Double=15.0) {
    private var reference: Double?=null
    fun update(yawDeg: Double, stationary: Boolean): Boolean {
        if(!stationary) return false
        if(reference==null){ reference=yawDeg; return false }
        val d=kotlin.math.abs(((yawDeg-reference!!+540.0)%360.0)-180.0)
        return d>thresholdDeg
    }
}
