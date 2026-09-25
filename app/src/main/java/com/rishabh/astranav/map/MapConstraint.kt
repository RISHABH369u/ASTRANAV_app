package com.rishabh.astranav.map
data class MapCorrection(val eastM:Double,val northM:Double,val sigmaM:Double)
class MapConstraint {
    fun correction(distanceAlongNormalM:Double,confidence:Double):MapCorrection {
        val c=confidence.coerceIn(0.05,1.0)
        return MapCorrection(0.0,-distanceAlongNormalM*c,(20.0/c).coerceAtMost(100.0))
    }
}
