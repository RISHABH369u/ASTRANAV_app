package com.rishabh.astranav.ml
object MlUncertainty {
    fun varianceFromConfidence(confidence: Double, floor: Double=0.25): Double {
        val c=confidence.coerceIn(0.01,1.0)
        return floor/(c*c)
    }
}
