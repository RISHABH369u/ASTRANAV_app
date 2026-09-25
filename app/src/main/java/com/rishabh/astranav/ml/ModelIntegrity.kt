package com.rishabh.astranav.ml
class ModelIntegrity {
    fun validSpeed(speedMps: Double): Boolean = speedMps.isFinite() && speedMps in 0.0..70.0
    fun confidence(variance: Double): Double = if(!variance.isFinite() || variance<=0.0) 0.0 else (1.0/(1.0+variance)).coerceIn(0.0,1.0)
}
