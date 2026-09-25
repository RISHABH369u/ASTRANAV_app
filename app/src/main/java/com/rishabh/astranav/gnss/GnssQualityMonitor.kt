package com.rishabh.astranav.gnss
data class GnssQuality(val available:Boolean,val accuracyM:Double,val score:Double)
class GnssQualityMonitor {
    fun assess(accuracyM:Double):GnssQuality {
        val ok=accuracyM.isFinite() && accuracyM<100.0
        val score=if(!ok)0.0 else (1.0-accuracyM/50.0).coerceIn(0.0,1.0)
        return GnssQuality(ok,accuracyM,score)
    }
}
