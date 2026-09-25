package com.rishabh.astranav.integrity
data class IntegritySnapshot(val scorePercent: Int, val ml: Double, val physics: Double, val map: Double, val gnss: Double)
class IntegrityMonitor {
    fun evaluate(ml: Double, physics: Double, map: Double, gnss: Double): IntegritySnapshot {
        val score = (0.35*ml + 0.25*physics + 0.20*map + 0.20*gnss).coerceIn(0.0,1.0)
        return IntegritySnapshot((score*100).toInt(),ml,physics,map,gnss)
    }
}
