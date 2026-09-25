package com.rishabh.astranav.constraints
class TurningConservatism {
    fun weight(yawRateRadSec: Double): Double {
        val a = kotlin.math.abs(yawRateRadSec)
        return when { a < 0.10 -> 1.0; a > 0.35 -> 0.5; else -> 1.0 - (a - 0.10) / 0.50 }
    }
}
