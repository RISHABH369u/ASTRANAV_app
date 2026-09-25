package com.rishabh.astranav.constraints
class NonHolonomicConstraint(private val gain: Double = 0.7) {
    fun correctLateralVelocity(lateralMps: Double): Double = lateralMps * (1.0 - gain)
}
