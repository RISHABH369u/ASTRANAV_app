package com.rishabh.astranav.constraints
class ZaruConstraint {
    fun active(gyroMagnitudeRadSec: Double, stationary: Boolean): Boolean =
        stationary && gyroMagnitudeRadSec < 0.08
}
