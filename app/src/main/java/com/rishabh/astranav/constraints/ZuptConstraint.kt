package com.rishabh.astranav.constraints
class ZuptConstraint {
    fun active(speedMps: Double, gyroMagnitudeRadSec: Double, accelVariance: Double): Boolean =
        speedMps < 0.25 && gyroMagnitudeRadSec < 0.08 && accelVariance < 0.08
}
