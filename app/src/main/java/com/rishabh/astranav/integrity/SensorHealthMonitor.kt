package com.rishabh.astranav.integrity
data class SensorHealth(val imu: Boolean, val samplingHz: Double, val gravity: Boolean)
class SensorHealthMonitor {
    fun assess(samplingHz: Double, gravityMagnitude: Double): SensorHealth =
        SensorHealth(samplingHz > 5.0 && samplingHz < 500.0, samplingHz, kotlin.math.abs(gravityMagnitude-9.81) < 1.0)
}
