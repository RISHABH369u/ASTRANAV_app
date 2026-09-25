package com.rishabh.astranav.sensor

data class ImuSample(
    val timestampNanos: Long,
    val accelX: Double, val accelY: Double, val accelZ: Double,
    val gyroX: Double, val gyroY: Double, val gyroZ: Double,
    val gravityX: Double = 0.0, val gravityY: Double = 0.0, val gravityZ: Double = 9.81
)

data class GnssSample(
    val timestampMillis: Long,
    val latitudeDeg: Double, val longitudeDeg: Double,
    val altitudeM: Double = 0.0,
    val speedMps: Double = 0.0,
    val bearingDeg: Double = 0.0,
    val accuracyM: Double = 99.0
)

data class SensorFrame(
    val imu: ImuSample,
    val gnss: GnssSample? = null
)
