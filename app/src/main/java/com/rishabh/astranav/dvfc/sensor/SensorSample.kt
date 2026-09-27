package com.rishabh.astranav.dvfc.sensor

import com.rishabh.astranav.dvfc.math.Quat

data class DeviceSensorSample(
    val timestampNs: Long,

    // Raw/normalized IMU
    val acceleration: FloatArray,
    val angularVelocity: FloatArray,
    val gravity: FloatArray,

    // Gravity-compensated acceleration
    val linearAcceleration: FloatArray,

    // Orientation
    val quaternion: Quat,

    // Gravity diagnostics
    val gravityMagnitude: Float,
    val gravityStable: Boolean,
    val gravityLevelRollDeg: Float,
    val gravityLevelPitchDeg: Float,

    // Adapter diagnostics
    val estimatedSampleHz: Float,
    val timestampJitterMs: Float,
    val dataGapCount: Int,
    val maxGapMs: Float,
    val duplicateTimestampCount: Int,

    // Processing state
    val resamplingActive: Boolean,
    val resamplingRateHz: Float,

    // Sensor availability
    val accelerationAvailable: Boolean,
    val gyroscopeAvailable: Boolean,
    val gravityAvailable: Boolean,
    val rotationVectorAvailable: Boolean,

    val rotationAccuracy: Int,




    val adapterResamplingActive: Boolean = false,
    val adapterResamplingRateHz: Float = 0f,

    val adapterEstimatedSampleHz: Float = 0f,
    val adapterTimestampJitterMs: Float = 0f,
    val adapterDataGapCount: Int = 0,
    val adapterMaxGapMs: Float = 0f,
    val adapterDuplicateTimestampCount: Int = 0,


)