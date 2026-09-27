package com.rishabh.astranav.dvfc.sensor

import com.rishabh.astranav.dvfc.math.Quat

/**
 * One synchronized, validated sensor sample produced by SensorAdapter.
 *
 * All values are already normalized into the units expected by the
 * downstream DVFC / ESKF pipeline.
 */
data class DeviceSensorSample(

    // ---------------------------------------------------------
    // TIMESTAMP
    // ---------------------------------------------------------

    val timestampNs: Long,

    // ---------------------------------------------------------
    // IMU
    // ---------------------------------------------------------

    /** Linear acceleration in PHONE frame, m/s². */
    val acceleration: FloatArray,

    /** Angular velocity in PHONE frame, rad/s. */
    val angularVelocity: FloatArray,

    /** Gravity vector in PHONE frame, m/s². */
    val gravity: FloatArray,

    // ---------------------------------------------------------
    // ORIENTATION
    // ---------------------------------------------------------

    /** Rotation-vector orientation converted to quaternion. */
    val quaternion: Quat,

    // ---------------------------------------------------------
    // SENSOR STATUS
    // ---------------------------------------------------------

    val rotationAccuracy: Int,

    val accelerationAvailable: Boolean,

    val gyroscopeAvailable: Boolean,

    val gravityAvailable: Boolean,

    val rotationVectorAvailable: Boolean,

    // ---------------------------------------------------------
    // ADAPTER DIAGNOSTICS
    // ---------------------------------------------------------

    val estimatedSampleHz: Float,

    val timestampJitterMs: Float,

    val dataGapCount: Int,

    val maxGapMs: Float,

    val gravityMagnitude: Float,

    val gravityStable: Boolean,
)