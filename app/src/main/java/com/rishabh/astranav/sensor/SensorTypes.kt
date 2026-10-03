package com.rishabh.astranav.sensor

/**
 * Where an IMU measurement originated.
 *
 * REPLAY is intentionally included so IO-VNBD can travel through the
 * exact same adapter pipeline as a real external sensor.
 */
enum class SensorSourceType {
    SMARTPHONE,
    BLUETOOTH_IMU,
    USB_IMU,
    CAN_BRIDGE,
    REPLAY
}

/**
 * Quality of the output sample after normalization/resampling.
 */
enum class SampleQuality {
    VALID,
    RESAMPLED,
    GAP_RECOVERY,
    DEGRADED
}

/**
 * Raw/common IMU representation used throughout ASTRANAV.
 *
 * Accelerometer:
 *   m/s²
 *
 * Gyroscope:
 *   rad/s
 *
 * Timestamp:
 *   Android/monotonic nanoseconds.
 */
data class ImuSample(
    val timestampNanos: Long,

    val accelX: Double,
    val accelY: Double,
    val accelZ: Double,

    val gyroX: Double,
    val gyroY: Double,
    val gyroZ: Double,

    val gravityX: Double = 0.0,
    val gravityY: Double = 0.0,
    val gravityZ: Double = 9.81
)

/**
 * Output of SensorAdapter.
 *
 * The contained ImuSample is always expressed in the common ASTRANAV
 * sensor convention. It has NOT yet passed through DVFC.
 */
data class NormalizedImuSample(
    val imu: ImuSample,

    val source: SensorSourceType,

    /**
     * Timestamp of the fixed-rate output sample.
     */
    val outputTimestampNanos: Long,

    /**
     * Timestamp of the most recent actual source sample used.
     */
    val sourceTimestampNanos: Long,

    /**
     * Original source interval that surrounded this output sample.
     */
    val sourceIntervalNanos: Long,

    val quality: SampleQuality,

    /**
     * True when the output was generated at a fixed 10 Hz timestamp
     * rather than being passed through directly.
     */
    val isResampled: Boolean,

    /**
     * Size of the detected source gap, if applicable.
     */
    val gapDurationNanos: Long = 0L
)

/**
 * Runtime health information from SensorAdapter.
 */
data class SensorAdapterStats(
    val inputSamples: Long = 0L,
    val outputSamples: Long = 0L,
    val rejectedSamples: Long = 0L,

    val interpolatedSamples: Long = 0L,
    val gapCount: Long = 0L,

    val totalGapNanos: Long = 0L,
    val maxGapNanos: Long = 0L,

    val lastInputTimestampNanos: Long? = null,
    val lastOutputTimestampNanos: Long? = null,

    val sourceRateHz: Double? = null,
    val outputRateHz: Double? = null
)

data class GnssSample(
    val timestampMillis: Long,
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val altitudeM: Double = 0.0,
    val speedMps: Double = 0.0,
    val bearingDeg: Double = 0.0,
    val accuracyM: Double = 99.0
)

data class SensorFrame(
    val imu: ImuSample,
    val gnss: GnssSample? = null
)