package com.rishabh.astranav.sensor

import kotlin.math.ceil

/**
 * ASTRANAV fixed-rate sensor adapter.
 *
 * Responsibilities:
 *  - validate timestamps and finite sensor values
 *  - convert arbitrary source rates into a fixed 10 Hz timeline
 *  - interpolate small timing gaps
 *  - explicitly report large gaps
 *  - preserve source/quality metadata
 *
 * This class deliberately knows NOTHING about:
 *  - Bluetooth
 *  - USB
 *  - CAN
 *  - Android SensorManager
 *  - DVFC
 *  - ESKF
 *  - ML
 *
 * That separation is important.
 */
class SensorAdapter(
    private val targetHz: Double = DEFAULT_TARGET_HZ,
    private val maxInterpolationGapMs: Long = DEFAULT_MAX_INTERPOLATION_GAP_MS
) {

    companion object {
        const val DEFAULT_TARGET_HZ = 10.0

        /**
         * We allow interpolation across small timing gaps.
         *
         * Anything larger is considered a real source dropout and is NOT
         * fabricated into a long sequence of synthetic IMU samples.
         */
        const val DEFAULT_MAX_INTERPOLATION_GAP_MS = 300L
    }

    private val periodNanos: Long =
        (1_000_000_000.0 / targetHz).toLong()

    private val maxInterpolationGapNanos: Long =
        maxInterpolationGapMs * 1_000_000L

    private var previousSample: ImuSample? = null

    /**
     * Timestamp at which the next fixed-rate output sample is expected.
     */
    private var nextOutputTimestampNanos: Long? = null

    private var inputSamples = 0L
    private var outputSamples = 0L
    private var rejectedSamples = 0L
    private var interpolatedSamples = 0L
    private var gapCount = 0L

    private var totalGapNanos = 0L
    private var maxGapNanos = 0L

    private var lastInputTimestampNanos: Long? = null
    private var lastOutputTimestampNanos: Long? = null

    /**
     * Feed one source sample.
     *
     * A single input sample may generate:
     *  - zero output samples
     *  - one output sample
     *
     * We intentionally do not generate an arbitrary number of synthetic
     * samples after a large dropout.
     */
    @Synchronized
    fun feed(
        sample: ImuSample,
        source: SensorSourceType = SensorSourceType.SMARTPHONE
    ): List<NormalizedImuSample> {

        inputSamples++

        if (!isValid(sample)) {
            rejectedSamples++
            return emptyList()
        }

        val previous = previousSample

        if (previous == null) {
            previousSample = sample
            lastInputTimestampNanos = sample.timestampNanos

            /*
             * Start the fixed-rate timeline immediately from the first
             * valid measurement.
             */
            nextOutputTimestampNanos = sample.timestampNanos

            return emitDirectFirstSample(sample, source)
        }

        val dtNanos =
            sample.timestampNanos - previous.timestampNanos

        if (dtNanos <= 0L) {
            rejectedSamples++
            return emptyList()
        }

        lastInputTimestampNanos = sample.timestampNanos

        val outputs = mutableListOf<NormalizedImuSample>()

        /*
         * NORMAL / SMALL-GAP PATH
         *
         * Interpolate onto the fixed 10 Hz timeline.
         */
        if (dtNanos <= maxInterpolationGapNanos) {

            var outputTimestamp = nextOutputTimestampNanos
                ?: (previous.timestampNanos + periodNanos)

            while (outputTimestamp <= sample.timestampNanos) {

                val alpha =
                    if (dtNanos > 0L) {
                        (
                                outputTimestamp - previous.timestampNanos
                                ).toDouble() / dtNanos.toDouble()
                    } else {
                        1.0
                    }

                val clampedAlpha =
                    alpha.coerceIn(0.0, 1.0)

                val interpolated =
                    interpolate(
                        previous = previous,
                        current = sample,
                        timestampNanos = outputTimestamp,
                        alpha = clampedAlpha
                    )

                val isExactSourceSample =
                    outputTimestamp == sample.timestampNanos

                val quality =
                    if (isExactSourceSample) {
                        SampleQuality.VALID
                    } else {
                        interpolatedSamples++
                        SampleQuality.RESAMPLED
                    }

                outputs += NormalizedImuSample(
                    imu = interpolated,
                    source = source,
                    outputTimestampNanos = outputTimestamp,
                    sourceTimestampNanos = sample.timestampNanos,
                    sourceIntervalNanos = dtNanos,
                    quality = quality,
                    isResampled = !isExactSourceSample
                )

                outputSamples++
                lastOutputTimestampNanos = outputTimestamp

                outputTimestamp += periodNanos
            }

            nextOutputTimestampNanos = outputTimestamp

        } else {

            /*
             * LARGE-GAP PATH
             *
             * Do NOT generate hundreds of fake inertial measurements.
             *
             * We resume from the newest real sample and explicitly mark
             * the first resumed output as GAP_RECOVERY.
             */
            gapCount++

            totalGapNanos += dtNanos
            maxGapNanos = maxOf(maxGapNanos, dtNanos)

            val recoverySample = NormalizedImuSample(
                imu = sample,
                source = source,
                outputTimestampNanos = sample.timestampNanos,
                sourceTimestampNanos = sample.timestampNanos,
                sourceIntervalNanos = dtNanos,
                quality = SampleQuality.GAP_RECOVERY,
                isResampled = false,
                gapDurationNanos = dtNanos
            )

            outputs += recoverySample

            outputSamples++
            lastOutputTimestampNanos = sample.timestampNanos

            /*
             * Restart the fixed-rate grid after the actual recovery sample.
             */
            nextOutputTimestampNanos =
                sample.timestampNanos + periodNanos
        }

        previousSample = sample

        return outputs
    }

    /**
     * Compatibility method retained for existing code.
     *
     * Returns the actual source delta in seconds.
     */
    @Synchronized
    fun dtSeconds(sample: ImuSample): Double? {

        if (!isValid(sample)) {
            rejectedSamples++
            return null
        }

        val previous = previousSample

        if (previous == null) {
            previousSample = sample
            lastInputTimestampNanos = sample.timestampNanos
            return null
        }

        val dtNanos =
            sample.timestampNanos - previous.timestampNanos

        if (dtNanos <= 0L) {
            rejectedSamples++
            return null
        }

        previousSample = sample
        lastInputTimestampNanos = sample.timestampNanos

        return dtNanos.toDouble() * 1e-9
    }

    /**
     * Compatibility method retained for existing code.
     */
    fun validate(sample: ImuSample): Boolean {
        return isValid(sample)
    }

    /**
     * Reset the adapter when:
     *  - changing sensor source
     *  - starting a new trip
     *  - seeking in replay
     *  - reconnecting an external sensor
     */
    @Synchronized
    fun reset() {
        previousSample = null
        nextOutputTimestampNanos = null

        inputSamples = 0L
        outputSamples = 0L
        rejectedSamples = 0L
        interpolatedSamples = 0L
        gapCount = 0L

        totalGapNanos = 0L
        maxGapNanos = 0L

        lastInputTimestampNanos = null
        lastOutputTimestampNanos = null
    }

    @Synchronized
    fun stats(): SensorAdapterStats {

        val sourceRateHz =
            calculateRateHz(
                inputSamples,
                previousSample?.timestampNanos,
                lastInputTimestampNanos
            )

        val outputRateHz =
            calculateRateHz(
                outputSamples,
                null,
                lastOutputTimestampNanos
            )

        return SensorAdapterStats(
            inputSamples = inputSamples,
            outputSamples = outputSamples,
            rejectedSamples = rejectedSamples,
            interpolatedSamples = interpolatedSamples,
            gapCount = gapCount,
            totalGapNanos = totalGapNanos,
            maxGapNanos = maxGapNanos,
            lastInputTimestampNanos = lastInputTimestampNanos,
            lastOutputTimestampNanos = lastOutputTimestampNanos,
            sourceRateHz = sourceRateHz,
            outputRateHz = outputRateHz
        )
    }

    private fun emitDirectFirstSample(
        sample: ImuSample,
        source: SensorSourceType
    ): List<NormalizedImuSample> {

        outputSamples++
        lastOutputTimestampNanos = sample.timestampNanos

        return listOf(
            NormalizedImuSample(
                imu = sample,
                source = source,
                outputTimestampNanos = sample.timestampNanos,
                sourceTimestampNanos = sample.timestampNanos,
                sourceIntervalNanos = 0L,
                quality = SampleQuality.VALID,
                isResampled = false
            )
        )
    }

    private fun interpolate(
        previous: ImuSample,
        current: ImuSample,
        timestampNanos: Long,
        alpha: Double
    ): ImuSample {

        fun lerp(a: Double, b: Double): Double =
            a + (b - a) * alpha

        return ImuSample(
            timestampNanos = timestampNanos,

            accelX = lerp(previous.accelX, current.accelX),
            accelY = lerp(previous.accelY, current.accelY),
            accelZ = lerp(previous.accelZ, current.accelZ),

            gyroX = lerp(previous.gyroX, current.gyroX),
            gyroY = lerp(previous.gyroY, current.gyroY),
            gyroZ = lerp(previous.gyroZ, current.gyroZ),

            gravityX = lerp(previous.gravityX, current.gravityX),
            gravityY = lerp(previous.gravityY, current.gravityY),
            gravityZ = lerp(previous.gravityZ, current.gravityZ)
        )
    }

    private fun isValid(sample: ImuSample): Boolean {

        if (sample.timestampNanos < 0L) {
            return false
        }

        return sample.accelX.isFinite() &&
                sample.accelY.isFinite() &&
                sample.accelZ.isFinite() &&
                sample.gyroX.isFinite() &&
                sample.gyroY.isFinite() &&
                sample.gyroZ.isFinite() &&
                sample.gravityX.isFinite() &&
                sample.gravityY.isFinite() &&
                sample.gravityZ.isFinite()
    }

    private fun calculateRateHz(
        count: Long,
        firstTimestamp: Long?,
        lastTimestamp: Long?
    ): Double? {

        if (count < 2L) return null

        if (firstTimestamp == null || lastTimestamp == null) {
            return null
        }

        val durationSeconds =
            (lastTimestamp - firstTimestamp).toDouble() * 1e-9

        if (durationSeconds <= 0.0) return null

        return ((count - 1L).toDouble() / durationSeconds)
            .takeIf { it.isFinite() && it > 0.0 }
    }
}