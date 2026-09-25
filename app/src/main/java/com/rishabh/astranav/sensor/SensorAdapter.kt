package com.rishabh.astranav.sensor

class SensorAdapter {
    private var lastTimestampNanos = Long.MIN_VALUE

    fun validate(sample: ImuSample): Boolean {
        if (sample.timestampNanos <= lastTimestampNanos) return false
        return sample.accelX.isFinite() && sample.accelY.isFinite() && sample.accelZ.isFinite() &&
            sample.gyroX.isFinite() && sample.gyroY.isFinite() && sample.gyroZ.isFinite()
    }

    fun dtSeconds(sample: ImuSample): Double? {
        if (!validate(sample)) return null
        val previous = lastTimestampNanos
        lastTimestampNanos = sample.timestampNanos
        if (previous == Long.MIN_VALUE) return null
        val dt = (sample.timestampNanos - previous).toDouble() * 1e-9
        return if (dt.isFinite() && dt in 0.001..0.2) dt else null
    }
}