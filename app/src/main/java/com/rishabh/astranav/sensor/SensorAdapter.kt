package com.rishabh.astranav.sensor

class SensorAdapter {
    private var lastTimestampNanos = Long.MIN_VALUE

    fun validate(sample: ImuSample): Boolean {
        if (sample.timestampNanos <= lastTimestampNanos) return false
        lastTimestampNanos = sample.timestampNanos
        return sample.accelX.isFinite() && sample.accelY.isFinite() && sample.accelZ.isFinite() &&
            sample.gyroX.isFinite() && sample.gyroY.isFinite() && sample.gyroZ.isFinite()
    }

    fun dtSeconds(sample: ImuSample): Double? {
        if (!validate(sample)) return null
        val dt = (sample.timestampNanos - lastTimestampNanos).toDouble() * 1e-9
        return dt.coerceIn(0.001, 0.2)
    }
}
