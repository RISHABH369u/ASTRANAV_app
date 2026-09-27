package com.rishabh.astranav.dvfc.sensor

import kotlin.math.sqrt

/**
 * Flags "stable" (spec STEP 3 — hold still before estimating orientation)
 * once the gyroscope magnitude has stayed below a threshold across a
 * rolling window of consecutive samples.
 */
class StabilityDetector(
    private val windowSize: Int = 30,
    private val stillThresholdRadPerSec: Float = 0.05f,
) {
    private val window = ArrayDeque<Float>()

    fun update(angularVelocity: FloatArray): Boolean {
        val mag = sqrt(
            angularVelocity[0] * angularVelocity[0] +
                angularVelocity[1] * angularVelocity[1] +
                angularVelocity[2] * angularVelocity[2],
        )
        window.addLast(mag)
        if (window.size > windowSize) window.removeFirst()
        return window.size >= windowSize && window.all { it < stillThresholdRadPerSec }
    }

    fun reset() = window.clear()
}
