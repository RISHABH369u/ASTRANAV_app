package com.rishabh.astranav.navigation

import com.rishabh.astranav.sensor.ImuSample
import kotlin.math.cos
import kotlin.math.sin

data class MechanizationState(
    var east: Double = 0.0, var north: Double = 0.0,
    var ve: Double = 0.0, var vn: Double = 0.0,
    var yawRad: Double = 0.0
)

class ImuMechanization {
    val state = MechanizationState()
    private var initialized = false

    fun initialize(yawRad: Double = 0.0) {
        state.yawRad = yawRad
        initialized = true
    }

    fun propagate(sample: ImuSample, dt: Double) {
        if (!initialized) initialize()
        state.yawRad = wrap(state.yawRad + sample.gyroZ * dt)
        val ax = sample.accelX - sample.gravityX
        val ay = sample.accelY - sample.gravityY
        val c = cos(state.yawRad); val s = sin(state.yawRad)
        val ae = c * ax - s * ay
        val an = s * ax + c * ay
        state.ve += ae * dt
        state.vn += an * dt
        state.east += state.ve * dt
        state.north += state.vn * dt
    }

    private fun wrap(x: Double): Double = ((x + Math.PI) % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI) - Math.PI
}
