package com.rishabh.astranav.sensorcheck.util

import android.view.Choreographer
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Kotlin port of the `useAnimatedNumber` React hook you included: eases a
 * displayed value toward a target via exponential smoothing, driven frame by
 * frame like the original requestAnimationFrame loop (Choreographer is
 * Android's equivalent of rAF).
 *
 * Usage:
 *   val anim = AnimatedNumber(initial = 0f, decimals = 0, stiffness = 6f) { value ->
 *       telemetryText.text = value.toInt().toString()
 *   }
 *   anim.animateTo(42f)   // eases toward 42 instead of jumping
 *   anim.stop()           // call from onDestroy/onDetachedFromWindow
 */
class AnimatedNumber(
    initial: Float,
    private val decimals: Int = 0,
    private val stiffness: Float = 6f,
    private val onUpdate: (Float) -> Unit,
) {
    private var current = initial
    private var target = initial
    private var lastFrameNanos = 0L
    private var running = false

    private val callback = object : Choreographer.FrameCallback {
        override fun doFrame(frameTimeNanos: Long) {
            if (!running) return
            if (lastFrameNanos == 0L) lastFrameNanos = frameTimeNanos
            val dt = min((frameTimeNanos - lastFrameNanos) / 1_000_000_000f, 0.1f)
            lastFrameNanos = frameTimeNanos

            current += (target - current) * min(dt * stiffness, 1f)
            if (abs(current - target) < 0.01f) current = target

            onUpdate(round(current))

            if (current != target) {
                Choreographer.getInstance().postFrameCallback(this)
            } else {
                running = false
            }
        }
    }

    private fun round(value: Float): Float =
        if (decimals == 0) {
            value.roundToInt().toFloat()
        } else {
            val factor = Math.pow(10.0, decimals.toDouble()).toFloat()
            (value * factor).roundToInt() / factor
        }

    /** Push a new target; the display eases toward it rather than jumping. */
    fun animateTo(newTarget: Float) {
        target = newTarget
        if (!running) {
            running = true
            lastFrameNanos = 0L
            Choreographer.getInstance().postFrameCallback(callback)
        }
    }

    fun stop() {
        running = false
        Choreographer.getInstance().removeFrameCallback(callback)
    }
}
