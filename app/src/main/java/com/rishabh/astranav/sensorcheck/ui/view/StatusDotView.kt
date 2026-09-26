package com.rishabh.astranav.sensorcheck.ui.view

import android.animation.ValueAnimator
import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import android.view.animation.LinearInterpolator
import androidx.core.content.ContextCompat
import com.rishabh.astranav.sensorcheck.model.Health
import com.rishabh.astranav.sensorcheck.model.colorRes

/**
 * Kotlin/Canvas port of the React `StatusDot` component: a filled, glowing dot
 * with an optional looping expand-and-fade ring (`animate-ring` + `animate-pulse-dot`).
 */
class StatusDotView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private val dotPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private var ringAnimator: ValueAnimator? = null
    private var ringProgress = 0f
    private var pulseEnabled = false

    init {
        setLayerType(LAYER_TYPE_SOFTWARE, dotPaint) // required for Paint.setShadowLayer glow
    }

    var health: Health = Health.IDLE
        set(value) {
            field = value
            val color = ContextCompat.getColor(context, value.colorRes())
            dotPaint.color = color
            dotPaint.setShadowLayer(8f, 0f, 0f, color)
            ringPaint.color = color
            invalidate()
        }

    fun setPulse(enabled: Boolean) {
        pulseEnabled = enabled
        if (enabled) startPulse() else stopPulse()
    }

    private fun startPulse() {
        stopPulse()
        ringAnimator = ValueAnimator.ofFloat(0f, 1f).apply {
            duration = 1600
            repeatCount = ValueAnimator.INFINITE
            interpolator = LinearInterpolator()
            addUpdateListener {
                ringProgress = it.animatedValue as Float
                invalidate()
            }
            start()
        }
    }

    private fun stopPulse() {
        ringAnimator?.cancel()
        ringAnimator = null
        ringProgress = 0f
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val baseRadius = minOf(width, height) / 2f

        if (pulseEnabled) {
            ringPaint.alpha = ((1f - ringProgress) * 140).toInt().coerceIn(0, 255)
            canvas.drawCircle(cx, cy, baseRadius * (1f + ringProgress * 1.6f), ringPaint)
        }
        canvas.drawCircle(cx, cy, baseRadius, dotPaint)
    }

    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        stopPulse()
    }
}
