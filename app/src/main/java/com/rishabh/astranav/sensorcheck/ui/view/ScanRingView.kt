package com.rishabh.astranav.sensorcheck.ui.view

import android.animation.ValueAnimator
import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import android.view.animation.LinearInterpolator
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R

/**
 * Mirrors the CSS `animate-ring` keyframe used for the two concentric scan
 * pulses in the React `PhoneScan` component: a circle that expands and fades
 * out on a loop. Call [start] with a `startDelay` to stagger a second ring.
 */
class ScanRingView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.cyan)
        style = Paint.Style.FILL
    }
    private var progress = 0f
    private var animator: ValueAnimator? = null

    fun start(startDelay: Long = 0L) {
        stop()
        animator = ValueAnimator.ofFloat(0f, 1f).apply {
            duration = 2000
            this.startDelay = startDelay
            repeatCount = ValueAnimator.INFINITE
            interpolator = LinearInterpolator()
            addUpdateListener {
                progress = it.animatedValue as Float
                invalidate()
            }
            start()
        }
    }

    fun stop() {
        animator?.cancel()
        animator = null
        progress = 0f
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (animator == null) return
        val cx = width / 2f
        val cy = height / 2f
        val maxRadius = minOf(width, height) / 2f
        val radius = maxRadius * (0.6f + progress * 0.9f)
        paint.alpha = ((1f - progress) * 90).toInt().coerceIn(0, 255)
        canvas.drawCircle(cx, cy, radius, paint)
    }

    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        stop()
    }
}
