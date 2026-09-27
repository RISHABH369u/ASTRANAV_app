package com.rishabh.astranav.ui.view

import android.animation.ValueAnimator
import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import android.view.animation.DecelerateInterpolator
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R

/**
 * A ring-style gauge for the DVFC quality hero card. Draws a dim full-circle
 * "track" and a colored progress arc on top, starting at 12 o'clock and
 * sweeping clockwise. Animates smoothly whenever [setProgress] changes value,
 * mirroring the calm HUD feel used across the rest of ASTRA NAV.
 */
class CircularGaugeView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private val trackPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        color = ContextCompat.getColor(context, R.color.hairline)
    }

    private val progressPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        color = ContextCompat.getColor(context, R.color.cyan)
    }

    private val arcBounds = RectF()
    private var strokeWidthPx = dp(9f)

    private var displayedProgress = 0f
    private var animator: ValueAnimator? = null

    init {
        trackPaint.strokeWidth = strokeWidthPx
        progressPaint.strokeWidth = strokeWidthPx
    }

    /** Animates the ring to [value] (0-100) and tints it with [colorRes]. */
    fun setProgress(value: Int, colorRes: Int, animate: Boolean = true) {
        val target = value.coerceIn(0, 100).toFloat()
        progressPaint.color = ContextCompat.getColor(context, colorRes)

        animator?.cancel()

        if (!animate) {
            displayedProgress = target
            invalidate()
            return
        }

        animator = ValueAnimator.ofFloat(displayedProgress, target).apply {
            duration = 650
            interpolator = DecelerateInterpolator()
            addUpdateListener {
                displayedProgress = it.animatedValue as Float
                invalidate()
            }
            start()
        }
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        val inset = strokeWidthPx / 2f
        arcBounds.set(inset, inset, w - inset, h - inset)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawOval(arcBounds, trackPaint)

        val sweep = 360f * (displayedProgress / 100f)
        if (sweep > 0f) {
            canvas.drawArc(arcBounds, -90f, sweep, false, progressPaint)
        }
    }

    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        animator?.cancel()
    }

    private fun dp(value: Float): Float = value * resources.displayMetrics.density
}
