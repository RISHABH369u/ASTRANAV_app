package com.rishabh.astranav.home

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R

/**
 * A minimal bar-style sparkline — the small live trend strip under the
 * "AI speed" / "Motion" tiles, inspired by the reference dashboard's
 * Vehicle Signal card. Values are expected pre-normalized by the caller
 * (0f..1f for motion energy) or raw (km/h for speed, auto-scaled here).
 */
class SparklineView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var values: List<Float> = emptyList()
    private var autoScale = true

    private val barPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.cyan)
    }

    /** @param autoScaleToMax true for raw units (e.g. km/h) where the tallest bar should hit the top; false when values already arrive 0f..1f. */
    fun setColorRes(colorRes: Int) {
        barPaint.color = ContextCompat.getColor(context, colorRes)
        invalidate()
    }

    fun submit(newValues: List<Float>, autoScaleToMax: Boolean = true) {
        values = newValues
        autoScale = autoScaleToMax
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (values.isEmpty()) return

        val maxVal = if (autoScale) (values.maxOrNull()?.takeIf { it > 0f } ?: 1f) else 1f
        val barCount = values.size
        val gap = 3f
        val barWidth = (width - gap * (barCount - 1)) / barCount
        if (barWidth <= 0f) return

        values.forEachIndexed { i, v ->
            val ratio = (v / maxVal).coerceIn(0.06f, 1f) // floor so a zero value still shows a faint nub
            val barHeight = height * ratio
            val left = i * (barWidth + gap)
            val top = height - barHeight
            barPaint.alpha = 90 + ((i.toFloat() / barCount) * 165).toInt() // older bars fade, newest is brightest
            canvas.drawRoundRect(left, top, left + barWidth, height.toFloat(), barWidth / 2.2f, barWidth / 2.2f, barPaint)
        }
    }
}
