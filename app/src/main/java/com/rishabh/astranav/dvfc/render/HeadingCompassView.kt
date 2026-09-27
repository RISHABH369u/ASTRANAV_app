package com.rishabh.astranav.dvfc.render

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * A 2D compass overlay showing the vehicle's fixed forward direction (top,
 * blue "Zv" marker — doesn't move) against the phone's current heading
 * (rotating needle, cyan while off, green once aligned) — so the person can
 * see at a glance which way to turn the phone to match the vehicle frame.
 * Same idea as the web reference's top-down view, drawn natively instead of
 * as SVG, and driven by `DVFCController`'s live `headingOffsetDeg` rather
 * than a fixed demo value.
 */
class HeadingCompassView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var offsetDeg = 0f

    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = ContextCompat.getColor(context, R.color.hairline)
    }
    private val vehiclePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.blue)
    }
    private val needlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.cyan)
    }
    private val centerDotPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.fg_faint)
    }

    /** Call from DVFCActivity's state collector with DvfcUiState.headingOffsetDeg. */
    fun setHeadingOffsetDeg(deg: Float) {
        offsetDeg = deg
        val aligned = abs(deg) < ALIGN_THRESHOLD_DEG
        needlePaint.color = ContextCompat.getColor(context, if (aligned) R.color.good else R.color.cyan)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val r = minOf(width, height) / 2f - 14f
        if (r <= 0f) return

        canvas.drawCircle(cx, cy, r, ringPaint)
        for (deg in 0 until 360 step 30) {
            val rad = Math.toRadians(deg - 90.0)
            val x1 = cx + (r - 6f) * cos(rad).toFloat()
            val y1 = cy + (r - 6f) * sin(rad).toFloat()
            val x2 = cx + r * cos(rad).toFloat()
            val y2 = cy + r * sin(rad).toFloat()
            canvas.drawLine(x1, y1, x2, y2, ringPaint)
        }

        // Fixed vehicle-forward marker — Zv, always points up, never rotates.
        drawArrow(canvas, cx, cy, innerR = r * 0.8f, angleDeg = 0f, length = r * 0.34f, paint = vehiclePaint)

        // Device-forward needle — rotates live with the current heading offset.
        drawArrow(canvas, cx, cy, innerR = r * 0.6f, angleDeg = offsetDeg, length = r * 0.5f, paint = needlePaint)

        canvas.drawCircle(cx, cy, 5f, centerDotPaint)
    }

    /** Draws a small arrow from the center outward at `angleDeg` (0 = up/12 o'clock, clockwise-positive, matching yaw sign). */
    private fun drawArrow(canvas: Canvas, cx: Float, cy: Float, innerR: Float, angleDeg: Float, length: Float, paint: Paint) {
        val rad = Math.toRadians((angleDeg - 90).toDouble())
        val baseX = cx + innerR * cos(rad).toFloat()
        val baseY = cy + innerR * sin(rad).toFloat()
        val tipX = cx + (innerR + length) * cos(rad).toFloat()
        val tipY = cy + (innerR + length) * sin(rad).toFloat()

        paint.style = Paint.Style.STROKE
        paint.strokeWidth = 4f
        canvas.drawLine(baseX, baseY, tipX, tipY, paint)

        val headSize = 9f
        val leftRad = rad + Math.toRadians(150.0)
        val rightRad = rad - Math.toRadians(150.0)
        val head = Path().apply {
            moveTo(tipX, tipY)
            lineTo(tipX + headSize * cos(leftRad).toFloat(), tipY + headSize * sin(leftRad).toFloat())
            lineTo(tipX + headSize * cos(rightRad).toFloat(), tipY + headSize * sin(rightRad).toFloat())
            close()
        }
        paint.style = Paint.Style.FILL
        canvas.drawPath(head, paint)
    }

    companion object {
        private const val ALIGN_THRESHOLD_DEG = 3f
    }
}
