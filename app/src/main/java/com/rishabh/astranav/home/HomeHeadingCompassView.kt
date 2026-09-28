package com.rishabh.astranav.home

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R
import kotlin.math.cos
import kotlin.math.sin

/**
 * A North-up compass rose showing the vehicle's current heading (from
 * NavigationState.headingDegrees) — distinct from DVFC's
 * HeadingCompassView, which shows an *offset* between phone and vehicle
 * during calibration. This one shows absolute direction of travel, for the
 * Home dashboard.
 */
class HomeHeadingCompassView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var headingDeg = 0f
    private var needleColor = ContextCompat.getColor(context, R.color.cyan)

    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2.5f
        color = ContextCompat.getColor(context, R.color.hairline)
    }
    private val tickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f
        color = ContextCompat.getColor(context, R.color.hairline)
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.fg_faint)
        textSize = 22f
        textAlign = Paint.Align.CENTER
    }
    private val needlePaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val needleTailPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.surface_3)
    }
    private val centerDotPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.fg_dim)
    }

    /** health: "good" | "warn" | "cyan" — reuses the same semantics as the nav-mode status card. */
    fun setHeading(deg: Double, health: String) {
        headingDeg = deg.toFloat()
        needleColor = ContextCompat.getColor(
            context,
            when (health) {
                "good" -> R.color.good
                "warn" -> R.color.warn
                else -> R.color.cyan
            },
        )
        needlePaint.color = needleColor
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val r = minOf(width, height) / 2f - 22f
        if (r <= 0f) return

        canvas.drawCircle(cx, cy, r, ringPaint)

        val cardinals = listOf("N" to 0f, "E" to 90f, "S" to 180f, "W" to 270f)
        for ((label, deg) in cardinals) {
            val rad = Math.toRadians(deg - 90.0)
            val tickInnerX = cx + (r - 10f) * cos(rad).toFloat()
            val tickInnerY = cy + (r - 10f) * sin(rad).toFloat()
            val tickOuterX = cx + r * cos(rad).toFloat()
            val tickOuterY = cy + r * sin(rad).toFloat()
            canvas.drawLine(tickInnerX, tickInnerY, tickOuterX, tickOuterY, tickPaint)

            val labelR = r - 24f
            val lx = cx + labelR * cos(rad).toFloat()
            val ly = cy + labelR * sin(rad).toFloat() + 7f // small baseline correction
            canvas.drawText(label, lx, ly, labelPaint)
        }
        for (deg in 0 until 360 step 30) {
            if (deg % 90 == 0) continue
            val rad = Math.toRadians(deg - 90.0)
            val x1 = cx + (r - 6f) * cos(rad).toFloat()
            val y1 = cy + (r - 6f) * sin(rad).toFloat()
            val x2 = cx + r * cos(rad).toFloat()
            val y2 = cy + r * sin(rad).toFloat()
            canvas.drawLine(x1, y1, x2, y2, tickPaint)
        }

        // Needle: bright half points in the heading direction, dim half is the tail.
        drawSpoke(canvas, cx, cy, headingDeg, r * 0.62f, needlePaint)
        drawSpoke(canvas, cx, cy, headingDeg + 180f, r * 0.34f, needleTailPaint)

        canvas.drawCircle(cx, cy, 6f, centerDotPaint)
    }

    private fun drawSpoke(canvas: Canvas, cx: Float, cy: Float, angleDeg: Float, length: Float, paint: Paint) {
        val rad = Math.toRadians((angleDeg - 90).toDouble())
        val tipX = cx + length * cos(rad).toFloat()
        val tipY = cy + length * sin(rad).toFloat()

        paint.style = Paint.Style.STROKE
        paint.strokeWidth = 5f
        canvas.drawLine(cx, cy, tipX, tipY, paint)

        val headSize = 10f
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
}
