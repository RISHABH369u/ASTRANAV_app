package com.rishabh.astranav.sensorcheck.ui.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.Shader
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.rishabh.astranav.R

/** Dashed crosshair + radial glow backdrop — mirrors the <svg> HUD grid in PhoneScan. */
class HudGridView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private val linePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.hairline)
        strokeWidth = 2f
        pathEffect = DashPathEffect(floatArrayOf(10f, 16f), 0f)
    }
    private val glowPaint = Paint(Paint.ANTI_ALIAS_FLAG)

    init {
        alpha = 0.5f
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        if (w > 0 && h > 0) {
            glowPaint.shader = RadialGradient(
                w / 2f, h / 2f, minOf(w, h) / 1.6f,
                intArrayOf(Color.parseColor("#2E35E0D0"), Color.parseColor("#0035E0D0")),
                floatArrayOf(0f, 1f),
                Shader.TileMode.CLAMP,
            )
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), glowPaint)
        val midY = height / 2f
        val midX = width / 2f
        canvas.drawLine(0f, midY, width.toFloat(), midY, linePaint)
        canvas.drawLine(midX, 0f, midX, height.toFloat(), linePaint)
    }
}
