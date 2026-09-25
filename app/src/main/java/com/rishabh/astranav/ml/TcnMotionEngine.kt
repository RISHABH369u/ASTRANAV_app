package com.rishabh.astranav.ml

import android.content.Context
import com.microsoft.onnxruntime.*
import com.rishabh.astranav.sensor.ImuSample
import java.nio.FloatBuffer

class TcnMotionEngine(context: Context) {
    private val env = OrtEnvironment.getEnvironment()
    private var session: OrtSession? = null
    private val window = ArrayDeque<FloatArray>()
    private val windowSize = 20
    private val featureCount = 12

    init {
        try {
            val bytes = context.assets.open("models/astranav_tcn_speed.onnx").use { it.readBytes() }
            session = env.createSession(bytes, OrtSession.SessionOptions())
        } catch (_: Throwable) {
            session = null
        }
    }

    fun add(sample: ImuSample): MlMeasurement {
        val f = floatArrayOf(
            sample.accelX.toFloat(), sample.accelY.toFloat(), sample.accelZ.toFloat(),
            sample.gravityX.toFloat(), sample.gravityY.toFloat(), sample.gravityZ.toFloat(),
            sample.gyroZ.toFloat(), sample.gyroY.toFloat(), sample.gyroX.toFloat(),
            (sample.accelX - sample.gravityX).toFloat(),
            (sample.accelY - sample.gravityY).toFloat(),
            (sample.accelZ - sample.gravityZ).toFloat()
        )
        if (window.size == windowSize) window.removeFirst()
        window.addLast(f)
        if (session == null || window.size < windowSize) {
            return MlMeasurement(0.0, 25.0, 0.0, false)
        }
        return runInference()
    }

    private fun runInference(): MlMeasurement {
        val flat = FloatArray(windowSize * featureCount)
        var k = 0
        for (row in window) for (v in row) flat[k++] = v

        val input = OnnxTensor.createTensor(
            env, FloatBuffer.wrap(flat),
            longArrayOf(1L, windowSize.toLong(), featureCount.toLong())
        )
        input.use {
            val name = session!!.inputNames.first()
            session!!.run(mapOf(name to input)).use { outputs ->
                val raw = outputs[0].value
                val speed = when (raw) {
                    is FloatArray -> raw.firstOrNull()?.toDouble()
                    is DoubleArray -> raw.firstOrNull()
                    is Array<*> -> {
                        val first = raw.firstOrNull()
                        when (first) {
                            is FloatArray -> first.firstOrNull()?.toDouble()
                            is DoubleArray -> first.firstOrNull()
                            is Array<*> -> (first.firstOrNull() as? Number)?.toDouble()
                            is Number -> first.toDouble()
                            else -> null
                        }
                    }
                    is Number -> raw.toDouble()
                    else -> null
                } ?: return MlMeasurement(0.0, 25.0, 0.0, false)

                val valid = speed.isFinite() && speed in 0.0..70.0
                return if (valid) MlMeasurement(speed, 1.0, 1.0, true)
                else MlMeasurement(0.0, 25.0, 0.0, false)
            }
        }
    }
}
