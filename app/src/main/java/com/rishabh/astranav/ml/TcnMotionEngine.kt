package com.rishabh.astranav.ml

import android.content.Context
import com.rishabh.astranav.sensor.ImuSample
import com.microsoft.onnxruntime.*
import java.nio.FloatBuffer

class TcnMotionEngine(context: Context) {
    private val env=OrtEnvironment.getEnvironment()
    private var session: OrtSession?=null
    private val window=ArrayDeque<FloatArray>()
    private val windowSize=20
    private val featureCount=12
    init {
        try {
            val bytes=context.assets.open("models/astranav_tcn_speed.onnx").use{it.readBytes()}
            session=env.createSession(bytes,OrtSession.SessionOptions())
        } catch(_:Throwable){ session=null }
    }
    fun add(sample: ImuSample): MlMeasurement {
        val f=floatArrayOf(
            sample.accelX.toFloat(),sample.accelY.toFloat(),sample.accelZ.toFloat(),
            sample.gravityX.toFloat(),sample.gravityY.toFloat(),sample.gravityZ.toFloat(),
            sample.gyroZ.toFloat(),sample.gyroY.toFloat(),sample.gyroX.toFloat(),
            (sample.accelX-sample.gravityX).toFloat(),
            (sample.accelY-sample.gravityY).toFloat(),
            (sample.accelZ-sample.gravityZ).toFloat()
        )
        if(window.size==windowSize) window.removeFirst()
        window.addLast(f)
        if(session==null || window.size<windowSize) return MlMeasurement(0.0,25.0,0.0,false)
        return try { infer() } catch(_:Throwable) { MlMeasurement(0.0,25.0,0.0,false) }
    }
    private fun infer(): MlMeasurement {
        val flat=FloatArray(windowSize*featureCount); var k=0
        for(row in window) for(v in row) flat[k++]=v
        val tensor=OnnxTensor.createTensor(env,FloatBuffer.wrap(flat),longArrayOf(1,windowSize.toLong(),featureCount.toLong()))
        tensor.use {
            val inputName=session!!.inputNames.iterator().next()
            val result=session!!.run(mapOf(inputName to tensor))
            result.use {
                val value=(it[0].value as Array<*>)[0]
                val speed=((value as? FloatArray)?.firstOrNull() ?: (value as? Array<*>)?.firstOrNull() as? Float ?: 0f).toDouble().coerceAtLeast(0.0)
                return MlMeasurement(speed,1.0,1.0,true)
            }
        }
    }
}
