package com.rishabh.astranav.ml

import ai.onnxruntime.OnnxTensor
import java.nio.FloatBuffer

object OnnxModelUtils {

    fun createFloatTensor(
        env: ai.onnxruntime.OrtEnvironment,
        data: FloatArray,
        shape: LongArray
    ): OnnxTensor {
        return OnnxTensor.createTensor(
            env,
            FloatBuffer.wrap(data),
            shape
        )
    }

    fun flattenOutput(value: Any?): FloatArray {
        return when (value) {

            is FloatArray -> value

            is DoubleArray -> FloatArray(value.size) {
                value[it].toFloat()
            }

            is Float -> floatArrayOf(value)

            is Double -> floatArrayOf(value.toFloat())

            is Number -> floatArrayOf(value.toFloat())

            is Array<*> -> {
                val result = ArrayList<Float>()

                fun walk(v: Any?) {
                    when (v) {
                        is FloatArray -> v.forEach { result.add(it) }
                        is DoubleArray -> v.forEach { result.add(it.toFloat()) }
                        is Float -> result.add(v)
                        is Double -> result.add(v.toFloat())
                        is Number -> result.add(v.toFloat())
                        is Array<*> -> v.forEach { walk(it) }
                    }
                }

                walk(value)
                result.toFloatArray()
            }

            else -> FloatArray(0)
        }
    }
}