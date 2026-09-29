package com.rishabh.astranav.ml

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OnnxValue
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

    /**
     * Extracts numeric tensor data from an ONNX Runtime output.
     *
     * IMPORTANT:
     * OrtSession.Result gives us OnnxValue/OnnxTensor.
     * We must unwrap the OnnxTensor first.
     */
    fun flattenOutput(value: Any?): FloatArray {

        // ---------------------------------------------------------
        // CASE 1: ONNX Tensor - THIS IS THE IMPORTANT FIX
        // ---------------------------------------------------------
        if (value is OnnxTensor) {
            return try {
                val buffer = value.getFloatBuffer()

                if (buffer == null) {
                    // Fallback for tensors which aren't directly float-readable
                    flattenOutput(value.getValue())
                } else {
                    val copy = buffer.duplicate()
                    val result = FloatArray(copy.remaining())
                    copy.get(result)
                    result
                }

            } catch (e: Exception) {
                e.printStackTrace()
                FloatArray(0)
            }
        }

        // ---------------------------------------------------------
        // CASE 2: Generic OnnxValue
        // ---------------------------------------------------------
        if (value is OnnxValue) {
            return try {
                flattenOutput(value.getValue())
            } catch (e: Exception) {
                e.printStackTrace()
                FloatArray(0)
            }
        }

        // ---------------------------------------------------------
        // CASE 3: FloatBuffer
        // ---------------------------------------------------------
        if (value is FloatBuffer) {
            val copy = value.duplicate()
            val result = FloatArray(copy.remaining())
            copy.get(result)
            return result
        }

        // ---------------------------------------------------------
        // CASE 4: Primitive arrays / scalars
        // ---------------------------------------------------------
        return when (value) {

            is FloatArray -> {
                value.copyOf()
            }

            is DoubleArray -> {
                FloatArray(value.size) {
                    value[it].toFloat()
                }
            }

            is IntArray -> {
                FloatArray(value.size) {
                    value[it].toFloat()
                }
            }

            is LongArray -> {
                FloatArray(value.size) {
                    value[it].toFloat()
                }
            }

            is ShortArray -> {
                FloatArray(value.size) {
                    value[it].toFloat()
                }
            }

            is Float -> {
                floatArrayOf(value)
            }

            is Double -> {
                floatArrayOf(value.toFloat())
            }

            is Int -> {
                floatArrayOf(value.toFloat())
            }

            is Long -> {
                floatArrayOf(value.toFloat())
            }

            is Number -> {
                floatArrayOf(value.toFloat())
            }

            is Array<*> -> {

                val result = ArrayList<Float>()

                fun walk(v: Any?) {
                    when (v) {

                        is FloatArray ->
                            v.forEach { result.add(it) }

                        is DoubleArray ->
                            v.forEach { result.add(it.toFloat()) }

                        is IntArray ->
                            v.forEach { result.add(it.toFloat()) }

                        is LongArray ->
                            v.forEach { result.add(it.toFloat()) }

                        is ShortArray ->
                            v.forEach { result.add(it.toFloat()) }

                        is Float ->
                            result.add(v)

                        is Double ->
                            result.add(v.toFloat())

                        is Int ->
                            result.add(v.toFloat())

                        is Long ->
                            result.add(v.toFloat())

                        is Number ->
                            result.add(v.toFloat())

                        is Array<*> ->
                            v.forEach { walk(it) }

                        is FloatBuffer -> {
                            val copy = v.duplicate()
                            while (copy.hasRemaining()) {
                                result.add(copy.get())
                            }
                        }
                    }
                }

                walk(value)

                result.toFloatArray()
            }

            else -> {
                FloatArray(0)
            }
        }
    }
}