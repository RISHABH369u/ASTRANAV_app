package com.rishabh.astranav.ml.astrasphm

import com.rishabh.astranav.sensor.ImuSample
import java.util.ArrayDeque

class AstraSphmFeatureBuilder {

    private val window =
        ArrayDeque<DoubleArray>(
            AstraSphmModelMetadata.WINDOW_SIZE
        )

    fun addSample(
        sample: ImuSample
    ): Boolean {

        val features = doubleArrayOf(
            sample.accelX,
            sample.accelY,
            sample.accelZ,

            sample.gyroX,
            sample.gyroY,
            sample.gyroZ
        )

        return addFeatures(features)
    }

    fun addFeatures(
        features: DoubleArray
    ): Boolean {

        require(
            features.size ==
                    AstraSphmModelMetadata.FEATURE_COUNT
        ) {
            "ASTRA-SPHM expects " +
                    "${AstraSphmModelMetadata.FEATURE_COUNT} features"
        }

        if (window.size >= AstraSphmModelMetadata.WINDOW_SIZE) {
            window.removeFirst()
        }

        window.addLast(features.copyOf())

        return isReady()
    }

    fun isReady(): Boolean {
        return window.size >=
                AstraSphmModelMetadata.WINDOW_SIZE
    }

    fun size(): Int {
        return window.size
    }

    fun progress(): Float {

        return (
                window.size.toFloat() /
                        AstraSphmModelMetadata.WINDOW_SIZE.toFloat()
                ).coerceIn(0f, 1f)
    }

    fun buildWindow(): Array<DoubleArray>? {

        if (!isReady()) {
            return null
        }

        return window.map {
            it.copyOf()
        }.toTypedArray()
    }

    fun clear() {
        window.clear()
    }

    fun latest(): DoubleArray? {
        return window.lastOrNull()?.copyOf()
    }
}