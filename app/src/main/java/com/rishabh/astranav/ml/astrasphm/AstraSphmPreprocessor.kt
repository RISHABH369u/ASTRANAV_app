package com.rishabh.astranav.ml.astrasphm

import kotlin.math.max

class AstraSphmPreprocessor {

    private val imuMean = doubleArrayOf(
        0.04105671867728233,
        -0.05652690678834915,
        9.789615631103516,
        0.0004609735624399036,
        -0.004508985206484795,
        0.0010660206899046898
    )

    private val imuStd = doubleArrayOf(
        1.6698585748672485,
        1.5978151559829712,
        0.8167657852172852,
        0.12872757017612457,
        0.226267009973526,
        0.1430339515209198
    )

    private val speedMean = 11.62447452545166

    private val speedStd = 8.732809066772461

    private val positionMean = doubleArrayOf(
        -2.996354579925537,
        3.694854974746704
    )

    private val positionStd = doubleArrayOf(
        19.846437454223633,
        18.584957122802734
    )

    private val yawMean = -0.0026945548597723246

    private val yawStd = 0.10826282203197479

    fun normalizeImu(
        window: Array<DoubleArray>
    ): Array<FloatArray> {

        require(window.size == AstraSphmModelMetadata.WINDOW_SIZE) {
            "ASTRA-SPHM requires exactly " +
                    "${AstraSphmModelMetadata.WINDOW_SIZE} IMU samples"
        }

        return Array(window.size) { rowIndex ->

            require(
                window[rowIndex].size ==
                        AstraSphmModelMetadata.FEATURE_COUNT
            ) {
                "ASTRA-SPHM requires exactly " +
                        "${AstraSphmModelMetadata.FEATURE_COUNT} IMU features"
            }

            FloatArray(
                AstraSphmModelMetadata.FEATURE_COUNT
            ) { featureIndex ->

                val value = window[rowIndex][featureIndex]

                val std = max(
                    imuStd[featureIndex],
                    1e-8
                )

                ((value - imuMean[featureIndex]) / std)
                    .toFloat()
            }
        }
    }

    fun normalizeInitialSpeed(
        speedMps: Double
    ): Float {

        val safeSpeed = speedMps
            .coerceAtLeast(0.0)

        return (
                (safeSpeed - speedMean) /
                        max(speedStd, 1e-8)
                ).toFloat()
    }

    fun denormalizeSpeed(
        normalizedSpeed: Double
    ): Double {

        return (
                normalizedSpeed * speedStd +
                        speedMean
                ).coerceAtLeast(0.0)
    }

    fun denormalizePosition(
        normalizedX: Double,
        normalizedY: Double
    ): Pair<Double, Double> {

        val x =
            normalizedX * positionStd[0] +
                    positionMean[0]

        val y =
            normalizedY * positionStd[1] +
                    positionMean[1]

        return Pair(x, y)
    }

    fun denormalizeHeadingDelta(
        normalizedYaw: Double
    ): Double {

        return normalizedYaw * yawStd + yawMean
    }

    fun getImuMean(): DoubleArray =
        imuMean.copyOf()

    fun getImuStd(): DoubleArray =
        imuStd.copyOf()

    fun getSpeedMean(): Double =
        speedMean

    fun getSpeedStd(): Double =
        speedStd
}