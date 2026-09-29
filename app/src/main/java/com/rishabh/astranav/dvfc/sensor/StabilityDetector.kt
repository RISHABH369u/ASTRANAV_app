package com.rishabh.astranav.dvfc.sensor

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Robust DVFC stationary / mounting stability detector.
 *
 * Uses:
 *  - Gyroscope magnitude
 *  - Gravity magnitude
 *  - Gravity-compensated linear acceleration
 *
 * The score is calculated over a recent rolling window.
 *
 * IMPORTANT:
 * This measures the current calibration window.
 * It does NOT use the complete lifetime sample history.
 */
class StabilityDetector(
    private val windowSize: Int = 30,
    private val gyroLimitRadPerSec: Double = 0.08,
    private val gravityToleranceMps2: Double = 0.50,
    private val linearAccelerationLimitMps2: Double = 0.30
) {

    data class Metrics(
        val score: Double,
        val stable: Boolean,
        val sampleCount: Int,
        val gyroScore: Double,
        val gravityScore: Double,
        val accelerationScore: Double
    )

    private data class Sample(
        val gyroScore: Double,
        val gravityScore: Double,
        val accelerationScore: Double
    )

    private val window =
        ArrayDeque<Sample>()

    fun update(
        angularVelocity: FloatArray,
        gravity: FloatArray,
        linearAcceleration: FloatArray
    ): Metrics {

        if (
            angularVelocity.size < 3 ||
            gravity.size < 3 ||
            linearAcceleration.size < 3
        ) {
            return Metrics(
                score = 0.0,
                stable = false,
                sampleCount = window.size,
                gyroScore = 0.0,
                gravityScore = 0.0,
                accelerationScore = 0.0
            )
        }

        val gx = angularVelocity[0].toDouble()
        val gy = angularVelocity[1].toDouble()
        val gz = angularVelocity[2].toDouble()

        val gyroMagnitude =
            sqrt(
                gx * gx +
                        gy * gy +
                        gz * gz
            )

        val gravityX = gravity[0].toDouble()
        val gravityY = gravity[1].toDouble()
        val gravityZ = gravity[2].toDouble()

        val gravityMagnitude =
            sqrt(
                gravityX * gravityX +
                        gravityY * gravityY +
                        gravityZ * gravityZ
            )

        val laX = linearAcceleration[0].toDouble()
        val laY = linearAcceleration[1].toDouble()
        val laZ = linearAcceleration[2].toDouble()

        val linearAccelerationMagnitude =
            sqrt(
                laX * laX +
                        laY * laY +
                        laZ * laZ
            )

        val gyroScore =
            (
                    1.0 -
                            gyroMagnitude /
                            gyroLimitRadPerSec
                    )
                .coerceIn(0.0, 1.0)

        val gravityError =
            abs(
                gravityMagnitude -
                        9.80665
            )

        val gravityScore =
            (
                    1.0 -
                            gravityError /
                            gravityToleranceMps2
                    )
                .coerceIn(0.0, 1.0)

        val accelerationScore =
            (
                    1.0 -
                            linearAccelerationMagnitude /
                            linearAccelerationLimitMps2
                    )
                .coerceIn(0.0, 1.0)

        window.addLast(
            Sample(
                gyroScore = gyroScore,
                gravityScore = gravityScore,
                accelerationScore = accelerationScore
            )
        )

        while (window.size > windowSize) {
            window.removeFirst()
        }

        val averageGyro =
            window.averageOf {
                it.gyroScore
            }

        val averageGravity =
            window.averageOf {
                it.gravityScore
            }

        val averageAcceleration =
            window.averageOf {
                it.accelerationScore
            }

        /*
         * Mount stability is primarily gyro/gravity driven.
         * Linear acceleration prevents accepting a phone that
         * is rotating very slowly while still being disturbed.
         */
        val score =
            (
                    0.45 * averageGyro +
                            0.35 * averageGravity +
                            0.20 * averageAcceleration
                    )
                .coerceIn(0.0, 1.0)

        /*
         * 30 samples at 10 Hz = approximately 3 seconds.
         */
        val stable =
            window.size >= windowSize &&
                    score >= 0.80 &&
                    averageGyro >= 0.75 &&
                    averageGravity >= 0.80 &&
                    averageAcceleration >= 0.65

        return Metrics(
            score = score,
            stable = stable,
            sampleCount = window.size,
            gyroScore = averageGyro,
            gravityScore = averageGravity,
            accelerationScore = averageAcceleration
        )
    }

    /**
     * Compatibility overload.
     *
     * If only gyro data is available, use gyro-only stability.
     */
    fun update(
        angularVelocity: FloatArray
    ): Boolean {

        if (angularVelocity.size < 3) {
            return false
        }

        val gx = angularVelocity[0].toDouble()
        val gy = angularVelocity[1].toDouble()
        val gz = angularVelocity[2].toDouble()

        val magnitude =
            sqrt(
                gx * gx +
                        gy * gy +
                        gz * gz
            )

        val score =
            (
                    1.0 -
                            magnitude /
                            gyroLimitRadPerSec
                    )
                .coerceIn(0.0, 1.0)

        window.addLast(
            Sample(
                gyroScore = score,
                gravityScore = 1.0,
                accelerationScore = 1.0
            )
        )

        while (window.size > windowSize) {
            window.removeFirst()
        }

        return window.size >= windowSize &&
                window.all {
                    it.gyroScore >= 0.75
                }
    }

    fun reset() {
        window.clear()
    }

    private inline fun <T> Iterable<T>.averageOf(
        selector: (T) -> Double
    ): Double {

        if (!iterator().hasNext()) {
            return 0.0
        }

        var sum = 0.0
        var count = 0

        for (item in this) {
            sum += selector(item)
            count++
        }

        return if (count == 0) {
            0.0
        } else {
            sum / count
        }
    }
}