package com.rishabh.astranav.dvfc.gnss

import android.location.Location
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Real GNSS COG + speed + gyro yaw-rate based
 * Device -> Vehicle azimuth estimator.
 *
 * GNSS bearing is accepted only when:
 * - location is fresh
 * - accuracy is good
 * - vehicle is actually moving
 * - bearing is available
 *
 * Gyro is used to check that consecutive COG observations
 * are physically consistent with the phone's yaw-rate.
 */
class GnssAzimuthEstimator {

    data class Result(
        val gnssAvailable: Boolean,
        val accuracyM: Double?,
        val speedMps: Double?,
        val validSamples: Int,
        val automaticAzimuthDeg: Double?,
        val residualDeg: Double?,
        val circularConsistency: Double?,
        val ready: Boolean
    )

    companion object {

        private const val MIN_SPEED_MPS = 3.0
        private const val MAX_ACCURACY_M = 15.0

        private const val MIN_VALID_SAMPLES = 8

        private const val MAX_LOCATION_AGE_MS = 2500L

        private const val MAX_RESIDUAL_DEG = 35.0
        private const val MIN_CONSISTENCY = 0.85

        private const val MAX_HISTORY = 32

        private const val RAD_TO_DEG =
            180.0 / Math.PI
    }

    private data class Observation(
        val offsetDeg: Double,
        val residualDeg: Double
    )

    private val observations =
        ArrayDeque<Observation>()

    private var gyroDeltaDeg = 0.0

    private var lastCogDeg: Double? = null
    private var lastDeviceYawDeg: Double? = null

    private var latestAccuracyM: Double? = null
    private var latestSpeedMps: Double? = null

    fun reset() {

        observations.clear()

        gyroDeltaDeg = 0.0

        lastCogDeg = null
        lastDeviceYawDeg = null

        latestAccuracyM = null
        latestSpeedMps = null
    }

    /**
     * Android TYPE_GYROSCOPE values are rad/s.
     */
    fun updateGyro(
        yawRateRadPerSec: Double,
        dtSec: Double
    ) {

        if (
            !dtSec.isFinite() ||
            dtSec <= 0.0 ||
            dtSec > 0.5
        ) {
            return
        }

        if (!yawRateRadPerSec.isFinite()) {
            return
        }

        gyroDeltaDeg +=
            yawRateRadPerSec *
                    dtSec *
                    RAD_TO_DEG

        gyroDeltaDeg =
            wrapSigned(
                gyroDeltaDeg
            )
    }

    /**
     * Feed real Android Location.
     *
     * Vehicle heading:
     *     GNSS COG
     *
     * Device heading:
     *     rotation-vector yaw
     *
     * Mount azimuth:
     *     COG - device yaw
     */
    fun updateLocation(
        location: Location,
        deviceYawDeg: Double
    ): Result {

        val nowMs =
            System.currentTimeMillis()

        val ageMs =
            if (location.time > 0L) {

                abs(
                    nowMs -
                            location.time
                )

            } else {

                Long.MAX_VALUE
            }

        val accuracy =
            if (location.hasAccuracy()) {

                location.accuracy.toDouble()

            } else {

                null
            }

        val speed =
            if (location.hasSpeed()) {

                location.speed.toDouble()

            } else {

                null
            }

        latestAccuracyM =
            accuracy

        latestSpeedMps =
            speed

        /*
         * Stationary GNSS bearing is NOT usable
         * for automatic mount azimuth.
         */
        val validFix =
            ageMs <= MAX_LOCATION_AGE_MS &&
                    accuracy != null &&
                    accuracy <= MAX_ACCURACY_M &&
                    speed != null &&
                    speed >= MIN_SPEED_MPS &&
                    location.hasBearing() &&
                    deviceYawDeg.isFinite()

        if (!validFix) {
            return result()
        }

        val cogDeg =
            normalize360(
                location.bearing.toDouble()
            )

        val previousYaw =
            lastDeviceYawDeg

        val previousCog =
            lastCogDeg

        /*
         * Predict the change in vehicle heading from gyro.
         */
        val gyroPredictedCog =
            if (
                previousYaw != null &&
                previousCog != null
            ) {

                normalize360(
                    previousCog +
                            gyroDeltaDeg
                )

            } else {

                cogDeg
            }

        val gyroResidual =
            abs(
                signedAngleDeg(
                    cogDeg,
                    gyroPredictedCog
                )
            )

        /*
         * Device -> Vehicle azimuth.
         *
         * Vehicle heading - device heading.
         */
        val offsetDeg =
            normalize360(
                cogDeg -
                        normalize360(
                            deviceYawDeg
                        )
            )

        /*
         * First valid moving GNSS point establishes
         * the absolute reference.
         *
         * Later points must agree with gyro dynamics.
         */
        val accept =
            previousYaw == null ||
                    gyroResidual <= MAX_RESIDUAL_DEG

        if (accept) {

            observations.addLast(
                Observation(
                    offsetDeg =
                        offsetDeg,

                    residualDeg =
                        gyroResidual
                )
            )

            while (
                observations.size >
                MAX_HISTORY
            ) {

                observations.removeFirst()
            }

            lastCogDeg =
                cogDeg

            lastDeviceYawDeg =
                deviceYawDeg

            gyroDeltaDeg =
                0.0

        } else {

            /*
             * Reject GNSS outlier.
             */
            gyroDeltaDeg =
                0.0
        }

        return result()
    }

    private fun result(): Result {

        if (observations.isEmpty()) {

            return Result(
                gnssAvailable =
                    latestAccuracyM != null,

                accuracyM =
                    latestAccuracyM,

                speedMps =
                    latestSpeedMps,

                validSamples =
                    0,

                automaticAzimuthDeg =
                    null,

                residualDeg =
                    null,

                circularConsistency =
                    null,

                ready =
                    false
            )
        }

        var sumSin =
            0.0

        var sumCos =
            0.0

        var residualSum =
            0.0

        observations.forEach { observation ->

            val weight =
                1.0 /
                        (
                                1.0 +
                                        observation.residualDeg /
                                        MAX_RESIDUAL_DEG
                                )

            val radians =
                Math.toRadians(
                    observation.offsetDeg
                )

            sumSin +=
                sin(radians) *
                        weight

            sumCos +=
                cos(radians) *
                        weight

            residualSum +=
                observation.residualDeg
        }

        val automaticAzimuth =
            normalize360(
                atan2(
                    sumSin,
                    sumCos
                ) *
                        RAD_TO_DEG
            )

        val weightTotal =
            observations.sumOf {

                1.0 /
                        (
                                1.0 +
                                        it.residualDeg /
                                        MAX_RESIDUAL_DEG
                                )
            }

        val consistency =
            sqrt(
                sumSin * sumSin +
                        sumCos * sumCos
            ) /
                    weightTotal.coerceAtLeast(
                        1e-9
                    )

        val meanResidual =
            residualSum /
                    observations.size.toDouble()

        val ready =
            observations.size >=
                    MIN_VALID_SAMPLES &&
                    consistency >=
                    MIN_CONSISTENCY &&
                    meanResidual <=
                    MAX_RESIDUAL_DEG

        return Result(
            gnssAvailable =
                latestAccuracyM != null,

            accuracyM =
                latestAccuracyM,

            speedMps =
                latestSpeedMps,

            validSamples =
                observations.size,

            automaticAzimuthDeg =
                automaticAzimuth,

            residualDeg =
                meanResidual,

            circularConsistency =
                consistency,

            ready =
                ready
        )
    }

    private fun normalize360(
        value: Double
    ): Double {

        var x =
            value % 360.0

        if (x < 0.0) {
            x += 360.0
        }

        return x
    }

    private fun wrapSigned(
        value: Double
    ): Double {

        var x =
            value % 360.0

        if (x > 180.0) {
            x -= 360.0
        }

        if (x < -180.0) {
            x += 360.0
        }

        return x
    }

    private fun signedAngleDeg(
        fromDeg: Double,
        toDeg: Double
    ): Double {

        return wrapSigned(
            toDeg -
                    fromDeg
        )
    }
}