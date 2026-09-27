package com.rishabh.astranav.calibration

import kotlin.math.abs

class MountChangeDetector(
    private val thresholdDeg: Double = 12.0,
    private val confirmationSamples: Int = 20
) {

    private var referenceYawDeg: Double? = null

    private var suspectSamples = 0

    private var changeConfirmed = false

    fun update(
        calibratedYawDeg: Double,
        currentYawDeg: Double,
        stationary: Boolean,
        gravityChanged: Boolean = false
    ): Boolean {

        if (!stationary) {
            suspectSamples = 0
            return changeConfirmed
        }

        if (referenceYawDeg == null) {
            referenceYawDeg = calibratedYawDeg
            return false
        }

        val delta = abs(
            normalizeAngle(
                currentYawDeg -
                        referenceYawDeg!!
            )
        )

        val suspicious =
            delta > thresholdDeg ||
                    gravityChanged

        if (suspicious) {
            suspectSamples++

            if (suspectSamples >= confirmationSamples) {
                changeConfirmed = true
            }
        } else {
            suspectSamples = 0
        }

        return changeConfirmed
    }

    fun reset(
        newReferenceYawDeg: Double
    ) {
        referenceYawDeg =
            newReferenceYawDeg

        suspectSamples = 0
        changeConfirmed = false
    }

    fun isChangeConfirmed(): Boolean =
        changeConfirmed

    private fun normalizeAngle(
        angle: Double
    ): Double {

        var a = angle % 360.0

        if (a > 180.0) {
            a -= 360.0
        }

        if (a < -180.0) {
            a += 360.0
        }

        return a
    }
}