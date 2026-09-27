package com.rishabh.astranav.calibration

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

enum class CalibrationState {
    UNINITIALIZED,
    MANUAL_ALIGNMENT,
    WAITING_FOR_MOTION,
    CALIBRATING,
    CALIBRATED,
    REFINING,
    REQUIRES_RECALIBRATION
}

data class VehicleAlignment(
    val rotationDeviceToVehicle: Mat3 = Mat3.identity(),

    val levelingMatrix: Mat3 = Mat3.identity(),

    val yawOffsetDeg: Double = 0.0,

    val pitchOffsetDeg: Double = 0.0,

    val rollOffsetDeg: Double = 0.0,

    val confidence: Double = 0.0,

    val sampleCount: Int = 0,

    val gyroBias: Vec3 = Vec3(0.0, 0.0, 0.0),

    val state: CalibrationState = CalibrationState.UNINITIALIZED,

    val isCalibrated: Boolean = false
)

class VehicleAlignmentCalibrator(
    private val gravityMagnitude: Double = 9.80665
) {

    private var levelingMatrix = Mat3.identity()

    private var mountYawRad = 0.0

    private var manualYawRad = 0.0

    private var manualAlignmentSet = false

    private var automaticRefinementEnabled = true

    private var streamDotX = 0.0
    private var streamDotY = 0.0
    private var accelerationEnergy = 0.0

    private var sampleCount = 0

    private var gyroBiasX = 0.0
    private var gyroBiasY = 0.0
    private var gyroBiasZ = 0.0

    private var gyroBiasSamples = 0

    private var state = CalibrationState.UNINITIALIZED

    private var lastEstimatedYawRad = 0.0

    private var stableYawSamples = 0

    /**
     * -------------------------------------------------------------
     * MANUAL ALIGNMENT
     * -------------------------------------------------------------
     *
     * User physically aligns the phone's forward direction
     * with the vehicle forward direction.
     *
     * Call this when the phone is physically mounted/aligned.
     *
     * The gravity vector establishes vertical.
     * manualYawDeg represents any intentional horizontal correction.
     */
    fun setManualAlignment(
        gravity: Vec3,
        manualYawDeg: Double = 0.0
    ) {

        levelingMatrix = computeLevelingMatrix(gravity)

        manualYawRad = Math.toRadians(
            normalizeAngleDeg(manualYawDeg)
        )

        mountYawRad = manualYawRad

        manualAlignmentSet = true

        state = CalibrationState.MANUAL_ALIGNMENT
    }

    /**
     * Manual yaw adjustment without recalculating gravity.
     *
     * Useful for UI buttons:
     *
     * LEFT  -> -1°
     * RIGHT -> +1°
     */
    fun adjustManualYaw(deltaDeg: Double) {

        manualYawRad += Math.toRadians(deltaDeg)

        manualYawRad = normalizeAngleRad(manualYawRad)

        mountYawRad = manualYawRad

        manualAlignmentSet = true

        state = CalibrationState.MANUAL_ALIGNMENT
    }

    fun getManualYawDeg(): Double {
        return Math.toDegrees(mountYawRad)
    }

    /**
     * Lock manual alignment and allow automatic GNSS refinement.
     */
    fun enableAutomaticRefinement(enabled: Boolean) {

        automaticRefinementEnabled = enabled

        if (enabled && manualAlignmentSet) {
            state = CalibrationState.WAITING_FOR_MOTION
        }
    }

    /**
     * -------------------------------------------------------------
     * GRAVITY LEVELING
     * -------------------------------------------------------------
     */
    fun updateGravity(gravity: Vec3) {

        if (gravity.norm() < 0.5) return

        levelingMatrix = computeLevelingMatrix(gravity)

        if (!manualAlignmentSet) {
            state = CalibrationState.MANUAL_ALIGNMENT
        }
    }

    /**
     * Convert gravity vector into device->level rotation.
     */
    fun computeLevelingMatrix(gravity: Vec3): Mat3 {

        val up = gravity.normalized()

        val targetUp = Vec3(
            0.0,
            0.0,
            1.0
        )

        return FrameTransform.rotationBetween(
            up,
            targetUp
        )
    }

    /**
     * -------------------------------------------------------------
     * GYRO BIAS
     * -------------------------------------------------------------
     */
    fun updateStationaryGyroBias(
        gyro: Vec3,
        stationary: Boolean
    ) {

        if (!stationary) return

        /*
         * Reject obviously moving/rotating samples.
         */
        if (gyro.norm() > 0.06) return

        val alpha = if (gyroBiasSamples < 100) {
            1.0 / (gyroBiasSamples + 1)
        } else {
            0.01
        }

        gyroBiasX =
            (1.0 - alpha) * gyroBiasX +
                    alpha * gyro.x

        gyroBiasY =
            (1.0 - alpha) * gyroBiasY +
                    alpha * gyro.y

        gyroBiasZ =
            (1.0 - alpha) * gyroBiasZ +
                    alpha * gyro.z

        gyroBiasSamples++
    }

    fun getGyroBias(): Vec3 {
        return Vec3(
            gyroBiasX,
            gyroBiasY,
            gyroBiasZ
        )
    }

    /**
     * -------------------------------------------------------------
     * VEHICLE YAW RATE
     * -------------------------------------------------------------
     *
     * Projection of gyro onto vehicle vertical axis.
     */
    fun extractVehicleYawRate(
        gyro: Vec3,
        gravity: Vec3
    ): Double {

        val correctedGyro = gyro - getGyroBias()

        val up = gravity.normalized()

        return correctedGyro.dot(up)
    }

    /**
     * -------------------------------------------------------------
     * AUTOMATIC MOUNT AZIMUTH REFINEMENT
     * -------------------------------------------------------------
     *
     * GNSS provides vehicle longitudinal acceleration.
     *
     * We solve:
     *
     * psi = atan2(
     *      Σ ay * a_ref,
     *      Σ ax * a_ref
     * )
     */
    fun updateAutomaticAzimuth(
        linearAccelerationDevice: Vec3,
        gravity: Vec3,
        vehicleSpeedMps: Double,
        vehicleForwardAccelerationMps2: Double,
        gnssAccuracyM: Double
    ): Boolean {

        if (!automaticRefinementEnabled) {
            return false
        }

        if (!manualAlignmentSet) {
            return false
        }

        /*
         * GNSS quality gate.
         */
        if (gnssAccuracyM > 15.0) {
            return false
        }

        /*
         * Vehicle must be moving.
         */
        if (vehicleSpeedMps < 3.0) {
            state = CalibrationState.WAITING_FOR_MOTION
            return false
        }

        /*
         * Need actual longitudinal excitation.
         */
        if (abs(vehicleForwardAccelerationMps2) < 0.25) {
            state = CalibrationState.WAITING_FOR_MOTION
            return false
        }

        val levelled =
            levelingMatrix * linearAccelerationDevice

        val ax = levelled.x
        val ay = levelled.y

        streamDotX +=
            ax * vehicleForwardAccelerationMps2

        streamDotY +=
            ay * vehicleForwardAccelerationMps2

        accelerationEnergy +=
            vehicleForwardAccelerationMps2 *
                    vehicleForwardAccelerationMps2

        sampleCount++

        state = CalibrationState.CALIBRATING

        /*
         * Need enough samples and excitation.
         */
        if (
            sampleCount < 30 ||
            accelerationEnergy < 2.0
        ) {
            return false
        }

        val estimatedYaw =
            atan2(
                streamDotY,
                streamDotX
            )

        /*
         * Check convergence.
         */
        val yawDelta = abs(
            normalizeAngleRad(
                estimatedYaw -
                        lastEstimatedYawRad
            )
        )

        if (yawDelta < Math.toRadians(0.5)) {
            stableYawSamples++
        } else {
            stableYawSamples = 0
        }

        lastEstimatedYawRad = estimatedYaw

        /*
         * Blend automatic estimate with
         * existing manual alignment.
         *
         * This avoids violent jumps.
         */
        val oldYaw = mountYawRad

        val refinementWeight =
            if (stableYawSamples >= 10) {
                0.20
            } else {
                0.05
            }

        mountYawRad =
            circularBlend(
                oldYaw,
                estimatedYaw,
                refinementWeight
            )

        if (stableYawSamples >= 10) {
            state = CalibrationState.CALIBRATED
        } else {
            state = CalibrationState.REFINING
        }

        return state == CalibrationState.CALIBRATED
    }

    /**
     * -------------------------------------------------------------
     * VEHICLE FRAME PROJECTION
     * -------------------------------------------------------------
     */
    fun projectVehicleAcceleration(
        linearAccelerationDevice: Vec3,
        gravity: Vec3
    ): Vec3 {

        /*
         * Always update leveling from latest gravity.
         */
        updateGravity(gravity)

        return FrameTransform.deviceToVehicle(
            linearAccelerationDevice,
            levelingMatrix,
            mountYawRad
        )
    }

    /**
     * Returns:
     *
     * x = forward
     * y = lateral
     * z = vertical
     */
    fun projectVehicleBodyAccel(
        linearAccelerationDevice: Vec3,
        gravity: Vec3
    ): Triple<Double, Double, Double> {

        val vehicle =
            projectVehicleAcceleration(
                linearAccelerationDevice,
                gravity
            )

        return Triple(
            vehicle.x,
            vehicle.y,
            vehicle.z
        )
    }

    /**
     * -------------------------------------------------------------
     * CONFIDENCE
     * -------------------------------------------------------------
     */
    fun confidence(): Double {

        if (sampleCount == 0) {
            return if (manualAlignmentSet) {
                0.35
            } else {
                0.0
            }
        }

        val vectorMagnitude =
            sqrt(
                streamDotX * streamDotX +
                        streamDotY * streamDotY
            )

        /*
         * Circular consistency.
         */
        val consistency =
            (
                    vectorMagnitude /
                            (sqrt(accelerationEnergy) *
                                    sqrt(
                                        max(
                                            1.0,
                                            sampleCount.toDouble()
                                        )
                                    ))
                    )
                .coerceIn(0.0, 1.0)

        val sampleConfidence =
            (sampleCount / 150.0)
                .coerceIn(0.0, 1.0)

        val stabilityConfidence =
            (stableYawSamples / 20.0)
                .coerceIn(0.0, 1.0)

        val manualConfidence =
            if (manualAlignmentSet) {
                0.35
            } else {
                0.0
            }

        return (
                0.35 * consistency +
                        0.25 * sampleConfidence +
                        0.25 * stabilityConfidence +
                        0.15 * manualConfidence
                ).coerceIn(0.0, 1.0)
    }

    fun result(): VehicleAlignment {

        val confidence =
            confidence()

        return VehicleAlignment(
            rotationDeviceToVehicle =
                Mat3.yaw(mountYawRad) *
                        levelingMatrix,

            levelingMatrix =
                levelingMatrix,

            yawOffsetDeg =
                Math.toDegrees(mountYawRad),

            confidence =
                confidence,

            sampleCount =
                sampleCount,

            gyroBias =
                getGyroBias(),

            state =
                state,

            isCalibrated =
                state == CalibrationState.CALIBRATED
        )
    }

    /**
     * -------------------------------------------------------------
     * RESET
     * -------------------------------------------------------------
     */
    fun resetAutomaticCalibration() {

        streamDotX = 0.0
        streamDotY = 0.0
        accelerationEnergy = 0.0

        sampleCount = 0

        stableYawSamples = 0

        lastEstimatedYawRad = mountYawRad

        state =
            if (manualAlignmentSet) {
                CalibrationState.WAITING_FOR_MOTION
            } else {
                CalibrationState.UNINITIALIZED
            }
    }

    fun resetEverything() {

        levelingMatrix =
            Mat3.identity()

        mountYawRad = 0.0
        manualYawRad = 0.0

        manualAlignmentSet = false

        streamDotX = 0.0
        streamDotY = 0.0
        accelerationEnergy = 0.0

        sampleCount = 0

        gyroBiasX = 0.0
        gyroBiasY = 0.0
        gyroBiasZ = 0.0

        gyroBiasSamples = 0

        stableYawSamples = 0

        state =
            CalibrationState.UNINITIALIZED
    }

    /**
     * -------------------------------------------------------------
     * HELPERS
     * -------------------------------------------------------------
     */

    private fun normalizeAngleRad(
        angle: Double
    ): Double {

        var a = angle

        while (a > Math.PI) {
            a -= 2.0 * Math.PI
        }

        while (a < -Math.PI) {
            a += 2.0 * Math.PI
        }

        return a
    }

    private fun normalizeAngleDeg(
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

    private fun circularBlend(
        old: Double,
        target: Double,
        alpha: Double
    ): Double {

        val delta =
            normalizeAngleRad(
                target - old
            )

        return normalizeAngleRad(
            old + alpha * delta
        )
    }

    private fun max(
        a: Double,
        b: Double
    ): Double {
        return if (a > b) a else b
    }
}