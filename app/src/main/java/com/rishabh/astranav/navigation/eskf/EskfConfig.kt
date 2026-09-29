package com.rishabh.astranav.navigation.eskf

/**
 * Configuration for ASTRA-Core Error-State Kalman Filter.
 *
 * All values are SI units unless explicitly stated.
 *
 * Coordinate convention:
 *   Navigation frame = NED
 *   Body frame      = FRD
 *
 * ESKF nominal state:
 *   position
 *   velocity
 *   attitude
 *   gyro bias
 *   accelerometer bias
 *
 * Error state:
 *   [δp, δv, δθ, δbg, δba] = 15 states
 */
data class EskfConfig(

    /**
     * Gravity magnitude used by the mechanization.
     *
     * NED convention means gravity acceleration is:
     *
     *   [0, 0, +g]
     *
     * because Down is positive.
     */
    val gravityMps2: Double = 9.80665,

    /**
     * Maximum accepted IMU timestep.
     *
     * Protects the estimator against sensor timestamp jumps.
     */
    val maxDeltaTimeSeconds: Double = 0.25,

    /**
     * Minimum accepted timestep.
     *
     * Very small / duplicate timestamps are ignored.
     */
    val minDeltaTimeSeconds: Double = 0.001,

    /**
     * Gyroscope continuous-time white-noise density.
     *
     * rad/s/sqrt(Hz)
     */
    val gyroNoiseDensity: Double = 0.015,

    /**
     * Accelerometer continuous-time white-noise density.
     *
     * m/s²/sqrt(Hz)
     */
    val accelNoiseDensity: Double = 0.15,

    /**
     * Gyroscope bias random walk.
     *
     * rad/s²/sqrt(Hz)
     */
    val gyroBiasRandomWalk: Double = 0.0005,

    /**
     * Accelerometer bias random walk.
     *
     * m/s³/sqrt(Hz)
     */
    val accelBiasRandomWalk: Double = 0.01,

    /**
     * Initial position standard deviation.
     *
     * meters.
     */
    val initialPositionStdM: Double = 5.0,

    /**
     * Initial velocity standard deviation.
     *
     * m/s.
     */
    val initialVelocityStdMps: Double = 1.0,

    /**
     * Initial attitude standard deviation.
     *
     * radians.
     */
    val initialAttitudeStdRad: Double =
        Math.toRadians(5.0),

    /**
     * Initial gyro bias standard deviation.
     *
     * rad/s.
     */
    val initialGyroBiasStdRadPerSec: Double =
        0.02,

    /**
     * Initial accelerometer bias standard deviation.
     *
     * m/s².
     */
    val initialAccelBiasStdMps2: Double =
        0.20,

    /**
     * ZUPT velocity measurement standard deviation.
     *
     * This is intentionally configurable.
     *
     * We do NOT directly force velocity to zero.
     */
    val zuptVelocityStdMps: Double = 0.05,

    /**
     * Maximum allowed absolute velocity.
     *
     * Safety / numerical sanity check only.
     */
    val maxVelocityMps: Double = 100.0,

    /**
     * Maximum allowed absolute position magnitude
     * relative to the local navigation origin.
     *
     * This is a numerical safety guard, not a navigation limit.
     */
    val maxPositionM: Double = 1_000_000.0,

    /**
     * Maximum allowed gyro bias magnitude.
     *
     * rad/s.
     */
    val maxGyroBiasRadPerSec: Double = 1.0,

    /**
     * Maximum allowed accelerometer bias magnitude.
     *
     * m/s².
     */
    val maxAccelBiasMps2: Double = 10.0,

    /**
     * Small numerical epsilon used by matrix/math operations.
     */
    val numericalEpsilon: Double = 1e-12
) {

    init {

        require(gravityMps2 > 0.0) {
            "gravityMps2 must be > 0"
        }

        require(
            minDeltaTimeSeconds > 0.0 &&
                    maxDeltaTimeSeconds > minDeltaTimeSeconds
        ) {
            "Invalid timestep limits"
        }

        require(
            gyroNoiseDensity >= 0.0 &&
                    accelNoiseDensity >= 0.0
        ) {
            "IMU noise densities cannot be negative"
        }

        require(
            gyroBiasRandomWalk >= 0.0 &&
                    accelBiasRandomWalk >= 0.0
        ) {
            "Bias random walks cannot be negative"
        }

        require(
            initialPositionStdM > 0.0 &&
                    initialVelocityStdMps > 0.0 &&
                    initialAttitudeStdRad > 0.0 &&
                    initialGyroBiasStdRadPerSec > 0.0 &&
                    initialAccelBiasStdMps2 > 0.0
        ) {
            "Initial standard deviations must be > 0"
        }

        require(zuptVelocityStdMps > 0.0) {
            "ZUPT velocity standard deviation must be > 0"
        }

        require(maxVelocityMps > 0.0) {
            "maxVelocityMps must be > 0"
        }

        require(maxPositionM > 0.0) {
            "maxPositionM must be > 0"
        }

        require(maxGyroBiasRadPerSec > 0.0) {
            "maxGyroBiasRadPerSec must be > 0"
        }

        require(maxAccelBiasMps2 > 0.0) {
            "maxAccelBiasMps2 must be > 0"
        }

        require(numericalEpsilon > 0.0) {
            "numericalEpsilon must be > 0"
        }
    }

    /**
     * Gravity vector in NED coordinates.
     */
    fun gravityVector(): Vec3 {
        return Vec3(
            x = 0.0,
            y = 0.0,
            z = gravityMps2
        )
    }

    /**
     * Returns the initial 15-state covariance diagonal.
     *
     * State ordering:
     *
     *   0..2   position
     *   3..5   velocity
     *   6..8   attitude error
     *   9..11  gyro bias
     *   12..14 accelerometer bias
     */
    fun initialCovarianceDiagonal(): DoubleArray {

        return doubleArrayOf(

            // Position
            initialPositionStdM * initialPositionStdM,
            initialPositionStdM * initialPositionStdM,
            initialPositionStdM * initialPositionStdM,

            // Velocity
            initialVelocityStdMps * initialVelocityStdMps,
            initialVelocityStdMps * initialVelocityStdMps,
            initialVelocityStdMps * initialVelocityStdMps,

            // Attitude
            initialAttitudeStdRad * initialAttitudeStdRad,
            initialAttitudeStdRad * initialAttitudeStdRad,
            initialAttitudeStdRad * initialAttitudeStdRad,

            // Gyroscope bias
            initialGyroBiasStdRadPerSec *
                    initialGyroBiasStdRadPerSec,

            initialGyroBiasStdRadPerSec *
                    initialGyroBiasStdRadPerSec,

            initialGyroBiasStdRadPerSec *
                    initialGyroBiasStdRadPerSec,

            // Accelerometer bias
            initialAccelBiasStdMps2 *
                    initialAccelBiasStdMps2,

            initialAccelBiasStdMps2 *
                    initialAccelBiasStdMps2,

            initialAccelBiasStdMps2 *
                    initialAccelBiasStdMps2
        )
    }

    /**
     * Process-noise standard deviations used for one
     * discrete timestep.
     *
     * These are converted from continuous noise densities
     * using sqrt(dt).
     */
    fun processNoiseStd(dtSeconds: Double): DoubleArray {

        require(dtSeconds > 0.0) {
            "dtSeconds must be > 0"
        }

        val sqrtDt = kotlin.math.sqrt(dtSeconds)

        return doubleArrayOf(

            // Position process noise.
            //
            // Position is indirectly driven by velocity,
            // so these are kept at zero here.
            0.0,
            0.0,
            0.0,

            // Velocity driven by accelerometer noise.
            accelNoiseDensity * sqrtDt,
            accelNoiseDensity * sqrtDt,
            accelNoiseDensity * sqrtDt,

            // Attitude driven by gyro noise.
            gyroNoiseDensity * sqrtDt,
            gyroNoiseDensity * sqrtDt,
            gyroNoiseDensity * sqrtDt,

            // Gyro bias random walk.
            gyroBiasRandomWalk * sqrtDt,
            gyroBiasRandomWalk * sqrtDt,
            gyroBiasRandomWalk * sqrtDt,

            // Accelerometer bias random walk.
            accelBiasRandomWalk * sqrtDt,
            accelBiasRandomWalk * sqrtDt,
            accelBiasRandomWalk * sqrtDt
        )
    }
}