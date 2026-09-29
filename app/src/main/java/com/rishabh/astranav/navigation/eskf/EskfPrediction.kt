package com.rishabh.astranav.navigation.eskf

/**
 * Nominal-state inertial mechanization for ASTRA-Core.
 *
 * Responsibilities:
 *
 *   - validate timestamps
 *   - remove estimated IMU biases
 *   - integrate attitude
 *   - rotate body-frame specific force into navigation frame
 *   - add gravity
 *   - integrate velocity
 *   - integrate position
 *
 * This class does NOT:
 *
 *   - propagate covariance
 *   - perform Kalman correction
 *   - perform ZUPT
 *   - perform GNSS correction
 *   - perform ML fusion
 *
 * Those operations belong to other ESKF layers.
 *
 * Coordinate convention:
 *
 * Navigation = NED
 *   X = North
 *   Y = East
 *   Z = Down
 *
 * Body = FRD
 *   X = Forward
 *   Y = Right
 *   Z = Down
 */
class EskfPrediction(
    private val config: EskfConfig
) {

    data class PredictionResult(
        val accepted: Boolean,
        val deltaTimeSeconds: Double,
        val state: NavigationState,
        val specificForceBody: Vec3,
        val specificForceNavigation: Vec3,
        val gyroRateBody: Vec3,
        val accelerationNavigation: Vec3,
        val reason: String? = null
    )

    /**
     * Predict nominal navigation state from one IMU sample.
     *
     * accelerationBody:
     *     accelerometer specific force, m/s²
     *
     * gyroBody:
     *     angular rate, rad/s
     */
    fun predict(
        state: NavigationState,
        timestampNanos: Long,
        accelerationBody: Vec3,
        gyroBody: Vec3
    ): PredictionResult {

        if (!accelerationBody.isFinite()) {

            return rejected(
                state = state,
                reason = "Non-finite accelerometer input"
            )
        }

        if (!gyroBody.isFinite()) {

            return rejected(
                state = state,
                reason = "Non-finite gyroscope input"
            )
        }

        if (timestampNanos <= 0L) {

            return rejected(
                state = state,
                reason = "Invalid timestamp"
            )
        }

        /*
         * First sample:
         *
         * We establish the estimator clock but do not integrate.
         */
        if (state.timestampNanos <= 0L) {

            state.timestampNanos =
                timestampNanos

            return PredictionResult(
                accepted = true,
                deltaTimeSeconds = 0.0,
                state = state.copyState(),
                specificForceBody = accelerationBody,
                specificForceNavigation = Vec3.ZERO,
                gyroRateBody = gyroBody,
                accelerationNavigation = Vec3.ZERO,
                reason = "Initial timestamp accepted"
            )
        }

        /*
         * Reject duplicate or backwards timestamps.
         */
        if (
            timestampNanos <=
            state.timestampNanos
        ) {

            return rejected(
                state = state,
                reason = "Non-monotonic IMU timestamp"
            )
        }

        val deltaTimeSeconds =
            (
                    timestampNanos -
                            state.timestampNanos
                    ) * 1e-9

        /*
         * Protect the estimator against timestamp jumps.
         */
        if (
            !deltaTimeSeconds.isFinite() ||
            deltaTimeSeconds <
            config.minDeltaTimeSeconds ||
            deltaTimeSeconds >
            config.maxDeltaTimeSeconds
        ) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Invalid IMU timestep: " +
                            "$deltaTimeSeconds s"
            )
        }

        /*
         * Remove current bias estimates.
         */
        val correctedGyro =
            gyroBody -
                    state.gyroBias

        val correctedAcceleration =
            accelerationBody -
                    state.accelBias

        /*
         * ---------------------------------------------------------
         * ATTITUDE INTEGRATION
         * ---------------------------------------------------------
         *
         * dtheta = omega * dt
         *
         * q_new = q_old * dq
         *
         * dq represents the incremental body-frame rotation.
         */
        val deltaTheta =
            correctedGyro *
                    deltaTimeSeconds

        val deltaQuaternion =
            Quaternion.fromRotationVector(
                deltaTheta
            )

        val newAttitude =
            (
                    state.attitude.normalized() *
                            deltaQuaternion
                    ).normalized()

        /*
         * ---------------------------------------------------------
         * BODY -> NAVIGATION
         * ---------------------------------------------------------
         *
         * Accelerometer measures specific force:
         *
         *     f = a - g
         *
         * Therefore:
         *
         *     a = f + g
         */
        val specificForceNavigation =
            newAttitude.rotate(
                correctedAcceleration
            )

        /*
         * NED gravity:
         *
         *     [0, 0, +g]
         */
        val accelerationNavigation =
            specificForceNavigation +
                    config.gravityVector()

        /*
         * ---------------------------------------------------------
         * VELOCITY
         * ---------------------------------------------------------
         *
         *     v(k+1) = v(k) + a*dt
         */
        val newVelocity =
            state.velocity +
                    accelerationNavigation *
                    deltaTimeSeconds

        /*
         * ---------------------------------------------------------
         * POSITION
         * ---------------------------------------------------------
         *
         * Constant acceleration:
         *
         * p(k+1) =
         *     p(k)
         *     + v(k)dt
         *     + 0.5*a*dt²
         */
        val dtSquared =
            deltaTimeSeconds *
                    deltaTimeSeconds

        val newPosition =
            state.position +
                    state.velocity *
                    deltaTimeSeconds +
                    accelerationNavigation *
                    (0.5 * dtSquared)

        /*
         * ---------------------------------------------------------
         * NUMERICAL VALIDATION
         * ---------------------------------------------------------
         */
        if (!newAttitude.isFinite()) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Predicted attitude became non-finite"
            )
        }

        if (!newVelocity.isFinite()) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Predicted velocity became non-finite"
            )
        }

        if (!newPosition.isFinite()) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Predicted position became non-finite"
            )
        }

        /*
         * Safety bounds.
         */
        if (
            newVelocity.norm() >
            config.maxVelocityMps
        ) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Predicted velocity exceeded safety limit"
            )
        }

        if (
            newPosition.norm() >
            config.maxPositionM
        ) {

            return rejected(
                state = state,
                deltaTimeSeconds =
                    deltaTimeSeconds,
                reason =
                    "Predicted position exceeded safety limit"
            )
        }

        /*
         * ---------------------------------------------------------
         * COMMIT STATE
         * ---------------------------------------------------------
         */
        state.position =
            newPosition

        state.velocity =
            newVelocity

        state.attitude =
            newAttitude

        state.timestampNanos =
            timestampNanos

        return PredictionResult(
            accepted = true,
            deltaTimeSeconds =
                deltaTimeSeconds,
            state =
                state.copyState(),
            specificForceBody =
                correctedAcceleration,
            specificForceNavigation =
                specificForceNavigation,
            gyroRateBody =
                correctedGyro,
            accelerationNavigation =
                accelerationNavigation
        )
    }

    /**
     * Convenience overload.
     */
    fun predict(
        state: NavigationState,
        timestampNanos: Long,
        accelX: Double,
        accelY: Double,
        accelZ: Double,
        gyroX: Double,
        gyroY: Double,
        gyroZ: Double
    ): PredictionResult {

        return predict(
            state = state,
            timestampNanos = timestampNanos,
            accelerationBody = Vec3(
                x = accelX,
                y = accelY,
                z = accelZ
            ),
            gyroBody = Vec3(
                x = gyroX,
                y = gyroY,
                z = gyroZ
            )
        )
    }

    /**
     * Resets only timestamp handling.
     *
     * Useful when the sensor stream restarts.
     */
    fun resetTimestamp(
        state: NavigationState
    ) {
        state.timestampNanos = 0L
    }

    private fun rejected(
        state: NavigationState,
        deltaTimeSeconds: Double = 0.0,
        reason: String
    ): PredictionResult {

        return PredictionResult(
            accepted = false,
            deltaTimeSeconds =
                deltaTimeSeconds,
            state =
                state.copyState(),
            specificForceBody =
                Vec3.ZERO,
            specificForceNavigation =
                Vec3.ZERO,
            gyroRateBody =
                Vec3.ZERO,
            accelerationNavigation =
                Vec3.ZERO,
            reason =
                reason
        )
    }
}