package com.rishabh.astranav.navigation.eskf

import kotlin.math.sqrt

/**
 * GNSS measurement adapter for ASTRA-Core ESKF.
 *
 * Coordinate convention:
 * - Position: local NED, metres
 * - Velocity: local NED, metres/second
 *
 * This class does NOT convert latitude/longitude to NED.
 * Coordinate conversion belongs outside the ESKF measurement layer.
 *
 * The ESKF remains the source of the navigation state.
 * GNSS is only a measurement used for correction.
 */
class EskfGnssUpdate(
    private val positionStdM: Double = 5.0,
    private val velocityStdMps: Double = 1.5,
    private val positionNisGate: Double = 16.27,
    private val velocityNisGate: Double = 16.27
) {

    data class PositionUpdateResult(
        val accepted: Boolean,
        val measurement: Vec3,
        val predictedPosition: Vec3,
        val innovation: Vec3,
        val innovationNormM: Double,
        val nis: Double,
        val reason: String
    )

    data class VelocityUpdateResult(
        val accepted: Boolean,
        val measurement: Vec3,
        val predictedVelocity: Vec3,
        val innovation: Vec3,
        val innovationNormMps: Double,
        val nis: Double,
        val reason: String
    )

    private val measurementUpdater = EskfMeasurementUpdate()

    /**
     * Correct ESKF position using a local-NED GNSS position.
     */
    @Synchronized
    fun updatePosition(
        state: NavigationState,
        covariance: EskfCovariance,
        gnssPositionNed: Vec3,
        horizontalAccuracyM: Double? = null,
        verticalAccuracyM: Double? = null
    ): PositionUpdateResult {

        if (!gnssPositionNed.isFinite()) {
            return PositionUpdateResult(
                accepted = false,
                measurement = gnssPositionNed,
                predictedPosition = state.position,
                innovation = Vec3.ZERO,
                innovationNormM = Double.NaN,
                nis = Double.NaN,
                reason = "non-finite GNSS position"
            )
        }

        if (!state.position.isFinite()) {
            return PositionUpdateResult(
                accepted = false,
                measurement = gnssPositionNed,
                predictedPosition = state.position,
                innovation = Vec3.ZERO,
                innovationNormM = Double.NaN,
                nis = Double.NaN,
                reason = "non-finite ESKF position"
            )
        }

        val northStd = validatedPositionStd(horizontalAccuracyM)
        val eastStd = validatedPositionStd(horizontalAccuracyM)
        val downStd = validatedPositionStd(verticalAccuracyM)

        val measurement = EskfMeasurementUpdate.Measurement(
            measurement = doubleArrayOf(
                gnssPositionNed.x,
                gnssPositionNed.y,
                gnssPositionNed.z
            ),

            predictedMeasurement = doubleArrayOf(
                state.position.x,
                state.position.y,
                state.position.z
            ),

            jacobian = identityPositionJacobian(),

            measurementCovariance = diagonal3(
                northStd * northStd,
                eastStd * eastStd,
                downStd * downStd
            ),

            nisGate = positionNisGate,
            name = "GNSS_POSITION"
        )

        val result = measurementUpdater.update(
            state = state,
            covariance = covariance,
            measurement = measurement
        )

        val innovation = Vec3(
            result.innovation.getOrElse(0) { 0.0 },
            result.innovation.getOrElse(1) { 0.0 },
            result.innovation.getOrElse(2) { 0.0 }
        )

        return PositionUpdateResult(
            accepted = result.accepted,
            measurement = gnssPositionNed,
            predictedPosition = state.position,
            innovation = innovation,
            innovationNormM = innovation.norm(),
            nis = result.nis,
            reason = result.reason ?: "unknown"
        )
    }

    /**
     * Correct ESKF velocity using a local-NED GNSS velocity.
     */
    @Synchronized
    fun updateVelocity(
        state: NavigationState,
        covariance: EskfCovariance,
        gnssVelocityNed: Vec3,
        speedAccuracyMps: Double? = null
    ): VelocityUpdateResult {

        if (!gnssVelocityNed.isFinite()) {
            return VelocityUpdateResult(
                accepted = false,
                measurement = gnssVelocityNed,
                predictedVelocity = state.velocity,
                innovation = Vec3.ZERO,
                innovationNormMps = Double.NaN,
                nis = Double.NaN,
                reason = "non-finite GNSS velocity"
            )
        }

        if (!state.velocity.isFinite()) {
            return VelocityUpdateResult(
                accepted = false,
                measurement = gnssVelocityNed,
                predictedVelocity = state.velocity,
                innovation = Vec3.ZERO,
                innovationNormMps = Double.NaN,
                nis = Double.NaN,
                reason = "non-finite ESKF velocity"
            )
        }

        val std = validatedVelocityStd(speedAccuracyMps)

        val measurement = EskfMeasurementUpdate.Measurement(
            measurement = doubleArrayOf(
                gnssVelocityNed.x,
                gnssVelocityNed.y,
                gnssVelocityNed.z
            ),

            predictedMeasurement = doubleArrayOf(
                state.velocity.x,
                state.velocity.y,
                state.velocity.z
            ),

            jacobian = identityVelocityJacobian(),

            measurementCovariance = diagonal3(
                std * std,
                std * std,
                std * std
            ),

            nisGate = velocityNisGate,
            name = "GNSS_VELOCITY"
        )

        val result = measurementUpdater.update(
            state = state,
            covariance = covariance,
            measurement = measurement
        )

        val innovation = Vec3(
            result.innovation.getOrElse(0) { 0.0 },
            result.innovation.getOrElse(1) { 0.0 },
            result.innovation.getOrElse(2) { 0.0 }
        )

        return VelocityUpdateResult(
            accepted = result.accepted,
            measurement = gnssVelocityNed,
            predictedVelocity = state.velocity,
            innovation = innovation,
            innovationNormMps = innovation.norm(),
            nis = result.nis,
            reason = result.reason ?: "unknown"
        )
    }

    private fun identityPositionJacobian(): Array<DoubleArray> {
        return Array(3) { row ->
            DoubleArray(15).apply {
                this[row] = 1.0
            }
        }
    }

    private fun identityVelocityJacobian(): Array<DoubleArray> {
        return Array(3) { row ->
            DoubleArray(15).apply {
                this[3 + row] = 1.0
            }
        }
    }

    private fun diagonal3(
        x: Double,
        y: Double,
        z: Double
    ): Array<DoubleArray> {
        return arrayOf(
            doubleArrayOf(x, 0.0, 0.0),
            doubleArrayOf(0.0, y, 0.0),
            doubleArrayOf(0.0, 0.0, z)
        )
    }

    private fun validatedPositionStd(value: Double?): Double {
        return if (value != null && value.isFinite() && value > 0.05) {
            value.coerceAtMost(100.0)
        } else {
            positionStdM
        }
    }

    private fun validatedVelocityStd(value: Double?): Double {
        return if (value != null && value.isFinite() && value > 0.01) {
            value.coerceAtMost(50.0)
        } else {
            velocityStdMps
        }
    }
}