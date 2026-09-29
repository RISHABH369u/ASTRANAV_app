package com.rishabh.astranav.navigation.gnss

import android.location.Location
import android.os.SystemClock
import com.rishabh.astranav.navigation.eskf.Eskf
import com.rishabh.astranav.navigation.eskf.Vec3
import kotlin.math.abs

/**
 * Runtime GNSS bridge for ASTRA-Core.
 *
 * Responsibilities:
 *
 * Android Location
 *      ↓
 * validity / freshness checks
 *      ↓
 * GnssLocalFrame
 *      ↓
 * local NED position + velocity
 *      ↓
 * ESKF GNSS measurement updates
 *
 * This class does NOT:
 * - modify ESKF state directly
 * - perform Kalman filtering
 * - convert GNSS into navigation state
 * - bypass NIS gating
 */
class GnssRuntimeCoordinator(
    private val eskf: Eskf,
    private val localFrame: GnssLocalFrame = GnssLocalFrame(),
    private val config: Config = Config()
) {

    data class Config(

        /**
         * Maximum accepted GNSS horizontal accuracy.
         *
         * Fixes worse than this are rejected before entering ESKF.
         */
        val maxHorizontalAccuracyM: Double = 50.0,

        /**
         * Maximum accepted vertical accuracy.
         */
        val maxVerticalAccuracyM: Double = 100.0,

        /**
         * Maximum age of a GNSS measurement.
         */
        val maxAgeMillis: Long = 2000L,

        /**
         * Minimum accepted GNSS speed.
         *
         * Velocity updates are still allowed at low speed, but
         * speed/bearing quality is handled separately.
         */
        val maxSpeedMps: Double = 100.0,

        /**
         * Minimum interval between accepted GNSS position updates.
         *
         * GNSS may arrive faster than the navigation loop.
         */
        val minPositionUpdateIntervalMillis: Long = 100L,

        /**
         * Minimum interval between accepted GNSS velocity updates.
         */
        val minVelocityUpdateIntervalMillis: Long = 100L
    )

    data class UpdateResult(
        val accepted: Boolean,
        val positionAccepted: Boolean,
        val velocityAccepted: Boolean,
        val originEstablished: Boolean,
        val positionInnovationM: Double?,
        val velocityInnovationMps: Double?,
        val positionNis: Double?,
        val velocityNis: Double?,
        val horizontalAccuracyM: Double?,
        val verticalAccuracyM: Double?,
        val speedMps: Double?,
        val reason: String
    )

    private var lastPositionUpdateElapsedNanos = 0L
    private var lastVelocityUpdateElapsedNanos = 0L

    private var acceptedLocationCount = 0L
    private var rejectedLocationCount = 0L

    private var acceptedPositionCount = 0L
    private var rejectedPositionCount = 0L

    private var acceptedVelocityCount = 0L
    private var rejectedVelocityCount = 0L

    private var lastResult: UpdateResult? = null

    /**
     * Process one Android Location sample.
     *
     * Call this whenever a new GNSS Location arrives.
     */
    @Synchronized
    fun processLocation(
        location: Location
    ): UpdateResult {

        val validationReason =
            validateLocation(location)

        if (validationReason != null) {

            rejectedLocationCount++

            return publish(
                UpdateResult(
                    accepted = false,
                    positionAccepted = false,
                    velocityAccepted = false,
                    originEstablished = false,
                    positionInnovationM = null,
                    velocityInnovationMps = null,
                    positionNis = null,
                    velocityNis = null,
                    horizontalAccuracyM =
                        locationAccuracy(location),

                    verticalAccuracyM =
                        verticalAccuracy(location),

                    speedMps =
                        locationSpeed(location),

                    reason = validationReason
                )
            )
        }

        val originWasMissing =
            !localFrame.hasOrigin()

        val measurement =
            localFrame.convert(location)

        if (measurement == null) {

            rejectedLocationCount++

            return publish(
                UpdateResult(
                    accepted = false,
                    positionAccepted = false,
                    velocityAccepted = false,
                    originEstablished = false,
                    positionInnovationM = null,
                    velocityInnovationMps = null,
                    positionNis = null,
                    velocityNis = null,
                    horizontalAccuracyM =
                        locationAccuracy(location),

                    verticalAccuracyM =
                        verticalAccuracy(location),

                    speedMps =
                        locationSpeed(location),

                    reason = "GNSS local-frame conversion failed"
                )
            )
        }

        val originEstablished =
            originWasMissing &&
                    localFrame.hasOrigin()

        /*
         * The first fix establishes the local origin.
         *
         * Its position is therefore [0,0,0].
         *
         * We can safely use it as the initial local reference,
         * but we do not need to force an ESKF correction immediately.
         */
        val nowElapsedNanos =
            SystemClock.elapsedRealtimeNanos()

        var positionAccepted = false
        var velocityAccepted = false

        var positionInnovationM: Double? = null
        var velocityInnovationMps: Double? = null

        var positionNis: Double? = null
        var velocityNis: Double? = null

        /*
         * --------------------------------------------------------------
         * Position update
         * --------------------------------------------------------------
         */

        val position =
            measurement.position

        if (position != null &&
            position.isFinite() &&
            shouldAcceptPosition(
                locationElapsedNanos =
                    location.elapsedRealtimeNanos
            )
        ) {

            val result =
                eskf.applyGnssPosition(
                    gnssPositionNed =
                        Vec3(
                            x = position.northM,
                            y = position.eastM,
                            z = position.downM
                        ),

                    horizontalAccuracyM =
                        measurement.horizontalAccuracyM,

                    verticalAccuracyM =
                        measurement.verticalAccuracyM
                )

            positionAccepted =
                result.accepted

            positionInnovationM =
                result.innovationNormM

            positionNis =
                result.nis

            if (result.accepted) {

                acceptedPositionCount++

                lastPositionUpdateElapsedNanos =
                    location.elapsedRealtimeNanos

            } else {

                rejectedPositionCount++
            }
        }

        /*
         * --------------------------------------------------------------
         * Velocity update
         * --------------------------------------------------------------
         *
         * Only use velocity when Android provides both speed and bearing.
         */
        val velocity =
            measurement.velocity

        if (velocity != null &&
            velocity.isFinite() &&
            shouldAcceptVelocity(
                locationElapsedNanos =
                    location.elapsedRealtimeNanos
            )
        ) {

            val result =
                eskf.applyGnssVelocity(
                    gnssVelocityNed =
                        Vec3(
                            x = velocity.northMps,
                            y = velocity.eastMps,
                            z = velocity.downMps
                        ),

                    speedAccuracyMps =
                        measurement.speedAccuracyMps
                )

            velocityAccepted =
                result.accepted

            velocityInnovationMps =
                result.innovationNormMps

            velocityNis =
                result.nis

            if (result.accepted) {

                acceptedVelocityCount++

                lastVelocityUpdateElapsedNanos =
                    location.elapsedRealtimeNanos

            } else {

                rejectedVelocityCount++
            }
        }

        val accepted =
            positionAccepted ||
                    velocityAccepted

        if (accepted) {
            acceptedLocationCount++
        } else {
            rejectedLocationCount++
        }

        val reason =
            when {
                positionAccepted &&
                        velocityAccepted ->
                    "GNSS position and velocity accepted"

                positionAccepted ->
                    "GNSS position accepted"

                velocityAccepted ->
                    "GNSS velocity accepted"

                originEstablished ->
                    "GNSS origin established; no correction accepted"

                else ->
                    "GNSS measurement rejected by ESKF gating or update policy"
            }

        return publish(
            UpdateResult(
                accepted = accepted,
                positionAccepted = positionAccepted,
                velocityAccepted = velocityAccepted,
                originEstablished = originEstablished,
                positionInnovationM = positionInnovationM,
                velocityInnovationMps = velocityInnovationMps,
                positionNis = positionNis,
                velocityNis = velocityNis,
                horizontalAccuracyM =
                    measurement.horizontalAccuracyM,

                verticalAccuracyM =
                    measurement.verticalAccuracyM,

                speedMps =
                    if (measurement.hasSpeed) {
                        location.speed.toDouble()
                    } else {
                        null
                    },

                reason = reason
            )
        )
    }

    /**
     * Returns the local-frame origin.
     */
    @Synchronized
    fun getOrigin(): GnssLocalFrame.Origin? {
        return localFrame.getOrigin()
    }

    @Synchronized
    fun getLastResult(): UpdateResult? {
        return lastResult
    }

    @Synchronized
    fun getAcceptedLocationCount(): Long {
        return acceptedLocationCount
    }

    @Synchronized
    fun getRejectedLocationCount(): Long {
        return rejectedLocationCount
    }

    @Synchronized
    fun getAcceptedPositionCount(): Long {
        return acceptedPositionCount
    }

    @Synchronized
    fun getRejectedPositionCount(): Long {
        return rejectedPositionCount
    }

    @Synchronized
    fun getAcceptedVelocityCount(): Long {
        return acceptedVelocityCount
    }

    @Synchronized
    fun getRejectedVelocityCount(): Long {
        return rejectedVelocityCount
    }

    /**
     * Resets GNSS local origin and runtime counters.
     */
    @Synchronized
    fun reset() {

        localFrame.reset()

        lastPositionUpdateElapsedNanos = 0L
        lastVelocityUpdateElapsedNanos = 0L

        acceptedLocationCount = 0L
        rejectedLocationCount = 0L

        acceptedPositionCount = 0L
        rejectedPositionCount = 0L

        acceptedVelocityCount = 0L
        rejectedVelocityCount = 0L

        lastResult = null
    }

    private fun publish(
        result: UpdateResult
    ): UpdateResult {

        lastResult = result
        return result
    }

    private fun validateLocation(
        location: Location
    ): String? {

        if (!location.latitude.isFinite() ||
            !location.longitude.isFinite()
        ) {
            return "Non-finite GNSS coordinates"
        }

        if (location.latitude < -90.0 ||
            location.latitude > 90.0
        ) {
            return "Invalid GNSS latitude"
        }

        if (location.longitude < -180.0 ||
            location.longitude > 180.0
        ) {
            return "Invalid GNSS longitude"
        }

        val accuracy =
            locationAccuracy(location)

        if (accuracy != null &&
            accuracy > config.maxHorizontalAccuracyM
        ) {
            return "Poor GNSS horizontal accuracy"
        }

        val verticalAccuracy =
            verticalAccuracy(location)

        if (verticalAccuracy != null &&
            verticalAccuracy > config.maxVerticalAccuracyM
        ) {
            return "Poor GNSS vertical accuracy"
        }

        val speed =
            locationSpeed(location)

        if (speed != null &&
            speed > config.maxSpeedMps
        ) {
            return "GNSS speed exceeds physical bound"
        }

        val ageMillis =
            locationAgeMillis(location)

        if (ageMillis > config.maxAgeMillis) {
            return "Stale GNSS measurement"
        }

        return null
    }

    private fun shouldAcceptPosition(
        locationElapsedNanos: Long
    ): Boolean {

        if (lastPositionUpdateElapsedNanos == 0L) {
            return true
        }

        val elapsedMillis =
            (
                    locationElapsedNanos -
                            lastPositionUpdateElapsedNanos
                    ) / 1_000_000L

        return elapsedMillis >=
                config.minPositionUpdateIntervalMillis
    }

    private fun shouldAcceptVelocity(
        locationElapsedNanos: Long
    ): Boolean {

        if (lastVelocityUpdateElapsedNanos == 0L) {
            return true
        }

        val elapsedMillis =
            (
                    locationElapsedNanos -
                            lastVelocityUpdateElapsedNanos
                    ) / 1_000_000L

        return elapsedMillis >=
                config.minVelocityUpdateIntervalMillis
    }

    private fun locationAgeMillis(
        location: Location
    ): Long {

        val now =
            SystemClock.elapsedRealtimeNanos()

        val timestamp =
            location.elapsedRealtimeNanos

        if (timestamp <= 0L ||
            timestamp > now
        ) {
            return Long.MAX_VALUE
        }

        return (
                now - timestamp
                ) / 1_000_000L
    }

    private fun locationAccuracy(
        location: Location
    ): Double? {

        return if (
            location.hasAccuracy() &&
            location.accuracy.isFinite()
        ) {
            location.accuracy.toDouble()
        } else {
            null
        }
    }

    private fun verticalAccuracy(
        location: Location
    ): Double? {

        return if (
            android.os.Build.VERSION.SDK_INT >= 26 &&
            location.hasVerticalAccuracy() &&
            location.verticalAccuracyMeters.isFinite()
        ) {
            location.verticalAccuracyMeters.toDouble()
        } else {
            null
        }
    }

    private fun locationSpeed(
        location: Location
    ): Double? {

        return if (
            location.hasSpeed() &&
            location.speed.isFinite()
        ) {
            location.speed.toDouble()
        } else {
            null
        }
    }
}