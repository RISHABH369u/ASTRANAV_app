package com.rishabh.astranav.navigation.gnss

import android.location.Location
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Converts Android GNSS Location measurements into the local
 * Navigation frame used by ASTRA-Core.
 *
 * Navigation frame:
 *   N = North
 *   E = East
 *   D = Down
 *
 * Position:
 *   metres relative to a chosen local origin.
 *
 * Velocity:
 *   metres/second in local NED.
 *
 * Coordinate convention is intentionally kept independent from
 * the ESKF itself.
 */
class GnssLocalFrame {

    companion object {

        // WGS84 semi-major axis.
        private const val WGS84_A = 6378137.0

        // WGS84 first eccentricity squared.
        private const val WGS84_E2 =
            6.6943799901413165e-3

        private const val DEG_TO_RAD =
            Math.PI / 180.0

        private const val RAD_TO_DEG =
            180.0 / Math.PI

        private const val MIN_VALID_LATITUDE = -90.0
        private const val MAX_VALID_LATITUDE = 90.0

        private const val MIN_VALID_LONGITUDE = -180.0
        private const val MAX_VALID_LONGITUDE = 180.0
    }

    /**
     * Local geographic origin.
     *
     * The first accepted GNSS fix can be used as the origin.
     */
    data class Origin(
        val latitudeDeg: Double,
        val longitudeDeg: Double,
        val altitudeM: Double
    )

    /**
     * Position in local NED coordinates.
     */
    data class NedPosition(
        val northM: Double,
        val eastM: Double,
        val downM: Double
    ) {
        fun isFinite(): Boolean {
            return northM.isFinite() &&
                    eastM.isFinite() &&
                    downM.isFinite()
        }
    }

    /**
     * Velocity in local NED coordinates.
     */
    data class NedVelocity(
        val northMps: Double,
        val eastMps: Double,
        val downMps: Double
    ) {
        fun isFinite(): Boolean {
            return northMps.isFinite() &&
                    eastMps.isFinite() &&
                    downMps.isFinite()
        }
    }

    /**
     * Complete converted GNSS measurement.
     */
    data class GnssMeasurement(
        val timestampNanos: Long,
        val position: NedPosition?,
        val velocity: NedVelocity?,
        val horizontalAccuracyM: Double?,
        val verticalAccuracyM: Double?,
        val speedAccuracyMps: Double?,
        val hasSpeed: Boolean,
        val hasBearing: Boolean,
        val origin: Origin
    )

    private data class Ecef(
        val x: Double,
        val y: Double,
        val z: Double
    )

    private var origin: Origin? = null

    /**
     * Returns the currently configured local origin.
     */
    @Synchronized
    fun getOrigin(): Origin? {
        return origin
    }

    /**
     * Returns true if a local origin has been established.
     */
    @Synchronized
    fun hasOrigin(): Boolean {
        return origin != null
    }

    /**
     * Sets the local geographic origin explicitly.
     *
     * Useful when navigation starts from a known reference position.
     */
    @Synchronized
    fun setOrigin(
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeM: Double
    ) {

        require(validLatitude(latitudeDeg)) {
            "Invalid origin latitude: $latitudeDeg"
        }

        require(validLongitude(longitudeDeg)) {
            "Invalid origin longitude: $longitudeDeg"
        }

        require(altitudeM.isFinite()) {
            "Invalid origin altitude: $altitudeM"
        }

        origin = Origin(
            latitudeDeg = latitudeDeg,
            longitudeDeg = longitudeDeg,
            altitudeM = altitudeM
        )
    }

    /**
     * Establishes the origin from an Android Location.
     */
    @Synchronized
    fun setOrigin(location: Location) {

        require(validLocation(location)) {
            "Invalid GNSS origin location"
        }

        setOrigin(
            latitudeDeg = location.latitude,
            longitudeDeg = location.longitude,
            altitudeM = safeAltitude(location)
        )
    }

    /**
     * Automatically establishes the origin from the first valid fix.
     *
     * Returns true if an origin was created.
     */
    @Synchronized
    fun establishOriginIfNeeded(location: Location): Boolean {

        if (origin != null) {
            return false
        }

        if (!validLocation(location)) {
            return false
        }

        setOrigin(location)

        return true
    }

    /**
     * Converts Android Location to local NED.
     *
     * If no origin exists, this method establishes the current location
     * as the origin and therefore returns a zero position.
     */
    @Synchronized
    fun convert(location: Location): GnssMeasurement? {

        if (!validLocation(location)) {
            return null
        }

        if (origin == null) {
            setOrigin(location)
        }

        val currentOrigin =
            origin ?: return null

        val position =
            convertPosition(
                latitudeDeg = location.latitude,
                longitudeDeg = location.longitude,
                altitudeM = safeAltitude(location),
                origin = currentOrigin
            )

        val velocity =
            convertVelocity(location)

        return GnssMeasurement(
            timestampNanos = location.elapsedRealtimeNanos,
            position = position,
            velocity = velocity,
            horizontalAccuracyM =
                if (location.hasAccuracy() &&
                    location.accuracy.isFinite()
                ) {
                    location.accuracy.toDouble()
                } else {
                    null
                },
            verticalAccuracyM =
                if (android.os.Build.VERSION.SDK_INT >= 26 &&
                    location.hasVerticalAccuracy() &&
                    location.verticalAccuracyMeters.isFinite()
                ) {
                    location.verticalAccuracyMeters.toDouble()
                } else {
                    null
                },
            speedAccuracyMps =
                if (android.os.Build.VERSION.SDK_INT >= 26 &&
                    location.hasSpeedAccuracy() &&
                    location.speedAccuracyMetersPerSecond.isFinite()
                ) {
                    location.speedAccuracyMetersPerSecond.toDouble()
                } else {
                    null
                },
            hasSpeed =
                location.hasSpeed() &&
                        location.speed.isFinite(),

            hasBearing =
                location.hasBearing() &&
                        location.bearing.isFinite(),

            origin = currentOrigin
        )
    }

    /**
     * Converts only position.
     */
    @Synchronized
    fun convertPosition(
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeM: Double
    ): NedPosition? {

        val currentOrigin =
            origin ?: return null

        return convertPosition(
            latitudeDeg = latitudeDeg,
            longitudeDeg = longitudeDeg,
            altitudeM = altitudeM,
            origin = currentOrigin
        )
    }

    /**
     * Converts GNSS speed + bearing into local NED velocity.
     *
     * Android bearing:
     *   degrees clockwise from true north.
     *
     * Therefore:
     *
     *   Vnorth = speed * cos(bearing)
     *   Veast  = speed * sin(bearing)
     *
     * Vertical GNSS velocity is not inferred from position.
     * We use 0 m/s for the Down component because Android Location
     * does not directly provide a reliable vertical velocity here.
     */
    private fun convertVelocity(
        location: Location
    ): NedVelocity? {

        if (!location.hasSpeed() ||
            !location.speed.isFinite() ||
            location.speed < 0.0
        ) {
            return null
        }

        if (!location.hasBearing() ||
            !location.bearing.isFinite()
        ) {
            return null
        }

        val speed =
            location.speed.toDouble()

        val bearingRad =
            normalizeBearing(location.bearing.toDouble()) *
                    DEG_TO_RAD

        val north =
            speed * cos(bearingRad)

        val east =
            speed * sin(bearingRad)

        /*
         * Android Location does not give us a direct vertical
         * velocity measurement through this API.
         *
         * Keep vertical velocity neutral rather than inventing it.
         */
        val down = 0.0

        val result =
            NedVelocity(
                northMps = north,
                eastMps = east,
                downMps = down
            )

        return if (result.isFinite()) {
            result
        } else {
            null
        }
    }

    /**
     * WGS84 geodetic coordinates -> ECEF.
     */
    private fun geodeticToEcef(
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeM: Double
    ): Ecef {

        val latitude =
            latitudeDeg * DEG_TO_RAD

        val longitude =
            longitudeDeg * DEG_TO_RAD

        val sinLatitude =
            sin(latitude)

        val cosLatitude =
            cos(latitude)

        val sinLongitude =
            sin(longitude)

        val cosLongitude =
            cos(longitude)

        val primeVerticalRadius =
            WGS84_A /
                    sqrt(
                        1.0 -
                                WGS84_E2 *
                                sinLatitude *
                                sinLatitude
                    )

        val x =
            (primeVerticalRadius + altitudeM) *
                    cosLatitude *
                    cosLongitude

        val y =
            (primeVerticalRadius + altitudeM) *
                    cosLatitude *
                    sinLongitude

        val z =
            (
                    primeVerticalRadius *
                            (1.0 - WGS84_E2) +
                            altitudeM
                    ) *
                    sinLatitude

        return Ecef(
            x = x,
            y = y,
            z = z
        )
    }

    /**
     * ECEF difference -> local NED.
     */
    private fun convertPosition(
        latitudeDeg: Double,
        longitudeDeg: Double,
        altitudeM: Double,
        origin: Origin
    ): NedPosition {

        val originEcef =
            geodeticToEcef(
                latitudeDeg = origin.latitudeDeg,
                longitudeDeg = origin.longitudeDeg,
                altitudeM = origin.altitudeM
            )

        val currentEcef =
            geodeticToEcef(
                latitudeDeg = latitudeDeg,
                longitudeDeg = longitudeDeg,
                altitudeM = altitudeM
            )

        val dx =
            currentEcef.x - originEcef.x

        val dy =
            currentEcef.y - originEcef.y

        val dz =
            currentEcef.z - originEcef.z

        val originLatitude =
            origin.latitudeDeg * DEG_TO_RAD

        val originLongitude =
            origin.longitudeDeg * DEG_TO_RAD

        val sinLatitude =
            sin(originLatitude)

        val cosLatitude =
            cos(originLatitude)

        val sinLongitude =
            sin(originLongitude)

        val cosLongitude =
            cos(originLongitude)

        /*
         * ECEF -> NED rotation.
         */
        val north =
            -sinLatitude * cosLongitude * dx -
                    sinLatitude * sinLongitude * dy +
                    cosLatitude * dz

        val east =
            -sinLongitude * dx +
                    cosLongitude * dy

        val down =
            -cosLatitude * cosLongitude * dx -
                    cosLatitude * sinLongitude * dy -
                    sinLatitude * dz

        return NedPosition(
            northM = north,
            eastM = east,
            downM = down
        )
    }

    /**
     * Clears the current local frame origin.
     */
    @Synchronized
    fun reset() {
        origin = null
    }

    private fun validLocation(
        location: Location
    ): Boolean {

        return validLatitude(location.latitude) &&
                validLongitude(location.longitude) &&
                safeAltitude(location).isFinite()
    }

    private fun validLatitude(
        latitudeDeg: Double
    ): Boolean {

        return latitudeDeg.isFinite() &&
                latitudeDeg >= MIN_VALID_LATITUDE &&
                latitudeDeg <= MAX_VALID_LATITUDE
    }

    private fun validLongitude(
        longitudeDeg: Double
    ): Boolean {

        return longitudeDeg.isFinite() &&
                longitudeDeg >= MIN_VALID_LONGITUDE &&
                longitudeDeg <= MAX_VALID_LONGITUDE
    }

    private fun safeAltitude(
        location: Location
    ): Double {

        return if (location.hasAltitude() &&
            location.altitude.isFinite()
        ) {
            location.altitude
        } else {
            0.0
        }
    }

    private fun normalizeBearing(
        bearingDeg: Double
    ): Double {

        var value =
            bearingDeg % 360.0

        if (value < 0.0) {
            value += 360.0
        }

        return value
    }
}