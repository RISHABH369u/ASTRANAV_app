package com.rishabh.astranav.navigation.gnss

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import androidx.core.content.ContextCompat
import com.rishabh.astranav.navigation.eskf.Eskf

/**
 * Android GNSS location source for ASTRA-Core.
 *
 * Responsibilities:
 *
 * Android LocationManager
 *        ↓
 * GPS provider
 *        ↓
 * Location
 *        ↓
 * GnssRuntimeCoordinator
 *        ↓
 * ASTRA-Core ESKF
 *
 * This class does NOT perform navigation math.
 */
class GnssLocationSource(
    context: Context,
    private val eskf: Eskf,
    private val config: Config = Config()
) {

    data class Config(
        val minTimeMillis: Long = 200L,
        val minDistanceMeters: Float = 0.0f
    )

    data class SourceState(
        val running: Boolean,
        val permissionGranted: Boolean,
        val providerEnabled: Boolean,
        val locationReceived: Boolean,
        val updateCount: Long,
        val lastLatitude: Double?,
        val lastLongitude: Double?,
        val lastAccuracyM: Double?,
        val lastSpeedMps: Double?,
        val lastResult: GnssRuntimeCoordinator.UpdateResult?
    )

    private val appContext =
        context.applicationContext

    private val locationManager =
        appContext.getSystemService(
            Context.LOCATION_SERVICE
        ) as LocationManager

    private val coordinator =
        GnssRuntimeCoordinator(
            eskf = eskf
        )

    private var running = false

    private var updateCount = 0L

    private var lastLocation: Location? = null

    private var lastResult:
            GnssRuntimeCoordinator.UpdateResult? = null

    private val locationListener =
        object : LocationListener {

            override fun onLocationChanged(
                location: Location
            ) {

                updateCount++

                lastLocation =
                    Location(location)

                lastResult =
                    coordinator.processLocation(
                        location
                    )
            }

            override fun onProviderEnabled(
                provider: String
            ) {
                // Provider state is queried dynamically.
            }

            override fun onProviderDisabled(
                provider: String
            ) {
                // Provider state is queried dynamically.
            }

            @Deprecated("Deprecated in Android API 29")
            override fun onStatusChanged(
                provider: String?,
                status: Int,
                extras: Bundle?
            ) {
                // Kept for compatibility with older Android APIs.
            }
        }

    /**
     * Starts GPS location updates.
     *
     * Returns false when location permission is missing or GPS
     * provider is unavailable.
     */
    @Synchronized
    fun start(): Boolean {

        if (!hasLocationPermission()) {
            return false
        }

        if (!isGpsProviderEnabled()) {
            return false
        }

        if (running) {
            return true
        }

        try {

            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER,
                config.minTimeMillis,
                config.minDistanceMeters,
                locationListener
            )

            running = true

            return true

        } catch (
            securityException: SecurityException
        ) {

            running = false

            return false
        }
    }

    /**
     * Stops GPS updates.
     */
    @Synchronized
    fun stop() {

        if (!running) {
            return
        }

        try {

            locationManager.removeUpdates(
                locationListener
            )

        } catch (
            securityException: SecurityException
        ) {
            // Permission may have been revoked while running.
        }

        running = false
    }

    /**
     * Resets the GNSS source and local-frame state.
     *
     * Does not reset the ESKF itself.
     */
    @Synchronized
    fun reset() {

        stop()

        coordinator.reset()

        updateCount = 0L

        lastLocation = null

        lastResult = null
    }

    /**
     * Current runtime state.
     */
    @Synchronized
    fun getState(): SourceState {

        val location =
            lastLocation

        return SourceState(
            running = running,

            permissionGranted =
                hasLocationPermission(),

            providerEnabled =
                isGpsProviderEnabled(),

            locationReceived =
                location != null,

            updateCount =
                updateCount,

            lastLatitude =
                location?.latitude,

            lastLongitude =
                location?.longitude,

            lastAccuracyM =
                if (
                    location != null &&
                    location.hasAccuracy()
                ) {
                    location.accuracy.toDouble()
                } else {
                    null
                },

            lastSpeedMps =
                if (
                    location != null &&
                    location.hasSpeed()
                ) {
                    location.speed.toDouble()
                } else {
                    null
                },

            lastResult =
                lastResult
        )
    }

    /**
     * Last Android Location received.
     *
     * Returns a copy so callers cannot mutate the internal object.
     */
    @Synchronized
    fun getLastLocation(): Location? {

        return lastLocation?.let {
            Location(it)
        }
    }

    /**
     * Last GNSS runtime processing result.
     */
    @Synchronized
    fun getLastResult():
            GnssRuntimeCoordinator.UpdateResult? {

        return lastResult
    }

    /**
     * GNSS local-frame origin.
     */
    @Synchronized
    fun getOrigin():
            GnssLocalFrame.Origin? {

        return coordinator.getOrigin()
    }

    /**
     * Number of Android location callbacks received.
     */
    @Synchronized
    fun getUpdateCount(): Long {
        return updateCount
    }

    /**
     * Whether fine or coarse location permission is available.
     *
     * GPS navigation should preferably have ACCESS_FINE_LOCATION.
     */
    private fun hasLocationPermission(): Boolean {

        return ContextCompat.checkSelfPermission(
            appContext,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED ||
                ContextCompat.checkSelfPermission(
                    appContext,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                ) == PackageManager.PERMISSION_GRANTED
    }

    /**
     * Checks whether the GPS provider is enabled.
     */
    private fun isGpsProviderEnabled(): Boolean {

        return try {

            locationManager.isProviderEnabled(
                LocationManager.GPS_PROVIDER
            )

        } catch (
            exception: Exception
        ) {

            false
        }
    }
}