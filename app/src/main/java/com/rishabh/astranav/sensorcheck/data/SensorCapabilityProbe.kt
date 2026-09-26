package com.rishabh.astranav.sensorcheck.data

import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.LocationManager
import com.rishabh.astranav.R
import com.rishabh.astranav.sensorcheck.model.AvailabilitySensor
import com.rishabh.astranav.sensorcheck.model.SensorResult

/**
 * Real, on-device capability probe — replaces the static SENSOR_AVAILABILITY
 * stub with actual SensorManager/LocationManager queries.
 *
 * IMU sampling rate: Android sensors don't report a fixed "spec" Hz — they
 * report `minDelay`, the shortest interval (in microseconds) the driver can
 * deliver samples at. We convert that to Hz, which is the number a user
 * would recognise as "kitne Hz ki sampling hai".
 */
object SensorCapabilityProbe {

    fun probe(context: Context): List<AvailabilitySensor> {
        val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
        return listOf(
            probeMotionSensor(
                sensorManager, Sensor.TYPE_ACCELEROMETER,
                key = "accelerometer", label = "Accelerometer", sub = "Linear motion & tilt", required = true,
            ),
            probeMotionSensor(
                sensorManager, Sensor.TYPE_GYROSCOPE,
                key = "gyroscope", label = "Gyroscope", sub = "Rotational velocity", required = true,
            ),
            probeMotionSensor(
                sensorManager, Sensor.TYPE_MAGNETIC_FIELD,
                key = "magnetometer", label = "Magnetometer", sub = "Compass heading", required = true,
            ),
            probeMotionSensor(
                sensorManager, Sensor.TYPE_PRESSURE,
                key = "barometer", label = "Barometer", sub = "Altitude assist", required = false,
            ),
            probeGnss(context),
        )
    }

    private fun probeMotionSensor(
        sensorManager: SensorManager,
        type: Int,
        key: String,
        label: String,
        sub: String,
        required: Boolean,
    ): AvailabilitySensor {
        val sensor = sensorManager.getDefaultSensor(type)
            ?: return AvailabilitySensor(
                key = key, label = label, sub = sub, detail = null,
                icon = R.drawable.ic_sensor_default, required = required, result = SensorResult.UNAVAILABLE,
            )

        // minDelay is microseconds-per-sample at the sensor's fastest supported rate.
        val hz = if (sensor.minDelay > 0) 1_000_000 / sensor.minDelay else 0
        val result = when {
            hz >= 50 -> SensorResult.AVAILABLE   // good enough for smooth motion tracking
            hz in 1..49 -> SensorResult.DEGRADED // present but slower than ideal
            else -> SensorResult.DEGRADED        // on-change only (minDelay == 0), e.g. some barometers
        }
        val detail = if (hz > 0) "${hz}Hz" else "On-change"
        return AvailabilitySensor(key, label, sub, detail, R.drawable.ic_sensor_default, required, result)
    }

    private fun probeGnss(context: Context): AvailabilitySensor {
        val hasGps = context.packageManager.hasSystemFeature(PackageManager.FEATURE_LOCATION_GPS)
        val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as? LocationManager
        val providerEnabled = hasGps && locationManager?.isProviderEnabled(LocationManager.GPS_PROVIDER) == true

        val result = when {
            !hasGps -> SensorResult.UNAVAILABLE
            !providerEnabled -> SensorResult.DEGRADED
            else -> SensorResult.DEGRADED // AVAILABLE only once GnssMonitor reports a real fix
        }
        val detail = when {
            !hasGps -> null
            !providerEnabled -> "Location off"
            else -> "Searching…"
        }
        return AvailabilitySensor(
            key = "gnss",
            label = "GNSS",
            sub = "Satellite position fix",
            detail = detail,
            icon = R.drawable.ic_satellite,
            required = false,
            result = result,
        )
    }
}
