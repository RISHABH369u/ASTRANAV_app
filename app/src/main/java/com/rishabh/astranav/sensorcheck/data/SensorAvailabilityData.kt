package com.rishabh.astranav.sensorcheck.data

import com.rishabh.astranav.R
import com.rishabh.astranav.sensorcheck.model.AvailabilitySensor
import com.rishabh.astranav.sensorcheck.model.SensorResult

/**
 * Static demo data — handy for Compose/View previews or a device-less unit
 * test where you don't want to touch SensorManager/LocationManager.
 *
 * SensorCheckActivity uses [SensorCapabilityProbe.probe] by default for real
 * on-device values (actual IMU Hz, actual GNSS availability). Swap the
 * `sensors` property in the Activity back to `SensorAvailabilityData.SENSORS`
 * if you want this static list instead.
 */
object SensorAvailabilityData {
    val SENSORS = listOf(
        AvailabilitySensor(
            key = "accelerometer",
            label = "Accelerometer",
            sub = "Linear motion & tilt",
            detail = "100Hz",
            icon = R.drawable.ic_sensor_default,
            required = true,
            result = SensorResult.AVAILABLE,
        ),
        AvailabilitySensor(
            key = "gyroscope",
            label = "Gyroscope",
            sub = "Rotational velocity",
            detail = "100Hz",
            icon = R.drawable.ic_sensor_default,
            required = true,
            result = SensorResult.AVAILABLE,
        ),
        AvailabilitySensor(
            key = "magnetometer",
            label = "Magnetometer",
            sub = "Compass heading",
            icon = R.drawable.ic_sensor_default,
            required = true,
            result = SensorResult.DEGRADED,
        ),
        AvailabilitySensor(
            key = "barometer",
            label = "Barometer",
            sub = "Altitude assist",
            icon = R.drawable.ic_sensor_default,
            required = false,
            result = SensorResult.UNAVAILABLE,
        ),
        AvailabilitySensor(
            key = "gnss",
            label = "GNSS",
            sub = "Satellite position fix",
            detail = "12/14 sats",
            icon = R.drawable.ic_satellite,
            required = false,
            result = SensorResult.AVAILABLE,
        ),
    )
}
