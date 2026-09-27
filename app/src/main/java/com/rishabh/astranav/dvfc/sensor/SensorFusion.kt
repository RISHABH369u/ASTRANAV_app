package com.rishabh.astranav.dvfc.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.rishabh.astranav.dvfc.math.Quat

data class DeviceOrientationSample(
    val quaternion: Quat,            // device orientation relative to Earth (ENU) frame
    val angularVelocity: FloatArray, // rad/s, most recent gyroscope sample — used for stability detection
    val timestampNs: Long,
    val accuracy: Int,
)

/**
 * Wraps Android's built-in sensor fusion (TYPE_ROTATION_VECTOR — a fused
 * accelerometer + gyroscope + magnetometer estimate) to produce a device
 * orientation quaternion in the Earth (East-North-Up) frame, per spec §3.
 *
 * TYPE_ROTATION_VECTOR (not GAME_ROTATION_VECTOR) is used deliberately:
 * DVFC needs an absolute heading reference to relate device yaw to a real
 * bearing at all — even though, per spec §5, that heading is not trusted as
 * perfect ground truth on its own (see DeviceVehicleTransform's docs on
 * preferring GNSS course-over-ground where available).
 */
class SensorFusion(
    context: Context,
    private val onSample: (DeviceOrientationSample) -> Unit,
) : SensorEventListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private var lastGyro = FloatArray(3)

    val isAvailable: Boolean get() = rotationSensor != null

    fun start() {
        rotationSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        gyroSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> {
                lastGyro = event.values.copyOf()
            }
            Sensor.TYPE_ROTATION_VECTOR -> {
                val q = FloatArray(4)
                // Android returns Q = [w, x, y, z].
                SensorManager.getQuaternionFromVector(q, event.values)
                val quat = Quat(x = q[1], y = q[2], z = q[3], w = q[0]).normalized()
                onSample(
                    DeviceOrientationSample(
                        quaternion = quat,
                        angularVelocity = lastGyro,
                        timestampNs = event.timestamp,
                        accuracy = event.accuracy,
                    ),
                )
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
