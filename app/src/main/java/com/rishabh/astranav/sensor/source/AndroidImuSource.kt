package com.rishabh.astranav.sensor.source

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.rishabh.astranav.sensor.ImuSample
import com.rishabh.astranav.sensor.SensorSourceType

/**
 * Android phone IMU source.
 *
 * This class ONLY reads Android sensors.
 *
 * It does not:
 *  - resample
 *  - perform DVFC
 *  - run ML
 *  - update ESKF
 *
 * SensorAdapter owns timing normalization.
 */
class AndroidImuSource(
    context: Context
) : SensorSource, SensorEventListener {

    private val sensorManager =
        context.applicationContext
            .getSystemService(Context.SENSOR_SERVICE) as SensorManager

    override val sourceType =
        SensorSourceType.SMARTPHONE

    override var isConnected: Boolean = false
        private set

    private var listener: SensorSource.Listener? = null

    private var latestGyro =
        doubleArrayOf(0.0, 0.0, 0.0)

    private var latestGravity =
        doubleArrayOf(0.0, 0.0, 9.81)

    private var hasAccelerometer = false
    private var hasGyroscope = false
    private var hasGravity = false

    override fun start(listener: SensorSource.Listener) {

        this.listener = listener

        val accelerometer =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_ACCELEROMETER
            )

        val gyroscope =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_GYROSCOPE
            )

        val gravity =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_GRAVITY
            )

        hasAccelerometer = accelerometer != null
        hasGyroscope = gyroscope != null
        hasGravity = gravity != null

        if (!hasAccelerometer || !hasGyroscope) {
            isConnected = false

            listener.onConnectionChanged(false)
            listener.onError(
                "Required accelerometer/gyroscope unavailable"
            )

            return
        }

        sensorManager.registerListener(
            this,
            accelerometer,
            SensorManager.SENSOR_DELAY_GAME
        )

        sensorManager.registerListener(
            this,
            gyroscope,
            SensorManager.SENSOR_DELAY_GAME
        )

        gravity?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }

        isConnected = true
        listener.onConnectionChanged(true)
    }

    override fun stop() {

        sensorManager.unregisterListener(this)

        listener = null
        isConnected = false

        latestGyro =
            doubleArrayOf(0.0, 0.0, 0.0)

        latestGravity =
            doubleArrayOf(0.0, 0.0, 9.81)
    }

    override fun onSensorChanged(event: SensorEvent) {

        when (event.sensor.type) {

            Sensor.TYPE_GYROSCOPE -> {

                latestGyro =
                    doubleArrayOf(
                        event.values[0].toDouble(),
                        event.values[1].toDouble(),
                        event.values[2].toDouble()
                    )
            }

            Sensor.TYPE_GRAVITY -> {

                latestGravity =
                    doubleArrayOf(
                        event.values[0].toDouble(),
                        event.values[1].toDouble(),
                        event.values[2].toDouble()
                    )
            }

            Sensor.TYPE_ACCELEROMETER -> {

                val sample =
                    ImuSample(
                        timestampNanos = event.timestamp,

                        accelX = event.values[0].toDouble(),
                        accelY = event.values[1].toDouble(),
                        accelZ = event.values[2].toDouble(),

                        gyroX = latestGyro[0],
                        gyroY = latestGyro[1],
                        gyroZ = latestGyro[2],

                        gravityX = latestGravity[0],
                        gravityY = latestGravity[1],
                        gravityZ = latestGravity[2]
                    )

                listener?.onImuSample(sample)
            }
        }
    }

    override fun onAccuracyChanged(
        sensor: Sensor?,
        accuracy: Int
    ) = Unit
}