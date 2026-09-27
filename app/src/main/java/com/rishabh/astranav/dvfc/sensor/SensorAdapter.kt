package com.rishabh.astranav.dvfc.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.rishabh.astranav.dvfc.math.Quat
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * SENSOR ADAPTER
 *
 * Responsibility:
 *
 * Accelerometer
 * Gyroscope
 * Gravity
 * Rotation Vector
 *
 *        ↓
 *
 * Timestamp synchronization
 * Unit validation
 * Gap detection
 * 10 Hz resampling
 * Gravity monitoring
 *
 *        ↓
 *
 * DeviceSensorSample
 *
 *        ↓
 *
 * DVFC / downstream estimator
 */
class SensorAdapter(
    context: Context,
    private val onSample: (DeviceSensorSample) -> Unit,
) : SensorEventListener {

    private val sensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    private val accelerometer =
        sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)

    private val gyroscope =
        sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private val gravitySensor =
        sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)

    private val rotationVector =
        sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    // ---------------------------------------------------------
    // TARGET RESAMPLING
    // ---------------------------------------------------------

    /**
     * ASTRANAV adapter output rate.
     *
     * 10 Hz = one synchronized sample every 100 ms.
     */
    private val targetPeriodNs = 100_000_000L

    private var nextOutputTimestampNs = 0L

    // ---------------------------------------------------------
    // LATEST SENSOR VALUES
    // ---------------------------------------------------------

    private var latestAcceleration: FloatArray? = null
    private var latestGyroscope: FloatArray? = null
    private var latestGravity: FloatArray? = null
    private var latestQuaternion: Quat? = null

    private var latestRotationAccuracy =
        SensorManager.SENSOR_STATUS_UNRELIABLE

    // ---------------------------------------------------------
    // TIMESTAMP DIAGNOSTICS
    // ---------------------------------------------------------

    private var previousSensorTimestampNs = 0L

    private var sampleCount = 0

    private var intervalCount = 0

    private var intervalSumNs = 0L

    private var jitterSumNs = 0.0

    private var gapCount = 0

    private var maxGapNs = 0L

    // ---------------------------------------------------------
    // GRAVITY
    // ---------------------------------------------------------

    private var gravityMagnitude = 0f

    private var gravityStable = false

    private var gravityStableCounter = 0

    // ---------------------------------------------------------
    // AVAILABILITY
    // ---------------------------------------------------------

    val isAvailable: Boolean
        get() = rotationVector != null &&
                gyroscope != null &&
                accelerometer != null

    val hasGravitySensor: Boolean
        get() = gravitySensor != null

    // ---------------------------------------------------------
    // START
    // ---------------------------------------------------------

    fun start() {

        nextOutputTimestampNs = 0L

        previousSensorTimestampNs = 0L

        sampleCount = 0
        intervalCount = 0
        intervalSumNs = 0L
        jitterSumNs = 0.0

        gapCount = 0
        maxGapNs = 0L

        gravityStableCounter = 0

        accelerometer?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME,
            )
        }

        gyroscope?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME,
            )
        }

        gravitySensor?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME,
            )
        }

        rotationVector?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME,
            )
        }
    }

    // ---------------------------------------------------------
    // STOP
    // ---------------------------------------------------------

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    // ---------------------------------------------------------
    // SENSOR CALLBACK
    // ---------------------------------------------------------

    override fun onSensorChanged(event: SensorEvent) {

        if (event.values.isEmpty()) {
            return
        }

        if (!isFinite(event.values)) {
            return
        }

        // -----------------------------------------------------
        // TIMESTAMP DIAGNOSTICS
        // -----------------------------------------------------

        updateTimestampDiagnostics(event.timestamp)

        // -----------------------------------------------------
        // SENSOR DATA
        // -----------------------------------------------------

        when (event.sensor.type) {

            Sensor.TYPE_ACCELEROMETER -> {

                if (event.values.size >= 3) {

                    latestAcceleration = floatArrayOf(
                        event.values[0],
                        event.values[1],
                        event.values[2],
                    )
                }
            }

            Sensor.TYPE_GYROSCOPE -> {

                if (event.values.size >= 3) {

                    latestGyroscope = floatArrayOf(
                        event.values[0],
                        event.values[1],
                        event.values[2],
                    )
                }
            }

            Sensor.TYPE_GRAVITY -> {

                if (event.values.size >= 3) {

                    latestGravity = floatArrayOf(
                        event.values[0],
                        event.values[1],
                        event.values[2],
                    )

                    updateGravityMonitor(latestGravity!!)
                }
            }

            Sensor.TYPE_ROTATION_VECTOR -> {

                if (event.values.size >= 3) {

                    latestQuaternion =
                        rotationVectorToQuaternion(event.values)

                    latestRotationAccuracy =
                        event.accuracy
                }
            }
        }

        // -----------------------------------------------------
        // SYNCHRONIZED OUTPUT
        // -----------------------------------------------------

        emitResampledSamples(event.timestamp)
    }

    // ---------------------------------------------------------
    // TIMESTAMP SYNCHRONIZATION / DIAGNOSTICS
    // ---------------------------------------------------------

    private fun updateTimestampDiagnostics(timestampNs: Long) {

        if (previousSensorTimestampNs != 0L) {

            val intervalNs =
                timestampNs - previousSensorTimestampNs

            // Ignore impossible timestamp ordering.
            if (intervalNs <= 0L) {
                return
            }

            intervalCount++

            intervalSumNs += intervalNs

            val expectedIntervalNs =
                100_000_000L

            val jitterNs =
                abs(intervalNs - expectedIntervalNs)

            jitterSumNs += jitterNs.toDouble()

            /*
             * A gap larger than 2.5x our target 10 Hz period
             * is considered a data gap.
             */
            if (intervalNs > expectedIntervalNs * 2.5) {

                gapCount++

                if (intervalNs > maxGapNs) {
                    maxGapNs = intervalNs
                }
            }
        }

        previousSensorTimestampNs = timestampNs
    }

    // ---------------------------------------------------------
    // RESAMPLING
    // ---------------------------------------------------------

    private fun emitResampledSamples(timestampNs: Long) {

        /*
         * We cannot produce a synchronized sample until the
         * required sensor streams have supplied data.
         */
        val acceleration = latestAcceleration ?: return
        val gyro = latestGyroscope ?: return
        val gravity = latestGravity ?: return
        val quaternion = latestQuaternion ?: return

        if (nextOutputTimestampNs == 0L) {

            nextOutputTimestampNs = timestampNs
            return
        }

        /*
         * Emit samples at a deterministic 10 Hz timeline.
         *
         * Current implementation uses the latest validated
         * sample for each sensor at the output timestamp.
         *
         * This gives us a stable common timeline. Later the
         * same interface can be upgraded to linear interpolation
         * without changing DVFC.
         */
        while (timestampNs >= nextOutputTimestampNs) {

            val sample = DeviceSensorSample(

                timestampNs = nextOutputTimestampNs,

                acceleration = acceleration.copyOf(),

                angularVelocity = gyro.copyOf(),

                gravity = gravity.copyOf(),

                quaternion = quaternion,

                rotationAccuracy = latestRotationAccuracy,

                accelerationAvailable =
                    accelerometer != null,

                gyroscopeAvailable =
                    gyroscope != null,

                gravityAvailable =
                    gravitySensor != null,

                rotationVectorAvailable =
                    rotationVector != null,

                estimatedSampleHz =
                    calculateSampleHz(),

                timestampJitterMs =
                    calculateTimestampJitterMs(),

                dataGapCount =
                    gapCount,

                maxGapMs =
                    maxGapNs / 1_000_000f,

                gravityMagnitude =
                    gravityMagnitude,

                gravityStable =
                    gravityStable,
            )

            onSample(sample)

            sampleCount++

            nextOutputTimestampNs += targetPeriodNs

            /*
             * Prevent an enormous catch-up loop if the phone
             * was paused or sensor delivery was interrupted.
             */
            if (nextOutputTimestampNs < timestampNs - targetPeriodNs * 10) {
                nextOutputTimestampNs =
                    timestampNs + targetPeriodNs

                break
            }
        }
    }

    // ---------------------------------------------------------
    // GRAVITY MONITOR
    // ---------------------------------------------------------

    private fun updateGravityMonitor(gravity: FloatArray) {

        val x = gravity[0].toDouble()
        val y = gravity[1].toDouble()
        val z = gravity[2].toDouble()

        gravityMagnitude =
            sqrt(
                x * x +
                        y * y +
                        z * z
            ).toFloat()

        /*
         * Earth's gravity should be approximately
         * 9.81 m/s².
         */
        val magnitudeError =
            abs(gravityMagnitude - 9.81f)

        if (magnitudeError <= 0.35f) {

            gravityStableCounter++

        } else {

            gravityStableCounter = 0
        }

        /*
         * Require several consecutive valid samples before
         * calling gravity stable.
         */
        gravityStable =
            gravityStableCounter >= 5
    }

    // ---------------------------------------------------------
    // ROTATION VECTOR → QUATERNION
    // ---------------------------------------------------------

    private fun rotationVectorToQuaternion(
        values: FloatArray,
    ): Quat {

        val q = FloatArray(4)

        SensorManager.getQuaternionFromVector(
            q,
            values,
        )

        /*
         * Android:
         *
         * q[0] = w
         * q[1] = x
         * q[2] = y
         * q[3] = z
         */
        return Quat(
            x = q[1],
            y = q[2],
            z = q[3],
            w = q[0],
        ).normalized()
    }

    // ---------------------------------------------------------
    // DIAGNOSTICS
    // ---------------------------------------------------------

    private fun calculateSampleHz(): Float {

        if (intervalCount == 0) {
            return 0f
        }

        val averageIntervalNs =
            intervalSumNs.toDouble() / intervalCount

        if (averageIntervalNs <= 0.0) {
            return 0f
        }

        return (
                1_000_000_000.0 /
                        averageIntervalNs
                ).toFloat()
    }

    private fun calculateTimestampJitterMs(): Float {

        if (intervalCount == 0) {
            return 0f
        }

        return (
                jitterSumNs /
                        intervalCount /
                        1_000_000.0
                ).toFloat()
    }

    // ---------------------------------------------------------
    // FINITE CHECK
    // ---------------------------------------------------------

    private fun isFinite(values: FloatArray): Boolean {

        for (value in values) {

            if (!value.isFinite()) {
                return false
            }
        }

        return true
    }

    override fun onAccuracyChanged(
        sensor: Sensor?,
        accuracy: Int,
    ) {

        if (sensor?.type == Sensor.TYPE_ROTATION_VECTOR) {

            latestRotationAccuracy = accuracy
        }
    }
}