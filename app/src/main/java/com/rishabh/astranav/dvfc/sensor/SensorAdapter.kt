package com.rishabh.astranav.dvfc.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.rishabh.astranav.dvfc.math.Quat
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * ASTRANAV Sensor Adapter
 *
 * RAW SENSORS
 *
 * Accelerometer
 * Gyroscope
 * Gravity
 * Rotation Vector
 *
 *        ↓
 *
 * Timestamp diagnostics
 * Unit validation
 * Per-sensor gap detection
 * Duplicate timestamp detection
 * Explicit 10 Hz processing timeline
 * Gravity handling
 *
 *        ↓
 *
 * DeviceSensorSample
 *
 *        ↓
 *
 * DVFC
 */
class SensorAdapter(
    context: Context,
    private val onSample: (DeviceSensorSample) -> Unit,
) : SensorEventListener {

    private val sensorManager =
        context.getSystemService(
            Context.SENSOR_SERVICE
        ) as SensorManager

    // =========================================================
    // SENSOR HARDWARE REFERENCES
    // =========================================================

    private val accelerometer =
        sensorManager.getDefaultSensor(
            Sensor.TYPE_ACCELEROMETER
        )

    private val gyroscopeSensor =
        sensorManager.getDefaultSensor(
            Sensor.TYPE_GYROSCOPE
        )

    private val gravitySensor =
        sensorManager.getDefaultSensor(
            Sensor.TYPE_GRAVITY
        )

    private val rotationVector =
        sensorManager.getDefaultSensor(
            Sensor.TYPE_ROTATION_VECTOR
        )

    // =========================================================
    // TARGET PROCESSING RATE
    // =========================================================

    companion object {

        private const val TARGET_HZ = 10.0f

        private const val TARGET_PERIOD_NS =
            100_000_000L

        /*
         * Maximum acceptable distance between the target
         * processing timestamp and the newest synchronized
         * sensor sample.
         */
        private const val SYNC_TOLERANCE_NS =
            50_000_000L

        /*
         * Gap is considered significant when a sensor's
         * timestamp interval is > 3 × its nominal period.
         */
        private const val GAP_MULTIPLIER = 3.0

        private const val GRAVITY_NOMINAL = 9.80665f

        private const val GRAVITY_TOLERANCE = 0.35f
    }

    // =========================================================
    // LATEST SENSOR VALUES
    // =========================================================

    private var acceleration: FloatArray? = null

    /*
     * Latest gyroscope measurement.
     *
     * IMPORTANT:
     * This is intentionally named differently from
     * gyroscopeSensor, which represents the Android Sensor
     * hardware object.
     */
    private var latestGyroscope: FloatArray? = null

    private var gravity: FloatArray? = null

    private var quaternion: Quat? = null

    private var rotationAccuracy =
        SensorManager.SENSOR_STATUS_UNRELIABLE

    // =========================================================
    // PER SENSOR TIMESTAMPS
    // =========================================================

    private var lastAccelTimestampNs = 0L

    private var lastGyroTimestampNs = 0L

    private var lastGravityTimestampNs = 0L

    private var lastRotationTimestampNs = 0L

    // =========================================================
    // PER SENSOR DIAGNOSTICS
    // =========================================================

    private var accelSampleCount = 0
    private var gyroSampleCount = 0
    private var gravitySampleCount = 0
    private var rotationSampleCount = 0

    private var accelGapCount = 0
    private var gyroGapCount = 0
    private var gravityGapCount = 0
    private var rotationGapCount = 0

    private var accelMaxGapNs = 0L
    private var gyroMaxGapNs = 0L
    private var gravityMaxGapNs = 0L
    private var rotationMaxGapNs = 0L

    private var duplicateTimestampCount = 0

    // =========================================================
    // RATE / JITTER
    // =========================================================

    private var intervalCount = 0

    private var intervalSumNs = 0.0

    private var jitterSumSquaredNs = 0.0

    // =========================================================
    // OUTPUT TIMELINE
    // =========================================================

    private var nextOutputTimestampNs = 0L

    private var outputSampleCount = 0

    // =========================================================
    // GRAVITY
    // =========================================================

    private var gravityMagnitude = 0f

    private var gravityStableCounter = 0

    private var gravityStable = false

    // =========================================================
    // AVAILABILITY
    // =========================================================

    val isAvailable: Boolean
        get() =
            accelerometer != null &&
                    gyroscopeSensor != null &&
                    rotationVector != null

    val hasGravitySensor: Boolean
        get() =
            gravitySensor != null

    // =========================================================
    // START
    // =========================================================

    fun start() {

        resetDiagnostics()

        accelerometer?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }

        gyroscopeSensor?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }

        gravitySensor?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }

        rotationVector?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }
    }

    // =========================================================
    // STOP
    // =========================================================

    fun stop() {

        sensorManager.unregisterListener(
            this
        )
    }

    // =========================================================
    // SENSOR CALLBACK
    // =========================================================

    override fun onSensorChanged(
        event: SensorEvent
    ) {

        if (event.values.isEmpty()) {
            return
        }

        if (!valuesAreFinite(event.values)) {
            return
        }

        when (event.sensor.type) {

            Sensor.TYPE_ACCELEROMETER -> {

                if (event.values.size >= 3) {

                    if (
                        !updateSensorTimestamp(
                            event.timestamp,
                            SensorType.ACCELEROMETER
                        )
                    ) {
                        return
                    }

                    acceleration =
                        event.values.copyOfRange(
                            0,
                            3
                        )
                }
            }

            Sensor.TYPE_GYROSCOPE -> {

                if (event.values.size >= 3) {

                    if (
                        !updateSensorTimestamp(
                            event.timestamp,
                            SensorType.GYROSCOPE
                        )
                    ) {
                        return
                    }

                    latestGyroscope =
                        event.values.copyOfRange(
                            0,
                            3
                        )
                }
            }

            Sensor.TYPE_GRAVITY -> {

                if (event.values.size >= 3) {

                    if (
                        !updateSensorTimestamp(
                            event.timestamp,
                            SensorType.GRAVITY
                        )
                    ) {
                        return
                    }

                    gravity =
                        event.values.copyOfRange(
                            0,
                            3
                        )

                    updateGravityDiagnostics(
                        gravity!!
                    )
                }
            }

            Sensor.TYPE_ROTATION_VECTOR -> {

                if (event.values.size >= 3) {

                    if (
                        !updateSensorTimestamp(
                            event.timestamp,
                            SensorType.ROTATION
                        )
                    ) {
                        return
                    }

                    quaternion =
                        rotationVectorToQuaternion(
                            event.values
                        )

                    rotationAccuracy =
                        event.accuracy
                }
            }
        }

        emitSynchronizedSample(
            event.timestamp
        )
    }

    // =========================================================
    // TIMESTAMP HANDLING
    // =========================================================

    private fun updateSensorTimestamp(
        timestampNs: Long,
        type: SensorType
    ): Boolean {

        val previous =
            when (type) {

                SensorType.ACCELEROMETER ->
                    lastAccelTimestampNs

                SensorType.GYROSCOPE ->
                    lastGyroTimestampNs

                SensorType.GRAVITY ->
                    lastGravityTimestampNs

                SensorType.ROTATION ->
                    lastRotationTimestampNs
            }

        /*
         * Duplicate / out-of-order timestamp.
         */
        if (
            previous != 0L &&
            timestampNs <= previous
        ) {

            duplicateTimestampCount++

            return false
        }

        if (previous != 0L) {

            val dt =
                timestampNs - previous

            val nominalPeriod =
                estimateNominalPeriodNs(
                    type
                )

            if (
                dt >
                nominalPeriod * GAP_MULTIPLIER
            ) {

                registerGap(
                    type,
                    dt
                )
            }

            /*
             * Global rate diagnostics are based only on
             * rotation-vector samples, because those are
             * the samples that drive the DVFC orientation
             * stream.
             */
            if (
                type ==
                SensorType.ROTATION
            ) {

                intervalCount++

                intervalSumNs +=
                    dt.toDouble()

                val meanSoFar =
                    intervalSumNs /
                            intervalCount

                val error =
                    dt.toDouble() -
                            meanSoFar

                jitterSumSquaredNs +=
                    error * error
            }
        }

        when (type) {

            SensorType.ACCELEROMETER -> {

                lastAccelTimestampNs =
                    timestampNs

                accelSampleCount++
            }

            SensorType.GYROSCOPE -> {

                lastGyroTimestampNs =
                    timestampNs

                gyroSampleCount++
            }

            SensorType.GRAVITY -> {

                lastGravityTimestampNs =
                    timestampNs

                gravitySampleCount++
            }

            SensorType.ROTATION -> {

                lastRotationTimestampNs =
                    timestampNs

                rotationSampleCount++
            }
        }

        return true
    }

    // =========================================================
    // NOMINAL SENSOR RATE
    // =========================================================

    private fun estimateNominalPeriodNs(
        type: SensorType
    ): Long {

        /*
         * Android SENSOR_DELAY_GAME typically delivers
         * considerably faster than the 10 Hz navigation
         * processing rate.
         *
         * We use a conservative 50 Hz diagnostic baseline.
         */
        return 20_000_000L
    }

    // =========================================================
    // GAP REGISTRATION
    // =========================================================

    private fun registerGap(
        type: SensorType,
        gapNs: Long
    ) {

        when (type) {

            SensorType.ACCELEROMETER -> {

                accelGapCount++

                accelMaxGapNs =
                    maxOf(
                        accelMaxGapNs,
                        gapNs
                    )
            }

            SensorType.GYROSCOPE -> {

                gyroGapCount++

                gyroMaxGapNs =
                    maxOf(
                        gyroMaxGapNs,
                        gapNs
                    )
            }

            SensorType.GRAVITY -> {

                gravityGapCount++

                gravityMaxGapNs =
                    maxOf(
                        gravityMaxGapNs,
                        gapNs
                    )
            }

            SensorType.ROTATION -> {

                rotationGapCount++

                rotationMaxGapNs =
                    maxOf(
                        rotationMaxGapNs,
                        gapNs
                    )
            }
        }
    }

    // =========================================================
    // SYNCHRONIZED OUTPUT
    // =========================================================

    private fun emitSynchronizedSample(
        eventTimestampNs: Long
    ) {

        val acc =
            acceleration ?: return

        val gyro =
            latestGyroscope ?: return

        val grav =
            gravity ?: return

        val quat =
            quaternion ?: return

        /*
         * We need all streams close enough to the same
         * processing timestamp.
         */
        if (
            !streamsSynchronized(
                eventTimestampNs
            )
        ) {
            return
        }

        if (
            nextOutputTimestampNs == 0L
        ) {

            nextOutputTimestampNs =
                eventTimestampNs
        }

        /*
         * Explicit 10 Hz processing timeline.
         *
         * We do not claim interpolation here.
         * The synchronized latest sample is emitted only
         * when all streams are within tolerance.
         */
        if (
            eventTimestampNs <
            nextOutputTimestampNs
        ) {
            return
        }

        val linearAcceleration =
            computeLinearAcceleration(
                acc,
                grav
            )

        val level =
            computeGravityLevel(
                grav
            )

        val sample =
            DeviceSensorSample(

                timestampNs =
                    nextOutputTimestampNs,

                acceleration =
                    acc.copyOf(),

                angularVelocity =
                    gyro.copyOf(),

                gravity =
                    grav.copyOf(),

                linearAcceleration =
                    linearAcceleration,

                quaternion =
                    quat,

                gravityMagnitude =
                    gravityMagnitude,

                gravityStable =
                    gravityStable,

                gravityLevelRollDeg =
                    level.first,

                gravityLevelPitchDeg =
                    level.second,

                estimatedSampleHz =
                    calculateSampleHz(),

                timestampJitterMs =
                    calculateJitterMs(),

                dataGapCount =
                    totalGapCount(),

                maxGapMs =
                    maxGapMs(),

                duplicateTimestampCount =
                    duplicateTimestampCount,

                resamplingActive =
                    true,

                resamplingRateHz =
                    TARGET_HZ,

                accelerationAvailable =
                    accelerometer != null,

                gyroscopeAvailable =
                    gyroscopeSensor != null,

                gravityAvailable =
                    gravitySensor != null,

                rotationVectorAvailable =
                    rotationVector != null,

                rotationAccuracy =
                    rotationAccuracy
            )

        onSample(
            sample
        )

        outputSampleCount++

        nextOutputTimestampNs +=
            TARGET_PERIOD_NS
    }

    // =========================================================
    // STREAM SYNCHRONIZATION
    // =========================================================

    private fun streamsSynchronized(
        timestampNs: Long
    ): Boolean {

        val timestamps =
            longArrayOf(
                lastAccelTimestampNs,
                lastGyroTimestampNs,
                lastGravityTimestampNs,
                lastRotationTimestampNs
            )

        if (
            timestamps.any {
                it == 0L
            }
        ) {
            return false
        }

        val newest =
            timestamps.maxOrNull()
                ?: return false

        val oldest =
            timestamps.minOrNull()
                ?: return false

        /*
         * All sensor streams must be close enough to one
         * another to form a valid synchronized sample.
         */
        if (
            newest - oldest >
            SYNC_TOLERANCE_NS
        ) {
            return false
        }

        /*
         * Also make sure the event itself has not moved
         * too far away from the synchronized sensor group.
         */
        return abs(
            timestampNs - newest
        ) <= SYNC_TOLERANCE_NS
    }

    // =========================================================
    // GRAVITY COMPENSATION
    // =========================================================

    private fun computeLinearAcceleration(
        acceleration: FloatArray,
        gravity: FloatArray
    ): FloatArray {

        return floatArrayOf(

            acceleration[0] -
                    gravity[0],

            acceleration[1] -
                    gravity[1],

            acceleration[2] -
                    gravity[2]
        )
    }

    // =========================================================
    // GRAVITY MONITOR
    // =========================================================

    private fun updateGravityDiagnostics(
        value: FloatArray
    ) {

        val x =
            value[0].toDouble()

        val y =
            value[1].toDouble()

        val z =
            value[2].toDouble()

        gravityMagnitude =
            sqrt(
                x * x +
                        y * y +
                        z * z
            ).toFloat()

        val error =
            abs(
                gravityMagnitude -
                        GRAVITY_NOMINAL
            )

        if (
            error <=
            GRAVITY_TOLERANCE
        ) {

            gravityStableCounter++

        } else {

            gravityStableCounter = 0
        }

        gravityStable =
            gravityStableCounter >= 5
    }

    // =========================================================
    // GRAVITY LEVELING
    // =========================================================

    /**
     * Estimates roll/pitch required to level the device
     * using the measured gravity vector.
     *
     * This is a leveling diagnostic.
     * It does not replace the Device → Vehicle yaw
     * calibration.
     */
    private fun computeGravityLevel(
        gravity: FloatArray
    ): Pair<Float, Float> {

        val gx =
            gravity[0].toDouble()

        val gy =
            gravity[1].toDouble()

        val gz =
            gravity[2].toDouble()

        val roll =
            Math.toDegrees(
                atan2(
                    gy,
                    gz
                )
            ).toFloat()

        val pitch =
            Math.toDegrees(
                atan2(
                    -gx,
                    sqrt(
                        gy * gy +
                                gz * gz
                    )
                )
            ).toFloat()

        return Pair(
            roll,
            pitch
        )
    }

    // =========================================================
    // ROTATION VECTOR → QUATERNION
    // =========================================================

    private fun rotationVectorToQuaternion(
        values: FloatArray
    ): Quat {

        val q =
            FloatArray(4)

        SensorManager.getQuaternionFromVector(
            q,
            values
        )

        return Quat(
            x = q[1],
            y = q[2],
            z = q[3],
            w = q[0]
        ).normalized()
    }

    // =========================================================
    // DIAGNOSTICS
    // =========================================================

    private fun calculateSampleHz(): Float {

        if (
            intervalCount <= 0
        ) {
            return 0f
        }

        val meanNs =
            intervalSumNs /
                    intervalCount

        if (
            meanNs <= 0.0
        ) {
            return 0f
        }

        return (
                1_000_000_000.0 /
                        meanNs
                ).toFloat()
    }

    private fun calculateJitterMs(): Float {

        if (
            intervalCount <= 1
        ) {
            return 0f
        }

        val variance =
            jitterSumSquaredNs /
                    intervalCount

        return (
                sqrt(
                    variance
                ) /
                        1_000_000.0
                ).toFloat()
    }

    private fun totalGapCount(): Int {

        return accelGapCount +
                gyroGapCount +
                gravityGapCount +
                rotationGapCount
    }

    private fun maxGapMs(): Float {

        val maxNs =
            maxOf(
                accelMaxGapNs,
                gyroMaxGapNs,
                gravityMaxGapNs,
                rotationMaxGapNs
            )

        return maxNs /
                1_000_000f
    }

    // =========================================================
    // VALIDATION
    // =========================================================

    private fun valuesAreFinite(
        values: FloatArray
    ): Boolean {

        return values.all {
            it.isFinite()
        }
    }

    // =========================================================
    // RESET
    // =========================================================

    private fun resetDiagnostics() {

        acceleration = null
        latestGyroscope = null
        gravity = null
        quaternion = null

        lastAccelTimestampNs = 0L
        lastGyroTimestampNs = 0L
        lastGravityTimestampNs = 0L
        lastRotationTimestampNs = 0L

        accelSampleCount = 0
        gyroSampleCount = 0
        gravitySampleCount = 0
        rotationSampleCount = 0

        accelGapCount = 0
        gyroGapCount = 0
        gravityGapCount = 0
        rotationGapCount = 0

        accelMaxGapNs = 0L
        gyroMaxGapNs = 0L
        gravityMaxGapNs = 0L
        rotationMaxGapNs = 0L

        duplicateTimestampCount = 0

        intervalCount = 0
        intervalSumNs = 0.0
        jitterSumSquaredNs = 0.0

        nextOutputTimestampNs = 0L
        outputSampleCount = 0

        gravityMagnitude = 0f
        gravityStableCounter = 0
        gravityStable = false

        rotationAccuracy =
            SensorManager.SENSOR_STATUS_UNRELIABLE
    }

    // =========================================================
    // TYPES
    // =========================================================

    private enum class SensorType {
        ACCELEROMETER,
        GYROSCOPE,
        GRAVITY,
        ROTATION
    }

    override fun onAccuracyChanged(
        sensor: Sensor?,
        accuracy: Int
    ) {

        if (
            sensor?.type ==
            Sensor.TYPE_ROTATION_VECTOR
        ) {

            rotationAccuracy =
                accuracy
        }
    }
}