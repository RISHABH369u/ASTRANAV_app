package com.rishabh.astranav.dvfc.sensor

import android.content.Context
import com.rishabh.astranav.dvfc.math.Quat

/**
 * Backward-compatible orientation interface for DVFC.
 *
 * The actual raw sensor processing is now performed by
 * SensorAdapter.
 */
data class DeviceOrientationSample(
    val quaternion: Quat,

    val angularVelocity: FloatArray,

    val timestampNs: Long,

    val accuracy: Int,

    // ---------------------------------------------------------
    // SENSOR ADAPTER DATA
    // ---------------------------------------------------------

    val acceleration: FloatArray = FloatArray(3),

    val gravity: FloatArray = FloatArray(3),

    val estimatedSampleHz: Float = 0f,

    val timestampJitterMs: Float = 0f,

    val dataGapCount: Int = 0,

    val maxGapMs: Float = 0f,

    val gravityMagnitude: Float = 0f,

    val gravityStable: Boolean = false,

    val accelerationAvailable: Boolean = false,

    val gyroscopeAvailable: Boolean = false,

    val gravityAvailable: Boolean = false,

    val rotationVectorAvailable: Boolean = false,
)

/**
 * Compatibility facade.
 *
 * DVFCController can continue using SensorFusion while the
 * implementation underneath has been upgraded to SensorAdapter.
 */
class SensorFusion(
    context: Context,
    private val onSample: (DeviceOrientationSample) -> Unit,
) {

    private val adapter = SensorAdapter(context) { sample ->

        onSample(
            DeviceOrientationSample(

                quaternion =
                    sample.quaternion,

                angularVelocity =
                    sample.angularVelocity.copyOf(),

                timestampNs =
                    sample.timestampNs,

                accuracy =
                    sample.rotationAccuracy,

                acceleration =
                    sample.acceleration.copyOf(),

                gravity =
                    sample.gravity.copyOf(),

                estimatedSampleHz =
                    sample.estimatedSampleHz,

                timestampJitterMs =
                    sample.timestampJitterMs,

                dataGapCount =
                    sample.dataGapCount,

                maxGapMs =
                    sample.maxGapMs,

                gravityMagnitude =
                    sample.gravityMagnitude,

                gravityStable =
                    sample.gravityStable,

                accelerationAvailable =
                    sample.accelerationAvailable,

                gyroscopeAvailable =
                    sample.gyroscopeAvailable,

                gravityAvailable =
                    sample.gravityAvailable,

                rotationVectorAvailable =
                    sample.rotationVectorAvailable,
            ),
        )
    }

    val isAvailable: Boolean
        get() = adapter.isAvailable

    fun start() {
        adapter.start()
    }

    fun stop() {
        adapter.stop()
    }
}