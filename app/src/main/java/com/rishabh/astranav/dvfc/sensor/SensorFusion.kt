package com.rishabh.astranav.dvfc.sensor

import android.content.Context
import com.rishabh.astranav.dvfc.math.Quat

data class DeviceOrientationSample(

    val quaternion: Quat,

    val angularVelocity: FloatArray,

    val timestampNs: Long,

    val accuracy: Int,

    // ---------------------------------------------------------
    // SENSOR ADAPTER
    // ---------------------------------------------------------

    val acceleration: FloatArray =
        FloatArray(3),

    val gravity: FloatArray =
        FloatArray(3),

    val linearAcceleration: FloatArray =
        FloatArray(3),

    // ---------------------------------------------------------
    // GRAVITY
    // ---------------------------------------------------------

    val gravityMagnitude: Float =
        0f,

    val gravityStable: Boolean =
        false,

    val gravityLevelRollDeg: Float =
        0f,

    val gravityLevelPitchDeg: Float =
        0f,

    // ---------------------------------------------------------
    // TIMESTAMP / RATE
    // ---------------------------------------------------------

    val estimatedSampleHz: Float =
        0f,

    val timestampJitterMs: Float =
        0f,

    val dataGapCount: Int =
        0,

    val maxGapMs: Float =
        0f,

    val duplicateTimestampCount: Int =
        0,

    // ---------------------------------------------------------
    // RESAMPLING
    // ---------------------------------------------------------

    val resamplingActive: Boolean =
        false,

    val resamplingRateHz: Float =
        0f,

    // ---------------------------------------------------------
    // AVAILABILITY
    // ---------------------------------------------------------

    val accelerationAvailable: Boolean =
        false,

    val gyroscopeAvailable: Boolean =
        false,

    val gravityAvailable: Boolean =
        false,

    val rotationVectorAvailable: Boolean =
        false
)

/**
 * Compatibility facade.
 *
 * Raw Android sensors are handled exclusively by
 * SensorAdapter.
 */
class SensorFusion(
    context: Context,
    private val onSample:
        (DeviceOrientationSample) -> Unit
) {

    private val adapter =
        SensorAdapter(context) { sample ->

            onSample(

                DeviceOrientationSample(

                    quaternion =
                        sample.quaternion,

                    angularVelocity =
                        sample.angularVelocity
                            .copyOf(),

                    timestampNs =
                        sample.timestampNs,

                    accuracy =
                        sample.rotationAccuracy,

                    acceleration =
                        sample.acceleration
                            .copyOf(),

                    gravity =
                        sample.gravity
                            .copyOf(),

                    linearAcceleration =
                        sample.linearAcceleration
                            .copyOf(),

                    gravityMagnitude =
                        sample.gravityMagnitude,

                    gravityStable =
                        sample.gravityStable,

                    gravityLevelRollDeg =
                        sample.gravityLevelRollDeg,

                    gravityLevelPitchDeg =
                        sample.gravityLevelPitchDeg,

                    estimatedSampleHz =
                        sample.estimatedSampleHz,

                    timestampJitterMs =
                        sample.timestampJitterMs,

                    dataGapCount =
                        sample.dataGapCount,

                    maxGapMs =
                        sample.maxGapMs,

                    duplicateTimestampCount =
                        sample.duplicateTimestampCount,

                    resamplingActive =
                        sample.resamplingActive,

                    resamplingRateHz =
                        sample.resamplingRateHz,

                    accelerationAvailable =
                        sample.accelerationAvailable,

                    gyroscopeAvailable =
                        sample.gyroscopeAvailable,

                    gravityAvailable =
                        sample.gravityAvailable,

                    rotationVectorAvailable =
                        sample.rotationVectorAvailable
                )
            )
        }

    val isAvailable: Boolean
        get() =
            adapter.isAvailable

    fun start() {
        adapter.start()
    }

    fun stop() {
        adapter.stop()
    }
}