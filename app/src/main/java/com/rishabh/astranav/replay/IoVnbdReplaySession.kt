package com.rishabh.astranav.replay

import android.content.Context
import android.net.Uri

import com.rishabh.astranav.dvfc.math.Quat
import com.rishabh.astranav.dvfc.sensor.DeviceSensorSample
import com.rishabh.astranav.navigation.AstraNavigationEngine
import com.rishabh.astranav.navigation.NavigationSolution

import kotlin.math.sqrt

private var replayStartTimeMs = 0L
/**
 * =============================================================
 * IO-VNBD REPLAY DATASET
 * =============================================================
 */
data class ReplayDataset(
    val id: String,
    val phone: List<IovnbdPhoneSample>,
    val vehicle: List<IovnbdVehicleSample>
) {

    val durationMs: Long
        get() {
            if (phone.size < 2) {
                return 0L
            }

            val firstTimeMs =
                phone.first().timeMs.toLong()

            val lastTimeMs =
                phone.last().timeMs.toLong()

            return lastTimeMs - firstTimeMs
        }
}

/**
 * =============================================================
 * ONE REPLAY FRAME
 * =============================================================
 */
data class ReplayFrame(
    val phone: IovnbdPhoneSample,
    val vehicle: IovnbdVehicleSample?,
    val solution: NavigationSolution,
    val metrics: ReplayMetricSnapshot?
)


/**
 * =============================================================
 * IO-VNBD REPLAY SESSION
 * =============================================================
 *
 * S-CSV
 *   ↓
 * IO-VNBD parser
 *   ↓
 * DeviceSensorSample
 *   ↓
 * SAME AstraNavigationEngine
 *   ↓
 * NavigationSolution
 *
 * V-CSV is used only as reference truth.
 */
class IoVnbdReplaySession(
    context: Context,
    private val engine: AstraNavigationEngine,
    private val frameMode: ReplayFrameMode =
        ReplayFrameMode.DATASET_FRAME
) {

    private val parser =
        IovnbdCsvParser(context)

    val metrics =
        ReplayMetricsEngine()


    private var dataset:
            ReplayDataset? = null

    private var index =
        -1

    private var truthIndex =
        0

    private var metricIndex =
        -1


    fun frameMode(): ReplayFrameMode {
        return frameMode
    }

    // =========================================================
    // LOAD
    // =========================================================

    fun load(
        phoneUri: Uri,
        vehicleUri: Uri?,
        id: String = "IO-VNBD"
    ): ReplayDataset {

        val phone =
            parser.readPhone(
                phoneUri
            )


        require(
            phone.isNotEmpty()
        ) {
            "No valid smartphone samples found"
        }


        val vehicle =
            vehicleUri?.let {
                parser.readVehicle(it)
            } ?: emptyList()


        dataset =
            ReplayDataset(
                id = id,
                phone = phone,
                vehicle = vehicle
            )


        reset()


        return dataset!!
    }


    // =========================================================
    // RESET
    // =========================================================

    fun reset() {

        index =
            -1

        truthIndex =
            0

        metricIndex =
            -1

        metrics.reset()

        engine.resetForReplay(
            frameMode = frameMode
        )

        replayStartTimeMs =
            dataset
                ?.phone
                ?.firstOrNull()
                ?.timeMs
                ?.toLong()
                ?: 0L
    }


    // =========================================================
    // SAMPLE AT PLAYBACK PROGRESS
    // =========================================================

    fun sampleAt(
        progress: Float
    ): ReplayFrame? {

        val d =
            dataset
                ?: return null


        if (d.phone.isEmpty()) {
            return null
        }


        val startTime =
            d.phone
                .first()
                .timeMs


        val targetTime =
            startTime +
                    (
                            d.durationMs *
                                    progress
                                        .coerceIn(
                                            0f,
                                            1f
                                        )
                            ).toLong()


        val targetIndex =
            d.phone
                .indexOfLast {
                    it.timeMs <= targetTime
                }
                .coerceAtLeast(0)


        /*
         * If the user dragged backwards,
         * rebuild the navigation state from the beginning.
         */
        if (
            targetIndex < index
        ) {

            reset()
        }


        /*
         * Process every phone sample between
         * the current replay position and target.
         */
        while (
            index < targetIndex
        ) {

            index++

            engine.processReplaySample(
                toDeviceSample(
                    d.phone[index]
                )
            )
        }


        /*
         * Initial sample.
         */
        if (index < 0) {

            index = 0

            engine.processReplaySample(
                toDeviceSample(
                    d.phone[0]
                )
            )
        }


        val phoneSample =
            d.phone[index]


        // -----------------------------------------------------
        // FIND MATCHING V-CSV TRUTH SAMPLE
        // -----------------------------------------------------

        while (
            truthIndex + 1 < d.vehicle.size &&
            relativeVehicleTimeMs(
                d.vehicle[truthIndex + 1]
            ) <=
            phoneSample.timeMs -
            startTime
        ) {

            truthIndex++
        }


        val truth =
            d.vehicle.getOrNull(
                truthIndex
            )


        // -----------------------------------------------------
        // CURRENT NAVIGATION SOLUTION
        // -----------------------------------------------------

        val solution =
            engine.state()


        // -----------------------------------------------------
        // METRICS
        // -----------------------------------------------------

        val snapshot =
            if (
                truth != null &&
                index != metricIndex
            ) {

                metricIndex =
                    index


                metrics.update(

                    north =
                        solution
                            .position
                            .x,

                    east =
                        solution
                            .position
                            .y,

                    speedMps =
                        solution
                            .speedMps,

                    heading =
                        solution
                            .headingDegrees,

                    truth =
                        truth
                )

            } else {
                null
            }


        return ReplayFrame(

            phone =
                phoneSample,

            vehicle =
                truth,

            solution =
                solution,

            metrics =
                snapshot
        )
    }


    // =========================================================
    // VEHICLE RELATIVE TIME
    // =========================================================

    private fun relativeVehicleTimeMs(
        sample: IovnbdVehicleSample
    ): Long {

        val d =
            dataset
                ?: return 0L


        if (
            d.vehicle.isEmpty()
        ) {
            return 0L
        }


        return (
                (
                        sample.timeSeconds -
                                d.vehicle
                                    .first()
                                    .timeSeconds
                        ) *
                        1000.0
                ).toLong()
    }


    // =========================================================
    // IO-VNBD PHONE → DEVICE SENSOR SAMPLE
    // =========================================================

    private fun toDeviceSample(
        sample: IovnbdPhoneSample
    ): DeviceSensorSample {

        val gravity =
            floatArrayOf(

                sample.gravityX
                    .toFloat(),

                sample.gravityY
                    .toFloat(),

                sample.gravityZ
                    .toFloat()
            )


        val acceleration =
            floatArrayOf(

                sample.accelX
                    .toFloat(),

                sample.accelY
                    .toFloat(),

                sample.accelZ
                    .toFloat()
            )


        val linearAcceleration =
            floatArrayOf(

                (
                        sample.accelX -
                                sample.gravityX
                        ).toFloat(),

                (
                        sample.accelY -
                                sample.gravityY
                        ).toFloat(),

                (
                        sample.accelZ -
                                sample.gravityZ
                        ).toFloat()
            )


        val gravityMagnitude =
            sqrt(

                sample.gravityX *
                        sample.gravityX +

                        sample.gravityY *
                        sample.gravityY +

                        sample.gravityZ *
                        sample.gravityZ
            ).toFloat()


        return DeviceSensorSample(

            timestampNs =
                (
                        sample.timeMs.toLong() -
                                replayStartTimeMs
                        ) * 1_000_000L,


            acceleration =
                acceleration,


            angularVelocity =
                floatArrayOf(

                    sample.gyroYaw
                        .toFloat(),

                    sample.gyroPitch
                        .toFloat(),

                    sample.gyroRoll
                        .toFloat()
                ),


            gravity =
                gravity,


            linearAcceleration =
                linearAcceleration,


            quaternion =
                Quat.IDENTITY,


            gravityMagnitude =
                gravityMagnitude,


            gravityStable =
                true,


            gravityLevelRollDeg =
                sample.orientationRollDeg
                    ?.toFloat()
                    ?: 0f,


            gravityLevelPitchDeg =
                sample.orientationPitchDeg
                    ?.toFloat()
                    ?: 0f,


            estimatedSampleHz =
                10f,


            timestampJitterMs =
                0f,


            dataGapCount =
                0,


            maxGapMs =
                0f,


            duplicateTimestampCount =
                0,


            resamplingActive =
                false,


            resamplingRateHz =
                10f,


            accelerationAvailable =
                true,


            gyroscopeAvailable =
                true,


            gravityAvailable =
                true,


            rotationVectorAvailable =
                false,


            rotationAccuracy =
                0
        )
    }


    // =========================================================
    // DATASET ACCESS
    // =========================================================

    fun dataset():
            ReplayDataset? {

        return dataset
    }
}