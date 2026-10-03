package com.rishabh.astranav.replay

import com.rishabh.astranav.sensor.ImuSample
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.util.Locale

class IovnbdCsvParser {

    fun parse(file: File): IovnbdTrip {
        require(file.exists()) {
            "IO-VNBD file does not exist: ${file.absolutePath}"
        }

        BufferedReader(FileReader(file)).use { reader ->

            val header = reader.readLine()
                ?: error("IO-VNBD CSV is empty: ${file.name}")

            val columns = header
                .split(',')
                .map { normalizeColumn(it) }

            val index = columns.withIndex().associate { it.value to it.index }

            val timeIndex = findRequired(
                index,
                "time", "timestamp", "timestamp_s", "time_s"
            )

            val axIndex = findRequired(
                index,
                "acc_x", "accel_x", "accelerometer_x"
            )

            val ayIndex = findRequired(
                index,
                "acc_y", "accel_y", "accelerometer_y"
            )

            val azIndex = findRequired(
                index,
                "acc_z", "accel_z", "accelerometer_z"
            )

            val gxIndex = findRequired(
                index,
                "gyro_yaw", "gyro_z", "gyroscope_z", "gyro_z"
            )

            val gyIndex = findRequired(
                index,
                "gyro_pitch", "gyro_x", "gyroscope_x", "gyro_x"
            )

            val gzIndex = findRequired(
                index,
                "gyro_roll", "gyro_y", "gyroscope_y", "gyro_y"
            )

            val samples = ArrayList<ImuSample>()

            reader.lineSequence().forEachIndexed { lineNumber, line ->

                if (line.isBlank()) return@forEachIndexed

                val values = line.split(',')

                try {
                    val time = values[timeIndex].toDouble()

                    val timestampNanos = timeToNanos(time)

                    val ax = values[axIndex].toDouble()
                    val ay = values[ayIndex].toDouble()
                    val az = values[azIndex].toDouble()

                    val gx = values[gxIndex].toDouble()
                    val gy = values[gyIndex].toDouble()
                    val gz = values[gzIndex].toDouble()

                    if (
                        !timestampNanos.isFiniteTimestamp() ||
                        !ax.isFinite() ||
                        !ay.isFinite() ||
                        !az.isFinite() ||
                        !gx.isFinite() ||
                        !gy.isFinite() ||
                        !gz.isFinite()
                    ) {
                        return@forEachIndexed
                    }

                    samples += ImuSample(
                        timestampNanos = timestampNanos,
                        accelX = ax,
                        accelY = ay,
                        accelZ = az,
                        gyroX = gx,
                        gyroY = gy,
                        gyroZ = gz
                    )

                } catch (_: Exception) {
                    // Ignore malformed rows.
                }
            }

            require(samples.size >= 2) {
                "Not enough valid IMU samples in ${file.name}"
            }

            val sortedSamples = samples
                .distinctBy { it.timestampNanos }
                .sortedBy { it.timestampNanos }

            val durationSeconds =
                (sortedSamples.last().timestampNanos -
                        sortedSamples.first().timestampNanos) * 1e-9

            val rateHz =
                if (durationSeconds > 0.0) {
                    (sortedSamples.size - 1) / durationSeconds
                } else {
                    0.0
                }

            return IovnbdTrip(
                tripId = file.nameWithoutExtension,
                samples = sortedSamples,
                sourceFile = file.absolutePath,
                sampleRateHz = rateHz
            )
        }
    }

    private fun normalizeColumn(value: String): String {
        return value
            .trim()
            .lowercase(Locale.US)
            .replace("\"", "")
            .replace(" ", "_")
            .replace("-", "_")
    }

    private fun findRequired(
        index: Map<String, Int>,
        vararg candidates: String
    ): Int {
        for (candidate in candidates) {
            index[candidate]?.let { return it }
        }

        error(
            "Required IO-VNBD column missing. " +
                    "Expected one of: ${candidates.joinToString()}"
        )
    }

    private fun timeToNanos(time: Double): Long {
        return when {
            time > 1e15 -> time.toLong()
            time > 1e12 -> (time * 1e3).toLong()
            time > 1e9 -> (time * 1e9).toLong()
            else -> (time * 1e9).toLong()
        }
    }

    private fun Long.isFiniteTimestamp(): Boolean {
        return this > 0L
    }
}