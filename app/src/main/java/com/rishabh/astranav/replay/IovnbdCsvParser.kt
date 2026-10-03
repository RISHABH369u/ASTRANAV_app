package com.rishabh.astranav.replay

import android.content.Context
import android.net.Uri
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.charset.StandardCharsets
import java.util.Locale

/**
 * =============================================================
 * IO-VNBD CSV PARSER
 * =============================================================
 *
 * Supports:
 *
 * S-*.csv
 * V-*.csv
 *
 * The parser is intentionally tolerant of:
 *
 * - UTF-8 / replacement-character unit corruption
 * - m/s² vs m/s2 vs m/s�
 * - spaces
 * - underscores
 * - hyphens
 * - degree / micro symbols
 * - optional unit suffixes
 *
 * The parser extracts only the fields required by ASTRANAV.
 */
class IovnbdCsvParser(
    private val context: Context
) {

    // =========================================================
    // PHONE / S-CSV
    // =========================================================

    fun readPhone(
        uri: Uri
    ): List<IovnbdPhoneSample> {

        return openReader(uri).use { reader ->

            val header =
                reader.readLine()
                    ?: error("Phone CSV is empty")

            val columns =
                parseCsvLine(header)
                    .map(::normalizeHeader)

            val index =
                columns
                    .withIndex()
                    .associate { it.value to it.index }

            // -------------------------------------------------
            // REQUIRED PHONE COLUMNS
            // -------------------------------------------------

            val time =
                required(
                    index,
                    "time_since_start_ms",
                    "time_since_start",
                    "timestamp_ms",
                    "time_ms"
                )

            val ax =
                required(
                    index,
                    "accelerometer_x",
                    "accel_x",
                    "acc_x"
                )

            val ay =
                required(
                    index,
                    "accelerometer_y",
                    "accel_y",
                    "acc_y"
                )

            val az =
                required(
                    index,
                    "accelerometer_z",
                    "accel_z",
                    "acc_z"
                )

            val gyroYaw =
                required(
                    index,
                    "gyroscope_yaw",
                    "gyro_yaw",
                    "gyroscope_yaw"
                )

            val gyroPitch =
                required(
                    index,
                    "gyroscope_pitch",
                    "gyro_pitch",
                    "gyroscope_pitch"
                )

            val gyroRoll =
                required(
                    index,
                    "gyroscope_roll",
                    "gyro_roll",
                    "gyroscope_roll"
                )

            // -------------------------------------------------
            // OPTIONAL GRAVITY
            // -------------------------------------------------

            val gravX =
                optional(
                    index,
                    "gravity_x",
                    "grav_x"
                )

            val gravY =
                optional(
                    index,
                    "gravity_y",
                    "grav_y"
                )

            val gravZ =
                optional(
                    index,
                    "gravity_z",
                    "grav_z"
                )

            // -------------------------------------------------
            // OPTIONAL GPS
            // -------------------------------------------------

            val latitude =
                optional(
                    index,
                    "gps_latitude",
                    "latitude"
                )

            val longitude =
                optional(
                    index,
                    "gps_longitude",
                    "longitude"
                )

            // -------------------------------------------------
            // OPTIONAL ORIENTATION
            // -------------------------------------------------

            val orientationYaw =
                optional(
                    index,
                    "orientation_yaw",
                    "yaw"
                )

            val orientationPitch =
                optional(
                    index,
                    "orientation_pitch",
                    "pitch"
                )

            val orientationRoll =
                optional(
                    index,
                    "orientation_roll",
                    "roll"
                )

            val samples =
                ArrayList<IovnbdPhoneSample>()

            // -------------------------------------------------
            // DATA ROWS
            // -------------------------------------------------

            reader.forEachLine { line ->

                if (line.isBlank()) {
                    return@forEachLine
                }

                val values =
                    parseCsvLine(line)

                try {

                    if (values.size < columns.size) {
                        return@forEachLine
                    }

                    val timeMs =
                        values[time]
                            .trim()
                            .toDouble()
                            .toLong()

                    val accelX =
                        values[ax]
                            .trim()
                            .toDouble()

                    val accelY =
                        values[ay]
                            .trim()
                            .toDouble()

                    val accelZ =
                        values[az]
                            .trim()
                            .toDouble()

                    val gyroYawValue =
                        values[gyroYaw]
                            .trim()
                            .toDouble()

                    val gyroPitchValue =
                        values[gyroPitch]
                            .trim()
                            .toDouble()

                    val gyroRollValue =
                        values[gyroRoll]
                            .trim()
                            .toDouble()

                    // -----------------------------------------
                    // GRAVITY
                    // -----------------------------------------

                    val gravityX =
                        gravX
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }
                            ?: 0.0

                    val gravityY =
                        gravY
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }
                            ?: 0.0

                    val gravityZ =
                        gravZ
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }
                            ?: 0.0

                    // -----------------------------------------
                    // GPS
                    // -----------------------------------------

                    val latitudeValue =
                        latitude
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }

                    val longitudeValue =
                        longitude
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }

                    // -----------------------------------------
                    // ORIENTATION
                    // -----------------------------------------

                    val yawValue =
                        orientationYaw
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }

                    val pitchValue =
                        orientationPitch
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }

                    val rollValue =
                        orientationRoll
                            ?.let {
                                values[it]
                                    .trim()
                                    .toDoubleOrNull()
                            }

                    samples +=
                        IovnbdPhoneSample(

                            timeMs =
                                timeMs,

                            accelX =
                                accelX,

                            accelY =
                                accelY,

                            accelZ =
                                accelZ,

                            gyroYaw =
                                gyroYawValue,

                            gyroPitch =
                                gyroPitchValue,

                            gyroRoll =
                                gyroRollValue,

                            gravityX =
                                gravityX,

                            gravityY =
                                gravityY,

                            gravityZ =
                                gravityZ,

                            latitude =
                                latitudeValue,

                            longitude =
                                longitudeValue,

                            orientationYawDeg =
                                yawValue,

                            orientationPitchDeg =
                                pitchValue,

                            orientationRollDeg =
                                rollValue
                        )

                } catch (_: Exception) {
                    // Ignore malformed rows.
                }
            }

            samples
                .filter {
                    it.timeMs >= 0L &&
                            it.accelX.isFinite() &&
                            it.accelY.isFinite() &&
                            it.accelZ.isFinite() &&
                            it.gyroYaw.isFinite() &&
                            it.gyroPitch.isFinite() &&
                            it.gyroRoll.isFinite()
                }
                .sortedBy {
                    it.timeMs
                }
                .distinctBy {
                    it.timeMs
                }
        }
    }


    // =========================================================
    // VEHICLE / V-CSV
    // =========================================================

    fun readVehicle(
        uri: Uri
    ): List<IovnbdVehicleSample> {

        return openReader(uri).use { reader ->

            val header =
                reader.readLine()
                    ?: error("Vehicle CSV is empty")

            val columns =
                parseCsvLine(header)
                    .map(::normalizeHeader)

            val index =
                columns
                    .withIndex()
                    .associate {
                        it.value to it.index
                    }

            // -------------------------------------------------
            // REQUIRED VEHICLE COLUMNS
            // -------------------------------------------------

            val time =
                required(
                    index,
                    "time_since_start_of_day_seconds",
                    "time_since_start_of_day",
                    "time_seconds",
                    "timestamp_s",
                    "time_s"
                )

            val latitude =
                required(
                    index,
                    "latitude",
                    "gps_latitude"
                )

            val longitude =
                required(
                    index,
                    "longitude",
                    "gps_longitude"
                )

            val velocity =
                required(
                    index,
                    "velocity_km_hr",
                    "velocity_kmh",
                    "velocity_km_h",
                    "velocity"
                )

            val heading =
                required(
                    index,
                    "heading",
                    "heading_deg"
                )

            val samples =
                ArrayList<IovnbdVehicleSample>()

            // -------------------------------------------------
            // DATA ROWS
            // -------------------------------------------------

            reader.forEachLine { line ->

                if (line.isBlank()) {
                    return@forEachLine
                }

                val values =
                    parseCsvLine(line)

                try {

                    if (values.size < columns.size) {
                        return@forEachLine
                    }

                    val timeSeconds =
                        values[time]
                            .trim()
                            .toDouble()

                    val latitudeValue =
                        values[latitude]
                            .trim()
                            .toDouble()

                    val longitudeValue =
                        values[longitude]
                            .trim()
                            .toDouble()

                    val velocityKmh =
                        values[velocity]
                            .trim()
                            .toDouble()

                    val headingDeg =
                        values[heading]
                            .trim()
                            .toDouble()

                    samples +=
                        IovnbdVehicleSample(

                            timeSeconds =
                                timeSeconds,

                            latitude =
                                latitudeValue,

                            longitude =
                                longitudeValue,

                            velocityKmh =
                                velocityKmh,

                            headingDeg =
                                headingDeg
                        )

                } catch (_: Exception) {
                    // Ignore malformed rows.
                }
            }

            samples
                .filter {
                    it.timeSeconds.isFinite() &&
                            it.latitude.isFinite() &&
                            it.longitude.isFinite() &&
                            it.velocityKmh.isFinite() &&
                            it.headingDeg.isFinite()
                }
                .sortedBy {
                    it.timeSeconds
                }
        }
    }


    // =========================================================
    // READER
    // =========================================================

    private fun openReader(
        uri: Uri
    ): BufferedReader {

        val input =
            context.contentResolver
                .openInputStream(uri)
                ?: error("Unable to open CSV")

        return BufferedReader(
            InputStreamReader(
                input,
                StandardCharsets.UTF_8
            )
        )
    }


    // =========================================================
    // CSV PARSER
    // =========================================================

    /**
     * Small CSV parser supporting quoted fields.
     *
     * This is important because DATE fields and other IO-VNBD
     * fields may contain characters that should remain inside
     * a single CSV field.
     */
    private fun parseCsvLine(
        line: String
    ): List<String> {

        val result =
            ArrayList<String>()

        val current =
            StringBuilder()

        var quoted =
            false

        var i = 0

        while (i < line.length) {

            val char =
                line[i]

            when {

                char == '"' -> {

                    // Handle escaped double quote ("")
                    if (
                        quoted &&
                        i + 1 < line.length &&
                        line[i + 1] == '"'
                    ) {
                        current.append('"')
                        i++
                    } else {
                        quoted = !quoted
                    }
                }

                char == ',' &&
                        !quoted -> {

                    result +=
                        current
                            .toString()
                            .trim()

                    current.clear()
                }

                else -> {
                    current.append(char)
                }
            }

            i++
        }

        result +=
            current
                .toString()
                .trim()

        return result
    }


    // =========================================================
    // HEADER NORMALIZATION
    // =========================================================

    /**
     * Converts different IO-VNBD header representations into
     * semantic names.
     *
     * Examples:
     *
     * ACCELEROMETER X (m/s²)
     * ACCELEROMETER X (m/s2)
     * ACCELEROMETER X (m/s�)
     *
     * all become:
     *
     * accelerometer_x
     */
    private fun normalizeHeader(
        value: String
    ): String {

        var result =
            value
                .trim()
                .removePrefix("\uFEFF")
                .lowercase(Locale.US)

        // -----------------------------------------------------
        // Remove everything inside parentheses.
        //
        // This intentionally removes units such as:
        // (m/s²)
        // (m/s�)
        // (rad/s)
        // (degrees)
        // (μT)
        // -----------------------------------------------------

        result =
            result.replace(
                Regex("\\([^)]*\\)"),
                ""
            )

        // -----------------------------------------------------
        // Remove common Unicode symbols.
        // -----------------------------------------------------

        result =
            result
                .replace("�", "")
                .replace("²", "2")
                .replace("°", "")
                .replace("μ", "u")
                .replace("µ", "u")

        // -----------------------------------------------------
        // Convert all non-alphanumeric characters to "_".
        // -----------------------------------------------------

        result =
            result.replace(
                Regex("[^a-z0-9]+"),
                "_"
            )

        // -----------------------------------------------------
        // Remove duplicate underscores.
        // -----------------------------------------------------

        result =
            result.replace(
                Regex("_+"),
                "_"
            )

        return result.trim('_')
    }


    // =========================================================
    // COLUMN HELPERS
    // =========================================================

    private fun required(
        index: Map<String, Int>,
        vararg candidates: String
    ): Int {

        for (candidate in candidates) {

            val normalizedCandidate =
                normalizeHeader(candidate)

            index[normalizedCandidate]
                ?.let {
                    return it
                }
        }

        error(
            "Required IO-VNBD CSV column missing. " +
                    "Expected one of: " +
                    candidates.joinToString()
        )
    }


    private fun optional(
        index: Map<String, Int>,
        vararg candidates: String
    ): Int? {

        for (candidate in candidates) {

            val normalizedCandidate =
                normalizeHeader(candidate)

            index[normalizedCandidate]
                ?.let {
                    return it
                }
        }

        return null
    }
}


/**
 * =============================================================
 * PHONE SAMPLE
 * =============================================================
 */
data class IovnbdPhoneSample(

    val timeMs: Long,

    val accelX: Double,

    val accelY: Double,

    val accelZ: Double,

    val gyroYaw: Double,

    val gyroPitch: Double,

    val gyroRoll: Double,

    val gravityX: Double,

    val gravityY: Double,

    val gravityZ: Double,

    val latitude: Double? = null,

    val longitude: Double? = null,

    val orientationYawDeg: Double? = null,

    val orientationPitchDeg: Double? = null,

    val orientationRollDeg: Double? = null
)


/**
 * =============================================================
 * VEHICLE / REFERENCE SAMPLE
 * =============================================================
 */
data class IovnbdVehicleSample(

    val timeSeconds: Double,

    val latitude: Double,

    val longitude: Double,

    val velocityKmh: Double,

    val headingDeg: Double
)