package com.rishabh.astranav.replay

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sqrt


/**
 * =============================================================
 * REPLAY METRIC SNAPSHOT
 * =============================================================
 */
data class ReplayMetricSnapshot(

    val positionErrorM: Double = 0.0,

    val speedErrorKmh: Double = 0.0,

    val headingErrorDeg: Double = 0.0,

    val samples: Int = 0,

    val positionRmseM: Double = 0.0,

    val positionMaeM: Double = 0.0,

    val position95M: Double = 0.0,

    val finalPositionErrorM: Double = 0.0,

    val speedMaeKmh: Double = 0.0,

    val headingMaeDeg: Double = 0.0
)


/**
 * =============================================================
 * REPLAY METRICS ENGINE
 * =============================================================
 *
 * Reference:
 * V-*.csv
 *
 * Estimated:
 * ASTRA-Core / NavigationSolution
 */
class ReplayMetricsEngine {

    private var originLat =
        Double.NaN

    private var originLon =
        Double.NaN


    private val positionErrors =
        ArrayList<Double>()

    private val speedErrors =
        ArrayList<Double>()

    private val headingErrors =
        ArrayList<Double>()


    // =========================================================
    // RESET
    // =========================================================

    fun reset() {

        originLat =
            Double.NaN

        originLon =
            Double.NaN

        positionErrors.clear()

        speedErrors.clear()

        headingErrors.clear()
    }


    // =========================================================
    // UPDATE
    // =========================================================

    fun update(

        north: Double,

        east: Double,

        speedMps: Double,

        heading: Double,

        truth: IovnbdVehicleSample

    ): ReplayMetricSnapshot {


        // -----------------------------------------------------
        // REFERENCE ORIGIN
        // -----------------------------------------------------

        if (
            !originLat.isFinite()
        ) {

            originLat =
                truth.latitude

            originLon =
                truth.longitude
        }


        // -----------------------------------------------------
        // TRUTH → LOCAL N/E
        // -----------------------------------------------------

        val truthLocal =
            localMeters(
                truth.latitude,
                truth.longitude
            )


        val truthNorth =
            truthLocal.first

        val truthEast =
            truthLocal.second


        // -----------------------------------------------------
        // POSITION ERROR
        // -----------------------------------------------------

        val positionError =
            hypot(

                north -
                        truthNorth,

                east -
                        truthEast
            )


        // -----------------------------------------------------
        // SPEED ERROR
        // -----------------------------------------------------

        val estimatedSpeedKmh =
            speedMps *
                    3.6


        val speedError =
            abs(
                estimatedSpeedKmh -
                        truth.velocityKmh
            )


        // -----------------------------------------------------
        // HEADING ERROR
        // -----------------------------------------------------

        val headingError =
            angularDifference(
                heading,
                truth.headingDeg
            )


        positionErrors +=
            positionError

        speedErrors +=
            speedError

        headingErrors +=
            headingError


        // -----------------------------------------------------
        // METRICS
        // -----------------------------------------------------

        val positionRmse =
            sqrt(

                positionErrors
                    .map {
                        it * it
                    }
                    .average()
            )


        val positionMae =
            positionErrors.average()


        val position95 =
            percentile(
                positionErrors,
                0.95
            )


        val speedMae =
            speedErrors.average()


        val headingMae =
            headingErrors.average()


        return ReplayMetricSnapshot(

            positionErrorM =
                positionError,

            speedErrorKmh =
                speedError,

            headingErrorDeg =
                headingError,

            samples =
                positionErrors.size,

            positionRmseM =
                positionRmse,

            positionMaeM =
                positionMae,

            position95M =
                position95,

            finalPositionErrorM =
                positionError,

            speedMaeKmh =
                speedMae,

            headingMaeDeg =
                headingMae
        )
    }


    // =========================================================
    // GEO → LOCAL N/E
    // =========================================================

    private fun localMeters(

        latitude: Double,

        longitude: Double

    ): Pair<Double, Double> {

        val earthRadius =
            6_371_000.0


        val deltaLat =
            Math.toRadians(
                latitude -
                        originLat
            )


        val deltaLon =
            Math.toRadians(
                longitude -
                        originLon
            )


        val meanLat =
            Math.toRadians(

                (
                        latitude +
                                originLat
                        ) /
                        2.0
            )


        val north =
            earthRadius *
                    deltaLat


        val east =
            earthRadius *
                    cos(meanLat) *
                    deltaLon


        return north to east
    }


    // =========================================================
    // ANGULAR DIFFERENCE
    // =========================================================

    private fun angularDifference(

        a: Double,

        b: Double

    ): Double {

        var difference =
            (a - b) %
                    360.0


        if (
            difference > 180.0
        ) {

            difference -=
                360.0
        }


        if (
            difference < -180.0
        ) {

            difference +=
                360.0
        }


        return abs(
            difference
        )
    }


    // =========================================================
    // PERCENTILE
    // =========================================================

    private fun percentile(

        values: List<Double>,

        percentile: Double

    ): Double {

        if (
            values.isEmpty()
        ) {
            return 0.0
        }


        val sorted =
            values.sorted()


        val index =
            (
                    (
                            sorted.size - 1
                            ) *
                            percentile
                    )
                .roundToInt()
                .coerceIn(
                    0,
                    sorted.lastIndex
                )


        return sorted[index]
    }
}