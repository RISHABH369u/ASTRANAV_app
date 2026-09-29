package com.rishabh.astranav.map

import kotlin.math.cos

/**
 * Converts NavigationEngine's local East/North (metres) frame into
 * approximate lat/lon so it can be drawn on a real map.
 *
 * This is a local equirectangular (tangent-plane) approximation anchored at
 * `originLat`/`originLon` — accurate to a few centimetres at city scale,
 * which is what this app operates at. It is NOT a general-purpose geodesic
 * conversion and shouldn't be reused for anything spanning hundreds of
 * kilometres.
 */
object GeoMath {
    private const val METERS_PER_DEGREE_LAT = 111_320.0

    fun toLatLon(originLat: Double, originLon: Double, eastMeters: Double, northMeters: Double): Pair<Double, Double> {
        val lat = originLat + (northMeters / METERS_PER_DEGREE_LAT)
        val metersPerDegreeLon = METERS_PER_DEGREE_LAT * cos(Math.toRadians(originLat)).coerceAtLeast(0.01)
        val lon = originLon + (eastMeters / metersPerDegreeLon)
        return lat to lon
    }
}
