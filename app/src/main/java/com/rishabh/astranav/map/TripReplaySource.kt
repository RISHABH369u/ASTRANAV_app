package com.rishabh.astranav.map

import com.mapbox.geojson.Point
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round
import kotlin.math.sin

enum class ReplayMode { FUSED, DEGRADED, DENIED }
enum class ReplayEventKind { FUSED, DEGRADED, DENIED, TURN }

data class ReplayEvent(val timeLabel: String, val pct: Float, val label: String, val kind: ReplayEventKind)

data class ReplaySample(
    val speedKmh: Float,
    val headingDeg: Float,
    val accuracyM: Float,
    val point: Point,
    val mode: ReplayMode,
)

/**
 * Source of truth for the Replay screen's timeline. There's no trip-recording
 * backend in this app yet (nothing persists a real drive), so
 * [DemoTripReplaySource] below is deterministic sample data — same shape and
 * numbers as the React reference's own mock (`replayEvents`, the sine-wave
 * speed curve), not a real recorded trip. TripReplayActivity labels this
 * "DEMO PLAYBACK" on screen rather than pretending otherwise.
 *
 * When a real trip-logging layer exists, implement this interface against
 * stored samples and swap it in — TripReplayActivity doesn't know or care
 * which implementation it's given.
 */
interface TripReplaySource {
    val isDemoData: Boolean
    val durationSeconds: Float
    val events: List<ReplayEvent>
    val routePoints: List<Point>
    fun sampleAt(pct: Float): ReplaySample
}

class DemoTripReplaySource(originLat: Double, originLon: Double) : TripReplaySource {

    override val isDemoData = true
    override val durationSeconds = 78f

    override val events = listOf(
        ReplayEvent("00:12", 0.14f, "GNSS healthy", ReplayEventKind.FUSED),
        ReplayEvent("00:24", 0.30f, "GNSS degradation", ReplayEventKind.DEGRADED),
        ReplayEvent("00:31", 0.40f, "GNSS lost", ReplayEventKind.DENIED),
        ReplayEvent("00:31", 0.42f, "Dead reckoning activated", ReplayEventKind.DENIED),
        ReplayEvent("00:52", 0.66f, "Turn detected", ReplayEventKind.TURN),
        ReplayEvent("01:04", 0.82f, "GNSS restored", ReplayEventKind.FUSED),
    )

    // A believable curved arterial, offset from the origin by a few hundred metres —
    // same idea as the reference's hand-authored SVG path, expressed as east/north waypoints.
    private val waypointsEastNorth = listOf(
        0.0 to 0.0,
        35.0 to 55.0,
        60.0 to 130.0,
        70.0 to 230.0,
        60.0 to 330.0,
        20.0 to 410.0,
        -20.0 to 480.0,
        10.0 to 560.0,
        80.0 to 610.0,
        180.0 to 630.0,
    )

    override val routePoints: List<Point> = waypointsEastNorth.map { (e, n) ->
        val (lat, lon) = GeoMath.toLatLon(originLat, originLon, e, n)
        Point.fromLngLat(lon, lat)
    }

    private fun pointAt(pct: Float): Point {
        val clamped = pct.coerceIn(0f, 1f)
        val seg = clamped * (routePoints.size - 1)
        val i = min(seg.toInt(), routePoints.size - 2).coerceAtLeast(0)
        val f = seg - i
        val a = routePoints[i]
        val b = routePoints[i + 1]
        val lon = a.longitude() + (b.longitude() - a.longitude()) * f
        val lat = a.latitude() + (b.latitude() - a.latitude()) * f
        return Point.fromLngLat(lon, lat)
    }

    private fun stateAt(pct: Float): ReplayMode = when {
        pct >= 0.4f && pct < 0.82f -> ReplayMode.DENIED
        pct >= 0.3f && pct < 0.4f -> ReplayMode.DEGRADED
        else -> ReplayMode.FUSED
    }

    override fun sampleAt(pct: Float): ReplaySample {
        val mode = stateAt(pct)
        val speed = 44f + sin(pct * PI.toFloat() * 3f) * 8f
        val heading = 30f + pct * 60f
        val accuracy = when (mode) {
            ReplayMode.DENIED -> round(10f + pct * 8f)
            ReplayMode.DEGRADED -> 12f
            ReplayMode.FUSED -> 6f
        }
        return ReplaySample(
            speedKmh = max(0f, speed),
            headingDeg = heading % 360f,
            accuracyM = accuracy,
            point = pointAt(pct),
            mode = mode,
        )
    }
}
