package com.rishabh.astranav

import android.graphics.Bitmap
import android.graphics.Canvas
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.mapbox.maps.plugin.annotation.annotations
import com.mapbox.maps.extension.style.layers.properties.generated.IconAnchor
import com.mapbox.maps.plugin.annotation.generated.PointAnnotation
import com.mapbox.maps.plugin.annotation.generated.PointAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.PointAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.createPointAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.createPolylineAnnotationManager
import com.mapbox.maps.CoordinateBounds
import com.rishabh.astranav.map.DemoTripReplaySource
import com.rishabh.astranav.map.ReplayEventKind
import com.rishabh.astranav.map.ReplayMode
import com.rishabh.astranav.map.ReplaySample
import com.rishabh.astranav.map.TripReplaySource
import java.util.Locale
import kotlin.math.roundToInt

/**
 * Trip Replay — matches TripReplay.tsx's structure (map + status pill + mode
 * toggle + telemetry strip + event chips + scrubbable timeline + transport
 * controls), but the underlying data is explicitly a [TripReplaySource],
 * currently [DemoTripReplaySource] since there's no trip-recording backend
 * in this app yet. The "DEMO PLAYBACK" label in the layout is not
 * decorative — it's the honest disclosure that this isn't a real recorded
 * drive. Swap in a real TripReplaySource implementation once one exists;
 * nothing else in this file needs to change.
 */
class TripReplayActivity : AppCompatActivity() {

    private lateinit var mapView: MapView
    private lateinit var replayStatusDot: View
    private lateinit var replayStatusLabel: TextView
    private lateinit var modeGnss: TextView
    private lateinit var modeRaw: TextView
    private lateinit var modeHybrid: TextView
    private lateinit var replaySpeedValue: TextView
    private lateinit var replayHeadingValue: TextView
    private lateinit var replayAccuracyValue: TextView
    private lateinit var eventChipRow: android.widget.LinearLayout
    private lateinit var replaySeekBar: SeekBar
    private lateinit var replayTimeElapsed: TextView
    private lateinit var replayTimeTotal: TextView
    private lateinit var btnPlayPause: View
    private lateinit var playPauseIcon: android.widget.ImageView

    private val source: TripReplaySource = DemoTripReplaySource(originLat = 28.6139, originLon = 77.2090)

    private var t = 0f
    private var playing = true
    private var seeking = false
    private var pointAnnotationManager: PointAnnotationManager? = null
    private var vehicleAnnotation: PointAnnotation? = null
    private var vehicleIcon: Bitmap? = null
    private var styleReady = false

    private val handler = Handler(Looper.getMainLooper())
    private var lastTickNanos = 0L
    private val tickRunnable = object : Runnable {
        override fun run() {
            if (playing) {
                val now = System.nanoTime()
                val dt = if (lastTickNanos == 0L) 0f else ((now - lastTickNanos) / 1_000_000_000f).coerceAtMost(0.1f)
                lastTickNanos = now
                t += dt * 3f // 3x playback speed, matching the reference
                if (t >= source.durationSeconds) t = 0f
                renderTick()
            } else {
                lastTickNanos = 0L
            }
            handler.postDelayed(this, 33L)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_trip_replay)

        mapView = findViewById(R.id.replayMapView)
        replayStatusDot = findViewById(R.id.replayStatusDot)
        replayStatusLabel = findViewById(R.id.replayStatusLabel)
        modeGnss = findViewById(R.id.modeGnss)
        modeRaw = findViewById(R.id.modeRaw)
        modeHybrid = findViewById(R.id.modeHybrid)
        replaySpeedValue = findViewById(R.id.replaySpeedValue)
        replayHeadingValue = findViewById(R.id.replayHeadingValue)
        replayAccuracyValue = findViewById(R.id.replayAccuracyValue)
        eventChipRow = findViewById(R.id.eventChipRow)
        replaySeekBar = findViewById(R.id.replaySeekBar)
        replayTimeElapsed = findViewById(R.id.replayTimeElapsed)
        replayTimeTotal = findViewById(R.id.replayTimeTotal)
        btnPlayPause = findViewById(R.id.btnPlayPause)
        playPauseIcon = findViewById(R.id.playPauseIcon)

        replayTimeTotal.text = fmtTime(source.durationSeconds)
        findViewById<View>(R.id.btnCloseReplay).setOnClickListener { finish() }

        setUpModeToggle()
        setUpEventChips()
        setUpSeekBar()
        setUpTransportControls()
        setUpMap()
    }

    private fun setUpModeToggle() {
        val buttons = listOf(modeGnss, modeRaw, modeHybrid)
        fun select(selected: TextView) {
            buttons.forEach {
                val isSel = it == selected
                it.setBackgroundResource(if (isSel) R.drawable.bg_pill_button else 0)
                it.setTextColor(ContextCompat.getColor(this, if (isSel) R.color.button_primary_text else R.color.fg_dim))
            }
        }
        buttons.forEach { it.setOnClickListener { v -> select(v as TextView) } }
        select(modeHybrid)
    }

    private fun setUpEventChips() {
        source.events.forEach { event ->
            val chip = TextView(this).apply {
                text = "${event.timeLabel}  ${event.label}"
                textSize = 10.5f
                setTextColor(ContextCompat.getColor(context, R.color.fg_dim))
                setPadding(28, 16, 28, 16)
                setBackgroundResource(R.drawable.bg_event_chip)
                val lp = android.widget.LinearLayout.LayoutParams(
                    android.widget.LinearLayout.LayoutParams.WRAP_CONTENT,
                    android.widget.LinearLayout.LayoutParams.WRAP_CONTENT,
                )
                lp.marginEnd = 8
                layoutParams = lp
                setOnClickListener { t = event.pct * source.durationSeconds; renderTick() }
            }
            eventChipRow.addView(chip)
        }
    }

    private fun setUpSeekBar() {
        replaySeekBar.max = 1000
        replaySeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                if (fromUser) {
                    t = (progress / 1000f) * source.durationSeconds
                    renderTick()
                }
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {
                seeking = true
                playing = false
                updatePlayPauseIcon()
            }
            override fun onStopTrackingTouch(seekBar: SeekBar) { seeking = false }
        })
    }

    private fun setUpTransportControls() {
        btnPlayPause.setOnClickListener {
            playing = !playing
            updatePlayPauseIcon()
        }
        findViewById<View>(R.id.btnPrevEvent).setOnClickListener {
            val marks = source.events.map { it.pct * source.durationSeconds }
            val target = marks.filter { it < t - 0.5f }.maxOrNull()
            if (target != null) { t = target; renderTick() }
        }
        findViewById<View>(R.id.btnNextEvent).setOnClickListener {
            val marks = source.events.map { it.pct * source.durationSeconds }
            val target = marks.filter { it > t + 0.5f }.minOrNull()
            if (target != null) { t = target; renderTick() }
        }
    }

    private fun updatePlayPauseIcon() {
        playPauseIcon.setImageResource(if (playing) R.drawable.ic_pause else R.drawable.ic_play)
    }

    private fun setUpMap() {
        mapView.mapboxMap.loadStyle(Style.DARK) {
            styleReady = true
            vehicleIcon = vectorToBitmap(R.drawable.ic_vehicle_marker)
            pointAnnotationManager = mapView.annotations.createPointAnnotationManager()

            // Draw the full demo route once.
            val lineManager = mapView.annotations.createPolylineAnnotationManager()
            lineManager.create(
                PolylineAnnotationOptions()
                    .withPoints(source.routePoints)
                    .withLineColor("#4F9DFF")
                    .withLineWidth(4.0),
            )

            fitCameraToRoute()
            renderTick()
        }
    }

    /**
     * VERSION NOTE — `cameraForCoordinateBounds` / `CoordinateBounds` /
     * `EdgeInsets` param order occasionally shifts between Mapbox Maps SDK
     * releases. If this doesn't compile as-is against your pinned SDK
     * version, the fix is a signature tweak here only — nothing else in
     * this file depends on it. A safe fallback if you want to skip this
     * entirely: just call `mapView.mapboxMap.setCamera(CameraOptions.Builder().center(source.routePoints[0]).zoom(14.0).build())`.
     */
    private fun fitCameraToRoute() {
        val points = source.routePoints
        if (points.isEmpty()) return
        var minLat = points[0].latitude(); var maxLat = minLat
        var minLon = points[0].longitude(); var maxLon = minLon
        points.forEach {
            minLat = minOf(minLat, it.latitude()); maxLat = maxOf(maxLat, it.latitude())
            minLon = minOf(minLon, it.longitude()); maxLon = maxOf(maxLon, it.longitude())
        }
        val bounds = CoordinateBounds(
            com.mapbox.geojson.Point.fromLngLat(minLon, minLat),
            com.mapbox.geojson.Point.fromLngLat(maxLon, maxLat),
        )
        val camera = mapView.mapboxMap.cameraForCoordinateBounds(bounds, com.mapbox.maps.EdgeInsets(80.0, 60.0, 260.0, 60.0))
        mapView.mapboxMap.setCamera(camera)
    }

    private fun renderTick() {
        val pct = (t / source.durationSeconds).coerceIn(0f, 1f)
        val sample: ReplaySample = source.sampleAt(pct)

        // ---- Status pill ----
        val (label, colorRes, dotRes) = when (sample.mode) {
            ReplayMode.DENIED -> Triple("DEAD RECKONING", R.color.cyan, R.drawable.dot_cyan)
            ReplayMode.DEGRADED -> Triple("GNSS DEGRADED", R.color.warn, R.drawable.dot_warn)
            ReplayMode.FUSED -> Triple("GNSS + INS", R.color.good, R.drawable.dot_good)
        }
        replayStatusLabel.text = label
        replayStatusLabel.setTextColor(ContextCompat.getColor(this, colorRes))
        replayStatusDot.setBackgroundResource(dotRes)

        // ---- Telemetry ----
        replaySpeedValue.text = "${sample.speedKmh.roundToInt()} km/h"
        replayHeadingValue.text = headingLabel(sample.headingDeg)
        replayAccuracyValue.text = "± ${sample.accuracyM.roundToInt()} m"
        replayAccuracyValue.setTextColor(
            ContextCompat.getColor(this, if (sample.mode == ReplayMode.DENIED) R.color.cyan else R.color.good),
        )

        // ---- Timeline ----
        if (!seeking) replaySeekBar.progress = (pct * 1000).roundToInt()
        replayTimeElapsed.text = fmtTime(t)

        // ---- Event chip highlight ----
        for (i in 0 until eventChipRow.childCount) {
            val chip = eventChipRow.getChildAt(i)
            val active = kotlin.math.abs(pct - source.events[i].pct) < 0.05f
            chip.setBackgroundResource(if (active) R.drawable.bg_event_chip_active else R.drawable.bg_event_chip)
        }

        // ---- Marker ----
        updateMarker(sample)
    }

    private fun updateMarker(sample: ReplaySample) {
        if (!styleReady) return
        val manager = pointAnnotationManager ?: return
        val icon = vehicleIcon ?: return
        val existing = vehicleAnnotation
        if (existing == null) {
            val options = PointAnnotationOptions()
                .withPoint(sample.point)
                .withIconImage(icon)
                .withIconRotate(sample.headingDeg.toDouble())
                .withIconAnchor(IconAnchor.CENTER)
            vehicleAnnotation = manager.create(options)
        } else {
            existing.point = sample.point
            existing.iconRotate = sample.headingDeg.toDouble()
            manager.update(existing)
        }
    }

    private fun vectorToBitmap(resId: Int): Bitmap {
        val drawable = ContextCompat.getDrawable(this, resId)!!
        val bitmap = Bitmap.createBitmap(drawable.intrinsicWidth, drawable.intrinsicHeight, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        drawable.setBounds(0, 0, canvas.width, canvas.height)
        drawable.draw(canvas)
        return bitmap
    }

    private fun fmtTime(seconds: Float): String {
        val s = seconds.roundToInt().coerceAtLeast(0)
        return String.format(Locale.US, "%02d:%02d", s / 60, s % 60)
    }

    private fun headingLabel(deg: Float): String {
        val headings = listOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        val idx = (Math.round(deg / 45f)).mod(8)
        return String.format(Locale.US, "%s · %03d°", headings[idx], deg.roundToInt().mod(360))
    }

    override fun onResume() {
        super.onResume()
        lastTickNanos = 0L
        handler.post(tickRunnable)
    }

    override fun onPause() {
        super.onPause()
        handler.removeCallbacks(tickRunnable)
    }
}
