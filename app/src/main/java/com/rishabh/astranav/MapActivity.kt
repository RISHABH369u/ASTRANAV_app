package com.rishabh.astranav

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Canvas
import android.os.Bundle
import android.os.SystemClock
import android.view.View
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.mapbox.geojson.Point
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.mapbox.maps.plugin.LocationPuck2D
import com.mapbox.maps.plugin.PuckBearing
import com.mapbox.maps.plugin.annotation.annotations
import com.mapbox.maps.extension.style.layers.properties.generated.IconAnchor
import com.mapbox.maps.plugin.annotation.generated.PointAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.createPointAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.createPolylineAnnotationManager
import com.mapbox.maps.plugin.locationcomponent.location
import com.rishabh.astranav.integrity.IntegritySnapshot
import com.rishabh.astranav.map.GeoMath
import com.rishabh.astranav.map.IntegrityLevel
import com.rishabh.astranav.map.level
import com.rishabh.astranav.navigation.HomeDashboardState
import com.rishabh.astranav.navigation.NavigationMode
import com.rishabh.astranav.navigation.NavigationSessionController
import com.rishabh.astranav.sensorcheck.util.AnimatedNumber
import kotlinx.coroutines.launch
import java.util.Locale
import kotlin.math.roundToInt

/**
 * The primary navigation HUD — real Mapbox map (not the SVG placeholder from
 * the React reference), fed entirely by NavigationSessionController's fused
 * state. Structure follows the priority order from your brief: speed
 * (biggest) → heading → map/trajectory → accuracy → trip distance → GNSS →
 * integrity, always visible; ESKF/AI/map-match/ZUPT/ZARU/DVFC pushed into
 * the expandable bottom sheet as "Subsystems" diagnostics.
 *
 * Two distinct markers on the map, intentionally:
 *  - The native Mapbox location puck = raw GPS, exactly what your phone's
 *    GNSS chip reports, with its own accuracy ring.
 *  - The custom heading-aware vehicle marker + breadcrumb trail = ASTRANAV's
 *    own fused/DR position (NavigationEngine's east/north state, converted
 *    to lat/lon — see GeoMath.kt). This is the marker that keeps moving
 *    through a GNSS outage, which is the entire point of the app.
 */
class MapActivity : AppCompatActivity() {

    private lateinit var mapView: MapView
    private lateinit var bottomSheetBehavior: BottomSheetBehavior<View>

    private lateinit var navModeDot: View
    private lateinit var navModeLabel: TextView
    private lateinit var integrityDot: View
    private lateinit var drBanner: View
    private lateinit var mapSpeedValue: TextView
    private lateinit var mapAccuracyValue: TextView
    private lateinit var mapHeadingValue: TextView
    private lateinit var mapThirdCellLabel: TextView
    private lateinit var mapThirdCellValue: TextView
    private lateinit var mapGnssValue: TextView
    private lateinit var integrityRowDot: View
    private lateinit var integrityRowLabel: TextView
    private lateinit var integrityScoreValue: TextView

    private lateinit var rowEskf: View
    private lateinit var rowAi: View
    private lateinit var rowMapMatch: View
    private lateinit var rowZupt: View
    private lateinit var rowZaru: View
    private lateinit var rowDvfc: View

    private var vehicleAnnotationManager: com.mapbox.maps.plugin.annotation.generated.PointAnnotationManager? = null
    private var breadcrumbManager: com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationManager? = null
    private var vehicleAnnotation: com.mapbox.maps.plugin.annotation.generated.PointAnnotation? = null
    private var breadcrumbAnnotation: com.mapbox.maps.plugin.annotation.generated.PolylineAnnotation? = null
    private var vehicleIconBitmap: Bitmap? = null

    private var originLat: Double? = null
    private var originLon: Double? = null
    private val breadcrumbPoints = mutableListOf<Point>()
    private var lastBreadcrumbPoint: Point? = null
    private var lastMapUpdateUptimeMs = 0L
    private var styleLoaded = false
    private var cameraCentered = false

    private lateinit var speedAnim: AnimatedNumber
    private lateinit var accuracyAnim: AnimatedNumber
    private lateinit var tripAnim: AnimatedNumber

    private val locationPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) {
                NavigationSessionController.onLocationPermissionGranted(this)
                enableLocationPuck()
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_map)

        mapView = findViewById(R.id.mapView)
        navModeDot = findViewById(R.id.navModeDot)
        navModeLabel = findViewById(R.id.navModeLabel)
        integrityDot = findViewById(R.id.integrityDot)
        drBanner = findViewById(R.id.drBanner)
        mapSpeedValue = findViewById(R.id.mapSpeedValue)
        mapAccuracyValue = findViewById(R.id.mapAccuracyValue)
        mapHeadingValue = findViewById(R.id.mapHeadingValue)
        mapThirdCellLabel = findViewById(R.id.mapThirdCellLabel)
        mapThirdCellValue = findViewById(R.id.mapThirdCellValue)
        mapGnssValue = findViewById(R.id.mapGnssValue)
        integrityRowDot = findViewById(R.id.integrityRowDot)
        integrityRowLabel = findViewById(R.id.integrityRowLabel)
        integrityScoreValue = findViewById(R.id.integrityScoreValue)
        rowEskf = findViewById(R.id.rowEskf)
        rowAi = findViewById(R.id.rowAi)
        rowMapMatch = findViewById(R.id.rowMapMatch)
        rowZupt = findViewById(R.id.rowZupt)
        rowZaru = findViewById(R.id.rowZaru)
        rowDvfc = findViewById(R.id.rowDvfc)

        bottomSheetBehavior = BottomSheetBehavior.from(findViewById(R.id.bottomSheet))

        findViewById<View>(R.id.btnBack).setOnClickListener { finish() }
        findViewById<View>(R.id.btnRecenter).setOnClickListener { recenterCamera() }

        speedAnim = AnimatedNumber(0f, decimals = 0, stiffness = 4f) { mapSpeedValue.text = it.roundToInt().toString() }
        accuracyAnim = AnimatedNumber(0f, decimals = 0, stiffness = 5f) { mapAccuracyValue.text = "± ${it.roundToInt()} m" }
        tripAnim = AnimatedNumber(0f, decimals = 1, stiffness = 8f) { mapThirdCellValue.text = String.format(Locale.US, "%.1f km", it) }

        setupMap()

        lifecycleScope.launch {
            NavigationSessionController.state.collect { render(it) }
        }
    }

    /**
     * VERSION NOTE — `mapView.annotations.createPointAnnotationManager()` /
     * `createPolylineAnnotationManager()` and the `PointAnnotationOptions`/
     * `PolylineAnnotationOptions` builder shape are the current documented
     * Mapbox Maps SDK v11 annotations-plugin API, but exact package paths
     * (`com.mapbox.maps.plugin.annotation.generated.*`) have moved before
     * across major versions. If imports don't resolve, check the
     * `annotation` plugin's current package in your pinned SDK version —
     * the two-manager, create/update shape stays the same either way.
     */
    private fun setupMap() {
        mapView.mapboxMap.loadStyle(Style.DARK) {
            styleLoaded = true
            vehicleIconBitmap = vectorToBitmap(R.drawable.ic_vehicle_marker)
            vehicleAnnotationManager = mapView.annotations.createPointAnnotationManager()
            breadcrumbManager = mapView.annotations.createPolylineAnnotationManager()
            enableLocationPuck()
        }
    }

    private fun enableLocationPuck() {
        if (!styleLoaded) return
        val granted = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (!granted) return
        mapView.location.updateSettings {
            enabled = true
            locationPuck = LocationPuck2D()
            puckBearing = PuckBearing.COURSE
            puckBearingEnabled = true
            showAccuracyRing = true
        }
    }

    override fun onResume() {
        super.onResume()
        NavigationSessionController.start(this)
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) {
            locationPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
        } else {
            enableLocationPuck()
        }
    }

    override fun onPause() {
        super.onPause()
        NavigationSessionController.stop()
    }

    override fun onDestroy() {
        super.onDestroy()
        speedAnim.stop()
        accuracyAnim.stop()
        tripAnim.stop()
    }

    private fun render(s: HomeDashboardState) {
        val denied = s.nav.mode == NavigationMode.GNSS_DENIED || s.nav.mode == NavigationMode.REACQUIRING
        val degraded = s.nav.mode == NavigationMode.DEGRADED || s.nav.mode == NavigationMode.TRANSITION

        // ---- Top pill ----
        val (modeLabel, modeColor, modeDot) = when {
            denied -> Triple("DEAD RECKONING", R.color.cyan, R.drawable.dot_cyan)
            degraded -> Triple("GNSS DEGRADED", R.color.warn, R.drawable.dot_warn)
            else -> Triple("GNSS + INS", R.color.good, R.drawable.dot_good)
        }
        navModeLabel.text = modeLabel
        navModeLabel.setTextColor(ContextCompat.getColor(this, modeColor))
        navModeDot.setBackgroundResource(modeDot)
        drBanner.visibility = if (denied) View.VISIBLE else View.GONE

        // ---- Integrity ----
        val integrity: IntegritySnapshot? = s.integrity
        val level = integrity?.level()
        val integrityDotRes = when (level) {
            IntegrityLevel.STABLE -> R.drawable.dot_good
            IntegrityLevel.CAUTION -> R.drawable.dot_warn
            IntegrityLevel.DEGRADED, null -> R.drawable.dot_warn
        }
        integrityDot.setBackgroundResource(integrityDotRes)
        integrityRowDot.setBackgroundResource(integrityDotRes)
        integrityRowLabel.text = when (level) {
            IntegrityLevel.STABLE -> "STABLE"
            IntegrityLevel.CAUTION -> "CAUTION"
            IntegrityLevel.DEGRADED -> "DEGRADED"
            null -> "—"
        }
        integrityRowLabel.setTextColor(
            ContextCompat.getColor(this, if (level == IntegrityLevel.STABLE) R.color.good else R.color.warn),
        )
        integrityScoreValue.text = integrity?.let { "${it.scorePercent}%" } ?: "--%"

        // ---- Primary telemetry ----
        speedAnim.animateTo((s.nav.speedMps * 3.6).toFloat())
        accuracyAnim.animateTo(s.nav.positionSigmaMeters.toFloat())
        mapAccuracyValue.setTextColor(ContextCompat.getColor(this, if (denied) R.color.cyan else R.color.good))
        mapHeadingValue.text = headingLabel(s.nav.headingDegrees)

        if (denied) {
            mapThirdCellLabel.text = "GNSS OUTAGE"
            mapThirdCellValue.text = "--:--" // no outage timer wired yet — see README
        } else {
            mapThirdCellLabel.text = "TRIP DIST."
            tripAnim.animateTo((s.tripDistanceMeters / 1000.0).toFloat())
        }
        mapGnssValue.text = if (s.gnssAvailable) "FIX" else "DENIED"
        mapGnssValue.setTextColor(ContextCompat.getColor(this, if (s.gnssAvailable) R.color.good else R.color.cyan))

        // ---- Subsystem diagnostics ----
        bindRow(rowEskf, "ESKF", if (s.imuAvailable) "ACTIVE" else "NO IMU", s.imuAvailable)
        bindRow(rowAi, "AI Velocity (TCN)", if (s.aiAvailable) "ACTIVE" else "WARMING UP", s.aiAvailable)
        bindRow(rowMapMatch, "Map Match", if (s.nav.mapMatched) "LOCKED" else "SEARCHING", s.nav.mapMatched)
        bindRow(rowZupt, "ZUPT", if (s.zuptActive) "ACTIVE" else "IDLE", s.zuptActive)
        bindRow(rowZaru, "ZARU", if (s.zaruActive) "ACTIVE" else "IDLE", s.zaruActive)
        bindRow(rowDvfc, "DVFC", if (s.dvfcCalibrated) "CALIBRATED" else "NEEDS SETUP", s.dvfcCalibrated)

        // ---- Map (throttled — annotation churn at 100Hz IMU rate would be wasteful) ----
        val now = SystemClock.elapsedRealtime()
        if (now - lastMapUpdateUptimeMs >= 150L) {
            lastMapUpdateUptimeMs = now
            updateMap(s)
        }
    }

    private fun bindRow(row: View, label: String, value: String, healthy: Boolean) {
        row.findViewById<TextView>(R.id.diagRowLabel).text = label
        val valueView = row.findViewById<TextView>(R.id.diagRowValue)
        valueView.text = value
        valueView.setTextColor(ContextCompat.getColor(this, if (healthy) R.color.good else R.color.fg_faint))
        row.findViewById<View>(R.id.diagRowDot).setBackgroundResource(if (healthy) R.drawable.dot_good else R.drawable.dot_warn)
    }

    private fun updateMap(s: HomeDashboardState) {
        val manager = vehicleAnnotationManager ?: return
        val breadcrumb = breadcrumbManager ?: return
        val icon = vehicleIconBitmap ?: return

        // Anchor the ENU→lat/lon conversion at the first real GNSS fix we ever see.
        if (originLat == null && s.gnssLatitude != null && s.gnssLongitude != null) {
            originLat = s.gnssLatitude
            originLon = s.gnssLongitude
        }
        val oLat = originLat ?: FALLBACK_ORIGIN_LAT.also { originLon = FALLBACK_ORIGIN_LON }
        val oLon = originLon ?: FALLBACK_ORIGIN_LON

        val (lat, lon) = GeoMath.toLatLon(oLat, oLon, s.nav.eastMeters, s.nav.northMeters)
        val point = Point.fromLngLat(lon, lat)

        val existing = vehicleAnnotation
        if (existing == null) {
            val options = PointAnnotationOptions()
                .withPoint(point)
                .withIconImage(icon)
                .withIconRotate(s.nav.headingDegrees)
                .withIconAnchor(IconAnchor.CENTER)
            vehicleAnnotation = manager.create(options)
        } else {
            existing.point = point
            existing.iconRotate = s.nav.headingDegrees
            manager.update(existing)
        }

        if (!cameraCentered) {
            cameraCentered = true
            mapView.mapboxMap.setCamera(CameraOptions.Builder().center(point).zoom(16.0).build())
        }

        // Breadcrumb: only append when we've actually moved, so a parked car doesn't spam annotations.
        val last = lastBreadcrumbPoint
        val movedEnough = last == null || distanceMeters(last, point) > 3.0
        if (movedEnough) {
            lastBreadcrumbPoint = point
            breadcrumbPoints.add(point)
            if (breadcrumbPoints.size > MAX_BREADCRUMB_POINTS) breadcrumbPoints.removeAt(0)

            val existingLine = breadcrumbAnnotation
            if (existingLine == null && breadcrumbPoints.size >= 2) {
                val lineColor = if (s.nav.mode == NavigationMode.GNSS_DENIED || s.nav.mode == NavigationMode.REACQUIRING) "#35E0D0" else "#4F9DFF"
                val options = PolylineAnnotationOptions()
                    .withPoints(breadcrumbPoints.toList())
                    .withLineColor(lineColor)
                    .withLineWidth(4.0)
                breadcrumbAnnotation = breadcrumb.create(options)
            } else if (existingLine != null) {
                existingLine.points = breadcrumbPoints.toList()
                breadcrumb.update(existingLine)
            }
        }
    }

    private fun recenterCamera() {
        val point = vehicleAnnotation?.point ?: return
        mapView.mapboxMap.setCamera(CameraOptions.Builder().center(point).zoom(16.5).build())
    }

    private fun vectorToBitmap(resId: Int): Bitmap {
        val drawable = ContextCompat.getDrawable(this, resId)!!
        val bitmap = Bitmap.createBitmap(drawable.intrinsicWidth, drawable.intrinsicHeight, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        drawable.setBounds(0, 0, canvas.width, canvas.height)
        drawable.draw(canvas)
        return bitmap
    }

    private fun distanceMeters(a: Point, b: Point): Double {
        // Flat-earth approximation, fine at the few-metre breadcrumb scale — see GeoMath.kt.
        val dLat = (b.latitude() - a.latitude()) * 111_320.0
        val dLon = (b.longitude() - a.longitude()) * 111_320.0 * Math.cos(Math.toRadians(a.latitude()))
        return Math.sqrt(dLat * dLat + dLon * dLon)
    }

    private fun headingLabel(deg: Double): String {
        val headings = listOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        val idx = (Math.round(deg / 45.0).toInt()).mod(8)
        return String.format(Locale.US, "%s · %03d°", headings[idx], deg.roundToInt().mod(360))
    }

    companion object {
        private const val MAX_BREADCRUMB_POINTS = 500

        // Only used if GNSS never becomes available at all, so the map isn't blank — swap for
        // wherever you're actually demoing. Not used once a real fix comes in.
        private const val FALLBACK_ORIGIN_LAT = 28.6139
        private const val FALLBACK_ORIGIN_LON = 77.2090
    }
}
