package com.rishabh.astranav

import android.Manifest
import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.rishabh.astranav.home.HomeHeadingCompassView
import com.rishabh.astranav.home.SparklineView
import com.rishabh.astranav.navigation.MotionActivity
import com.rishabh.astranav.navigation.NavigationMode
import com.rishabh.astranav.navigation.NavigationSessionController
import kotlinx.coroutines.launch
import java.util.Locale
import kotlin.math.roundToInt

/**
 * Home — the first screen driven by real, live NavigationSessionController
 * state instead of the mock data the original React reference (Home.tsx)
 * used. Structure follows Home.tsx (system status card → hero → last trip),
 * with the additions requested on top of it:
 *  - a heading compass (current travel direction, not the DVFC alignment one)
 *  - motion state: Stationary / Driving / Turning
 *  - AI (TCN) speed alongside the fused speed
 *  - live ZUPT / ZARU indicator pills
 *  - a DVFC status chip (Calibrated / Needs setup) instead of a static "Map" chip
 */
class HomeActivity : AppCompatActivity() {

    private lateinit var systemStatusDot: View
    private lateinit var systemStatusLabel: TextView
    private lateinit var chipGnssDot: View
    private lateinit var chipGnssValue: TextView
    private lateinit var chipImuDot: View
    private lateinit var chipImuValue: TextView
    private lateinit var chipAiDot: View
    private lateinit var chipAiValue: TextView
    private lateinit var chipDvfcDot: View
    private lateinit var chipDvfcValue: TextView
    private lateinit var speedValue: TextView
    private lateinit var aiSpeedValue: TextView
    private lateinit var headingValue: TextView
    private lateinit var motionValue: TextView
    private lateinit var zuptDot: View
    private lateinit var zaruDot: View
    private lateinit var homeCompass: HomeHeadingCompassView
    private lateinit var compassReadout: TextView

    private lateinit var positionValue: TextView
    private lateinit var badgeGnss: View
    private lateinit var badgeGnssValue: TextView
    private lateinit var badgeImu: View
    private lateinit var badgeImuValue: TextView
    private lateinit var badgeDvfc: View
    private lateinit var badgeDvfcValue: TextView
    private lateinit var readinessAccuracyValue: TextView
    private lateinit var readinessConfidenceValue: TextView
    private lateinit var readinessDvfcOffsetValue: TextView
    private lateinit var signalAiSpeedValue: TextView
    private lateinit var signalMotionValue: TextView
    private lateinit var aiSpeedSparkline: SparklineView
    private lateinit var motionSparkline: SparklineView

    private val locationPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) NavigationSessionController.onLocationPermissionGranted(this)
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_home)

        systemStatusDot = findViewById(R.id.systemStatusDot)
        systemStatusLabel = findViewById(R.id.systemStatusLabel)
        chipGnssDot = findViewById(R.id.chipGnssDot)
        chipGnssValue = findViewById(R.id.chipGnssValue)
        chipImuDot = findViewById(R.id.chipImuDot)
        chipImuValue = findViewById(R.id.chipImuValue)
        chipAiDot = findViewById(R.id.chipAiDot)
        chipAiValue = findViewById(R.id.chipAiValue)
        chipDvfcDot = findViewById(R.id.chipDvfcDot)
        chipDvfcValue = findViewById(R.id.chipDvfcValue)
        speedValue = findViewById(R.id.speedValue)
        aiSpeedValue = findViewById(R.id.aiSpeedValue)
        headingValue = findViewById(R.id.headingValue)
        motionValue = findViewById(R.id.motionValue)
        zuptDot = findViewById(R.id.zuptDot)
        zaruDot = findViewById(R.id.zaruDot)
        homeCompass = findViewById(R.id.homeCompass)
        compassReadout = findViewById(R.id.compassReadout)

        positionValue = findViewById(R.id.positionValue)
        badgeGnss = findViewById(R.id.badgeGnss)
        badgeGnssValue = findViewById(R.id.badgeGnssValue)
        badgeImu = findViewById(R.id.badgeImu)
        badgeImuValue = findViewById(R.id.badgeImuValue)
        badgeDvfc = findViewById(R.id.badgeDvfc)
        badgeDvfcValue = findViewById(R.id.badgeDvfcValue)
        readinessAccuracyValue = findViewById(R.id.readinessAccuracyValue)
        readinessConfidenceValue = findViewById(R.id.readinessConfidenceValue)
        readinessDvfcOffsetValue = findViewById(R.id.readinessDvfcOffsetValue)
        signalAiSpeedValue = findViewById(R.id.signalAiSpeedValue)
        signalMotionValue = findViewById(R.id.signalMotionValue)
        aiSpeedSparkline = findViewById(R.id.aiSpeedSparkline)
        motionSparkline = findViewById(R.id.motionSparkline)
        aiSpeedSparkline.setColorRes(R.color.cyan)
        motionSparkline.setColorRes(R.color.good)

        findViewById<View>(R.id.navHome).setOnClickListener { /* already here */ }
        findViewById<View>(R.id.navSensors).setOnClickListener { openSensorDiagnostics() }
        findViewById<View>(R.id.navDvfc).setOnClickListener {
            startActivity(Intent(this, DVFCActivity::class.java))
        }
        findViewById<View>(R.id.navTrips).setOnClickListener {
//            Toast.makeText(this, "Trips screen isn't built yet", Toast.LENGTH_SHORT).show()
            startActivity(Intent(this, TripReplayActivity::class.java))
        }
        findViewById<View>(R.id.navSettings).setOnClickListener {
            startActivity(Intent(this, SettingsActivity::class.java))
        }

        findViewById<View>(R.id.positionCard).setOnClickListener {
            // TODO: no map screen exists yet in this repo — wire it here once built.
            Toast.makeText(this, "Map view isn't built yet", Toast.LENGTH_SHORT).show()
        }


        findViewById<View>(R.id.btnInfo).setOnClickListener {
            startActivity(
                Intent(
                    this,
                    MLTestActivity::class.java
                )
            )
        }

        findViewById<View>(R.id.systemStatusCard).setOnClickListener { openSensorDiagnostics() }
        findViewById<View>(R.id.btnViewSystem).setOnClickListener { openSensorDiagnostics() }
        findViewById<View>(R.id.btnStartNav).setOnClickListener {
            // TODO: no Active Navigation screen exists yet in this repo — wire it here once built.
            Toast.makeText(this, "Active navigation screen isn't built yet", Toast.LENGTH_SHORT).show()
        }
        findViewById<View>(R.id.lastTripCard).setOnClickListener {
           // TODO: no Trip Analytics screen/storage exists yet — this card is still mock data, honestly.
            Toast.makeText(this, "Trip analytics isn't wired up yet", Toast.LENGTH_SHORT).show()

        }

        lifecycleScope.launch {
            NavigationSessionController.state.collect { render(it) }
        }
    }

    override fun onResume() {
        super.onResume()
        NavigationSessionController.start(this)
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != android.content.pm.PackageManager.PERMISSION_GRANTED
        ) {
            locationPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
        }
    }

    override fun onPause() {
        super.onPause()
        NavigationSessionController.stop()
    }

    /** Live telemetry screen — distinct from the boot-time SensorCheckActivity
     * capability scan. Diagnostics itself offers a way into that scan
     * ("Run full sensor capability scan") for anyone who wants it. */
    private fun openSensorDiagnostics() {
        startActivity(Intent(this, SensorDiagnosticsActivity::class.java))
    }

    private fun render(s: com.rishabh.astranav.navigation.HomeDashboardState) {
        // ---- Overall system status ----
        val (label, color, dotRes) = when (s.nav.mode) {
            NavigationMode.GNSS_AVAILABLE -> Triple("READY", R.color.good, R.drawable.dot_good)
            NavigationMode.DEGRADED, NavigationMode.TRANSITION -> Triple("GNSS DEGRADED", R.color.warn, R.drawable.dot_warn)
            NavigationMode.GNSS_DENIED, NavigationMode.REACQUIRING -> Triple("DEAD RECKONING", R.color.cyan, R.drawable.dot_cyan)
        }
        systemStatusLabel.text = label
        systemStatusLabel.setTextColor(ContextCompat.getColor(this, color))
        systemStatusDot.setBackgroundResource(dotRes)

        // ---- GNSS chip ----
        chipGnssValue.text = when {
            !s.gnssAvailable -> "No fix"
            s.gnssAccuracyM != null -> "±${s.gnssAccuracyM.roundToInt()} m"
            else -> "Locked"
        }
        chipGnssDot.setBackgroundResource(if (s.gnssAvailable) R.drawable.dot_good else R.drawable.dot_warn)

        // ---- IMU chip ----
        chipImuValue.text = if (s.imuAvailable && s.imuHz != null) "${s.imuHz.roundToInt()} Hz" else "No sensor"
        chipImuDot.setBackgroundResource(if (s.imuAvailable) R.drawable.dot_good else R.drawable.dot_warn)

        // ---- AI chip ----
        chipAiValue.text = if (s.aiAvailable) "Active" else "Warming up"
        chipAiDot.setBackgroundResource(if (s.aiAvailable) R.drawable.dot_good else R.drawable.dot_warn)

        // ---- DVFC chip ----
        chipDvfcValue.text = if (s.dvfcCalibrated) "Calibrated" else "Needs setup"
        chipDvfcDot.setBackgroundResource(if (s.dvfcCalibrated) R.drawable.dot_good else R.drawable.dot_warn)

        // ---- Hero: speed + AI speed ----
        val speedKmh = (s.nav.speedMps * 3.6).roundToInt()
        speedValue.text = speedKmh.toString()
        aiSpeedValue.text = if (s.aiAvailable && s.aiSpeedMps != null) {
            "AI · ${(s.aiSpeedMps * 3.6).roundToInt()} km/h"
        } else {
            "AI speed unavailable"
        }

        // ---- Heading + motion ----
        val headingText = headingLabel(s.nav.headingDegrees)
        headingValue.text = headingText
        compassReadout.text = headingText
        val compassHealth = when (s.nav.mode) {
            NavigationMode.GNSS_AVAILABLE -> "good"
            NavigationMode.DEGRADED, NavigationMode.TRANSITION -> "warn"
            else -> "cyan"
        }
        homeCompass.setHeading(s.nav.headingDegrees, compassHealth)

        motionValue.text = when (s.motion) {
            MotionActivity.STATIONARY -> "Stationary"
            MotionActivity.DRIVING -> "Driving"
            MotionActivity.TURNING -> "Turning"
        }

        // ---- ZUPT / ZARU pills ----
        zuptDot.setBackgroundResource(if (s.zuptActive) R.drawable.dot_good else R.drawable.dot_warn)
        zaruDot.setBackgroundResource(if (s.zaruActive) R.drawable.dot_good else R.drawable.dot_warn)

        // ---- Position card ----
        positionValue.text = if (s.gnssLatitude != null && s.gnssLongitude != null) {
            String.format(Locale.US, "%.4f, %.4f", s.gnssLatitude, s.gnssLongitude)
        } else {
            "Waiting for GNSS fix…"
        }

        // ---- System readiness badges ----
        bindBadge(badgeGnss, badgeGnssValue, s.gnssAvailable, if (s.gnssAvailable) "Online" else "No fix")
        bindBadge(badgeImu, badgeImuValue, s.imuAvailable, if (s.imuAvailable) "Online" else "Offline")
        bindBadge(badgeDvfc, badgeDvfcValue, s.dvfcCalibrated, if (s.dvfcCalibrated) "Calibrated" else "Needs setup")

        readinessAccuracyValue.text = if (s.nav.positionSigmaMeters > 0) {
            String.format(Locale.US, "%.1f m", s.nav.positionSigmaMeters)
        } else {
            "-- m"
        }
        readinessConfidenceValue.text = "${s.nav.confidencePercent}%"
        readinessDvfcOffsetValue.text = if (s.dvfcCalibrated && s.dvfcYawOffsetDeg != null) {
            String.format(Locale.US, "%+.1f°", s.dvfcYawOffsetDeg)
        } else {
            "Uncalibrated"
        }

        // ---- Vehicle signal sparklines ----
        signalAiSpeedValue.text = if (s.aiAvailable && s.aiSpeedMps != null) {
            String.format(Locale.US, "%.1f km/h", s.aiSpeedMps * 3.6)
        } else {
            "-- km/h"
        }
        signalMotionValue.text = motionValue.text
        aiSpeedSparkline.submit(s.aiSpeedHistory, autoScaleToMax = true)
        motionSparkline.submit(s.motionEnergyHistory, autoScaleToMax = false)
    }

    private fun bindBadge(badgeView: View, valueView: TextView, healthy: Boolean, text: String) {
        badgeView.setBackgroundResource(if (healthy) R.drawable.bg_badge_good else R.drawable.bg_badge_warn)
        valueView.text = text
        valueView.setTextColor(ContextCompat.getColor(this, if (healthy) R.color.good else R.color.warn))
    }

    private fun headingLabel(deg: Double): String {
        val headings = listOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        val idx = (Math.round(deg / 45.0).toInt()).mod(8)
        return String.format(Locale.US, "%s · %03d°", headings[idx], deg.roundToInt().mod(360))
    }
}
