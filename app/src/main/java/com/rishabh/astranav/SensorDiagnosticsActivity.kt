package com.rishabh.astranav

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.rishabh.astranav.dvfc.DvfcCalibrationStore
import com.rishabh.astranav.home.SparklineView
import com.rishabh.astranav.navigation.HomeDashboardState
import com.rishabh.astranav.navigation.NavigationSessionController
import kotlinx.coroutines.launch
import java.util.Locale
import kotlin.math.roundToInt

/**
 * "System Diagnostics" — a live read of the same NavigationSessionController
 * singleton HomeActivity already drives, not a re-run of the boot-time
 * SensorCheckActivity capability scan. Anything not actually wired into the
 * live pipeline yet (map matching) is labeled as such instead of shown with
 * invented numbers, matching how DVFCQualityActivity handles missing evidence.
 */
class SensorDiagnosticsActivity : AppCompatActivity() {

    private lateinit var imuStatusDot: View
    private lateinit var imuStatusText: TextView
    private lateinit var imuSampleRate: TextView
    private lateinit var imuAccelMag: TextView
    private lateinit var imuGyroMag: TextView

    private lateinit var gnssStatusDot: View
    private lateinit var gnssStatusText: TextView
    private lateinit var gnssAccuracy: TextView
    private lateinit var gnssFusedSpeed: TextView
    private lateinit var gnssLastFix: TextView

    private lateinit var aiStatusDot: View
    private lateinit var aiStatusText: TextView
    private lateinit var aiSpeed: TextView

    private lateinit var calibDot: View
    private lateinit var calibChip: TextView
    private lateinit var calibYawOffset: TextView
    private lateinit var calibAge: TextView

    private lateinit var accelValue: TextView
    private lateinit var gyroValue: TextView
    private lateinit var speedValueSignal: TextView
    private lateinit var accelSparkline: SparklineView
    private lateinit var gyroSparkline: SparklineView
    private lateinit var speedSparkline: SparklineView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_sensor_diagnostics)

        imuStatusDot = findViewById(R.id.imuStatusDot)
        imuStatusText = findViewById(R.id.imuStatusText)
        imuSampleRate = findViewById(R.id.imuSampleRate)
        imuAccelMag = findViewById(R.id.imuAccelMag)
        imuGyroMag = findViewById(R.id.imuGyroMag)

        gnssStatusDot = findViewById(R.id.gnssStatusDot)
        gnssStatusText = findViewById(R.id.gnssStatusText)
        gnssAccuracy = findViewById(R.id.gnssAccuracy)
        gnssFusedSpeed = findViewById(R.id.gnssFusedSpeed)
        gnssLastFix = findViewById(R.id.gnssLastFix)

        aiStatusDot = findViewById(R.id.aiStatusDot)
        aiStatusText = findViewById(R.id.aiStatusText)
        aiSpeed = findViewById(R.id.aiSpeed)

        calibDot = findViewById(R.id.calibDot)
        calibChip = findViewById(R.id.calibChip)
        calibYawOffset = findViewById(R.id.calibYawOffset)
        calibAge = findViewById(R.id.calibAge)

        accelValue = findViewById(R.id.accelValue)
        gyroValue = findViewById(R.id.gyroValue)
        speedValueSignal = findViewById(R.id.speedValueSignal)
        accelSparkline = findViewById(R.id.accelSparkline)
        gyroSparkline = findViewById(R.id.gyroSparkline)
        speedSparkline = findViewById(R.id.speedSparkline)
        accelSparkline.setColorRes(R.color.blue)
        gyroSparkline.setColorRes(R.color.cyan)
        speedSparkline.setColorRes(R.color.good)

        findViewById<View>(R.id.btnBack).setOnClickListener { finish() }

        findViewById<View>(R.id.btnRecalibrate).setOnClickListener {
            startActivity(Intent(this, DVFCActivity::class.java))
        }

        findViewById<View>(R.id.btnFullScan).setOnClickListener {
            startActivity(Intent(this, SensorCheckActivity::class.java))
        }

        renderCalibration()

        lifecycleScope.launch {
            NavigationSessionController.state.collect { render(it) }
        }
    }

    override fun onResume() {
        super.onResume()
        NavigationSessionController.start(this)
    }

    override fun onPause() {
        super.onPause()
        NavigationSessionController.stop()
    }

    private fun render(s: HomeDashboardState) {
        // ---- IMU ----
        bindDot(imuStatusDot, imuStatusText, s.imuAvailable, "ACTIVE", "NO SENSOR")
        imuSampleRate.text = s.imuHz?.let { String.format(Locale.US, "%.0f Hz", it) } ?: "—"
        imuAccelMag.text = s.accelMagnitude?.let { String.format(Locale.US, "%.2f m/s²", it) } ?: "—"
        imuGyroMag.text = s.gyroMagnitude?.let { String.format(Locale.US, "%.2f rad/s", it) } ?: "—"

        // ---- GNSS ----
        bindDot(gnssStatusDot, gnssStatusText, s.gnssAvailable, "FIX", "NO FIX")
        gnssAccuracy.text = s.gnssAccuracyM?.let { String.format(Locale.US, "±%.0f m", it) } ?: "—"
        gnssFusedSpeed.text = String.format(Locale.US, "%.0f km/h", s.nav.speedMps * 3.6)
        gnssLastFix.text = if (s.gnssLatitude != null && s.gnssLongitude != null) {
            String.format(Locale.US, "%.3f, %.3f", s.gnssLatitude, s.gnssLongitude)
        } else {
            "—"
        }

        // ---- AI ----
        bindDot(aiStatusDot, aiStatusText, s.aiAvailable, "ACTIVE", "WARMING UP")
        aiSpeed.text = s.aiSpeedMps?.let { String.format(Locale.US, "%.0f km/h", it * 3.6) } ?: "—"

        // ---- Live signal tiles ----
        accelValue.text = s.accelMagnitude?.let { String.format(Locale.US, "%.2f m/s²", it) } ?: "—"
        gyroValue.text = s.gyroMagnitude?.let { String.format(Locale.US, "%.2f rad/s", it) } ?: "—"
        speedValueSignal.text = String.format(Locale.US, "%.0f km/h", s.nav.speedMps * 3.6)

        accelSparkline.submit(s.accelHistory, autoScaleToMax = true)
        gyroSparkline.submit(s.gyroHistory, autoScaleToMax = true)
        speedSparkline.submit(s.speedHistory, autoScaleToMax = true)

        // Calibration's "locked" status can change (e.g. after a recalibration
        // in another activity), so refresh it alongside the live telemetry too.
        renderCalibration()
    }

    private fun renderCalibration() {
        val transform = DvfcCalibrationStore.load(this)
        val calibrated = transform != null
        val lockedAt = DvfcCalibrationStore.lockedAtMillis(this)

        calibDot.setBackgroundResource(if (calibrated) R.drawable.dot_good else R.drawable.dot_warn)
        calibChip.text = if (calibrated) "LOCKED" else "PENDING"
        calibChip.setBackgroundResource(
            if (calibrated) R.drawable.bg_dvfc_status_chip_pass else R.drawable.bg_dvfc_status_chip_waiting
        )
        calibChip.setTextColor(ContextCompat.getColor(this, if (calibrated) R.color.good else R.color.warn))

        calibYawOffset.text = transform?.toEulerDegrees()?.get(2)
            ?.let { String.format(Locale.US, "%+.1f°", it) }
            ?: "—"

        calibAge.text = when {
            lockedAt == null -> "Not yet"
            else -> relativeTime(lockedAt)
        }
    }

    private fun bindDot(dot: View, text: TextView, healthy: Boolean, onLabel: String, offLabel: String) {
        dot.setBackgroundResource(if (healthy) R.drawable.dot_good else R.drawable.dot_warn)
        text.text = if (healthy) onLabel else offLabel
        text.setTextColor(ContextCompat.getColor(this, if (healthy) R.color.good else R.color.warn))
    }

    private fun relativeTime(thenMillis: Long): String {
        val deltaS = ((System.currentTimeMillis() - thenMillis) / 1000L).coerceAtLeast(0L)
        return when {
            deltaS < 60 -> "Just now"
            deltaS < 3600 -> "${deltaS / 60}m ago"
            deltaS < 86400 -> "${deltaS / 3600}h ago"
            else -> "${deltaS / 86400}d ago"
        }
    }
}
