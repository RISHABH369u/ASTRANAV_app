package com.rishabh.astranav

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.animation.AnimationUtils
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

/** XML-layout host. State/navigation is deliberately kept outside the layouts. */
class MainActivity : AppCompatActivity() {
    private val handler = Handler(Looper.getMainLooper())

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        show("splash")
    }

    private fun show(screen: String) {
        val layout = resources.getIdentifier("screen_$screen", "layout", packageName)
        if (layout != 0) {
            setContentView(layout)

            val animRes = if (screen == "active_nav") {
                resources.getIdentifier("nav_in", "anim", packageName)
            } else {
                resources.getIdentifier("screen_in", "anim", packageName)
            }
            if (animRes != 0) {
                window.decorView.startAnimation(AnimationUtils.loadAnimation(this, animRes))
            }

            if (screen == "splash") {
                handler.postDelayed({ show("sensor_check") }, 1500)
                return
            }

            when (screen) {
                "home" -> bindHome()
                "active_nav" -> findViewById<View>(R.id.action_exit_nav)?.setOnClickListener { show("home") }
                "dvfc_quality" -> bindDvfcQuality()
                else -> bindList(screen)
            }
        }
    }

    private fun bindHome() {
        findViewById<View>(R.id.action_start_nav)?.setOnClickListener { show("active_nav") }
        findViewById<View>(R.id.action_sensors)?.setOnClickListener { show("diagnostics") }
        findViewById<View>(R.id.action_trips)?.setOnClickListener { show("trip_analytics") }
        bindBottomNav()
    }

    private fun bindBottomNav() {
        mapOf(
            R.id.nav_navigate to "home",
            R.id.nav_trips to "trip_analytics",
            R.id.nav_sensors to "diagnostics",
            R.id.nav_maps to "offline_maps",
            R.id.nav_settings to "settings"
        ).forEach { (id, screen) ->
            findViewById<View>(id)?.setOnClickListener { show(screen) }
        }
    }

    /**
     * DVFC validation station.
     *
     * Current values are UI/demo telemetry until the live SensorAdapter + DVFC
     * telemetry stream is connected. They are deliberately exposed as individual
     * fields so the real engine can replace them without changing the XML.
     */
    private fun bindDvfcQuality() {
        val score = 94

        findViewById<TextView>(R.id.dvfc_quality_score)?.text = score.toString()
        findViewById<TextView>(R.id.dvfc_quality_state)?.text =
            "READY · CALIBRATION CONVERGED"
        findViewById<ProgressBar>(R.id.dvfc_quality_progress)?.progress = score

        findViewById<TextView>(R.id.adapter_time_sync)?.text =
            "TIME SYNC     ✓ LOCKED     drift 1.8 ms"
        findViewById<TextView>(R.id.adapter_units)?.text =
            "UNITS         ✓ NORMALIZED  accel · m/s²  gyro · rad/s"
        findViewById<TextView>(R.id.adapter_gaps)?.text =
            "DATA GAPS     ✓ HEALTHY   max 8 ms · dropped 2"
        findViewById<TextView>(R.id.adapter_resampling)?.text =
            "RESAMPLING    ✓ STABLE    100 Hz → 100 Hz"
        findViewById<TextView>(R.id.adapter_gravity)?.text =
            "GRAVITY       ✓ STABLE    9.806 m/s² · 0.04% error"

        findViewById<TextView>(R.id.score_manual)?.text =
            "MANUAL ALIGNMENT        96%   ✓"
        findViewById<TextView>(R.id.score_gravity)?.text =
            "GRAVITY LEVELING        94%   ✓"
        findViewById<TextView>(R.id.score_gyro)?.text =
            "GYRO BIAS ESTIMATION    91%   ✓"
        findViewById<TextView>(R.id.score_azimuth)?.text =
            "MOUNT AZIMUTH           89%   ✓"
        findViewById<TextView>(R.id.score_refinement)?.text =
            "AUTO REFINEMENT          93%   ✓"
        findViewById<TextView>(R.id.score_mount)?.text =
            "MOUNT STABILITY          97%   ✓"

        findViewById<TextView>(R.id.gnss_accuracy)?.text = "4.2 m"
        findViewById<TextView>(R.id.gnss_details)?.text =
            "42.1 km/h · 284 valid samples"

        findViewById<TextView>(R.id.excitation_value)?.text = "GOOD"
        findViewById<TextView>(R.id.excitation_details)?.text =
            "3.82 m²/s⁴ · 72% valid"

        findViewById<TextView>(R.id.azimuth_estimate)?.text =
            "Manual +18.4°     Auto +17.9°     Δ 0.5°"
        findViewById<TextView>(R.id.azimuth_samples)?.text =
            "284 valid samples · residual 0.7° · circular consistency 94%"
        findViewById<ProgressBar>(R.id.azimuth_progress)?.progress = 94

        findViewById<TextView>(R.id.stationary_stability)?.text =
            "STATIONARY STABILITY     95%   ✓   18.4 s"
        findViewById<TextView>(R.id.gravity_stability)?.text =
            "GRAVITY STABILITY        96%   ✓   variance 0.014"
        findViewById<TextView>(R.id.mount_status)?.text =
            "MOUNT-CHANGE STATUS      ✓ STABLE   Δ yaw 0.4°"

        findViewById<TextView>(R.id.final_yaw)?.text = "Yaw offset       +17.9°"
        findViewById<TextView>(R.id.final_pitch)?.text = "Pitch correction  −2.1°"
        findViewById<TextView>(R.id.final_roll)?.text = "Roll correction   +4.7°"
        findViewById<TextView>(R.id.final_bias)?.text =
            "Gyro bias         X 0.008 · Y 0.011 · Z 0.013"
        findViewById<TextView>(R.id.final_output_state)?.text =
            "✓ VEHICLE FRAME READY · SAFE TO HAND OFF TO ESKF"

        findViewById<Button>(R.id.action_use_calibration)?.setOnClickListener {
            show("home")
        }

        findViewById<Button>(R.id.action_recalibrate)?.setOnClickListener {
            show("alignment")
        }
    }

    private fun bindList(screen: String) {
        val data = when (screen) {
            "sensor_check" -> Triple(
                "STARTUP · 1 / 3",
                "Checking sensors",
                listOf(
                    "GNSS receiver · Connected",
                    "Inertial measurement unit · Healthy",
                    "Vehicle motion model · Ready"
                )
            )
            "alignment", "calibration" -> Triple(
                "STARTUP · 2 / 3",
                "Device–Vehicle Alignment",
                listOf(
                    "Place phone in vehicle holder",
                    "Align phone forward axis with vehicle",
                    "Yaw · −22°    Pitch · 8°    Roll · −5°"
                )
            )
            "diagnostics" -> Triple(
                "SENSOR HEALTH",
                "Diagnostics",
                listOf(
                    "GNSS · Excellent · ±1.8 m",
                    "IMU · Nominal",
                    "AI fusion · 98% confidence"
                )
            )
            "trip_analytics" -> Triple(
                "TRIPS",
                "Trip analytics",
                listOf(
                    "Today · 48.2 km · 01:14",
                    "Dead-reckoning coverage · 100%",
                    "View trip replay"
                )
            )
            "offline_maps" -> Triple(
                "MAPS",
                "Offline maps",
                listOf(
                    "Greater London · Downloaded",
                    "Route corridor · 18 MB",
                    "Manage map downloads"
                )
            )
            "trip_replay" -> Triple(
                "TRIP REPLAY",
                "Morning commute",
                listOf(
                    "08:12 → 08:42 · 12.4 km",
                    "GNSS outage recovered with IMU + AI",
                    "Playback position · 06:18"
                )
            )
            "settings" -> Triple(
                "SETTINGS",
                "Navigation setup",
                listOf(
                    "Recalibrate alignment",
                    "Phone mounting changed",
                    "Units · km/h"
                )
            )
            "system_check" -> Triple(
                "SYSTEM CHECK",
                "Everything is ready",
                listOf(
                    "Device mounting · Verified",
                    "Sensor fusion · Online",
                    "Navigation permissions · Granted"
                )
            )
            "sensor_setup" -> Triple(
                "SENSOR SETUP",
                "Connect your sensors",
                listOf(
                    "Location access · Enabled",
                    "Motion activity · Enabled",
                    "Background navigation · Enabled"
                )
            )
            "mounting_change" -> Triple(
                "MOUNTING CHANGE",
                "Mount position changed",
                listOf(
                    "Your reference frame needs recalibration",
                    "Keep phone fixed in holder",
                    "Continue to alignment"
                )
            )
            "quick_check" -> Triple(
                "QUICK CHECK",
                "Ready before you drive",
                listOf(
                    "Signal quality · Excellent",
                    "Alignment · Verified",
                    "Battery optimization · Disabled"
                )
            )
            "system_initialization" -> Triple(
                "INITIALIZATION",
                "Preparing ASTRA NAV",
                listOf(
                    "Loading map corridor",
                    "Calibrating fusion pipeline",
                    "Validating vehicle constraints"
                )
            )
            else -> Triple(
                "ONBOARDING",
                "Navigation that keeps going",
                listOf(
                    "ASTRA NAV continues with dead reckoning",
                    "AI fusion combines GNSS, IMU and vehicle constraints",
                    "Set up your navigation system"
                )
            )
        }

        findViewById<TextView>(R.id.screen_kicker)?.text = data.first
        findViewById<TextView>(R.id.screen_title)?.text = data.second
        findViewById<TextView>(R.id.screen_subtitle)?.text =
            "Precision navigation designed for your vehicle."

        val cards = findViewById<LinearLayout>(R.id.content_cards)
        cards?.removeAllViews()
        data.third.forEach { addCard(cards, it) }

        findViewById<Button>(R.id.action_primary)?.apply {
            text = when (screen) {
                "sensor_check" -> "CONTINUE TO ALIGNMENT"
                "alignment", "calibration" -> "CHECK CALIBRATION QUALITY"
                else -> "CONTINUE"
            }
            setOnClickListener {
                show(
                    when (screen) {
                        "sensor_check" -> "alignment"
                        "alignment", "calibration" -> "dvfc_quality"
                        else -> "home"
                    }
                )
            }
        }
    }

    private fun addCard(parent: LinearLayout?, value: String) {
        if (parent == null) return
        val cardLayout = resources.getIdentifier("item_info_card", "layout", packageName)
        if (cardLayout != 0) {
            layoutInflater.inflate(cardLayout, parent, false).also { card ->
                card.findViewById<TextView>(R.id.card_title)?.text =
                    value.substringBefore(" ·")
                card.findViewById<TextView>(R.id.card_detail)?.text =
                    value.substringAfter(" ·", "Verified and operating normally")
                parent.addView(card)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacksAndMessages(null)
    }
}
