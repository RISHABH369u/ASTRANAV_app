package com.rishabh.astranav

import android.app.Activity
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.ViewGroup
import android.widget.Button
import android.widget.SeekBar
import com.astranav.ui.CalibrationSceneView

/** Deterministic navigation state machine: no sensor, location, or permission APIs are used. */
class MainActivity : Activity() {
    override fun onCreate(state: Bundle?) {
        super.onCreate(state)
        setContentView(R.layout.activity_main)
        Handler(Looper.getMainLooper()).postDelayed({ showSensorCheck() }, 700)
    }

    private fun showSensorCheck() {
        val root = findViewById<ViewGroup>(R.id.astra_root) ?: return
        root.removeAllViews()
        val layoutId = resources.getIdentifier("screen_sensor_check", "layout", packageName)
        if (layoutId != 0) {
            layoutInflater.inflate(layoutId, root, true)
            val btnId = resources.getIdentifier("sensor_continue", "id", packageName)
            if (btnId != 0) {
                root.findViewById<Button>(btnId)?.setOnClickListener { showAlignment() }
            }
        }
    }

    private fun showAlignment() {
        val root = findViewById<ViewGroup>(R.id.astra_root) ?: return
        root.removeAllViews()
        val layoutId = resources.getIdentifier("screen_alignment", "layout", packageName)
        if (layoutId != 0) {
            layoutInflater.inflate(layoutId, root, true)
            val sceneId = resources.getIdentifier("calibration_scene", "id", packageName)
            val scene = if (sceneId != 0) root.findViewById<CalibrationSceneView>(sceneId) else null

            fun bind(idName: String, axis: Int) {
                val id = resources.getIdentifier(idName, "id", packageName)
                if (id != 0) {
                    root.findViewById<SeekBar>(id)?.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                        override fun onProgressChanged(s: SeekBar, p: Int, user: Boolean) {
                            scene?.setAxis(axis, p - 45f)
                        }

                        override fun onStartTrackingTouch(s: SeekBar) = Unit
                        override fun onStopTrackingTouch(s: SeekBar) = Unit
                    })
                }
            }

            bind("yaw", 0)
            bind("pitch", 1)
            bind("roll", 2)

            val resetId = resources.getIdentifier("reset", "id", packageName)
            if (resetId != 0) {
                root.findViewById<Button>(resetId)?.setOnClickListener {
                    scene?.reset()
                    listOf("yaw", "pitch", "roll").forEach { idName ->
                        val id = resources.getIdentifier(idName, "id", packageName)
                        if (id != 0) {
                            root.findViewById<SeekBar>(id)?.progress = 45
                        }
                    }
                }
            }
        }
    }
}


//---------------------------------------------------------------


//
//
//package com.rishabh.astranav
//
//import android.os.Bundle
//import android.os.Handler
//import android.os.Looper
//import android.view.View
//import android.view.animation.AnimationUtils
//import android.widget.Button
//import android.widget.LinearLayout
//import android.widget.TextView
//import androidx.appcompat.app.AppCompatActivity
//
///** XML-layout host. State/navigation is deliberately kept outside the layouts. */
//class MainActivity : AppCompatActivity() {
//    private val handler = Handler(Looper.getMainLooper())
//
//    override fun onCreate(savedInstanceState: Bundle?) {
//        super.onCreate(savedInstanceState)
//        show("splash")
//    }
//
//    private fun show(screen: String) {
//        val layout = resources.getIdentifier("screen_$screen", "layout", packageName)
//        if (layout != 0) {
//            setContentView(layout)
//
//            // Safe animation resolution
//            val animRes = if (screen == "active_nav") {
//                resources.getIdentifier("nav_in", "anim", packageName)
//            } else {
//                resources.getIdentifier("screen_in", "anim", packageName)
//            }
//            if (animRes != 0) {
//                window.decorView.startAnimation(AnimationUtils.loadAnimation(this, animRes))
//            }
//
//            if (screen == "splash") {
//                handler.postDelayed({ show("sensor_check") }, 1500)
//                return
//            }
//            if (screen == "home") bindHome()
//            if (screen == "active_nav") findViewById<View>(R.id.action_exit_nav)?.setOnClickListener { show("home") }
//            if (screen !in setOf("home", "active_nav")) bindList(screen)
//        }
//    }
//
//    private fun bindHome() {
//        findViewById<View>(R.id.action_start_nav)?.setOnClickListener { show("active_nav") }
//        findViewById<View>(R.id.action_sensors)?.setOnClickListener { show("diagnostics") }
//        findViewById<View>(R.id.action_trips)?.setOnClickListener { show("trip_analytics") }
//        bindBottomNav()
//    }
//
//    private fun bindBottomNav() {
//        mapOf(
//            R.id.nav_navigate to "home",
//            R.id.nav_trips to "trip_analytics",
//            R.id.nav_sensors to "diagnostics",
//            R.id.nav_maps to "offline_maps",
//            R.id.nav_settings to "settings"
//        ).forEach { (id, screen) ->
//            findViewById<View>(id)?.setOnClickListener { show(screen) }
//        }
//    }
//
//    private fun bindList(screen: String) {
//        val data = when (screen) {
//            "sensor_check" -> Triple("STARTUP · 1 / 3", "Checking sensors", listOf("GNSS receiver · Connected", "Inertial measurement unit · Healthy", "Vehicle motion model · Ready"))
//            "alignment", "calibration" -> Triple("STARTUP · 2 / 3", "Device–Vehicle Alignment", listOf("Place phone in vehicle holder", "Align phone forward axis with vehicle", "Yaw · −22°    Pitch · 8°    Roll · −5°"))
//            "diagnostics" -> Triple("SENSOR HEALTH", "Diagnostics", listOf("GNSS · Excellent · ±1.8 m", "IMU · Nominal", "AI fusion · 98% confidence"))
//            "trip_analytics" -> Triple("TRIPS", "Trip analytics", listOf("Today · 48.2 km · 01:14", "Dead-reckoning coverage · 100%", "View trip replay"))
//            "offline_maps" -> Triple("MAPS", "Offline maps", listOf("Greater London · Downloaded", "Route corridor · 18 MB", "Manage map downloads"))
//            "trip_replay" -> Triple("TRIP REPLAY", "Morning commute", listOf("08:12 → 08:42 · 12.4 km", "GNSS outage recovered with IMU + AI", "Playback position · 06:18"))
//            "settings" -> Triple("SETTINGS", "Navigation setup", listOf("Recalibrate alignment", "Phone mounting changed", "Units · km/h"))
//            "system_check" -> Triple("SYSTEM CHECK", "Everything is ready", listOf("Device mounting · Verified", "Sensor fusion · Online", "Navigation permissions · Granted"))
//            "sensor_setup" -> Triple("SENSOR SETUP", "Connect your sensors", listOf("Location access · Enabled", "Motion activity · Enabled", "Background navigation · Enabled"))
//            "mounting_change" -> Triple("MOUNTING CHANGE", "Mount position changed", listOf("Your reference frame needs recalibration", "Keep phone fixed in holder", "Continue to alignment"))
//            "quick_check" -> Triple("QUICK CHECK", "Ready before you drive", listOf("Signal quality · Excellent", "Alignment · Verified", "Battery optimization · Disabled"))
//            "system_initialization" -> Triple("INITIALIZATION", "Preparing ASTRA NAV", listOf("Loading map corridor", "Calibrating fusion pipeline", "Validating vehicle constraints"))
//            else -> Triple("ONBOARDING", "Navigation that keeps going", listOf("ASTRA NAV continues with dead reckoning", "AI fusion combines GNSS, IMU and vehicle constraints", "Set up your navigation system"))
//        }
//
//        findViewById<TextView>(R.id.screen_kicker)?.text = data.first
//        findViewById<TextView>(R.id.screen_title)?.text = data.second
//        findViewById<TextView>(R.id.screen_subtitle)?.text = "Precision navigation designed for your vehicle."
//
//        val cards = findViewById<LinearLayout>(R.id.content_cards)
//        cards?.removeAllViews()
//        data.third.forEach { addCard(cards, it) }
//
//        findViewById<Button>(R.id.action_primary)?.apply {
//            text = if (screen == "sensor_check") "CONTINUE TO ALIGNMENT" else if (screen == "alignment") "CONFIRM ALIGNMENT" else "CONTINUE"
//            setOnClickListener {
//                show(
//                    if (screen in setOf("sensor_check", "alignment", "calibration")) {
//                        if (screen == "sensor_check") "alignment" else "home"
//                    } else "home"
//                )
//            }
//        }
//    }
//
//    private fun addCard(parent: LinearLayout?, value: String) {
//        if (parent == null) return
//        val cardLayout = resources.getIdentifier("item_info_card", "layout", packageName)
//        if (cardLayout != 0) {
//            layoutInflater.inflate(cardLayout, parent, false).also { card ->
//                card.findViewById<TextView>(R.id.card_title)?.text = value.substringBefore(" ·")
//                card.findViewById<TextView>(R.id.card_detail)?.text = value.substringAfter(" ·", "Verified and operating normally")
//                parent.addView(card)
//            }
//        }
//    }
//
//    override fun onDestroy() {
//        super.onDestroy()
//        handler.removeCallbacksAndMessages(null)
//    }
//}
//
