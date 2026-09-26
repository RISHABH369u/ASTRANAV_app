package com.rishabh.astranav

import android.animation.ObjectAnimator
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.view.animation.DecelerateInterpolator
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

/**
 * One row in the checklist. Either [sensorType] (checked via SensorManager)
 * or [customCheck] (for anything that isn't a Sensor object, e.g. GPS
 * hardware) should be supplied.
 */
private data class SensorCheckItem(
    val label: String,
    val sensorType: Int? = null,
    val essential: Boolean = true,
    val customCheck: (() -> Boolean)? = null
)

/**
 * Runs a real hardware check (not a decorative fake sequence) for the
 * sensors this app's dead-reckoning pipeline depends on, then lets the
 * user continue into the app.
 *
 * Note: this only checks for hardware *presence*. Runtime location
 * permission (for the GPS updates used elsewhere in the app) should still
 * be requested where that data is actually consumed, not here.
 */
class SensorCheckActivity : AppCompatActivity() {

    private val handler = Handler(Looper.getMainLooper())
    private lateinit var sensorManager: SensorManager
    private var missingEssential = false

    private val checklist by lazy {
        listOf(
            SensorCheckItem("Accelerometer", sensorType = Sensor.TYPE_ACCELEROMETER, essential = true),
            SensorCheckItem("Gyroscope", sensorType = Sensor.TYPE_GYROSCOPE, essential = true),
            SensorCheckItem("Magnetometer", sensorType = Sensor.TYPE_MAGNETIC_FIELD, essential = true),
            SensorCheckItem("Barometer", sensorType = Sensor.TYPE_PRESSURE, essential = false),
            SensorCheckItem("Step Detector", sensorType = Sensor.TYPE_STEP_DETECTOR, essential = false),
            SensorCheckItem(
                "GPS Receiver",
                essential = true,
                customCheck = { packageManager.hasSystemFeature(PackageManager.FEATURE_LOCATION_GPS) }
            )
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_sensor_check)

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager

        val list = findViewById<LinearLayout>(R.id.sensorList)
        val inflater = LayoutInflater.from(this)

        checklist.forEachIndexed { index, item ->
            val row = inflater.inflate(R.layout.item_sensor_check, list, false)
            row.findViewById<TextView>(R.id.sensorName).text = item.label
            list.addView(row)

            handler.postDelayed({ revealRow(row) }, 150L * index)
            handler.postDelayed({ resolveRow(row, item) }, 150L * index + 420L)
        }

        val totalDelay = 150L * checklist.size + 420L + 300L
        handler.postDelayed({ showSummary() }, totalDelay)

        findViewById<View>(R.id.continueButton).setOnClickListener { goToMain() }
    }

    private fun revealRow(row: View) {
        row.translationY = 8f
        ObjectAnimator.ofFloat(row, View.ALPHA, 0f, 1f).setDuration(350).start()
        ObjectAnimator.ofFloat(row, View.TRANSLATION_Y, 8f, 0f).apply {
            duration = 350
            interpolator = DecelerateInterpolator()
            start()
        }
    }

    private fun resolveRow(row: View, item: SensorCheckItem) {
        val statusDot = row.findViewById<View>(R.id.statusDot)
        val statusIcon = row.findViewById<ImageView>(R.id.statusIcon)
        val statusText = row.findViewById<TextView>(R.id.sensorStatus)

        val available = item.customCheck?.invoke()
            ?: item.sensorType?.let { sensorManager.getDefaultSensor(it) != null }
            ?: false

        statusDot.visibility = View.GONE
        statusIcon.visibility = View.VISIBLE

        if (available) {
            statusIcon.setImageResource(R.drawable.ic_status_check)
            statusText.text = "OK"
            statusText.setTextColor(getColor(R.color.astra_cyan))
        } else {
            statusIcon.setImageResource(R.drawable.ic_status_warning)
            statusText.text = if (item.essential) "MISSING" else "N/A"
            statusText.setTextColor(getColor(R.color.astra_warning))
            if (item.essential) missingEssential = true
        }

        statusIcon.alpha = 0f
        statusIcon.scaleX = 0.6f
        statusIcon.scaleY = 0.6f
        ObjectAnimator.ofFloat(statusIcon, View.ALPHA, 0f, 1f).setDuration(200).start()
        ObjectAnimator.ofFloat(statusIcon, View.SCALE_X, 0.6f, 1f).setDuration(250).start()
        ObjectAnimator.ofFloat(statusIcon, View.SCALE_Y, 0.6f, 1f).setDuration(250).start()
    }

    private fun showSummary() {
        val summary = findViewById<TextView>(R.id.summaryText)
        val button = findViewById<View>(R.id.continueButton)

        summary.text = if (missingEssential)
            "Some required sensors are unavailable — accuracy may be reduced."
        else
            "All required sensors detected."

        summary.setTextColor(
            getColor(if (missingEssential) R.color.astra_warning else R.color.astra_text_secondary)
        )

        ObjectAnimator.ofFloat(summary, View.ALPHA, 0f, 1f).setDuration(350).start()
        ObjectAnimator.ofFloat(button, View.ALPHA, 0f, 1f).setDuration(350).start()

        // Everything present -> move on automatically after a beat.
        // Something essential missing -> wait for the user to tap Continue.
        if (!missingEssential) {
            handler.postDelayed({ goToMain() }, 1100)
        }
    }

    private fun goToMain() {
        startActivity(Intent(this, MainActivity::class.java))
        overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out)
        finish()
    }

    override fun onDestroy() {
        handler.removeCallbacksAndMessages(null)
        super.onDestroy()
    }
}
