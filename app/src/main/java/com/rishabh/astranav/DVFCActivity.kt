package com.rishabh.astranav

import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import com.rishabh.astranav.dvfc.CalibrationStatus
import com.rishabh.astranav.dvfc.DVFCController
import com.rishabh.astranav.dvfc.render.DvfcSceneRenderer
import io.github.sceneview.SceneView
import kotlinx.coroutines.launch
import java.util.Locale

/**
 * Device → Vehicle Frame Calibration screen.
 *
 * This Activity is intentionally thin: it wires DVFCController's state to
 * the XML views and to DvfcSceneRenderer, and nothing else. All the actual
 * DVFC math/sensor-fusion logic lives in the dvfc.* classes, per the spec's
 * suggested architecture (§15):
 *
 *   DVFCActivity → DVFCController → SensorFusion / DeviceVehicleTransform → (nav pipeline)
 *                                 → DvfcSceneRenderer → 3D phone node
 */
class DVFCActivity : AppCompatActivity() {

    private lateinit var controller: DVFCController
    private lateinit var renderer: DvfcSceneRenderer

    private lateinit var rollValue: TextView
    private lateinit var pitchValue: TextView
    private lateinit var yawValue: TextView
    private lateinit var headingOffsetValue: TextView
    private lateinit var statusText: TextView
    private lateinit var statusDot: View
    private lateinit var statusCard: View
    private lateinit var recalibrateButton: Button
    private lateinit var sensorsUnavailableBanner: View

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_dvfcactivity)

        val sceneView = findViewById<SceneView>(R.id.sceneView)
        rollValue = findViewById(R.id.rollValue)
        pitchValue = findViewById(R.id.pitchValue)
        yawValue = findViewById(R.id.yawValue)
        headingOffsetValue = findViewById(R.id.headingOffsetValue)
        statusText = findViewById(R.id.calibrationStatusText)
        statusDot = findViewById(R.id.calibrationStatusDot)
        statusCard = findViewById(R.id.calibrationStatusCard)
        recalibrateButton = findViewById(R.id.btnRecalibrate)
        sensorsUnavailableBanner = findViewById(R.id.sensorsUnavailableBanner)

        renderer = DvfcSceneRenderer(sceneView, lifecycleScope)
        renderer.loadPhoneModel(
            onError = { e -> android.util.Log.e(TAG, "Failed to load phone GLB", e) },
        )

        controller = DVFCController(applicationContext)

        recalibrateButton.setOnClickListener { controller.recalibrate() }

        lifecycleScope.launch {
            controller.state.collect { state ->
                sensorsUnavailableBanner.visibility = if (state.sensorsAvailable) View.GONE else View.VISIBLE

                rollValue.text = formatDeg(state.rollDeg)
                pitchValue.text = formatDeg(state.pitchDeg)
                yawValue.text = formatDeg(state.yawDeg)
                headingOffsetValue.text = formatDeg(state.headingOffsetDeg)

                statusText.text = labelFor(state.status)
                statusDot.setBackgroundResource(dotFor(state.status))
                statusCard.setBackgroundResource(
                    if (state.status == CalibrationStatus.COMPLETE) R.drawable.bg_status_complete else R.drawable.bg_status_default,
                )

                renderer.updateOrientation(state.currentQuaternion)

                val complete = state.status == CalibrationStatus.COMPLETE
                recalibrateButton.isEnabled = complete
                recalibrateButton.alpha = if (complete) 1f else 0.5f
                recalibrateButton.text = if (complete) "Recalibrate" else "Calibrating…"
            }
        }
    }

    override fun onResume() {
        super.onResume()
        controller.start()
    }

    override fun onPause() {
        super.onPause()
        controller.stop()
    }

    private fun formatDeg(v: Float) = String.format(Locale.US, "%+.1f°", v)

    private fun labelFor(status: CalibrationStatus) = when (status) {
        CalibrationStatus.STABILIZING -> "Stabilizing"
        CalibrationStatus.ALIGNING -> "Aligning"
        CalibrationStatus.VALIDATING -> "Validating"
        CalibrationStatus.COMPLETE -> "Calibration Complete"
    }

    private fun dotFor(status: CalibrationStatus) = when (status) {
        CalibrationStatus.COMPLETE -> R.drawable.dot_good
        CalibrationStatus.VALIDATING -> R.drawable.dot_cyan
        else -> R.drawable.dot_warn
    }

    /**
     * Call this once CalibrationStatus.COMPLETE to hand the locked transform
     * off to the navigation/ESKF pipeline (spec §9). Not called
     * automatically — wire it to wherever your nav pipeline expects it.
     */
    fun exportTransformToNavigationPipeline() {
        val transform = controller.lockedDeviceToVehicleTransform() ?: return
        // TODO: feed `transform` into your ESKF / ASTRANAV nav pipeline.
    }

    companion object {
        private const val TAG = "DVFCActivity"
    }
}
