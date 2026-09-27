package com.rishabh.astranav

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import com.rishabh.astranav.dvfc.CalibrationStatus
import com.rishabh.astranav.dvfc.DVFCController
import com.rishabh.astranav.dvfc.render.DvfcSceneRenderer
import com.rishabh.astranav.dvfc.render.HeadingCompassView
import io.github.sceneview.SceneView
import kotlinx.coroutines.launch
import java.util.Locale

/**
 * Device → Vehicle Frame Calibration screen.
 *
 * Flow:
 *
 * SensorCheck
 *      ↓
 * DVFCActivity
 *      ↓
 * DVFCController
 *      ↓
 * SensorFusion / DeviceVehicleTransform
 *      ↓
 * CalibrationStatus.COMPLETE
 *      ↓
 * DVFCQualityActivity
 *
 * DVFCActivity is intentionally thin.
 * All calibration math remains inside dvfc.*.
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
    private lateinit var headingCompass: HeadingCompassView

    /**
     * Prevents DVFCQualityActivity from being launched
     * multiple times because StateFlow can emit COMPLETE
     * more than once.
     */
    private var qualityScreenOpened = false

    override fun onCreate(
        savedInstanceState: Bundle?
    ) {
        super.onCreate(savedInstanceState)

        setContentView(
            R.layout.activity_dvfc
        )

        // -------------------------------------------------
        // VIEW REFERENCES
        // -------------------------------------------------

        val sceneView =
            findViewById<SceneView>(
                R.id.sceneView
            )

        rollValue =
            findViewById(
                R.id.rollValue
            )

        pitchValue =
            findViewById(
                R.id.pitchValue
            )

        yawValue =
            findViewById(
                R.id.yawValue
            )

        headingOffsetValue =
            findViewById(
                R.id.headingOffsetValue
            )

        statusText =
            findViewById(
                R.id.calibrationStatusText
            )

        statusDot =
            findViewById(
                R.id.calibrationStatusDot
            )

        statusCard =
            findViewById(
                R.id.calibrationStatusCard
            )

        recalibrateButton =
            findViewById(
                R.id.btnRecalibrate
            )

        sensorsUnavailableBanner =
            findViewById(
                R.id.sensorsUnavailableBanner
            )

        headingCompass =
            findViewById(
                R.id.headingCompass
            )


        // -------------------------------------------------
        // 3D PHONE RENDERER
        // -------------------------------------------------

        renderer =
            DvfcSceneRenderer(
                sceneView,
                lifecycleScope
            )

        renderer.loadPhoneModel(
            onError = { error ->
                android.util.Log.e(
                    TAG,
                    "Failed to load phone GLB",
                    error
                )
            }
        )


        // -------------------------------------------------
        // DVFC CONTROLLER
        // -------------------------------------------------

        controller =
            DVFCController(
                applicationContext
            )


        // -------------------------------------------------
        // RECALIBRATE
        // -------------------------------------------------

        recalibrateButton.setOnClickListener {

            /*
             * If the user manually recalibrates,
             * allow the quality screen to open again
             * after the new calibration completes.
             */
            qualityScreenOpened = false

            controller.recalibrate()
        }


        // -------------------------------------------------
        // OBSERVE DVFC STATE
        // -------------------------------------------------

        lifecycleScope.launch {

            controller.state.collect { state ->

                // -----------------------------------------
                // SENSOR AVAILABILITY
                // -----------------------------------------

                sensorsUnavailableBanner.visibility =
                    if (state.sensorsAvailable) {
                        View.GONE
                    } else {
                        View.VISIBLE
                    }


                // -----------------------------------------
                // LIVE TELEMETRY
                // -----------------------------------------

                rollValue.text =
                    formatDeg(
                        state.rollDeg
                    )

                pitchValue.text =
                    formatDeg(
                        state.pitchDeg
                    )

                yawValue.text =
                    formatDeg(
                        state.yawDeg
                    )

                headingOffsetValue.text =
                    formatDeg(
                        state.headingOffsetDeg
                    )


                // -----------------------------------------
                // COMPASS
                // -----------------------------------------

                headingCompass.setHeadingOffsetDeg(
                    state.headingOffsetDeg
                )


                // -----------------------------------------
                // STATUS
                // -----------------------------------------

                statusText.text =
                    labelFor(
                        state.status
                    )

                statusDot.setBackgroundResource(
                    dotFor(
                        state.status
                    )
                )

                statusCard.setBackgroundResource(

                    if (
                        state.status ==
                        CalibrationStatus.COMPLETE
                    ) {

                        R.drawable.bg_status_complete

                    } else {

                        R.drawable.bg_status_default
                    }
                )


                // -----------------------------------------
                // 3D PHONE ORIENTATION
                // -----------------------------------------

                renderer.updateOrientation(
                    state.currentQuaternion
                )


                // -----------------------------------------
                // CALIBRATION BUTTON
                // -----------------------------------------

                val complete =
                    state.status ==
                            CalibrationStatus.COMPLETE

                recalibrateButton.isEnabled =
                    complete

                recalibrateButton.alpha =
                    if (complete) {
                        1f
                    } else {
                        0.5f
                    }

                recalibrateButton.text =
                    if (complete) {
                        "Recalibrate"
                    } else {
                        "Calibrating…"
                    }


                // -----------------------------------------
                // OPEN QUALITY SCREEN ONLY ONCE
                // -----------------------------------------

                if (
                    complete &&
                    !qualityScreenOpened
                ) {

                    qualityScreenOpened = true

                    openQualityScreen()
                }
            }
        }
    }


    // =====================================================
    // QUALITY SCREEN
    // =====================================================

    private fun openQualityScreen() {

        val transform =
            controller.lockedDeviceToVehicleTransform()

        val intent =
            Intent(
                this,
                DVFCQualityActivity::class.java
            )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_TRANSFORM_VALID,
            transform != null
        )

        startActivityForResult(
            intent,
            REQUEST_DVFC_QUALITY
        )
    }


    // =====================================================
    // QUALITY SCREEN RESULT
    // =====================================================

    @Deprecated(
        "Use Activity Result API when modernizing this Activity."
    )
    override fun onActivityResult(
        requestCode: Int,
        resultCode: Int,
        data: Intent?
    ) {
        super.onActivityResult(
            requestCode,
            resultCode,
            data
        )

        if (
            requestCode ==
            REQUEST_DVFC_QUALITY
        ) {

            if (
                resultCode ==
                RESULT_OK
            ) {

                /*
                 * Quality screen accepted the
                 * calibration.
                 *
                 * This is where the final transform
                 * should eventually be handed to the
                 * navigation / ESKF pipeline.
                 */
                exportTransformToNavigationPipeline()

            } else {

                /*
                 * User returned without accepting.
                 *
                 * Allow QualityActivity to be opened
                 * again if DVFC is still COMPLETE.
                 */
                qualityScreenOpened = false
            }
        }
    }


    // =====================================================
    // LIFECYCLE
    // =====================================================

    override fun onResume() {
        super.onResume()

        controller.start()
    }


    override fun onPause() {
        super.onPause()

        controller.stop()
    }


    // =====================================================
    // TRANSFORM → NAVIGATION
    // =====================================================

    /**
     * Hands the locked Device → Vehicle transform
     * to the navigation pipeline.
     *
     * Currently the navigation pipeline is not wired,
     * so this method only obtains the validated transform.
     *
     * TODO:
     * Feed this transform into the ESKF initialization
     * / navigation core once that module is connected.
     */
    private fun exportTransformToNavigationPipeline() {

        val transform =
            controller.lockedDeviceToVehicleTransform()
                ?: return

        /*
         * TODO:
         *
         * navigationCore.setDeviceToVehicleTransform(
         *     transform
         * )
         *
         * Do NOT implement fake navigation state here.
         */
    }


    // =====================================================
    // FORMATTERS
    // =====================================================

    private fun formatDeg(
        value: Float
    ): String {

        return String.format(
            Locale.US,
            "%+.1f°",
            value
        )
    }


    private fun labelFor(
        status: CalibrationStatus
    ): String {

        return when (status) {

            CalibrationStatus.STABILIZING ->
                "Stabilizing"

            CalibrationStatus.ALIGNING ->
                "Aligning"

            CalibrationStatus.VALIDATING ->
                "Validating"

            CalibrationStatus.COMPLETE ->
                "Calibration Complete"
        }
    }


    private fun dotFor(
        status: CalibrationStatus
    ): Int {

        return when (status) {

            CalibrationStatus.COMPLETE ->
                R.drawable.dot_good

            CalibrationStatus.VALIDATING ->
                R.drawable.dot_cyan

            else ->
                R.drawable.dot_warn
        }
    }


    companion object {

        private const val TAG =
            "DVFCActivity"

        private const val REQUEST_DVFC_QUALITY =
            2001
    }
}