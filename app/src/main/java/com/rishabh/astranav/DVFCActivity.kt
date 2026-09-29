package com.rishabh.astranav

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.TextView

import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope

import com.rishabh.astranav.dvfc.CalibrationStatus
import com.rishabh.astranav.dvfc.DVFCController
import com.rishabh.astranav.dvfc.render.DvfcSceneRenderer
import com.rishabh.astranav.dvfc.render.HeadingCompassView

import io.github.sceneview.SceneView

import kotlinx.coroutines.launch

import java.util.Locale


/**
 * Device → Vehicle Frame Calibration Activity.
 *
 * Flow:
 *
 * SensorCheck
 *      ↓
 * DVFCActivity
 *      ↓
 * DVFCController
 *      ↓
 * SensorFusion
 *      ↓
 * DeviceVehicleTransform
 *      ↓
 * CalibrationStatus.COMPLETE
 *      ↓
 * DVFCQualityActivity
 *
 * GNSS refinement:
 *
 * Android Location
 *      ↓
 * COG + speed + accuracy
 *      +
 * Gyroscope yaw-rate
 *      ↓
 * GnssAzimuthEstimator
 *      ↓
 * DVFCController StateFlow
 *      ↓
 * DVFCQualityActivity
 */
class DVFCActivity : AppCompatActivity() {

    // =========================================================
    // CONTROLLER / RENDERER
    // =========================================================

    private lateinit var controller:
            DVFCController

    private lateinit var renderer:
            DvfcSceneRenderer

    // =========================================================
    // UI
    // =========================================================

    private lateinit var rollValue:
            TextView

    private lateinit var pitchValue:
            TextView

    private lateinit var yawValue:
            TextView

    private lateinit var headingOffsetValue:
            TextView

    private lateinit var statusText:
            TextView

    private lateinit var statusDot:
            View

    private lateinit var statusCard:
            View

    private lateinit var recalibrateButton:
            Button

    private lateinit var sensorsUnavailableBanner:
            View

    private lateinit var headingCompass:
            HeadingCompassView

    // =========================================================
    // QUALITY SCREEN
    // =========================================================

    private var qualityScreenOpened =
        false

    // =========================================================
    // LOCATION PERMISSION
    // =========================================================

    private val locationPermissionLauncher =
        registerForActivityResult(
            ActivityResultContracts
                .RequestMultiplePermissions()
        ) {

            /*
             * Permission result received.
             *
             * Controller checks the actual permission
             * state itself before starting GNSS.
             */
            controller.refreshGnssPermission()
        }

    // =========================================================
    // ON CREATE
    // =========================================================

    override fun onCreate(
        savedInstanceState: Bundle?
    ) {

        super.onCreate(
            savedInstanceState
        )

        setContentView(
            R.layout.activity_dvfc
        )

        // =====================================================
        // VIEW REFERENCES
        // =====================================================

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

        // =====================================================
        // 3D PHONE RENDERER
        // =====================================================

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

        // =====================================================
        // CONTROLLER
        // =====================================================

        controller =
            DVFCController(
                applicationContext
            )

        // =====================================================
        // RECALIBRATE
        // =====================================================

        recalibrateButton.setOnClickListener {

            qualityScreenOpened =
                false

            controller.recalibrate()
        }

        // =====================================================
        // OBSERVE CONTROLLER
        // =====================================================

        lifecycleScope.launch {

            controller.state.collect { state ->

                // =================================================
                // SENSOR AVAILABILITY
                // =================================================

                sensorsUnavailableBanner.visibility =
                    if (
                        state.sensorsAvailable
                    ) {

                        View.GONE

                    } else {

                        View.VISIBLE
                    }

                // =================================================
                // LIVE ORIENTATION
                // =================================================

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

                // =================================================
                // COMPASS
                // =================================================

                headingCompass
                    .setHeadingOffsetDeg(
                        state.headingOffsetDeg
                    )

                // =================================================
                // STATUS
                // =================================================

                statusText.text =
                    labelFor(
                        state.status
                    )

                statusDot
                    .setBackgroundResource(
                        dotFor(
                            state.status
                        )
                    )

                statusCard
                    .setBackgroundResource(

                        if (
                            state.status ==
                            CalibrationStatus.COMPLETE
                        ) {

                            R.drawable
                                .bg_status_complete

                        } else {

                            R.drawable
                                .bg_status_default
                        }
                    )

                // =================================================
                // 3D PHONE
                // =================================================

                renderer.updateOrientation(
                    state.currentQuaternion
                )

                // =================================================
                // CALIBRATION BUTTON
                // =================================================

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

                // =================================================
                // OPEN QUALITY SCREEN
                // =================================================

                if (
                    complete &&
                    !qualityScreenOpened
                ) {

                    qualityScreenOpened =
                        true

                    openQualityScreen()
                }
            }
        }

        // =====================================================
        // LOCATION PERMISSION
        // =====================================================

        requestLocationPermissionIfNeeded()
    }

    // =========================================================
    // LOCATION PERMISSION
    // =========================================================

    private fun requestLocationPermissionIfNeeded() {

        val fineGranted =
            ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) ==
                    PackageManager.PERMISSION_GRANTED

        val coarseGranted =
            ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_COARSE_LOCATION
            ) ==
                    PackageManager.PERMISSION_GRANTED

        if (
            !fineGranted &&
            !coarseGranted
        ) {

            locationPermissionLauncher.launch(

                arrayOf(

                    Manifest.permission
                        .ACCESS_FINE_LOCATION,

                    Manifest.permission
                        .ACCESS_COARSE_LOCATION
                )
            )

        } else {

            controller
                .refreshGnssPermission()
        }
    }

    // =========================================================
    // QUALITY SCREEN
    // =========================================================

    private fun openQualityScreen() {

        val transform =
            controller
                .lockedDeviceToVehicleTransform()

        val state =
            controller.state.value

        val intent =
            Intent(
                this,
                DVFCQualityActivity::class.java
            )

        // =====================================================
        // TRANSFORM
        // =====================================================

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_TRANSFORM_VALID,

            transform != null
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_CALIBRATION_COMPLETE,

            state.status ==
                    CalibrationStatus.COMPLETE
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_YAW,

            state.yawDeg.toDouble()
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_PITCH,

            state.pitchDeg.toDouble()
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_ROLL,

            state.rollDeg.toDouble()
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_HEADING_OFFSET,

            state.headingOffsetDeg.toDouble()
        )

        // =====================================================
        // SENSOR ADAPTER
        // =====================================================

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_SAMPLE_COUNT,

            state.sampleCount
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_TIMESTAMP_JITTER,

            state.timestampJitterMs
                ?: -1.0
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_SAMPLE_HZ,

            state.estimatedSampleHz
                ?: -1.0
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GAP_COUNT,

            state.dataGapCount
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_MAX_GAP,

            state.maxGapMs
                ?: -1.0
        )

        // =====================================================
// RESAMPLING
// =====================================================

        intent.putExtra(
            DVFCQualityActivity.EXTRA_RESAMPLING_ACTIVE,
            state.resamplingActive
        )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_RESAMPLING_RATE_HZ,
            state.resamplingRateHz ?: -1.0
        )

// =====================================================
// GRAVITY / COMPENSATION
// =====================================================

        intent.putExtra(
            DVFCQualityActivity.EXTRA_GRAVITY_MAGNITUDE,
            state.gravityMagnitude ?: -1.0
        )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_GRAVITY_STABLE,
            state.gravityStable
        )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_GRAVITY_ROLL,
            state.gravityLevelRollDeg ?: 0.0
        )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_GRAVITY_PITCH,
            state.gravityLevelPitchDeg ?: 0.0
        )

        intent.putExtra(
            DVFCQualityActivity.EXTRA_LINEAR_ACCELERATION_MAGNITUDE,
            state.linearAccelerationMagnitude ?: -1.0
        )

        // =====================================================
        // STATIONARY
        // =====================================================

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_STATIONARY_SAMPLES,

            state.stationarySampleCount
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_STATIONARY_SCORE,

            state.stationaryScore
                ?: -1.0
        )

        // =====================================================
        // GYRO
        // =====================================================

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GYRO_BIAS_X,

            state.gyroBiasX
                ?: Double.NaN
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GYRO_BIAS_Y,

            state.gyroBiasY
                ?: Double.NaN
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GYRO_BIAS_Z,

            state.gyroBiasZ
                ?: Double.NaN
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GYRO_RMS,

            state.gyroMagnitudeRms
                ?: -1.0
        )

        // =====================================================
        // SENSOR AVAILABILITY
        // =====================================================

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_SENSORS_AVAILABLE,

            state.sensorsAvailable
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GRAVITY_AVAILABLE,

            state.gravityAvailable
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_ACCELERATION_AVAILABLE,

            state.accelerationAvailable
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_GNSS_AVAILABLE,

            state.gnssAvailable
        )

        intent.putExtra(

            DVFCQualityActivity
                .EXTRA_AUTO_AZIMUTH_AVAILABLE,

            state.automaticAzimuthAvailable
        )

        // =====================================================
        // GNSS TELEMETRY
        // =====================================================

        /*
         * These extras are only used as the initial snapshot.
         *
         * The actual GNSS estimator continues running in
         * DVFCController while the Quality screen is open
         * only if the controller is kept alive by the app.
         */

        intent.putExtra(
            "dvfc_gnss_accuracy_m",
            state.gnssAccuracyM
                ?: -1.0
        )

        intent.putExtra(
            "dvfc_gnss_speed_mps",
            state.gnssSpeedMps
                ?: -1.0
        )

        intent.putExtra(
            "dvfc_gnss_valid_samples",
            state.gnssValidSamples
                ?: 0
        )

        intent.putExtra(
            "dvfc_automatic_azimuth_deg",
            state.automaticAzimuthDeg
                ?: Double.NaN
        )

        intent.putExtra(
            "dvfc_azimuth_residual_deg",
            state.azimuthResidualDeg
                ?: Double.NaN
        )

        intent.putExtra(
            "dvfc_azimuth_consistency",
            state.azimuthConsistency
                ?: Double.NaN
        )

        // =====================================================
        // OPEN
        // =====================================================

        startActivityForResult(
            intent,
            REQUEST_DVFC_QUALITY
        )
    }

    // =========================================================
    // QUALITY RESULT
    // =========================================================

    @Deprecated(
        "Use Activity Result API when modernizing."
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

                exportTransformToNavigationPipeline()

            } else {

                /*
                 * User returned without accepting.
                 *
                 * Allow Quality screen to open again
                 * if calibration is still complete.
                 */
                qualityScreenOpened =
                    false
            }
        }
    }

    // =========================================================
    // LIFECYCLE
    // =========================================================

    override fun onResume() {

        super.onResume()

        controller.start()
    }

    override fun onPause() {

        super.onPause()

        /*
         * IMPORTANT:
         *
         * Do NOT stop the controller here.
         *
         * Quality screen is launched immediately after
         * calibration completion.
         *
         * If we call controller.stop() here,
         * GNSS refinement dies exactly when Quality screen
         * opens.
         *
         * The controller is stopped when this Activity is
         * actually destroyed / recalibration flow ends.
         */
    }

    override fun onDestroy() {

        /*
         * Only stop if this Activity is genuinely being
         * destroyed and no Quality screen is being kept alive.
         *
         * For now we leave the controller object alive during
         * the Quality flow.
         */
        super.onDestroy()
    }

    // =========================================================
    // TRANSFORM → NAVIGATION
    // =========================================================

    private fun exportTransformToNavigationPipeline() {

        val transform =
            controller
                .lockedDeviceToVehicleTransform()
                ?: return

        /*
         * TODO:
         *
         * navigationCore
         *     .setDeviceToVehicleTransform(
         *         transform
         *     )
         *
         * Do NOT create fake navigation state here.
         */
    }

    // =========================================================
    // FORMATTERS
    // =========================================================

    private fun formatDeg(
        value: Float
    ): String {

        return String.format(
            Locale.US,
            "%+.1f°",
            value
        )
    }

    // =========================================================
    // STATUS LABEL
    // =========================================================

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

    // =========================================================
    // STATUS DOT
    // =========================================================

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

    // =========================================================
    // CONSTANTS
    // =========================================================

    companion object {

        private const val TAG =
            "DVFCActivity"

        private const val REQUEST_DVFC_QUALITY =
            2001
    }
}