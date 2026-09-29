package com.rishabh.astranav

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

import com.rishabh.astranav.dvfcquality.AdapterEvidence
import com.rishabh.astranav.dvfcquality.AzimuthEvidence
import com.rishabh.astranav.dvfcquality.CalibrationEvidence
import com.rishabh.astranav.dvfcquality.DvfcQualityEngine
import com.rishabh.astranav.dvfcquality.DvfcQualityInput
import com.rishabh.astranav.dvfcquality.EvidenceState
import com.rishabh.astranav.dvfcquality.ExcitationEvidence
import com.rishabh.astranav.dvfcquality.GnssEvidence
import com.rishabh.astranav.dvfcquality.StabilityEvidence
import com.rishabh.astranav.dvfcquality.TransformEvidence
import com.rishabh.astranav.ui.view.CircularGaugeView

import java.util.Locale


class DVFCQualityActivity : AppCompatActivity() {

    // ============================================================
    // INTENT EXTRAS
    // ============================================================

    companion object {

        const val EXTRA_TRANSFORM_VALID =
            "dvfc_transform_valid"

        const val EXTRA_CALIBRATION_COMPLETE =
            "dvfc_calibration_complete"

        const val EXTRA_YAW =
            "dvfc_yaw"

        const val EXTRA_PITCH =
            "dvfc_pitch"

        const val EXTRA_ROLL =
            "dvfc_roll"

        const val EXTRA_HEADING_OFFSET =
            "dvfc_heading_offset"

        const val EXTRA_SAMPLE_COUNT =
            "dvfc_sample_count"

        const val EXTRA_TIMESTAMP_JITTER =
            "dvfc_timestamp_jitter"

        const val EXTRA_SAMPLE_HZ =
            "dvfc_sample_hz"

        const val EXTRA_GAP_COUNT =
            "dvfc_gap_count"

        const val EXTRA_MAX_GAP =
            "dvfc_max_gap"

        const val EXTRA_STATIONARY_SAMPLES =
            "dvfc_stationary_samples"

        const val EXTRA_STATIONARY_SCORE =
            "dvfc_stationary_score"

        const val EXTRA_GYRO_BIAS_X =
            "dvfc_gyro_bias_x"

        const val EXTRA_GYRO_BIAS_Y =
            "dvfc_gyro_bias_y"

        const val EXTRA_GYRO_BIAS_Z =
            "dvfc_gyro_bias_z"

        const val EXTRA_GYRO_RMS =
            "dvfc_gyro_rms"

        const val EXTRA_SENSORS_AVAILABLE =
            "dvfc_sensors_available"

        const val EXTRA_GRAVITY_AVAILABLE =
            "dvfc_gravity_available"

        const val EXTRA_ACCELERATION_AVAILABLE =
            "dvfc_acceleration_available"

        const val EXTRA_GNSS_AVAILABLE =
            "dvfc_gnss_available"

        const val EXTRA_AUTO_AZIMUTH_AVAILABLE =
            "dvfc_auto_azimuth_available"

        const val EXTRA_DUPLICATE_TIMESTAMPS =
            "dvfc_duplicate_timestamps"

        const val EXTRA_RESAMPLING_ACTIVE =
            "dvfc_resampling_active"

        const val EXTRA_RESAMPLING_RATE_HZ =
            "dvfc_resampling_rate_hz"

        const val EXTRA_GRAVITY_MAGNITUDE =
            "dvfc_gravity_magnitude"

        const val EXTRA_GRAVITY_STABLE =
            "dvfc_gravity_stable"

        const val EXTRA_GRAVITY_ROLL =
            "dvfc_gravity_roll"

        const val EXTRA_GRAVITY_PITCH =
            "dvfc_gravity_pitch"

        const val EXTRA_LINEAR_ACCELERATION_MAGNITUDE =
            "dvfc_linear_acceleration_magnitude"

        // ------------------------------------------------------------
        // GNSS REFINEMENT
        // ------------------------------------------------------------

        const val EXTRA_GNSS_ACCURACY_M =
            "dvfc_gnss_accuracy_m"

        const val EXTRA_GNSS_SPEED_MPS =
            "dvfc_gnss_speed_mps"

        const val EXTRA_GNSS_VALID_SAMPLES =
            "dvfc_gnss_valid_samples"

        const val EXTRA_AUTOMATIC_AZIMUTH_DEG =
            "dvfc_automatic_azimuth_deg"

        const val EXTRA_AZIMUTH_RESIDUAL_DEG =
            "dvfc_azimuth_residual_deg"

        const val EXTRA_AZIMUTH_CONSISTENCY =
            "dvfc_azimuth_consistency"

    }


    // ============================================================
    // UI
    // ============================================================

    private lateinit var score: TextView
    private lateinit var status: TextView
    private lateinit var hint: TextView
    private lateinit var gauge: CircularGaugeView


    // ============================================================
    // TRANSFORM
    // ============================================================

    private var transformValid = false
    private var calibrationComplete = false

    private var yawDeg = 0.0
    private var pitchDeg = 0.0
    private var rollDeg = 0.0
    private var headingOffsetDeg = 0.0


    // ============================================================
    // SENSOR ADAPTER TELEMETRY
    // ============================================================

    private var sampleCount = 0

    private var timestampJitterMs = -1.0

    private var sampleHz = -1.0

    private var gapCount = 0

    private var maxGapMs = -1.0

    private var duplicateTimestampCount = 0

    private var resamplingActive = false

    private var resamplingRateHz = -1.0

    private var gravityMagnitude = -1.0

    private var gravityStable = false

    private var gravityLevelRollDeg = 0.0

    private var gravityLevelPitchDeg = 0.0

    private var linearAccelerationMagnitude = -1.0


    // ============================================================
    // STATIONARY / GYRO
    // ============================================================

    private var stationarySamples = 0

    private var stationaryScore = -1.0

    private var gyroBiasX = Double.NaN

    private var gyroBiasY = Double.NaN

    private var gyroBiasZ = Double.NaN

    private var gyroRms = -1.0


    // ============================================================
    // SENSOR AVAILABILITY
    // ============================================================

    private var sensorsAvailable = false

    private var gravityAvailable = false

    private var accelerationAvailable = false

    private var gnssAvailable = false

    private var autoAzimuthAvailable = false


    // ============================================================
    // GNSS TELEMETRY
    // ============================================================

    private var gnssAccuracyM: Double? = null

    private var gnssSpeedMps: Double? = null

    private var gnssValidSamples: Int? = null


    // ============================================================
    // AUTOMATIC AZIMUTH
    // ============================================================

    private var automaticAzimuthDeg: Double? = null

    private var azimuthResidualDeg: Double? = null

    private var azimuthConsistency: Double? = null


    // ============================================================
    // ACTIVITY
    // ============================================================

    override fun onCreate(
        savedInstanceState: Bundle?
    ) {

        super.onCreate(
            savedInstanceState
        )

        setContentView(
            R.layout.activity_dvfcquality
        )


        // ========================================================
        // READ INTENT TELEMETRY
        // ========================================================

        readIntentTelemetry()


        // ========================================================
        // BIND UI
        // ========================================================

        score =
            findViewById(
                R.id.dvfcQualityScore
            )

        status =
            findViewById(
                R.id.dvfcQualityStatus
            )

        hint =
            findViewById(
                R.id.dvfcQualityHint
            )

        gauge =
            findViewById(
                R.id.dvfcQualityGauge
            )


        // ========================================================
        // USE CALIBRATION
        // ========================================================

        findViewById<Button>(
            R.id.actionUseCalibration
        ).setOnClickListener {

            evaluateAndUseCalibration()

        }




        // ========================================================
        // RECALIBRATE
        // ========================================================

        findViewById<Button>(
            R.id.actionRecalibrate
        ).setOnClickListener {

            startActivity(
                Intent(
                    this,
                    DVFCActivity::class.java
                )
            )

            finish()
        }


        // ========================================================
        // INITIAL RENDER
        // ========================================================

        render(
            buildQualityInput()
        )
    }




    // ============================================================
    // READ TELEMETRY
    // ============================================================

    private fun readIntentTelemetry() {

        transformValid =
            intent.getBooleanExtra(
                EXTRA_TRANSFORM_VALID,
                false
            )


        calibrationComplete =
            intent.getBooleanExtra(
                EXTRA_CALIBRATION_COMPLETE,
                false
            )


        yawDeg =
            intent.getDoubleExtra(
                EXTRA_YAW,
                0.0
            )


        pitchDeg =
            intent.getDoubleExtra(
                EXTRA_PITCH,
                0.0
            )


        rollDeg =
            intent.getDoubleExtra(
                EXTRA_ROLL,
                0.0
            )


        headingOffsetDeg =
            intent.getDoubleExtra(
                EXTRA_HEADING_OFFSET,
                0.0
            )


        // ========================================================
        // SENSOR ADAPTER
        // ========================================================

        sampleCount =
            intent.getIntExtra(
                EXTRA_SAMPLE_COUNT,
                0
            )


        timestampJitterMs =
            intent.getDoubleExtra(
                EXTRA_TIMESTAMP_JITTER,
                -1.0
            )


        sampleHz =
            intent.getDoubleExtra(
                EXTRA_SAMPLE_HZ,
                -1.0
            )


        gapCount =
            intent.getIntExtra(
                EXTRA_GAP_COUNT,
                0
            )


        maxGapMs =
            intent.getDoubleExtra(
                EXTRA_MAX_GAP,
                -1.0
            )


        duplicateTimestampCount =
            intent.getIntExtra(
                EXTRA_DUPLICATE_TIMESTAMPS,
                0
            )


        resamplingActive =
            intent.getBooleanExtra(
                EXTRA_RESAMPLING_ACTIVE,
                false
            )


        resamplingRateHz =
            intent.getDoubleExtra(
                EXTRA_RESAMPLING_RATE_HZ,
                -1.0
            )


        gravityMagnitude =
            intent.getDoubleExtra(
                EXTRA_GRAVITY_MAGNITUDE,
                -1.0
            )


        gravityStable =
            intent.getBooleanExtra(
                EXTRA_GRAVITY_STABLE,
                false
            )


        gravityLevelRollDeg =
            intent.getDoubleExtra(
                EXTRA_GRAVITY_ROLL,
                0.0
            )


        gravityLevelPitchDeg =
            intent.getDoubleExtra(
                EXTRA_GRAVITY_PITCH,
                0.0
            )


        linearAccelerationMagnitude =
            intent.getDoubleExtra(
                EXTRA_LINEAR_ACCELERATION_MAGNITUDE,
                -1.0
            )


        // ========================================================
        // STATIONARY
        // ========================================================

        stationarySamples =
            intent.getIntExtra(
                EXTRA_STATIONARY_SAMPLES,
                0
            )


        stationaryScore =
            intent.getDoubleExtra(
                EXTRA_STATIONARY_SCORE,
                -1.0
            )


        // ========================================================
        // GYRO BIAS
        // ========================================================

        gyroBiasX =
            intent.getDoubleExtra(
                EXTRA_GYRO_BIAS_X,
                Double.NaN
            )


        gyroBiasY =
            intent.getDoubleExtra(
                EXTRA_GYRO_BIAS_Y,
                Double.NaN
            )


        gyroBiasZ =
            intent.getDoubleExtra(
                EXTRA_GYRO_BIAS_Z,
                Double.NaN
            )


        gyroRms =
            intent.getDoubleExtra(
                EXTRA_GYRO_RMS,
                -1.0
            )


        // ========================================================
        // SENSOR AVAILABILITY
        // ========================================================

        sensorsAvailable =
            intent.getBooleanExtra(
                EXTRA_SENSORS_AVAILABLE,
                false
            )


        gravityAvailable =
            intent.getBooleanExtra(
                EXTRA_GRAVITY_AVAILABLE,
                false
            )


        accelerationAvailable =
            intent.getBooleanExtra(
                EXTRA_ACCELERATION_AVAILABLE,
                false
            )


        gnssAvailable =
            intent.getBooleanExtra(
                EXTRA_GNSS_AVAILABLE,
                false
            )


        autoAzimuthAvailable =
            intent.getBooleanExtra(
                EXTRA_AUTO_AZIMUTH_AVAILABLE,
                false
            )


        // ========================================================
        // GNSS REFINEMENT
        // ========================================================

        gnssAccuracyM =
            readNullableDoubleExtra(
                EXTRA_GNSS_ACCURACY_M
            )


        gnssSpeedMps =
            readNullableDoubleExtra(
                EXTRA_GNSS_SPEED_MPS
            )


        gnssValidSamples =
            readNullableIntExtra(
                EXTRA_GNSS_VALID_SAMPLES
            )


        // ========================================================
        // AUTOMATIC AZIMUTH
        // ========================================================

        automaticAzimuthDeg =
            readNullableDoubleExtra(
                EXTRA_AUTOMATIC_AZIMUTH_DEG
            )


        azimuthResidualDeg =
            readNullableDoubleExtra(
                EXTRA_AZIMUTH_RESIDUAL_DEG
            )


        azimuthConsistency =
            readNullableDoubleExtra(
                EXTRA_AZIMUTH_CONSISTENCY
            )
    }


    // ============================================================
    // NULLABLE DOUBLE EXTRA
    // ============================================================

    private fun readNullableDoubleExtra(
        key: String
    ): Double? {

        if (!intent.hasExtra(key)) {
            return null
        }

        val value =
            intent.getDoubleExtra(
                key,
                Double.NaN
            )

        return if (value.isFinite()) {
            value
        } else {
            null
        }
    }


    // ============================================================
    // NULLABLE INT EXTRA
    // ============================================================

    private fun readNullableIntExtra(
        key: String
    ): Int? {

        if (!intent.hasExtra(key)) {
            return null
        }

        return intent.getIntExtra(
            key,
            -1
        ).takeIf {
            it >= 0
        }
    }


    // ============================================================
    // USE CALIBRATION
    // ============================================================

    private fun evaluateAndUseCalibration() {

        val input =
            buildQualityInput()

        val result =
            DvfcQualityEngine.evaluate(
                input
            )

        if (
            result.status == "READY" ||
            result.status == "DEGRADED"
        ) {

            setResult(RESULT_OK)

            startActivity(
                Intent(
                    this,
                    HomeActivity::class.java
                ).apply {
                    flags =
                        Intent.FLAG_ACTIVITY_CLEAR_TOP or
                                Intent.FLAG_ACTIVITY_SINGLE_TOP
                }
            )

            finish()



        } else {

            status.text =
                result.hardGateFailure
                    ?: "VALIDATION REQUIRED"

            status.setBackgroundResource(
                R.drawable.bg_dvfc_status_chip_crit
            )

            hint.text =
                "Calibration cannot be accepted yet. Complete the required evidence and try again."
        }
    }


    // ============================================================
    // BUILD QUALITY INPUT
    // ============================================================

    private fun buildQualityInput(): DvfcQualityInput {

        // ========================================================
        // TIMESTAMP SYNC
        // ========================================================

        val timestampState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                timestampJitterMs < 3.0 ->
                    EvidenceState.PASS

                timestampJitterMs < 8.0 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // ========================================================
        // DATA GAPS
        // ========================================================

        val gapState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                gapCount == 0 ->
                    EvidenceState.PASS

                gapCount <= 3 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // ========================================================
        // RESAMPLING
        // ========================================================

        val resamplingState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                resamplingRateHz <= 0.0 ->
                    EvidenceState.WAITING

                resamplingRateHz in 9.0..11.0 ->
                    EvidenceState.PASS

                resamplingRateHz in 8.0..12.0 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // ========================================================
        // UNIT NORMALIZATION
        // ========================================================

        val unitState =
            if (sensorsAvailable) {

                EvidenceState.PASS

            } else {

                EvidenceState.WAITING
            }


        // ========================================================
        // GRAVITY
        // ========================================================

        val gravityState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                !gravityAvailable ->
                    EvidenceState.FAIL

                gravityMagnitude < 0.0 ->
                    EvidenceState.WAITING

                gravityMagnitude !in 8.5..11.0 ->
                    EvidenceState.FAIL

                gravityMagnitude in 9.2..10.4 ->
                    EvidenceState.PASS

                else ->
                    EvidenceState.WARN
            }


        // ========================================================
        // GRAVITY LEVELING
        // ========================================================

        val gravityLevelingState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                !gravityAvailable ->
                    EvidenceState.FAIL

                gravityMagnitude < 8.5 ||
                        gravityMagnitude > 11.0 ->
                    EvidenceState.FAIL

                kotlin.math.abs(gravityLevelRollDeg) <= 15.0 &&
                        kotlin.math.abs(gravityLevelPitchDeg) <= 15.0 ->
                    EvidenceState.PASS

                else ->
                    EvidenceState.WARN
            }


        // ========================================================
        // MANUAL ALIGNMENT
        // ========================================================

        val manualState =
            if (calibrationComplete) {

                EvidenceState.PASS

            } else {

                EvidenceState.WAITING
            }


        // ========================================================
        // GYRO BIAS
        // ========================================================

        val gyroBiasState =
            when {

                gyroRms < 0.0 ->
                    EvidenceState.WAITING

                gyroRms < 0.05 ->
                    EvidenceState.PASS

                gyroRms < 0.10 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // ========================================================
        // MOUNT STABILITY
        // ========================================================

        val mountStabilityState =
            when {

                stationaryScore < 0.0 ->
                    EvidenceState.WAITING

                stationaryScore >= 0.90 ->
                    EvidenceState.PASS

                stationaryScore >= 0.70 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // ========================================================
        // MANUAL MOUNT AZIMUTH
        // ========================================================

        val mountAzimuthState =
            if (calibrationComplete) {

                EvidenceState.PASS

            } else {

                EvidenceState.WAITING
            }


        // ========================================================
        // AUTOMATIC GNSS REFINEMENT
        // ========================================================

        val autoAzimuth = automaticAzimuthDeg
        val azimuthRes = azimuthResidualDeg
        val consistency = azimuthConsistency

        val automaticRefinementState =
            when {

                !gnssAvailable ->
                    EvidenceState.WAITING

                !autoAzimuthAvailable ->
                    EvidenceState.WAITING

                autoAzimuth == null ->
                    EvidenceState.WAITING

                azimuthRes == null ->
                    EvidenceState.WAITING

                consistency == null ->
                    EvidenceState.WAITING

                consistency < 0.50 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.PASS
            }


        // ========================================================
        // TRANSFORM
        // ========================================================

        val transform =
            TransformEvidence(

                yawDeg =
                    yawDeg,

                pitchDeg =
                    pitchDeg,

                rollDeg =
                    rollDeg,

                gyroBiasX =
                    if (gyroBiasX.isNaN()) {
                        null
                    } else {
                        gyroBiasX
                    },

                gyroBiasY =
                    if (gyroBiasY.isNaN()) {
                        null
                    } else {
                        gyroBiasY
                    },

                gyroBiasZ =
                    if (gyroBiasZ.isNaN()) {
                        null
                    } else {
                        gyroBiasZ
                    },

                valid =
                    transformValid
            )


        // ========================================================
        // RETURN FULL QUALITY INPUT
        // ========================================================

        return DvfcQualityInput(

            // ----------------------------------------------------
            // ADAPTER
            // ----------------------------------------------------

            adapter =
                AdapterEvidence(

                    timestampSync =
                        timestampState,

                    timestampJitterMs =
                        if (timestampJitterMs >= 0.0) {
                            timestampJitterMs
                        } else {
                            null
                        },

                    units =
                        unitState,

                    dataGaps =
                        gapState,

                    gapCount =
                        gapCount,

                    maxGapMs =
                        if (maxGapMs >= 0.0) {
                            maxGapMs
                        } else {
                            null
                        },

                    resampling =
                        resamplingState,

                    sourceHz =
                        if (sampleHz >= 0.0) {
                            sampleHz
                        } else {
                            null
                        },

                    outputHz =
                        if (resamplingRateHz > 0.0) {
                            resamplingRateHz
                        } else {
                            null
                        },

                    gravity =
                        gravityState
                ),


            // ----------------------------------------------------
            // CALIBRATION
            // ----------------------------------------------------

            calibration =
                CalibrationEvidence(

                    manualAlignment =
                        manualState,

                    gravityLeveling =
                        gravityLevelingState,

                    gyroBias =
                        gyroBiasState,

                    mountAzimuth =
                        mountAzimuthState,

                    automaticRefinement =
                        automaticRefinementState,

                    mountStability =
                        mountStabilityState
                ),


            // ----------------------------------------------------
            // GNSS
            // ----------------------------------------------------

            gnss =
                GnssEvidence(

                    accuracyM =
                        gnssAccuracyM,

                    speedMps =
                        gnssSpeedMps,

                    validSamples =
                        gnssValidSamples
                ),


            // ----------------------------------------------------
            // EXCITATION
            // ----------------------------------------------------

            excitation =
                ExcitationEvidence(

                    score =
                        null,

                    usableSamples =
                        null,

                    longitudinalAccelerationRms =
                        null
                ),


            // ----------------------------------------------------
            // AZIMUTH
            // ----------------------------------------------------

            azimuth =
                AzimuthEvidence(

                    manualEstimateDeg =
                        if (calibrationComplete) {
                            yawDeg
                        } else {
                            null
                        },

                    automaticEstimateDeg =
                        automaticAzimuthDeg,

                    residualDeg =
                        azimuthResidualDeg,

                    circularConsistency =
                        azimuthConsistency,

                    validSamples =
                        gnssValidSamples
                ),


            // ----------------------------------------------------
            // STABILITY
            // ----------------------------------------------------

            stability =
                StabilityEvidence(

                    stationaryScore =
                        if (stationaryScore >= 0.0) {
                            stationaryScore
                        } else {
                            null
                        },

                    stationarySamples =
                        stationarySamples,

                    gravityScore =
                        if (
                            gravityAvailable &&
                            gravityMagnitude >= 0.0
                        ) {

                            gravityStabilityScore()

                        } else {
                            null
                        },

                    gravityStd =
                        null,

                    mountChanged =
                        false,

                    mountChangeConfirmedSamples =
                        0
                ),


            // ----------------------------------------------------
            // TRANSFORM
            // ----------------------------------------------------

            transform =
                transform
        )
    }


    // ============================================================
    // GRAVITY SCORE
    // ============================================================

    private fun gravityStabilityScore(): Double {

        if (!gravityMagnitude.isFinite()) {
            return 0.0
        }

        val error =
            kotlin.math.abs(
                gravityMagnitude - 9.80665
            )

        return (
                1.0 -
                        (
                                error /
                                        2.0
                                )
                )
            .coerceIn(
                0.0,
                1.0
            )
    }


    // ============================================================
    // RENDER
    // ============================================================

    private fun render(
        input: DvfcQualityInput
    ) {

        val result =
            DvfcQualityEngine.evaluate(
                input
            )


        // ========================================================
        // SCORE
        // ========================================================

        val numericScore =
            result.score
                .coerceIn(
                    0,
                    100
                )

        /*
         * IMPORTANT:
         *
         * Previous implementation displayed "—"
         * whenever engine status was WAITING FOR TELEMETRY.
         *
         * That made the UI look broken.
         *
         * We always display the engine's numeric score.
         */

        score.text =
            numericScore.toString()


        // ========================================================
        // STATUS
        // ========================================================

        status.text =
            result.hardGateFailure
                ?: result.status


        // ========================================================
        // HINT
        // ========================================================

        hint.text =
            statusHint(
                result.status,
                result.hardGateFailure
            )


        // ========================================================
        // STATUS CHIP + GAUGE
        // ========================================================

        val (
            chipDrawable,
            gaugeColor
        ) =
            when {

                result.hardGateFailure != null ->

                    R.drawable.bg_dvfc_status_chip_crit to
                            R.color.crit


                result.status == "READY" ->

                    R.drawable.bg_dvfc_status_chip_pass to
                            R.color.good


                result.status == "DEGRADED" ->

                    R.drawable.bg_dvfc_status_chip_warn to
                            R.color.warn


                result.status == "WAITING FOR TELEMETRY" ->

                    R.drawable.bg_dvfc_status_chip_waiting to
                            R.color.idle


                else ->

                    R.drawable.bg_dvfc_status_chip_crit to
                            R.color.crit
            }


        status.setBackgroundResource(
            chipDrawable
        )


        gauge.setProgress(
            value = numericScore,
            colorRes = gaugeColor
        )


        // ========================================================
        // ADAPTER
        // ========================================================

        bindState(
            R.id.adapterTimeSync,
            input.adapter.timestampSync
        )


        bindState(
            R.id.adapterUnits,
            input.adapter.units
        )


        bindState(
            R.id.adapterGaps,
            input.adapter.dataGaps
        )


        bindState(
            R.id.adapterResampling,
            input.adapter.resampling
        )


        bindState(
            R.id.adapterGravity,
            input.adapter.gravity
        )


        bindSectionDot(
            R.id.sectionDotAdapter,
            listOf(
                input.adapter.timestampSync,
                input.adapter.units,
                input.adapter.dataGaps,
                input.adapter.resampling,
                input.adapter.gravity
            )
        )


        // ========================================================
        // CALIBRATION
        // ========================================================

        bindState(
            R.id.scoreManual,
            input.calibration.manualAlignment
        )


        bindState(
            R.id.scoreGravity,
            input.calibration.gravityLeveling
        )


        bindState(
            R.id.scoreGyro,
            input.calibration.gyroBias
        )


        bindState(
            R.id.scoreAzimuth,
            input.calibration.mountAzimuth
        )


        bindState(
            R.id.scoreRefinement,
            input.calibration.automaticRefinement
        )


        bindState(
            R.id.scoreMount,
            input.calibration.mountStability
        )


        bindSectionDot(
            R.id.sectionDotCalibration,
            listOf(
                input.calibration.manualAlignment,
                input.calibration.gravityLeveling,
                input.calibration.gyroBias,
                input.calibration.mountAzimuth,
                input.calibration.automaticRefinement,
                input.calibration.mountStability
            )
        )


        // ========================================================
        // GNSS
        // ========================================================

        findViewById<TextView>(
            R.id.gnssAccuracy
        ).text =
            input.gnss.accuracyM?.let {

                String.format(
                    Locale.US,
                    "%.1f m",
                    it
                )

            } ?: "—"


        findViewById<TextView>(
            R.id.gnssSpeed
        ).text =
            input.gnss.speedMps?.let {

                String.format(
                    Locale.US,
                    "%.2f m/s",
                    it
                )

            } ?: "—"


        findViewById<TextView>(
            R.id.gnssSamples
        ).text =
            input.gnss.validSamples
                ?.toString()
                ?: "—"


        // ========================================================
        // EXCITATION
        // ========================================================

        findViewById<TextView>(
            R.id.excitationValue
        ).text =
            input.excitation.score?.let {

                String.format(
                    Locale.US,
                    "%.2f",
                    it
                )

            } ?: "—"


        findViewById<TextView>(
            R.id.excitationSamples
        ).text =
            input.excitation.usableSamples
                ?.toString()
                ?: "—"


        // ========================================================
        // AZIMUTH
        // ========================================================

        findViewById<TextView>(
            R.id.azimuthManual
        ).text =
            input.azimuth.manualEstimateDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.azimuthAutomatic
        ).text =
            input.azimuth.automaticEstimateDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.azimuthResidual
        ).text =
            input.azimuth.residualDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.azimuthConsistency
        ).text =
            input.azimuth.circularConsistency
                ?.let {

                    String.format(
                        Locale.US,
                        "%.2f",
                        it
                    )

                } ?: "—"


        findViewById<TextView>(
            R.id.azimuthSamples
        ).text =
            input.azimuth.validSamples
                ?.toString()
                ?: "—"


        // ========================================================
        // STABILITY
        // ========================================================

        findViewById<TextView>(
            R.id.stationaryScore
        ).text =
            input.stability.stationaryScore
                ?.let {

                    String.format(
                        Locale.US,
                        "%.2f",
                        it
                    )

                } ?: "—"


        findViewById<TextView>(
            R.id.stationarySamples
        ).text =
            "samples: " +
                    (
                            input.stability.stationarySamples
                                ?.toString()
                                ?: "—"
                            )


        findViewById<TextView>(
            R.id.gravityScore
        ).text =
            input.stability.gravityScore
                ?.let {

                    String.format(
                        Locale.US,
                        "%.2f",
                        it
                    )

                } ?: "—"


        // ========================================================
        // MOUNT STATUS
        // ========================================================

        val mountBanner =
            findViewById<View>(
                R.id.mountBanner
            )


        val mountStatusView =
            findViewById<TextView>(
                R.id.mountStatus
            )


        when (
            input.stability.mountChanged
        ) {

            true -> {

                mountStatusView.text =
                    "CHANGE CONFIRMED"

                mountStatusView.setTextColor(
                    getColor(
                        R.color.crit
                    )
                )

                mountBanner.setBackgroundResource(
                    R.drawable.bg_dvfc_mount_banner_changed
                )
            }


            false -> {

                mountStatusView.text =
                    "STABLE"

                mountStatusView.setTextColor(
                    getColor(
                        R.color.good
                    )
                )

                mountBanner.setBackgroundResource(
                    R.drawable.bg_dvfc_mount_banner_stable
                )
            }


            null -> {

                mountStatusView.text =
                    "WAITING"

                mountStatusView.setTextColor(
                    getColor(
                        R.color.fg_faint
                    )
                )

                mountBanner.setBackgroundResource(
                    R.drawable.bg_dvfc_mount_banner_waiting
                )
            }
        }


        // ========================================================
        // FINAL TRANSFORM
        // ========================================================

        findViewById<TextView>(
            R.id.finalYaw
        ).text =
            input.transform.yawDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.finalPitch
        ).text =
            input.transform.pitchDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.finalRoll
        ).text =
            input.transform.rollDeg
                ?.let {
                    formatDeg(it)
                }
                ?: "—"


        findViewById<TextView>(
            R.id.finalBias
        ).text =
            if (
                input.transform.gyroBiasX != null
            ) {

                String.format(
                    Locale.US,
                    "%.4f / %.4f / %.4f rad/s",

                    input.transform.gyroBiasX,

                    input.transform.gyroBiasY
                        ?: 0.0,

                    input.transform.gyroBiasZ
                        ?: 0.0
                )

            } else {

                "—"
            }


        // ========================================================
        // USE BUTTON
        // ========================================================

        /*
         * Use Calibration is enabled from the actual launch gate:
         * no hard-gate failure + score >= 82.
         *
         * WAITING optional/runtime evidence (especially automatic
         * GNSS refinement) must not disable a valid calibration.
         */
        val calibrationUsable =
            result.hardGateFailure == null &&
                    result.input.transform.valid &&
                    result.score >= 82

        findViewById<Button>(
            R.id.actionUseCalibration
        ).isEnabled = calibrationUsable
    }


    // ============================================================
    // STATUS HINT
    // ============================================================

    private fun statusHint(
        statusText: String,
        hardGateFailure: String?
    ): String {

        if (
            hardGateFailure != null
        ) {

            return hardGateFailure
        }


        return when (statusText) {

            "READY" ->

                "Calibration accepted. DVFC is locked. GNSS auto-refinement can continue later while the vehicle is moving."


            "DEGRADED" ->

                "Calibration is usable, but some evidence is below the preferred target."


            "INVALID" ->

                "Calibration is below the minimum quality threshold. Recalibration is required."


            "WAITING FOR TELEMETRY" ->

                "Collecting sensor, timing and calibration evidence."


            else ->

                "Collecting evidence from the sensor adapter and calibration pipeline."
        }
    }


    // ============================================================
    // EVIDENCE STATE BINDING
    // ============================================================

    private fun bindState(
        id: Int,
        state: EvidenceState
    ) {

        val view =
            findViewById<TextView>(
                id
            )


        view.text =
            when (state) {

                EvidenceState.PASS ->
                    "PASS"

                EvidenceState.WARN ->
                    "WARN"

                EvidenceState.FAIL ->
                    "FAIL"

                EvidenceState.WAITING ->
                    "WAITING"
            }


        val (
            chipDrawable,
            textColor
        ) =
            when (state) {

                EvidenceState.PASS ->

                    R.drawable.bg_dvfc_status_chip_pass to
                            R.color.good


                EvidenceState.WARN ->

                    R.drawable.bg_dvfc_status_chip_warn to
                            R.color.warn


                EvidenceState.FAIL ->

                    R.drawable.bg_dvfc_status_chip_crit to
                            R.color.crit


                EvidenceState.WAITING ->

                    R.drawable.bg_dvfc_status_chip_waiting to
                            R.color.fg_faint
            }


        view.setBackgroundResource(
            chipDrawable
        )


        view.setTextColor(
            getColor(
                textColor
            )
        )
    }


    // ============================================================
    // SECTION DOT
    // ============================================================

    private fun bindSectionDot(
        id: Int,
        states: List<EvidenceState>
    ) {

        val worst =
            when {

                states.any {
                    it == EvidenceState.FAIL
                } ->
                    EvidenceState.FAIL


                states.any {
                    it == EvidenceState.WARN
                } ->
                    EvidenceState.WARN


                states.all {
                    it == EvidenceState.PASS
                } ->
                    EvidenceState.PASS


                else ->
                    EvidenceState.WAITING
            }


        val dotDrawable =
            when (worst) {

                EvidenceState.PASS ->

                    R.drawable.bg_dvfc_section_dot_pass


                EvidenceState.WARN ->

                    R.drawable.bg_dvfc_section_dot_warn


                EvidenceState.FAIL ->

                    R.drawable.bg_dvfc_section_dot_crit


                EvidenceState.WAITING ->

                    R.drawable.bg_dvfc_section_dot_waiting
            }


        findViewById<View>(
            id
        ).setBackgroundResource(
            dotDrawable
        )
    }


    // ============================================================
    // FORMAT DEGREES
    // ============================================================

    private fun formatDeg(
        value: Double
    ): String {

        return String.format(
            Locale.US,
            "%+.2f°",
            value
        )
    }
}
