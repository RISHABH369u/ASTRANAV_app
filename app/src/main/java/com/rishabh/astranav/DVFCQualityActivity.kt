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
    }

    private lateinit var score: TextView
    private lateinit var status: TextView
    private lateinit var hint: TextView
    private lateinit var gauge: CircularGaugeView

    private var transformValid = false
    private var calibrationComplete = false

    private var yawDeg = 0.0
    private var pitchDeg = 0.0
    private var rollDeg = 0.0

    private var sampleCount = 0
    private var timestampJitterMs = -1.0
    private var sampleHz = -1.0
    private var gapCount = 0
    private var maxGapMs = -1.0

    private var stationarySamples = 0
    private var stationaryScore = -1.0

    private var gyroBiasX = Double.NaN
    private var gyroBiasY = Double.NaN
    private var gyroBiasZ = Double.NaN
    private var gyroRms = -1.0

    private var sensorsAvailable = false
    private var gravityAvailable = false
    private var accelerationAvailable = false
    private var gnssAvailable = false
    private var autoAzimuthAvailable = false

    private var duplicateTimestampCount = 0

    private var resamplingActive = false

    private var resamplingRateHz = -1.0

    private var gravityMagnitude = -1.0

    private var gravityStable = false

    private var gravityLevelRollDeg = 0.0

    private var gravityLevelPitchDeg = 0.0

    private var linearAccelerationMagnitude = -1.0


    override fun onCreate(
        savedInstanceState: Bundle?
    ) {
        super.onCreate(savedInstanceState)

        setContentView(
            R.layout.activity_dvfcquality
        )


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


        score =
            findViewById(R.id.dvfcQualityScore)

        status =
            findViewById(R.id.dvfcQualityStatus)

        hint =
            findViewById(R.id.dvfcQualityHint)

        gauge =
            findViewById(R.id.dvfcQualityGauge)

        findViewById<Button>(
            R.id.actionUseCalibration
        ).setOnClickListener {

            val input =
                buildQualityInput()

            val result =
                DvfcQualityEngine.evaluate(input)

            if (
                result.status == "READY" ||
                result.status == "DEGRADED"
            ) {

                setResult(
                    RESULT_OK
                )

                finish()

            } else {

                status.text =
                    result.hardGateFailure
                        ?: "VALIDATION REQUIRED"

                status.setBackgroundResource(
                    R.drawable.bg_dvfc_status_chip_crit
                )
            }
        }

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

        render(
            buildQualityInput()
        )
    }

    /**
     * IMPORTANT:
     *
     * Current main branch does not yet expose all
     * Sensor Adapter + GNSS refinement telemetry.
     *
     * Therefore we intentionally DO NOT fabricate values.
     *
     * Replace this method with the shared runtime telemetry
     * once Sensor Adapter / DVFCController exposes it.
     */
    private fun buildQualityInput(): DvfcQualityInput {

        // =========================================================
        // TIMESTAMP
        // =========================================================

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


        // =========================================================
        // DATA GAPS
        // =========================================================

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


        // =========================================================
        // RESAMPLING
        // =========================================================
        //
        // IMPORTANT:
        // Current SensorFusion does not expose a resampling stage.
        // Therefore this is NOT claimed as PASS.
        //

        val resamplingState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                !resamplingActive ->
                    EvidenceState.WAITING

                resamplingRateHz <= 0.0 ->
                    EvidenceState.WAITING

                resamplingRateHz in 9.0..11.0 ->
                    EvidenceState.PASS

                else ->
                    EvidenceState.WARN
            }

        // =========================================================
        // UNIT NORMALIZATION
        // =========================================================
        //
        // Current SensorFusion callback gives gyro values from
        // Android's TYPE_GYROSCOPE contract.
        //
        // Android defines gyroscope values in rad/s.
        //
        // This is source-contract verification, not a runtime
        // calibration measurement.
        //

        val unitState =
            if (sensorsAvailable) {
                EvidenceState.PASS
            } else {
                EvidenceState.WAITING
            }


        // =========================================================
        // MANUAL / GRAVITY LEVELING
        // =========================================================

        val manualState =
            if (calibrationComplete) {
                EvidenceState.PASS
            } else {
                EvidenceState.WAITING
            }


        val gravityState =
            when {

                sampleCount < 10 ->
                    EvidenceState.WAITING

                !gravityAvailable ->
                    EvidenceState.WAITING

                gravityMagnitude < 0.0 ->
                    EvidenceState.WAITING

                gravityMagnitude in 9.2..10.4 &&
                        gravityStable ->
                    EvidenceState.PASS

                gravityMagnitude in 8.5..11.0 ->
                    EvidenceState.WARN

                else ->
                    EvidenceState.FAIL
            }


        // =========================================================
        // GYRO BIAS
        // =========================================================

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


        // =========================================================
        // MOUNT STABILITY
        // =========================================================

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


        // =========================================================
        // CURRENT TRANSFORM
        // =========================================================

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


        // =========================================================
        // RETURN
        // =========================================================

        return DvfcQualityInput(

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


            calibration =
                CalibrationEvidence(

                    manualAlignment =
                        manualState,

                    gravityLeveling =
                        gravityState,

                    gyroBias =
                        gyroBiasState,

                    mountAzimuth =
                        if (calibrationComplete) {
                            EvidenceState.PASS
                        } else {
                            EvidenceState.WAITING
                        },

                    automaticRefinement =
                        if (autoAzimuthAvailable) {
                            EvidenceState.PASS
                        } else {
                            EvidenceState.WAITING
                        },

                    mountStability =
                        mountStabilityState
                ),


            gnss =
                GnssEvidence(
                    accuracyM = null,
                    speedMps = null,
                    validSamples = null
                ),


            excitation =
                ExcitationEvidence(
                    score = null,
                    usableSamples = null,
                    longitudinalAccelerationRms = null
                ),


            azimuth =
                AzimuthEvidence(
                    manualEstimateDeg =
                        if (calibrationComplete) {
                            yawDeg
                        } else {
                            null
                        },

                    automaticEstimateDeg =
                        null,

                    residualDeg =
                        null,

                    circularConsistency =
                        null,

                    validSamples =
                        null
                ),


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
                        null,

                    gravityStd =
                        null,

                    mountChanged =
                        false,

                    mountChangeConfirmedSamples =
                        0
                ),


            transform =
                transform
        )
    }



    private fun render(
        input: DvfcQualityInput
    ) {

        val result =
            DvfcQualityEngine.evaluate(
                input
            )

        val waiting =
            result.status == "WAITING FOR TELEMETRY"

        score.text =
            if (waiting) {
                "—"
            } else {
                result.score.toString()
            }

        status.text =
            result.hardGateFailure
                ?: result.status

        hint.text =
            statusHint(
                result.status,
                result.hardGateFailure
            )

        val (chipDrawable, gaugeColor) =
            when {

                result.hardGateFailure != null ->
                    R.drawable.bg_dvfc_status_chip_crit to R.color.crit

                waiting ->
                    R.drawable.bg_dvfc_status_chip_waiting to R.color.idle

                result.status == "READY" ->
                    R.drawable.bg_dvfc_status_chip_pass to R.color.good

                result.status == "DEGRADED" ->
                    R.drawable.bg_dvfc_status_chip_warn to R.color.warn

                else ->
                    R.drawable.bg_dvfc_status_chip_crit to R.color.crit
            }

        status.setBackgroundResource(chipDrawable)

        gauge.setProgress(
            value = if (waiting) 0 else result.score,
            colorRes = gaugeColor
        )

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
            input.gnss.validSamples?.toString()
                ?: "—"

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

        findViewById<TextView>(
            R.id.azimuthManual
        ).text =
            input.azimuth.manualEstimateDeg
                ?.let { formatDeg(it) }
                ?: "—"

        findViewById<TextView>(
            R.id.azimuthAutomatic
        ).text =
            input.azimuth.automaticEstimateDeg
                ?.let { formatDeg(it) }
                ?: "—"

        findViewById<TextView>(
            R.id.azimuthResidual
        ).text =
            input.azimuth.residualDeg
                ?.let { formatDeg(it) }
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
                }
                ?: "—"

        findViewById<TextView>(
            R.id.azimuthSamples
        ).text =
            input.azimuth.validSamples
                ?.toString()
                ?: "—"

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
                }
                ?: "—"

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
                }
                ?: "—"

        val mountBanner =
            findViewById<View>(R.id.mountBanner)

        val mountStatusView =
            findViewById<TextView>(R.id.mountStatus)

        when (
            input.stability.mountChanged
        ) {

            true -> {
                mountStatusView.text = "CHANGE CONFIRMED"
                mountStatusView.setTextColor(getColor(R.color.crit))
                mountBanner.setBackgroundResource(R.drawable.bg_dvfc_mount_banner_changed)
            }

            false -> {
                mountStatusView.text = "STABLE"
                mountStatusView.setTextColor(getColor(R.color.good))
                mountBanner.setBackgroundResource(R.drawable.bg_dvfc_mount_banner_stable)
            }

            null -> {
                mountStatusView.text = "WAITING"
                mountStatusView.setTextColor(getColor(R.color.fg_faint))
                mountBanner.setBackgroundResource(R.drawable.bg_dvfc_mount_banner_waiting)
            }
        }

        findViewById<TextView>(
            R.id.finalYaw
        ).text =
            input.transform.yawDeg
                ?.let { formatDeg(it) }
                ?: "—"

        findViewById<TextView>(
            R.id.finalPitch
        ).text =
            input.transform.pitchDeg
                ?.let { formatDeg(it) }
                ?: "—"

        findViewById<TextView>(
            R.id.finalRoll
        ).text =
            input.transform.rollDeg
                ?.let { formatDeg(it) }
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

        findViewById<Button>(
            R.id.actionUseCalibration
        ).isEnabled =
            result.status == "READY" ||
                    result.status == "DEGRADED"
    }

    private fun statusHint(
        statusText: String,
        hardGateFailure: String?
    ): String {

        if (hardGateFailure != null) {
            return hardGateFailure
        }

        return when (statusText) {

            "READY" ->
                "Meets the navigation launch threshold. Safe to use this calibration."

            "DEGRADED" ->
                "Usable, but some evidence is below target. Consider recalibrating soon."

            "INVALID" ->
                "Below the minimum quality bar. Recalibration is required."

            else ->
                "Collecting evidence from the sensor adapter and calibration pipeline."
        }
    }

    private fun bindState(
        id: Int,
        state: EvidenceState
    ) {

        val view =
            findViewById<TextView>(id)

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

        val (chipDrawable, textColor) =
            when (state) {

                EvidenceState.PASS ->
                    R.drawable.bg_dvfc_status_chip_pass to R.color.good

                EvidenceState.WARN ->
                    R.drawable.bg_dvfc_status_chip_warn to R.color.warn

                EvidenceState.FAIL ->
                    R.drawable.bg_dvfc_status_chip_crit to R.color.crit

                EvidenceState.WAITING ->
                    R.drawable.bg_dvfc_status_chip_waiting to R.color.fg_faint
            }

        view.setBackgroundResource(chipDrawable)
        view.setTextColor(getColor(textColor))
    }

    /**
     * Colors a section-header dot with the worst evidence state among its
     * rows: any FAIL wins, else any WARN, else PASS only if every row has
     * reported in, otherwise the section is still WAITING.
     */
    private fun bindSectionDot(
        id: Int,
        states: List<EvidenceState>
    ) {

        val worst = when {

            states.any { it == EvidenceState.FAIL } ->
                EvidenceState.FAIL

            states.any { it == EvidenceState.WARN } ->
                EvidenceState.WARN

            states.all { it == EvidenceState.PASS } ->
                EvidenceState.PASS

            else ->
                EvidenceState.WAITING
        }

        val dotDrawable = when (worst) {

            EvidenceState.PASS ->
                R.drawable.bg_dvfc_section_dot_pass

            EvidenceState.WARN ->
                R.drawable.bg_dvfc_section_dot_warn

            EvidenceState.FAIL ->
                R.drawable.bg_dvfc_section_dot_crit

            EvidenceState.WAITING ->
                R.drawable.bg_dvfc_section_dot_waiting
        }

        findViewById<View>(id)
            .setBackgroundResource(dotDrawable)
    }

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
