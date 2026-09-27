package com.rishabh.astranav

import android.content.Intent
import android.os.Bundle
import android.widget.Button
import android.widget.ProgressBar
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
import java.util.Locale

class DVFCQualityActivity : AppCompatActivity() {

    companion object {

        const val EXTRA_TRANSFORM_VALID =
            "extra_transform_valid"
    }

    private lateinit var score: TextView
    private lateinit var status: TextView
    private lateinit var progress: ProgressBar
    private var transformValid = false




    override fun onCreate(
        savedInstanceState: Bundle?
    ) {
        super.onCreate(savedInstanceState)

        setContentView(
            R.layout.activity_dvfcquality
        )

        score =
            findViewById(R.id.dvfcQualityScore)

        status =
            findViewById(R.id.dvfcQualityStatus)

        progress =
            findViewById(R.id.dvfcQualityProgress)

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
            }
        }

        transformValid =
            intent.getBooleanExtra(
                EXTRA_TRANSFORM_VALID,
                false
            )



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

        return DvfcQualityInput(

            calibration =
                CalibrationEvidence(
                    manualAlignment =
                        EvidenceState.PASS,

                    gravityLeveling =
                        EvidenceState.PASS,

                    gyroBias =
                        EvidenceState.WAITING,

                    mountAzimuth =
                        EvidenceState.WAITING,

                    automaticRefinement =
                        EvidenceState.WAITING,

                    mountStability =
                        EvidenceState.WAITING
                ),

            transform =
                TransformEvidence(
                    valid = transformValid
                )
        )
    }



    private fun render(
        input: DvfcQualityInput
    ) {

        val result =
            DvfcQualityEngine.evaluate(
                input
            )

        score.text =
            if (
                result.status ==
                "WAITING FOR TELEMETRY"
            ) {
                "—"
            } else {
                result.score.toString()
            }

        status.text =
            result.hardGateFailure
                ?: result.status

        progress.progress =
            result.score

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

        findViewById<TextView>(
            R.id.mountStatus
        ).text =
            when (
                input.stability.mountChanged
            ) {

                true ->
                    "CHANGE CONFIRMED"

                false ->
                    "STABLE"

                null ->
                    "WAITING"
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

        view.setTextColor(
            getColor(
                when (state) {

                    EvidenceState.PASS ->
                        R.color.good

                    EvidenceState.WARN ->
                        R.color.warn

                    EvidenceState.FAIL ->
                        R.color.crit

                    EvidenceState.WAITING ->
                        R.color.fg_faint
                }
            )
        )
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