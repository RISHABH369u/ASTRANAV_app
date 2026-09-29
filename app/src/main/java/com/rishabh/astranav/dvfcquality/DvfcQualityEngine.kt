package com.rishabh.astranav.dvfcquality

import kotlin.math.abs
import kotlin.math.sqrt

enum class EvidenceState {
    PASS,
    WARN,
    FAIL,
    WAITING
}

data class AdapterEvidence(
    val timestampSync: EvidenceState = EvidenceState.WAITING,
    val timestampJitterMs: Double? = null,

    val units: EvidenceState = EvidenceState.WAITING,

    val dataGaps: EvidenceState = EvidenceState.WAITING,
    val gapCount: Int? = null,
    val maxGapMs: Double? = null,

    val resampling: EvidenceState = EvidenceState.WAITING,
    val sourceHz: Double? = null,
    val outputHz: Double? = null,

    val gravity: EvidenceState = EvidenceState.WAITING,
    val gravityMagnitude: Double? = null,
    val gravityStd: Double? = null
)

data class CalibrationEvidence(
    val manualAlignment: EvidenceState = EvidenceState.WAITING,
    val gravityLeveling: EvidenceState = EvidenceState.WAITING,
    val gyroBias: EvidenceState = EvidenceState.WAITING,
    val mountAzimuth: EvidenceState = EvidenceState.WAITING,

    /*
     * Runtime refinement.
     *
     * WAITING is valid here.
     * It does NOT block initial calibration.
     */
    val automaticRefinement: EvidenceState =
        EvidenceState.WAITING,

    val mountStability: EvidenceState =
        EvidenceState.WAITING
)

data class GnssEvidence(
    val accuracyM: Double? = null,
    val speedMps: Double? = null,
    val validSamples: Int? = null
)

data class ExcitationEvidence(
    val score: Double? = null,
    val usableSamples: Int? = null,
    val longitudinalAccelerationRms: Double? = null
)

data class AzimuthEvidence(
    val manualEstimateDeg: Double? = null,
    val automaticEstimateDeg: Double? = null,
    val residualDeg: Double? = null,
    val circularConsistency: Double? = null,
    val validSamples: Int? = null
)

data class StabilityEvidence(
    val stationaryScore: Double? = null,
    val stationarySamples: Int? = null,

    val gravityScore: Double? = null,
    val gravityStd: Double? = null,

    val mountChanged: Boolean? = null,
    val mountChangeConfirmedSamples: Int? = null
)

data class TransformEvidence(
    val yawDeg: Double? = null,
    val pitchDeg: Double? = null,
    val rollDeg: Double? = null,

    val gyroBiasX: Double? = null,
    val gyroBiasY: Double? = null,
    val gyroBiasZ: Double? = null,

    val valid: Boolean = false
)

data class DvfcQualityInput(
    val adapter: AdapterEvidence = AdapterEvidence(),
    val calibration: CalibrationEvidence =
        CalibrationEvidence(),
    val gnss: GnssEvidence =
        GnssEvidence(),
    val excitation: ExcitationEvidence =
        ExcitationEvidence(),
    val azimuth: AzimuthEvidence =
        AzimuthEvidence(),
    val stability: StabilityEvidence =
        StabilityEvidence(),
    val transform: TransformEvidence =
        TransformEvidence()
)

data class DvfcQualityResult(
    val score: Int,
    val status: String,
    val hardGateFailure: String?,
    val input: DvfcQualityInput
)

object DvfcQualityEngine {

    fun evaluate(
        input: DvfcQualityInput
    ): DvfcQualityResult {

        /*
         * =====================================================
         * INITIAL CALIBRATION HARD GATES
         * =====================================================
         *
         * These are the things that must be true before the
         * device → vehicle transform is accepted.
         */

        val hardGateFailure = when {

            !input.transform.valid ->
                "Device → Vehicle transform is not locked"

            input.calibration.manualAlignment == EvidenceState.FAIL ->
                "Manual alignment failed"

            input.stability.mountChanged == true ->
                "Mount change confirmed"

            else ->
                null
        }

        /*
         * =====================================================
         * CORE EVIDENCE
         * =====================================================
         */

        val components =
            mutableListOf<Double>()

        addState(
            components,
            input.adapter.timestampSync
        )

        addState(
            components,
            input.adapter.units
        )

        addState(
            components,
            input.adapter.dataGaps
        )

        addState(
            components,
            input.adapter.resampling
        )

        addState(
            components,
            input.adapter.gravity
        )

        addState(
            components,
            input.calibration.manualAlignment
        )

        addState(
            components,
            input.calibration.gravityLeveling
        )

        addState(
            components,
            input.calibration.gyroBias
        )

        addState(
            components,
            input.calibration.mountAzimuth
        )

        addState(
            components,
            input.calibration.mountStability
        )

        /*
         * IMPORTANT:
         *
         * automaticRefinement is deliberately NOT included
         * in the initial calibration score.
         *
         * It is a runtime GNSS refinement signal.
         */

        /*
         * =====================================================
         * NUMERIC EVIDENCE
         * =====================================================
         */

        numericWeight(
            input.stability.stationaryScore,
            bad = 0.55,
            good = 0.90
        )?.let {
            components.add(it)
        }

        numericWeight(
            input.stability.gravityScore,
            bad = 0.60,
            good = 0.95
        )?.let {
            components.add(it)
        }

        numericWeight(
            input.gnss.accuracyM,
            bad = 30.0,
            good = 3.0
        )?.let {
            /*
             * GNSS contributes to quality when available,
             * but is NOT an initial hard gate.
             */
            components.add(it)
        }

        numericWeight(
            input.azimuth.circularConsistency,
            bad = 0.50,
            good = 0.95
        )?.let {
            /*
             * Optional refinement quality.
             */
            components.add(it)
        }

        /*
         * Transform is fundamental.
         */
        components.add(
            if (input.transform.valid) {
                1.0
            } else {
                0.0
            }
        )

        val score =
            if (components.isEmpty()) {
                0
            } else {
                (
                        components.average() *
                                100.0
                        )
                    .toInt()
                    .coerceIn(
                        0,
                        100
                    )
            }

        /*
         * =====================================================
         * STATUS
         * =====================================================
         *
         * WAITING evidence is not automatically a launch blocker.
         * Runtime/optional evidence may legitimately remain WAITING.
         *
         * Initial calibration is accepted only when there is no
         * hard-gate failure and the quality score reaches the
         * launch threshold.
         *
         * Transform validity is already enforced as a hard gate
         * above, so a second "coreEvidenceReady" gate here would
         * incorrectly block a valid high-quality calibration.
         */

        val status = when {

            hardGateFailure != null ->
                "INVALID"

            score >= 82 ->
                "READY"

            score >= 60 ->
                "DEGRADED"

            else ->
                "INVALID"
        }

        return DvfcQualityResult(
            score = score,
            status = status,
            hardGateFailure = hardGateFailure,
            input = input
        )
    }

    private fun addState(
        components: MutableList<Double>,
        state: EvidenceState
    ) {

        when (state) {

            EvidenceState.PASS ->
                components.add(1.0)

            EvidenceState.WARN ->
                components.add(0.65)

            EvidenceState.FAIL ->
                components.add(0.0)

            /*
             * WAITING means unavailable/not-applicable.
             * Do not count it as zero quality.
             */
            EvidenceState.WAITING -> Unit
        }
    }

    private fun numericWeight(
        value: Double?,
        bad: Double,
        good: Double
    ): Double? {

        if (value == null) {
            return null
        }

        if (!value.isFinite()) {
            return null
        }

        if (bad == good) {
            return null
        }

        return if (bad < good) {

            (
                    (value - bad) /
                            (good - bad)
                    )
                .coerceIn(
                    0.0,
                    1.0
                )

        } else {

            (
                    (bad - value) /
                            (bad - good)
                    )
                .coerceIn(
                    0.0,
                    1.0
                )
        }
    }

    fun normalizeGravityMagnitude(
        g: Double
    ): Double {

        return abs(g)
    }

    fun vectorMagnitude(
        x: Double,
        y: Double,
        z: Double
    ): Double {

        return sqrt(
            x * x +
                    y * y +
                    z * z
        )
    }
}
