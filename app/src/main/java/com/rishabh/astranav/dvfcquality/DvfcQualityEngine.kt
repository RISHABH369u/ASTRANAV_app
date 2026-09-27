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
    val automaticRefinement: EvidenceState = EvidenceState.WAITING,
    val mountStability: EvidenceState = EvidenceState.WAITING
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
    val calibration: CalibrationEvidence = CalibrationEvidence(),
    val gnss: GnssEvidence = GnssEvidence(),
    val excitation: ExcitationEvidence = ExcitationEvidence(),
    val azimuth: AzimuthEvidence = AzimuthEvidence(),
    val stability: StabilityEvidence = StabilityEvidence(),
    val transform: TransformEvidence = TransformEvidence()
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

        val hardGateFailure = when {

            !input.transform.valid ->
                "Device → Vehicle transform is not locked"

            input.calibration.manualAlignment == EvidenceState.FAIL ->
                "Manual alignment failed"

            input.calibration.gravityLeveling == EvidenceState.FAIL ->
                "Gravity leveling failed"

            input.stability.mountChanged == true ->
                "Mount change confirmed"

            input.gnss.accuracyM != null &&
                    input.gnss.accuracyM > 15.0 ->
                "GNSS accuracy is outside calibration gate"

            input.excitation.score != null &&
                    input.excitation.score < 0.35 ->
                "Acceleration excitation is insufficient"

            input.azimuth.circularConsistency != null &&
                    input.azimuth.circularConsistency < 0.70 ->
                "Mount azimuth consistency is too low"

            else -> null
        }

        val components = listOf(

            stateWeight(input.adapter.timestampSync),
            stateWeight(input.adapter.units),
            stateWeight(input.adapter.dataGaps),
            stateWeight(input.adapter.resampling),
            stateWeight(input.adapter.gravity),

            stateWeight(input.calibration.manualAlignment),
            stateWeight(input.calibration.gravityLeveling),
            stateWeight(input.calibration.gyroBias),
            stateWeight(input.calibration.mountAzimuth),
            stateWeight(input.calibration.automaticRefinement),
            stateWeight(input.calibration.mountStability),

            numericWeight(
                input.gnss.accuracyM,
                bad = 15.0,
                good = 3.0
            ),

            numericWeight(
                input.excitation.score,
                bad = 0.35,
                good = 0.85
            ),

            numericWeight(
                input.azimuth.circularConsistency,
                bad = 0.70,
                good = 0.95
            ),

            numericWeight(
                input.stability.stationaryScore,
                bad = 0.75,
                good = 0.95
            ),

            numericWeight(
                input.stability.gravityScore,
                bad = 0.75,
                good = 0.95
            ),

            if (input.transform.valid) 1.0 else 0.0

        ).filterNotNull()

        val score =
            if (components.isEmpty()) {
                0
            } else {
                (components.average() * 100.0)
                    .toInt()
                    .coerceIn(0, 100)
            }

        val status = when {

            hardGateFailure != null ->
                "INVALID"

            components.isEmpty() ||
                    components.size < 17 ->
                "WAITING FOR TELEMETRY"

            score >= 85 ->
                "READY"

            score >= 65 ->
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

    private fun stateWeight(
        state: EvidenceState
    ): Double? {

        return when (state) {

            EvidenceState.PASS ->
                1.0

            EvidenceState.WARN ->
                0.65

            EvidenceState.FAIL ->
                0.0

            EvidenceState.WAITING ->
                null
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

        if (bad == good) {
            return null
        }

        return if (bad < good) {

            ((value - bad) /
                    (good - bad))
                .coerceIn(0.0, 1.0)

        } else {

            ((bad - value) /
                    (bad - good))
                .coerceIn(0.0, 1.0)
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