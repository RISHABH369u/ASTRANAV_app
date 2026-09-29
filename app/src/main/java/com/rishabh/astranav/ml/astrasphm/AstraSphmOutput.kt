package com.rishabh.astranav.ml.astrasphm

data class AstraSphmOutput(
    val valid: Boolean = false,

    // -----------------------------
    // SPEED
    // -----------------------------
    val speedMps: Double? = null,
    val speedKmh: Double? = null,
    val speedLogVariance: Double? = null,

    // -----------------------------
    // POSITION
    // -----------------------------
    val positionX: Double? = null,
    val positionY: Double? = null,

    val positionLogVarianceX: Double? = null,
    val positionLogVarianceY: Double? = null,

    // -----------------------------
    // HEADING
    // -----------------------------
    val headingDeltaRad: Double? = null,
    val headingDeltaDeg: Double? = null,

    val headingDeltaLogVariance: Double? = null,

    // -----------------------------
    // MOTION CLASSIFICATION
    // -----------------------------
    val motionLogits: FloatArray = FloatArray(0),

    // -----------------------------
    // RUNTIME
    // -----------------------------
    val latencyMs: Long = 0L,

    val errorMessage: String? = null
) {

    companion object {

        fun invalid(
            message: String,
            latencyMs: Long = 0L
        ): AstraSphmOutput {

            return AstraSphmOutput(
                valid = false,
                latencyMs = latencyMs,
                errorMessage = message
            )
        }
    }
}