package com.rishabh.astranav.ml.gru

data class GruSpeedOutput(
    val normalizedOutput: Double,
    val speedKmh: Double,
    val speedMps: Double,
    val valid: Boolean,
    val inferenceMs: Long,
    val samplesUsed: Int
)