package com.rishabh.astranav.integrity
data class NisResult(val value: Double, val accepted: Boolean)
class NisMonitor(private val gate: InnovationGate = InnovationGate()) {
    fun scalar(innovation: Double, variance: Double): NisResult {
        val v = variance.coerceAtLeast(1e-9)
        return NisResult(innovation * innovation / v, gate.accept(innovation, kotlin.math.sqrt(v)))
    }
}
