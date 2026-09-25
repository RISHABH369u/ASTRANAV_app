package com.rishabh.astranav.integrity
class InnovationGate(private val sigmaLimit: Double = 3.0) {
    fun accept(innovation: Double, sigma: Double): Boolean =
        innovation.isFinite() && sigma.isFinite() && sigma > 0.0 && kotlin.math.abs(innovation) <= sigmaLimit * sigma
}
