package com.rishabh.astranav.navigation

import kotlin.math.max

class ErrorStateEkf {
    private val p = DoubleArray(5) { 1.0 }
    private val q = doubleArrayOf(0.02, 0.02, 0.2, 0.2, 0.01)
    fun predict(dt: Double) { for (i in p.indices) p[i] += q[i] * dt.coerceAtLeast(0.0) }
    fun correctSpeed(measuredMps: Double, estimatedMps: Double, variance: Double): Boolean {
        val r = max(variance, 0.04)
        val k = p[2] / (p[2] + r)
        val innovation = measuredMps - estimatedMps
        if (!innovation.isFinite() || kotlin.math.abs(innovation) > 3.0 * kotlin.math.sqrt(p[2] + r)) return false
        p[2] *= (1.0 - k)
        return true
    }
    fun correctPosition(east: Double, north: Double, sigmaM: Double): Pair<Double,Double> =
        east to north
    fun positionSigma(): Double = kotlin.math.sqrt((p[0] + p[1]) * 0.5)
    fun speedSigma(): Double = kotlin.math.sqrt(p[2])
}
