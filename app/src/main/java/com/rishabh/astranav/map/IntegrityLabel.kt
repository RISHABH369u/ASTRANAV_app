package com.rishabh.astranav.map

import com.rishabh.astranav.integrity.IntegritySnapshot

enum class IntegrityLevel { STABLE, CAUTION, DEGRADED }

/** Thresholds are a starting point — tune once you have real driving data to calibrate against. */
fun IntegritySnapshot.level(): IntegrityLevel = when {
    scorePercent >= 80 -> IntegrityLevel.STABLE
    scorePercent >= 50 -> IntegrityLevel.CAUTION
    else -> IntegrityLevel.DEGRADED
}
