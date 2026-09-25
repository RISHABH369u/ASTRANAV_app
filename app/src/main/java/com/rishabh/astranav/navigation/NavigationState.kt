package com.rishabh.astranav.navigation

enum class NavigationMode { GNSS_AVAILABLE, DEGRADED, TRANSITION, GNSS_DENIED, REACQUIRING }

data class NavigationState(
    val timestampMillis: Long = 0L,
    val mode: NavigationMode = NavigationMode.GNSS_AVAILABLE,
    val eastMeters: Double = 0.0,
    val northMeters: Double = 0.0,
    val speedMps: Double = 0.0,
    val headingDegrees: Double = 0.0,
    val positionSigmaMeters: Double = 0.0,
    val headingSigmaDegrees: Double = 0.0,
    val driftMeters: Double = 0.0,
    val confidencePercent: Int = 0,
    val stationary: Boolean = false,
    val mlAvailable: Boolean = false,
    val mapMatched: Boolean = false
)
