package com.rishabh.astranav.navigation

class NavigationModeManager {
    private var mode = NavigationMode.GNSS_AVAILABLE
    private var lastGoodGnssMs = 0L
    fun update(nowMs: Long, gnssAvailable: Boolean, accuracyMeters: Double): NavigationMode {
        if (gnssAvailable && accuracyMeters <= 20.0) {
            mode = if (mode == NavigationMode.GNSS_DENIED) NavigationMode.REACQUIRING else NavigationMode.GNSS_AVAILABLE
            lastGoodGnssMs = nowMs
        } else {
            val outage = if (lastGoodGnssMs == 0L) Long.MAX_VALUE else nowMs - lastGoodGnssMs
            mode = when {
                outage < 1500L -> NavigationMode.DEGRADED
                mode == NavigationMode.REACQUIRING -> NavigationMode.REACQUIRING
                else -> NavigationMode.GNSS_DENIED
            }
        }
        return mode
    }
}
