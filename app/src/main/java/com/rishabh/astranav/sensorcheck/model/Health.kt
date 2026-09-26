package com.rishabh.astranav.sensorcheck.model

import androidx.annotation.ColorRes
import com.rishabh.astranav.R

/** Mirrors the TS `Health` type used for StatusDot / healthColor in lib/nav. */
enum class Health { IDLE, GOOD, WARN, CRIT }

@ColorRes
fun Health.colorRes(): Int = when (this) {
    Health.IDLE -> R.color.idle
    Health.GOOD -> R.color.good
    Health.WARN -> R.color.warn
    Health.CRIT -> R.color.crit
}

/** Mirrors `stateHealth` — only resolved states (not pending/checking) map to a health. */
fun RowState.toHealth(): Health = when (this) {
    RowState.AVAILABLE -> Health.GOOD
    RowState.DEGRADED -> Health.WARN
    RowState.UNAVAILABLE -> Health.CRIT
    RowState.PENDING, RowState.CHECKING -> Health.IDLE
}
