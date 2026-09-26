package com.rishabh.astranav.sensorcheck.model

/** Mirrors the TS `RowState` union: 'pending' | 'checking' | 'available' | 'degraded' | 'unavailable'. */
enum class RowState { PENDING, CHECKING, AVAILABLE, DEGRADED, UNAVAILABLE }

/** Terminal outcome a sensor probe can resolve to — mirrors `AvailabilitySensor['result']`. */
enum class SensorResult { AVAILABLE, DEGRADED, UNAVAILABLE }

fun SensorResult.toRowState(): RowState = when (this) {
    SensorResult.AVAILABLE -> RowState.AVAILABLE
    SensorResult.DEGRADED -> RowState.DEGRADED
    SensorResult.UNAVAILABLE -> RowState.UNAVAILABLE
}
