package com.rishabh.astranav.sensorcheck.model

import androidx.annotation.DrawableRes

/** Mirrors the `AvailabilitySensor` type imported from '../lib/boot' in the original code. */
data class AvailabilitySensor(
    val key: String,
    val label: String,
    val sub: String,
    val detail: String? = null,
    @DrawableRes val icon: Int,
    val required: Boolean,
    val result: SensorResult,
)
