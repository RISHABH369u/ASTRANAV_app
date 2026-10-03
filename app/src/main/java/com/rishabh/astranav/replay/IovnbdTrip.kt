package com.rishabh.astranav.replay

import com.rishabh.astranav.sensor.ImuSample

data class IovnbdTrip(
    val tripId: String,
    val samples: List<ImuSample>,
    val sourceFile: String,
    val sampleRateHz: Double
)