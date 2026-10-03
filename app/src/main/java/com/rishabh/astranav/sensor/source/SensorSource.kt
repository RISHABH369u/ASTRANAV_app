package com.rishabh.astranav.sensor.source

import com.rishabh.astranav.sensor.ImuSample
import com.rishabh.astranav.sensor.SensorSourceType

interface SensorSource {

    val sourceType: SensorSourceType

    val isConnected: Boolean

    fun start(listener: Listener)

    fun stop()

    interface Listener {

        fun onImuSample(sample: ImuSample)

        fun onConnectionChanged(connected: Boolean)

        fun onError(message: String)
    }
}