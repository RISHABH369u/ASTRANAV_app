package com.rishabh.astranav.dvfc

import android.content.Context

/**
 * Keeps one DVFCController alive between:
 *
 * DVFCActivity
 *      ↓
 * DVFCQualityActivity
 *
 * This allows the Quality screen to receive
 * live GNSS + gyro telemetry.
 */
object DvfcSession {

    @Volatile
    private var controller:
            DVFCController? = null

    @Synchronized
    fun getController(
        context: Context
    ): DVFCController {

        val existing =
            controller

        if (existing != null) {
            return existing
        }

        return DVFCController(
            context.applicationContext
        ).also {

            controller =
                it
        }
    }

    @Synchronized
    fun clear() {

        controller?.stop()

        controller =
            null
    }
}