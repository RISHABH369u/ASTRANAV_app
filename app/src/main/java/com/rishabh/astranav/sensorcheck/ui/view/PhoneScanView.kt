package com.rishabh.astranav.sensorcheck.ui.view

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import android.widget.FrameLayout
import com.rishabh.astranav.R

/**
 * Compound view for the central phone + scan visualization
 * (`PhoneScan` in the original React file). Inflates view_phone_scan.xml,
 * applies the 3D tilt, and drives the two staggered scan rings.
 */
class PhoneScanView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : FrameLayout(context, attrs) {

    private val scanRing1: ScanRingView
    private val scanRing2: ScanRingView
    private val phoneBody: View

    init {
        LayoutInflater.from(context).inflate(R.layout.view_phone_scan, this, true)
        scanRing1 = findViewById(R.id.scanRing1)
        scanRing2 = findViewById(R.id.scanRing2)
        phoneBody = findViewById(R.id.phoneBody)

        // Mirrors `perspective(520px) rotateX(14deg) rotateY(-13deg)`.
        // Android's rotationX sign convention is inverted relative to CSS.
        phoneBody.cameraDistance = 14000f
        phoneBody.rotationX = -14f
        phoneBody.rotationY = -13f
    }

    /** Mirrors the `{scanning && (...)}` conditional in the React component. */
    fun setScanning(scanning: Boolean) {
        if (scanning) {
            scanRing1.start(startDelay = 0L)
            scanRing2.start(startDelay = 1200L)
        } else {
            scanRing1.stop()
            scanRing2.stop()
        }
    }
}
