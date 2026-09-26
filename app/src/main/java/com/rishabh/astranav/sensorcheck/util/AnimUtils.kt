package com.rishabh.astranav.sensorcheck.util

import android.animation.ObjectAnimator
import android.view.View
import android.view.animation.LinearInterpolator

object AnimUtils {
    /** Equivalent of the `animate-fade-in` utility class used throughout the UI. */
    fun fadeIn(view: View, durationMs: Long = 400) {
        view.alpha = 0f
        view.translationY = 6f
        view.animate()
            .alpha(1f)
            .translationY(0f)
            .setDuration(durationMs)
            .start()
    }

    /** Continuous spin, e.g. for a custom "checking" indicator. */
    fun spin(view: View): ObjectAnimator =
        ObjectAnimator.ofFloat(view, View.ROTATION, 0f, 360f).apply {
            duration = 800
            repeatCount = ObjectAnimator.INFINITE
            interpolator = LinearInterpolator()
            start()
        }
}
