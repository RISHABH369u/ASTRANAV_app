package com.rishabh.astranav

import android.animation.Animator
import android.animation.AnimatorListenerAdapter
import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.graphics.drawable.Animatable
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.animation.PathInterpolator
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import android.content.Intent
import android.widget.ImageView

/**
 * Splash screen matching the web app's ink-900 / cyan design system:
 * an ambient trajectory line draws itself in behind a centered wordmark,
 * while a pill-shaped bar wipes in at the bottom. Navigates on to
 * MainActivity after the same ~1.1s the React version waits, independent
 * of whether the decorative animations have finished.
 */
class SplashScreen : AppCompatActivity() {

    private val handler = Handler(Looper.getMainLooper())

    // Matches the CSS cubic-bezier(0.22, 1, 0.36, 1) used for fade-up / fade-in.
    private val easeOutSoft = PathInterpolator(0.22f, 1f, 0.36f, 1f)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        enableEdgeToEdge()
        setContentView(R.layout.activity_splash_screen)

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { view, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            view.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }

        startSplash()
    }

    private fun startSplash() {

        val trajectoryBackdrop = findViewById<ImageView>(R.id.trajectoryBackdrop)
        val wordmark = findViewById<View>(R.id.wordmark)
        val bottomSection = findViewById<View>(R.id.bottomSection)
        val loadingFill = findViewById<View>(R.id.loadingFill)

        /* Trajectory line draws itself in — direct equivalent of the
           stroke-dashoffset keyframe on the SVG path. */
        (trajectoryBackdrop.drawable as? Animatable)?.start()

        /* Wordmark: fade in, no translation (animate-fade-in ~0.6s ease). */
        ObjectAnimator.ofFloat(wordmark, View.ALPHA, 0f, 1f).apply {
            duration = 1000
            start()
        }

        /* Bottom section: fade up (animate-fade-up ~0.5s, ease-out-soft). */
        AnimatorSet().apply {
            playTogether(
                ObjectAnimator.ofFloat(bottomSection, View.ALPHA, 0f, 1f),
                ObjectAnimator.ofFloat(bottomSection, View.TRANSLATION_Y, 10f, 0f),
            )
            duration = 700
            interpolator = easeOutSoft
            start()
        }

        /* Loading bar wipes in left-to-right over 2.4s — outlives the
           handoff below on purpose, same as the web version. */
        ObjectAnimator.ofFloat(loadingFill, View.SCALE_X, 0f, 1f).apply {
            duration = 3400
            interpolator = easeOutSoft
            start()
        }

        /* Hand off — mirrors the React setTimeout(onDone, 1100). */
        handler.postDelayed({ openSensorCheckActivity() }, 2100)
    }

    private fun openSensorCheckActivity() {

        val root = findViewById<View>(R.id.main)

        ObjectAnimator.ofFloat(
            root,
            View.ALPHA,
            1f,
            0f
        ).apply {

            duration = 220

            addListener(object : AnimatorListenerAdapter() {

                override fun onAnimationEnd(animation: Animator) {

                    val intent = Intent(
                        this@SplashScreen,
                        SensorCheckActivity::class.java
                    )

                    startActivity(intent)

                    applyExitTransition()

                    finish()
                }
            })

            start()
        }
    }

    @Suppress("DEPRECATION")
    private fun applyExitTransition() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            overrideActivityTransition(
                OVERRIDE_TRANSITION_OPEN,
                android.R.anim.fade_in,
                android.R.anim.fade_out,
            )
        } else {
            overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out)
        }
    }

    override fun onDestroy() {
        handler.removeCallbacksAndMessages(null)
        super.onDestroy()
    }
}