package com.rishabh.astranav

import android.animation.Animator
import android.animation.AnimatorListenerAdapter
import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.animation.ValueAnimator
import android.content.Intent
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.animation.AccelerateDecelerateInterpolator
import android.view.animation.DecelerateInterpolator
import android.view.animation.LinearInterpolator
import android.view.animation.PathInterpolator
import android.widget.ProgressBar
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

/**
 * Cinematic splash for ASTRANAV.
 *
 * Choreography is layered rather than simultaneous: ambient light first,
 * then the instrument (rings + sweep), then the mark, then the identity
 * (name / underline / tagline), then the loading readout. Two looping
 * animators (outer ring drift, radar sweep) keep the screen alive for
 * however long the real init work behind it takes.
 */
class SplashScreen : AppCompatActivity() {

    private val handler = Handler(Looper.getMainLooper())
    private val loopingAnimators = mutableListOf<Animator>()

    // Premium "ease-out expo"-style curve: fast start, long soft settle.
    private val easeOutExpo = PathInterpolator(0.16f, 1f, 0.3f, 1f)

    // Gentle overshoot for the logo's single confident "pop".
    private val easeOutBack = PathInterpolator(0.34f, 1.56f, 0.64f, 1f)

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

        val centerGlowOuter = findViewById<View>(R.id.centerGlowOuter)
        val centerGlow = findViewById<View>(R.id.centerGlow)
        val radarSweep = findViewById<View>(R.id.radarSweep)
        val ringOuter = findViewById<View>(R.id.ringOuter)
        val ringInner = findViewById<View>(R.id.ringInner)
        val orbitPivot = findViewById<View>(R.id.orbitPivot)
        val navigationPoint = findViewById<View>(R.id.navigationPoint)

        val logo = findViewById<View>(R.id.logo)
        val appName = findViewById<TextView>(R.id.appName)
        val nameUnderline = findViewById<View>(R.id.nameUnderline)
        val tagline = findViewById<TextView>(R.id.tagline)

        val loadingContainer = findViewById<View>(R.id.loadingContainer)
        val loadingBar = findViewById<ProgressBar>(R.id.loadingBar)

        /* ---------------------------------------------------------
         * Initial state — everything invisible / at rest.
         * --------------------------------------------------------- */

        centerGlowOuter.alpha = 0f
        centerGlow.alpha = 0f
        radarSweep.alpha = 0f

        ringOuter.alpha = 0f
        ringOuter.scaleX = 0.88f
        ringOuter.scaleY = 0.88f

        ringInner.alpha = 0f
        ringInner.scaleX = 0.88f
        ringInner.scaleY = 0.88f

        navigationPoint.alpha = 0f
        navigationPoint.scaleX = 0.2f
        navigationPoint.scaleY = 0.2f

        logo.alpha = 0f
        logo.scaleX = 0.74f
        logo.scaleY = 0.74f

        appName.alpha = 0f
        appName.translationY = 14f

        nameUnderline.alpha = 0f

        tagline.alpha = 0f
        tagline.translationY = 6f

        loadingContainer.alpha = 0f

        /* ---------------------------------------------------------
         * 1. Ambient light — the screen "warms up" before anything
         *    else appears.
         * --------------------------------------------------------- */

        val outerGlowFade = ObjectAnimator.ofFloat(centerGlowOuter, View.ALPHA, 0f, 1f).apply {
            duration = 1000
            interpolator = DecelerateInterpolator()
        }

        val coreGlowFade = ObjectAnimator.ofFloat(centerGlow, View.ALPHA, 0f, 1f).apply {
            duration = 900
            startDelay = 120
            interpolator = DecelerateInterpolator()
        }

        /* ---------------------------------------------------------
         * 2. The instrument — rings settle inward with the expo ease,
         *    sweep fades in behind them and starts its fast loop.
         * --------------------------------------------------------- */

        val outerRingIn = AnimatorSet().apply {
            playTogether(
                ObjectAnimator.ofFloat(ringOuter, View.ALPHA, 0f, 1f),
                ObjectAnimator.ofFloat(ringOuter, View.SCALE_X, 0.88f, 1f),
                ObjectAnimator.ofFloat(ringOuter, View.SCALE_Y, 0.88f, 1f),
            )
            duration = 900
            startDelay = 150
            interpolator = easeOutExpo
        }

        val innerRingIn = AnimatorSet().apply {
            playTogether(
                ObjectAnimator.ofFloat(ringInner, View.ALPHA, 0f, 1f),
                ObjectAnimator.ofFloat(ringInner, View.SCALE_X, 0.88f, 1f),
                ObjectAnimator.ofFloat(ringInner, View.SCALE_Y, 0.88f, 1f),
            )
            duration = 800
            startDelay = 220
            interpolator = easeOutExpo
        }

        val sweepFade = ObjectAnimator.ofFloat(radarSweep, View.ALPHA, 0f, 1f).apply {
            duration = 500
            startDelay = 260
        }

        /* ---------------------------------------------------------
         * 3. The mark — logo pops in with a single confident overshoot.
         * --------------------------------------------------------- */

        val logoIn = AnimatorSet().apply {
            playTogether(
                ObjectAnimator.ofFloat(logo, View.ALPHA, 0f, 1f).apply { duration = 650 },
                ObjectAnimator.ofFloat(logo, View.SCALE_X, 0.74f, 1f).apply { duration = 800 },
                ObjectAnimator.ofFloat(logo, View.SCALE_Y, 0.74f, 1f).apply { duration = 800 },
            )
            startDelay = 380
            interpolator = easeOutBack
        }

        /* Run the whole visual base as one set. */
        AnimatorSet().apply {
            playTogether(outerGlowFade, coreGlowFade, outerRingIn, innerRingIn, sweepFade, logoIn)
            start()
        }

        /* ---------------------------------------------------------
         * 4. Navigation point — snaps in with a pulse once the ring
         *    it lives on is visible, then the ring starts orbiting it.
         * --------------------------------------------------------- */

        handler.postDelayed({
            navigationPoint.alpha = 1f
            AnimatorSet().apply {
                playTogether(
                    ObjectAnimator.ofFloat(navigationPoint, View.SCALE_X, 0.2f, 1.3f, 1f),
                    ObjectAnimator.ofFloat(navigationPoint, View.SCALE_Y, 0.2f, 1.3f, 1f),
                )
                duration = 550
                interpolator = DecelerateInterpolator()
                start()
            }
        }, 650)

        /* ---------------------------------------------------------
         * 5. Two independent loops start once their host is visible:
         *    the sweep spins fast (a live scan), the orbit pivot
         *    drifts slowly (the ring itself feels alive, not static).
         * --------------------------------------------------------- */

        handler.postDelayed({
            loop(radarSweep, durationMs = 2600)
        }, 700)

        handler.postDelayed({
            loop(orbitPivot, durationMs = 12000)
        }, 750)

        /* ---------------------------------------------------------
         * 6. Identity — name, underline, tagline reveal in sequence.
         * --------------------------------------------------------- */

        handler.postDelayed({
            AnimatorSet().apply {
                playTogether(
                    ObjectAnimator.ofFloat(appName, View.ALPHA, 0f, 1f),
                    ObjectAnimator.ofFloat(appName, View.TRANSLATION_Y, 14f, 0f),
                )
                duration = 600
                interpolator = easeOutExpo
                start()
            }
        }, 950)

        handler.postDelayed({
            growUnderline(nameUnderline, targetDp = 44f, durationMs = 480)
        }, 1150)

        handler.postDelayed({
            AnimatorSet().apply {
                playTogether(
                    ObjectAnimator.ofFloat(tagline, View.ALPHA, 0f, 1f),
                    ObjectAnimator.ofFloat(tagline, View.TRANSLATION_Y, 6f, 0f),
                )
                duration = 450
                interpolator = DecelerateInterpolator()
                start()
            }
        }, 1350)

        /* ---------------------------------------------------------
         * 7. Loading readout — the last thing to appear, and the
         *    thing the eye rests on until launch.
         * --------------------------------------------------------- */

        handler.postDelayed({
            ObjectAnimator.ofFloat(loadingContainer, View.ALPHA, 0f, 1f).apply {
                duration = 400
                start()
            }
            animateProgress(loadingBar)
        }, 1600)

        /* ---------------------------------------------------------
         * 8. Hand off to MainActivity.
         * --------------------------------------------------------- */

        handler.postDelayed({ openMainActivity() }, 2750)
    }

    /** Starts an indefinite linear rotation on [view] and tracks it for cleanup. */
    private fun loop(view: View, durationMs: Long) {
        val animator = ObjectAnimator.ofFloat(view, View.ROTATION, 0f, 360f).apply {
            duration = durationMs
            repeatCount = ValueAnimator.INFINITE
            interpolator = LinearInterpolator()
        }
        loopingAnimators += animator
        animator.start()
    }

    /** Grows a 1dp placeholder view into a [targetDp]-wide accent rule. */
    private fun growUnderline(view: View, targetDp: Float, durationMs: Long) {
        val targetPx = (targetDp * resources.displayMetrics.density).toInt()
        view.alpha = 1f
        ValueAnimator.ofInt(view.layoutParams.width.coerceAtLeast(1), targetPx).apply {
            duration = durationMs
            interpolator = DecelerateInterpolator()
            addUpdateListener { animator ->
                view.layoutParams = view.layoutParams.apply {
                    width = animator.animatedValue as Int
                }
            }
            start()
        }
    }

    private fun animateProgress(progressBar: ProgressBar) {
        ValueAnimator.ofInt(0, 100).apply {
            duration = 1050
            interpolator = AccelerateDecelerateInterpolator()
            addUpdateListener { animator ->
                progressBar.progress = animator.animatedValue as Int
            }
            start()
        }
    }

    private fun openMainActivity() {
        val root = findViewById<View>(R.id.main)

        // Stop loops before the fade so they don't tick during the transition.
        loopingAnimators.forEach { it.cancel() }

        AnimatorSet().apply {
            playTogether(
                ObjectAnimator.ofFloat(root, View.ALPHA, 1f, 0f),
                ObjectAnimator.ofFloat(root, View.SCALE_X, 1f, 1.03f),
                ObjectAnimator.ofFloat(root, View.SCALE_Y, 1f, 1.03f),
            )
            duration = 380
            interpolator = AccelerateDecelerateInterpolator()
            addListener(object : AnimatorListenerAdapter() {
                override fun onAnimationEnd(animation: Animator) {
                    val intent = Intent(this@SplashScreen, MainActivity::class.java)
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
        loopingAnimators.forEach { it.cancel() }
        loopingAnimators.clear()
        handler.removeCallbacksAndMessages(null)
        super.onDestroy()
    }
}