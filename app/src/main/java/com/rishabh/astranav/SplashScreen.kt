package com.rishabh.astranav

import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.animation.ValueAnimator
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.animation.AccelerateDecelerateInterpolator
import android.view.animation.DecelerateInterpolator
import android.widget.ProgressBar
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

class SplashScreen : AppCompatActivity() {

    private val handler = Handler(Looper.getMainLooper())

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        enableEdgeToEdge()

        setContentView(R.layout.activity_splash_screen)

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { view, insets ->

            val systemBars =
                insets.getInsets(WindowInsetsCompat.Type.systemBars())

            view.setPadding(
                systemBars.left,
                systemBars.top,
                systemBars.right,
                systemBars.bottom
            )

            insets
        }

        startSplash()
    }

    private fun startSplash() {

        val logo = findViewById<View>(R.id.logo)
        val appName = findViewById<TextView>(R.id.appName)

        val ringOuter = findViewById<View>(R.id.ringOuter)
        val ringInner = findViewById<View>(R.id.ringInner)

        val centerGlow = findViewById<View>(R.id.centerGlow)
        val navigationPoint = findViewById<View>(R.id.navigationPoint)

        val loadingContainer =
            findViewById<View>(R.id.loadingContainer)

        val loadingBar =
            findViewById<ProgressBar>(R.id.loadingBar)

        /*
         * Initial state
         */

        logo.alpha = 0f
        logo.scaleX = 0.78f
        logo.scaleY = 0.78f

        appName.alpha = 0f
        appName.translationY = 12f

        ringOuter.alpha = 0f
        ringInner.alpha = 0f

        centerGlow.alpha = 0f

        navigationPoint.alpha = 0f
        navigationPoint.scaleX = 0.2f
        navigationPoint.scaleY = 0.2f

        loadingContainer.alpha = 0f

        /*
         * ------------------------------------------------
         * 1. Ambient glow
         * ------------------------------------------------
         */

        val glow = ObjectAnimator.ofFloat(
            centerGlow,
            View.ALPHA,
            0f,
            0.30f
        ).apply {

            duration = 900

            interpolator = DecelerateInterpolator()

        }

        /*
         * ------------------------------------------------
         * 2. Outer ring appears
         * ------------------------------------------------
         */

        val outerFade = ObjectAnimator.ofFloat(
            ringOuter,
            View.ALPHA,
            0f,
            0.30f
        ).apply {

            duration = 800

            interpolator = DecelerateInterpolator()

        }

        val outerScaleX = ObjectAnimator.ofFloat(
            ringOuter,
            View.SCALE_X,
            0.85f,
            1f
        ).apply {

            duration = 900

            interpolator = DecelerateInterpolator()

        }

        val outerScaleY = ObjectAnimator.ofFloat(
            ringOuter,
            View.SCALE_Y,
            0.85f,
            1f
        ).apply {

            duration = 900

            interpolator = DecelerateInterpolator()

        }

        /*
         * ------------------------------------------------
         * 3. Inner ring
         * ------------------------------------------------
         */

        val innerFade = ObjectAnimator.ofFloat(
            ringInner,
            View.ALPHA,
            0f,
            0.20f
        ).apply {

            duration = 650

        }

        /*
         * ------------------------------------------------
         * 4. Logo reveal
         * ------------------------------------------------
         */

        val logoFade = ObjectAnimator.ofFloat(
            logo,
            View.ALPHA,
            0f,
            1f
        ).apply {

            duration = 700

            interpolator = DecelerateInterpolator()

        }

        val logoScaleX = ObjectAnimator.ofFloat(
            logo,
            View.SCALE_X,
            0.78f,
            1.04f,
            1f
        ).apply {

            duration = 850

            interpolator = DecelerateInterpolator()

        }

        val logoScaleY = ObjectAnimator.ofFloat(
            logo,
            View.SCALE_Y,
            0.78f,
            1.04f,
            1f
        ).apply {

            duration = 850

            interpolator = DecelerateInterpolator()

        }

        /*
         * Run main visual reveal
         */

        AnimatorSet().apply {

            playTogether(
                glow,
                outerFade,
                outerScaleX,
                outerScaleY,
                innerFade,
                logoFade,
                logoScaleX,
                logoScaleY
            )

            start()
        }

        /*
         * ------------------------------------------------
         * 5. Navigation point pulse
         * ------------------------------------------------
         */

        handler.postDelayed({

            navigationPoint.alpha = 1f

            val pointScaleX = ObjectAnimator.ofFloat(
                navigationPoint,
                View.SCALE_X,
                0.2f,
                1f,
                1.25f,
                1f
            )

            val pointScaleY = ObjectAnimator.ofFloat(
                navigationPoint,
                View.SCALE_Y,
                0.2f,
                1f,
                1.25f,
                1f
            )

            AnimatorSet().apply {

                playTogether(
                    pointScaleX,
                    pointScaleY
                )

                duration = 850

                interpolator = DecelerateInterpolator()

                start()
            }

        }, 250)

        /*
         * ------------------------------------------------
         * 6. Slowly rotate outer ring
         * ------------------------------------------------
         */

        handler.postDelayed({

            ObjectAnimator.ofFloat(
                ringOuter,
                View.ROTATION,
                0f,
                360f
            ).apply {

                duration = 12000

                repeatCount = ValueAnimator.INFINITE

                interpolator =
                    android.view.animation.LinearInterpolator()

                start()

            }

        }, 500)

        /*
         * ------------------------------------------------
         * 7. App name reveal
         * ------------------------------------------------
         */

        handler.postDelayed({

            val nameFade = ObjectAnimator.ofFloat(
                appName,
                View.ALPHA,
                0f,
                1f
            )

            val nameMove = ObjectAnimator.ofFloat(
                appName,
                View.TRANSLATION_Y,
                12f,
                0f
            )

            AnimatorSet().apply {

                playTogether(
                    nameFade,
                    nameMove
                )

                duration = 650

                interpolator = DecelerateInterpolator()

                start()
            }

        }, 550)

        /*
         * ------------------------------------------------
         * 8. Loading section
         * ------------------------------------------------
         */

        handler.postDelayed({

            ObjectAnimator.ofFloat(
                loadingContainer,
                View.ALPHA,
                0f,
                1f
            ).apply {

                duration = 450

                start()
            }

        }, 850)

        /*
         * ------------------------------------------------
         * 9. Loading progress
         * ------------------------------------------------
         */

        handler.postDelayed({

            animateProgress(loadingBar)

        }, 900)

        /*
         * ------------------------------------------------
         * 10. Enter application
         * ------------------------------------------------
         */

        handler.postDelayed({

            openMainActivity()

        }, 2200)
    }

    private fun animateProgress(progressBar: ProgressBar) {

        ValueAnimator.ofInt(0, 100).apply {

            duration = 1100

            interpolator = AccelerateDecelerateInterpolator()

            addUpdateListener { animator ->

                progressBar.progress =
                    animator.animatedValue as Int

            }

            start()
        }
    }

    private fun openMainActivity() {

        val root = findViewById<View>(R.id.main)

        ObjectAnimator.ofFloat(
            root,
            View.ALPHA,
            1f,
            0f
        ).apply {

            duration = 350

            interpolator =
                AccelerateDecelerateInterpolator()

            addListener(object :
                android.animation.AnimatorListenerAdapter() {

                override fun onAnimationEnd(
                    animation: android.animation.Animator
                ) {

                    val intent = Intent(
                        this@SplashScreen,
                        MainActivity::class.java
                    )

                    startActivity(intent)

                    overridePendingTransition(
                        android.R.anim.fade_in,
                        android.R.anim.fade_out
                    )

                    finish()
                }
            })

            start()
        }
    }

    override fun onDestroy() {

        handler.removeCallbacksAndMessages(null)

        super.onDestroy()
    }
}