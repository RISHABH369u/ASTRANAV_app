package com.rishabh.astranav

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

import com.rishabh.astranav.dvfc.DvfcCalibrationStore
import com.rishabh.astranav.dvfc.math.Quat

import com.rishabh.astranav.ml.astramotion.AstraMotionEngine
import com.rishabh.astranav.ml.astramotion.AstraMotionOutput

import com.rishabh.astranav.ml.gru.GruSpeedEngine
import com.rishabh.astranav.ml.gru.GruSpeedOutput

import com.rishabh.astranav.ml.astrasphm.AstraSphmEngine
import com.rishabh.astranav.ml.astrasphm.AstraSphmOutput

import com.rishabh.astranav.navigation.zupt.ZuptDetector
import com.rishabh.astranav.navigation.zupt.ZuptResult

import com.rishabh.astranav.sensor.ImuSample

import java.util.Locale
import kotlin.math.sqrt


class MLTestActivity : AppCompatActivity(), SensorEventListener {

    // ============================================================
    // SENSOR SYSTEM
    // ============================================================

    private lateinit var sensorManager: SensorManager

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var gravitySensor: Sensor? = null


    // ============================================================
    // ML ENGINES
    // ============================================================

    private lateinit var astraSpeed: GruSpeedEngine
    private lateinit var astraMotion: AstraMotionEngine
    private lateinit var astraSphm: AstraSphmEngine


    // ============================================================
    // ZUPT
    // ============================================================

    private lateinit var zuptDetector: ZuptDetector

    private var latestZuptResult: ZuptResult? = null


    // ============================================================
    // SENSOR VALUES
    // ============================================================

    private var latestAccel = FloatArray(3)

    private var latestGyro = FloatArray(3)

    private var latestGravity =
        floatArrayOf(
            0f,
            0f,
            9.81f
        )


    // ============================================================
    // TIMING
    // ============================================================

    private var lastSensorTimestampNs = 0L

    private var lastModelSampleNs = 0L

    private var previousModelSampleNsForMotion = 0L


    // ============================================================
    // TEMPORAL MODEL STATE
    // ============================================================

    private var previousTrustedSpeedKmh = 0.0

    private var previousYawRate = 0.0


    // ============================================================
    // COUNTERS
    // ============================================================

    private var sampleCount = 0


    // ============================================================
    // LATEST MODEL OUTPUTS
    // ============================================================

    private var latestSpeedOutput: GruSpeedOutput? = null

    private var latestMotionOutput: AstraMotionOutput? = null

    private var latestSphmOutput: AstraSphmOutput? = null


    // ============================================================
    // DVFC
    // ============================================================

    private var dvfcTransform: Quat? = null


    // ============================================================
    // SYSTEM UI
    // ============================================================

    private lateinit var tvSystemStatus: TextView
    private lateinit var tvSensorStatus: TextView
    private lateinit var tvDvfcStatus: TextView


    // ============================================================
    // SENSOR UI
    // ============================================================

    private lateinit var tvAccel: TextView
    private lateinit var tvGravity: TextView
    private lateinit var tvGyro: TextView
    private lateinit var tvRate: TextView


    // ============================================================
    // ASTRA-SPEED UI
    // ============================================================

    private lateinit var tvSpeedWindow: TextView
    private lateinit var tvSpeedRaw: TextView
    private lateinit var tvSpeedKmh: TextView
    private lateinit var tvSpeedMps: TextView
    private lateinit var tvSpeedLatency: TextView


    // ============================================================
    // ASTRA-MOTION UI
    // ============================================================

    private lateinit var tvMotionWindow: TextView
    private lateinit var tvMotionRaw: TextView
    private lateinit var tvMotionProcessed: TextView
    private lateinit var tvMotionLatency: TextView


    // ============================================================
    // ASTRA-SPHM UI
    // ============================================================

    private lateinit var tvSphmWindow: TextView
    private lateinit var tvSphmRaw: TextView
    private lateinit var tvSphmProcessed: TextView
    private lateinit var tvSphmLatency: TextView


    // ============================================================
    // OTHER UI
    // ============================================================

    private lateinit var tvPriorSpeed: TextView


    // ============================================================
    // ACTIVITY CREATE
    // ============================================================

    override fun onCreate(savedInstanceState: Bundle?) {

        super.onCreate(savedInstanceState)

        setContentView(R.layout.activity_mltest)

        bindViews()


        // --------------------------------------------------------
        // SENSOR MANAGER
        // --------------------------------------------------------

        sensorManager =
            getSystemService(SENSOR_SERVICE) as SensorManager


        accelerometer =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_ACCELEROMETER
            )

        gyroscope =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_GYROSCOPE
            )

        gravitySensor =
            sensorManager.getDefaultSensor(
                Sensor.TYPE_GRAVITY
            )


        // --------------------------------------------------------
        // LOAD ML MODELS
        // --------------------------------------------------------

        try {

            astraSpeed =
                GruSpeedEngine(this)

            astraMotion =
                AstraMotionEngine(this)

            astraSphm =
                AstraSphmEngine(this)

            zuptDetector =
                ZuptDetector()


            tvSystemStatus.text =
                "● MODELS LOADED"

            tvSystemStatus.setTextColor(
                getColor(R.color.good)
            )

        } catch (e: Exception) {

            tvSystemStatus.text =
                "● MODEL LOAD ERROR"

            tvSystemStatus.setTextColor(
                getColor(R.color.warn)
            )

            tvMotionRaw.text =
                "ERROR\n${e.message}"

            tvSpeedRaw.text =
                "ERROR\n${e.message}"

            tvSphmRaw.text =
                "ERROR\n${e.message}"
        }


        // --------------------------------------------------------
        // LOAD DVFC
        // --------------------------------------------------------

        dvfcTransform =
            DvfcCalibrationStore.load(this)


        tvDvfcStatus.text =
            if (dvfcTransform != null) {
                "DVFC: ACTIVE"
            } else {
                "DVFC: NOT CALIBRATED"
            }


        // --------------------------------------------------------
        // RESET BUTTON
        // --------------------------------------------------------

        findViewById<Button>(
            R.id.btnResetMl
        ).setOnClickListener {

            resetModels()
        }


        // --------------------------------------------------------
        // CLOSE BUTTON
        // --------------------------------------------------------

        findViewById<Button>(
            R.id.btnCloseMl
        ).setOnClickListener {

            finish()
        }
    }


    // ============================================================
    // BIND UI
    // ============================================================

    private fun bindViews() {

        tvSystemStatus =
            findViewById(R.id.tvMlSystemStatus)

        tvSensorStatus =
            findViewById(R.id.tvMlSensorStatus)

        tvDvfcStatus =
            findViewById(R.id.tvMlDvfcStatus)


        tvAccel =
            findViewById(R.id.tvMlAccel)

        tvGravity =
            findViewById(R.id.tvMlGravity)

        tvGyro =
            findViewById(R.id.tvMlGyro)

        tvRate =
            findViewById(R.id.tvMlRate)


        // --------------------------------------------------------
        // SPEED
        // --------------------------------------------------------

        tvSpeedWindow =
            findViewById(R.id.tvSpeedWindow)

        tvSpeedRaw =
            findViewById(R.id.tvSpeedRaw)

        tvSpeedKmh =
            findViewById(R.id.tvSpeedKmh)

        tvSpeedMps =
            findViewById(R.id.tvSpeedMps)

        tvSpeedLatency =
            findViewById(R.id.tvSpeedLatency)


        // --------------------------------------------------------
        // MOTION
        // --------------------------------------------------------

        tvMotionWindow =
            findViewById(R.id.tvMotionWindow)

        tvMotionRaw =
            findViewById(R.id.tvMotionRaw)

        tvMotionProcessed =
            findViewById(R.id.tvMotionProcessed)

        tvMotionLatency =
            findViewById(R.id.tvMotionLatency)


        // --------------------------------------------------------
        // SPHM
        // --------------------------------------------------------

        tvSphmWindow =
            findViewById(R.id.tvSphmWindow)

        tvSphmRaw =
            findViewById(R.id.tvSphmRaw)

        tvSphmProcessed =
            findViewById(R.id.tvSphmProcessed)

        tvSphmLatency =
            findViewById(R.id.tvSphmLatency)


        // --------------------------------------------------------
        // PRIOR SPEED
        // --------------------------------------------------------

        tvPriorSpeed =
            findViewById(R.id.tvPriorSpeed)
    }


    // ============================================================
    // RESUME
    // ============================================================

    override fun onResume() {

        super.onResume()


        accelerometer?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }


        gyroscope?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }


        gravitySensor?.let {

            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_GAME
            )
        }
    }


    // ============================================================
    // PAUSE
    // ============================================================

    override fun onPause() {

        super.onPause()

        sensorManager.unregisterListener(this)
    }


    // ============================================================
    // SENSOR CALLBACK
    // ============================================================

    override fun onSensorChanged(
        event: SensorEvent
    ) {

        when (event.sensor.type) {

            Sensor.TYPE_ACCELEROMETER -> {

                latestAccel =
                    event.values.copyOf()
            }


            Sensor.TYPE_GYROSCOPE -> {

                latestGyro =
                    event.values.copyOf()
            }


            Sensor.TYPE_GRAVITY -> {

                latestGravity =
                    event.values.copyOf()
            }
        }


        // --------------------------------------------------------
        // TIMESTAMP VALIDATION
        // --------------------------------------------------------

        if (
            event.timestamp <=
            lastSensorTimestampNs
        ) {
            return
        }


        lastSensorTimestampNs =
            event.timestamp


        // --------------------------------------------------------
        // 10 Hz MODEL TIMELINE
        // --------------------------------------------------------

        if (
            lastModelSampleNs != 0L &&
            event.timestamp -
            lastModelSampleNs <
            100_000_000L
        ) {

            return
        }


        lastModelSampleNs =
            event.timestamp


        processModelSample(
            event.timestamp
        )
    }


    // ============================================================
    // MAIN MODEL PIPELINE
    // ============================================================

    private fun processModelSample(
        timestampNs: Long
    ) {

        sampleCount++


        // ========================================================
        // RAW SENSOR VALUES
        // ========================================================

        val ax =
            latestAccel[0].toDouble()

        val ay =
            latestAccel[1].toDouble()

        val az =
            latestAccel[2].toDouble()


        val gx =
            latestGyro[0].toDouble()

        val gy =
            latestGyro[1].toDouble()

        val gz =
            latestGyro[2].toDouble()


        val gravX =
            latestGravity[0].toDouble()

        val gravY =
            latestGravity[1].toDouble()

        val gravZ =
            latestGravity[2].toDouble()


        // ========================================================
        // IMU SAMPLE
        // ========================================================

        val sample =
            ImuSample(

                timestampNanos =
                    timestampNs,

                accelX = ax,
                accelY = ay,
                accelZ = az,

                gyroX = gx,
                gyroY = gy,
                gyroZ = gz,

                gravityX = gravX,
                gravityY = gravY,
                gravityZ = gravZ
            )


        updateSensorUi(sample)


        // ========================================================
        // ASTRA-SPEED
        // ========================================================

        try {

            val speedOutput =
                astraSpeed.add(

                    sample = sample,

                    previousTrustedSpeedKmh =
                        previousTrustedSpeedKmh
                )


            if (speedOutput != null) {

                latestSpeedOutput =
                    speedOutput


                /*
                 * TEST LAB TEMPORAL FEEDBACK
                 *
                 * Final navigation version should use
                 * trusted ESKF velocity instead.
                 */

                if (speedOutput.valid) {

                    previousTrustedSpeedKmh =
                        speedOutput.speedKmh
                }


                renderSpeed(
                    speedOutput
                )
            }

        } catch (e: Exception) {

            tvSpeedRaw.text =
                "INFERENCE ERROR\n${e.message}"
        }


        // ========================================================
        // ASTRA-MOTION
        // ========================================================

        /*
         * IMPORTANT:
         *
         * Declare outside try because ZUPT needs the
         * motion output after inference.
         */

        var motionOutput: AstraMotionOutput? = null


        try {

            // ----------------------------------------------------
            // DEVICE LINEAR ACCELERATION
            // ----------------------------------------------------

            val linearX =
                ax - gravX

            val linearY =
                ay - gravY

            val linearZ =
                az - gravZ


            val deviceLinear =
                floatArrayOf(

                    linearX.toFloat(),
                    linearY.toFloat(),
                    linearZ.toFloat()
                )


            val deviceGyro =
                floatArrayOf(

                    gx.toFloat(),
                    gy.toFloat(),
                    gz.toFloat()
                )


            // ----------------------------------------------------
            // DEVICE → VEHICLE
            // ----------------------------------------------------

            val vehicleLinear =
                dvfcTransform?.rotate(
                    deviceLinear
                ) ?: deviceLinear


            val vehicleGyro =
                dvfcTransform?.rotate(
                    deviceGyro
                ) ?: deviceGyro


            // ----------------------------------------------------
            // VEHICLE FRAME
            // ----------------------------------------------------

            val aFwd =
                vehicleLinear[0].toDouble()

            val aLat =
                vehicleLinear[1].toDouble()

            val wYaw =
                vehicleGyro[2].toDouble()


            // ----------------------------------------------------
            // DT
            // ----------------------------------------------------

            val dt =
                if (
                    previousModelSampleNsForMotion ==
                    0L
                ) {

                    0.1

                } else {

                    (
                            timestampNs -
                                    previousModelSampleNsForMotion
                            ).toDouble() * 1e-9
                }
                    .coerceIn(
                        0.05,
                        0.5
                    )


            // ----------------------------------------------------
            // YAW ACCELERATION
            // ----------------------------------------------------

            val yawAccel =
                (
                        wYaw -
                                previousYawRate
                        ) / dt


            previousYawRate =
                wYaw


            previousModelSampleNsForMotion =
                timestampNs


            // ----------------------------------------------------
            // ASTRA-MOTION INFERENCE
            // ----------------------------------------------------

            motionOutput =
                astraMotion.add(

                    aFwd = aFwd,

                    wYaw = wYaw,

                    aLat = aLat,

                    previousSpeedMps =
                        previousTrustedSpeedKmh /
                                3.6,

                    yawAccel = yawAccel
                )


            if (motionOutput != null) {

                latestMotionOutput =
                    motionOutput

                renderMotion(
                    motionOutput
                )
            }

        } catch (e: Exception) {

            tvMotionRaw.text =
                "INFERENCE ERROR\n${e.message}"
        }


        // ========================================================
        // ZUPT DETECTOR
        // ========================================================

        /*
         * ZUPT is an EVIDENCE GENERATOR here.
         *
         * It does NOT directly force ESKF velocity to zero.
         *
         * Actual ESKF ZUPT update will be implemented in
         * the navigation estimator layer.
         */

        try {

            val motionZuptLogit =
                motionOutput?.zuptScore


            val linearAccelX =
                sample.accelX -
                        sample.gravityX

            val linearAccelY =
                sample.accelY -
                        sample.gravityY

            val linearAccelZ =
                sample.accelZ -
                        sample.gravityZ


            val zuptResult =
                zuptDetector.update(

                    gyroX =
                        sample.gyroX,

                    gyroY =
                        sample.gyroY,

                    gyroZ =
                        sample.gyroZ,

                    linearAccelX =
                        linearAccelX,

                    linearAccelY =
                        linearAccelY,

                    linearAccelZ =
                        linearAccelZ,

                    trustedSpeedMps =
                        previousTrustedSpeedKmh /
                                3.6,

                    zuptLogit =
                        motionZuptLogit
                )


            latestZuptResult =
                zuptResult


            // ----------------------------------------------------
            // LOG
            // ----------------------------------------------------

            Log.i(
                "ASTRA_ZUPT",

                "active=${zuptResult.active} " +

                        "score=${
                            String.format(
                                Locale.US,
                                "%.3f",
                                zuptResult.score
                            )
                        } " +

                        "ml=${
                            zuptResult.mlProbability?.let {

                                String.format(
                                    Locale.US,
                                    "%.3f",
                                    it
                                )

                            } ?: "--"
                        } " +

                        "gyro=${
                            String.format(
                                Locale.US,
                                "%.3f",
                                zuptResult.gyroMagnitude
                            )
                        } " +

                        "linAcc=${
                            String.format(
                                Locale.US,
                                "%.3f",
                                zuptResult.linearAccelerationMagnitude
                            )
                        } " +

                        "speed=${
                            String.format(
                                Locale.US,
                                "%.3f",
                                zuptResult.trustedSpeedMps
                            )
                        } " +

                        "reason=${zuptResult.reason}"
            )

        } catch (e: Exception) {

            Log.e(
                "ASTRA_ZUPT",
                "ZUPT detector error",
                e
            )
        }


        // ========================================================
        // ASTRA-SPHM
        // ========================================================

        try {

            val sphmOutput =
                astraSphm.addSample(

                    sample = sample,

                    initialSpeedMps =
                        previousTrustedSpeedKmh /
                                3.6
                )


            latestSphmOutput =
                sphmOutput


            renderSphm(
                sphmOutput
            )

        } catch (e: Exception) {

            tvSphmRaw.text =
                "INFERENCE ERROR\n${e.message}"

            tvSphmProcessed.text =
                "ERROR"
        }


        // ========================================================
        // PRIOR SPEED
        // ========================================================

        tvPriorSpeed.text =
            String.format(
                Locale.US,

                "%.2f km/h",

                previousTrustedSpeedKmh
            )


        // ========================================================
        // SENSOR STATUS
        // ========================================================

        tvSensorStatus.text =
            "● LIVE · sample $sampleCount"
    }


    // ============================================================
    // SENSOR UI
    // ============================================================

    private fun updateSensorUi(
        sample: ImuSample
    ) {

        tvAccel.text =
            String.format(
                Locale.US,

                "X %+.3f   Y %+.3f   Z %+.3f m/s²",

                sample.accelX,
                sample.accelY,
                sample.accelZ
            )


        tvGravity.text =
            String.format(
                Locale.US,

                "X %+.3f   Y %+.3f   Z %+.3f m/s²",

                sample.gravityX,
                sample.gravityY,
                sample.gravityZ
            )


        tvGyro.text =
            String.format(
                Locale.US,

                "X %+.4f   Y %+.4f   Z %+.4f rad/s",

                sample.gyroX,
                sample.gyroY,
                sample.gyroZ
            )


        val gyroMag =
            sqrt(

                sample.gyroX *
                        sample.gyroX +

                        sample.gyroY *
                        sample.gyroY +

                        sample.gyroZ *
                        sample.gyroZ
            )


        tvRate.text =
            String.format(
                Locale.US,

                "GYRO |MAG| %.4f rad/s",

                gyroMag
            )
    }


    // ============================================================
    // ASTRA-SPEED UI
    // ============================================================

    private fun renderSpeed(
        output: GruSpeedOutput
    ) {

        tvSpeedWindow.text =
            "${output.samplesUsed} / 20"


        tvSpeedRaw.text =
            String.format(
                Locale.US,

                "normalized = %+.6f",

                output.normalizedOutput
            )


        tvSpeedKmh.text =
            String.format(
                Locale.US,

                "%.3f km/h",

                output.speedKmh
            )


        tvSpeedMps.text =
            String.format(
                Locale.US,

                "%.3f m/s",

                output.speedMps
            )


        tvSpeedLatency.text =
            "${output.inferenceMs} ms"
    }


    // ============================================================
    // ASTRA-MOTION UI
    // ============================================================

    private fun renderMotion(
        output: AstraMotionOutput
    ) {

        tvMotionWindow.text =
            "10 / 10"


        val raw =

            if (
                output.rawOutputs.isEmpty()
            ) {

                "No output"

            } else {

                output.rawOutputs
                    .mapIndexed { index, values ->

                        val formatted =
                            values.joinToString(

                                prefix = "[",

                                postfix = "]",

                                separator = ", "
                            ) {

                                String.format(
                                    Locale.US,
                                    "%.6f",
                                    it
                                )
                            }


                        "output[$index] = $formatted"
                    }
                    .joinToString(
                        "\n"
                    )
            }


        tvMotionRaw.text =
            raw


        tvMotionProcessed.text =
            buildString {

                append(
                    "displacement = "
                )


                append(

                    output.displacementM?.let {

                        String.format(
                            Locale.US,
                            "%.4f m",
                            it
                        )

                    } ?: "--"
                )


                append("\n")


                append(
                    "orientation Δ = "
                )


                append(

                    output.orientationChangeRad?.let {

                        String.format(
                            Locale.US,
                            "%.6f rad",
                            it
                        )

                    } ?: "--"
                )


                append("\n")


                append(
                    "orientation Δ = "
                )


                append(

                    output.orientationChangeRad?.let {

                        String.format(
                            Locale.US,
                            "%.3f°",

                            Math.toDegrees(
                                it
                            )
                        )

                    } ?: "--"
                )


                append("\n")


                append(
                    "ZUPT ML logit = "
                )


                append(

                    output.zuptScore?.let {

                        String.format(
                            Locale.US,
                            "%.6f",
                            it
                        )

                    } ?: "--"
                )


                append("\n")


                val zupt =
                    latestZuptResult


                append(
                    "ZUPT detector = "
                )


                append(

                    when {

                        zupt == null ->
                            "WAITING"

                        zupt.active ->
                            "ACTIVE"

                        else ->
                            "INACTIVE"
                    }
                )


                if (zupt != null) {

                    append("\n")

                    append(
                        "ZUPT score = "
                    )

                    append(

                        String.format(
                            Locale.US,
                            "%.3f",
                            zupt.score
                        )
                    )


                    append("\n")

                    append(
                        "ZUPT reason = "
                    )

                    append(
                        zupt.reason
                    )
                }
            }


        tvMotionLatency.text =
            "${output.inferenceMs} ms"
    }


    // ============================================================
    // ASTRA-SPHM UI
    // ============================================================

    private fun renderSphm(
        output: AstraSphmOutput
    ) {

        // --------------------------------------------------------
        // WINDOW
        // --------------------------------------------------------

        tvSphmWindow.text =
            "${astraSphm.windowSize()} / 20"


        // --------------------------------------------------------
        // RAW / STATUS
        // --------------------------------------------------------

        if (!output.valid) {

            tvSphmRaw.text =
                output.errorMessage
                    ?: "Waiting for 20 samples..."

        } else {

            tvSphmRaw.text =
                buildString {

                    append(
                        "speed = "
                    )


                    append(

                        output.speedMps?.let {

                            String.format(
                                Locale.US,
                                "%.6f m/s",
                                it
                            )

                        } ?: "--"
                    )


                    append("\n")


                    append(
                        "position = "
                    )


                    if (
                        output.positionX != null &&
                        output.positionY != null
                    ) {

                        append(

                            String.format(
                                Locale.US,

                                "[%.6f, %.6f]",

                                output.positionX,
                                output.positionY
                            )
                        )

                    } else {

                        append("--")
                    }


                    append("\n")


                    append(
                        "heading_delta = "
                    )


                    append(

                        output.headingDeltaRad?.let {

                            String.format(
                                Locale.US,
                                "%.6f rad",
                                it
                            )

                        } ?: "--"
                    )
                }
        }


        // --------------------------------------------------------
        // PROCESSED
        // --------------------------------------------------------

        tvSphmProcessed.text =
            buildString {

                append(
                    "STATUS = "
                )


                append(

                    if (output.valid) {

                        "VALID"

                    } else {

                        "WAITING"
                    }
                )


                append("\n")


                append(
                    "SPEED = "
                )


                append(

                    output.speedKmh?.let {

                        String.format(
                            Locale.US,
                            "%.3f km/h",
                            it
                        )

                    } ?: "--"
                )


                append("\n")


                append(
                    "POSITION = "
                )


                if (
                    output.positionX != null &&
                    output.positionY != null
                ) {

                    append(

                        String.format(
                            Locale.US,

                            "X %.3f m  Y %.3f m",

                            output.positionX,
                            output.positionY
                        )
                    )

                } else {

                    append("--")
                }


                append("\n")


                append(
                    "HEADING Δ = "
                )


                append(

                    output.headingDeltaDeg?.let {

                        String.format(
                            Locale.US,
                            "%+.4f°",
                            it
                        )

                    } ?: "--"
                )


                append("\n")


                append(
                    "MOTION LOGITS = "
                )


                if (
                    output.motionLogits.isNotEmpty()
                ) {

                    append(

                        output.motionLogits
                            .joinToString(

                                prefix = "[",

                                postfix = "]"
                            ) {

                                String.format(
                                    Locale.US,
                                    "%.4f",
                                    it
                                )
                            }
                    )

                } else {

                    append("--")
                }
            }


        // --------------------------------------------------------
        // LATENCY
        // --------------------------------------------------------

        tvSphmLatency.text =
            "${output.latencyMs} ms"
    }


    // ============================================================
    // RESET
    // ============================================================

    private fun resetModels() {

        try {

            astraSpeed.reset()

            astraMotion.reset()

            astraSphm.clearWindow()

            zuptDetector.reset()

        } catch (_: Exception) {
        }


        // --------------------------------------------------------
        // TEMPORAL STATE
        // --------------------------------------------------------

        previousTrustedSpeedKmh =
            0.0

        previousYawRate =
            0.0

        previousModelSampleNsForMotion =
            0L

        lastModelSampleNs =
            0L


        // --------------------------------------------------------
        // COUNTERS
        // --------------------------------------------------------

        sampleCount =
            0


        // --------------------------------------------------------
        // OUTPUTS
        // --------------------------------------------------------

        latestSpeedOutput =
            null

        latestMotionOutput =
            null

        latestSphmOutput =
            null

        latestZuptResult =
            null


        // --------------------------------------------------------
        // SPHM
        // --------------------------------------------------------

        tvSphmWindow.text =
            "0 / 20"

        tvSphmRaw.text =
            "Waiting for 20 samples..."

        tvSphmProcessed.text =
            "--"

        tvSphmLatency.text =
            "-- ms"


        // --------------------------------------------------------
        // SPEED
        // --------------------------------------------------------

        tvSpeedWindow.text =
            "0 / 20"

        tvSpeedRaw.text =
            "Waiting for 20 samples..."

        tvSpeedKmh.text =
            "-- km/h"

        tvSpeedMps.text =
            "-- m/s"

        tvSpeedLatency.text =
            "-- ms"


        // --------------------------------------------------------
        // MOTION
        // --------------------------------------------------------

        tvMotionWindow.text =
            "0 / 10"

        tvMotionRaw.text =
            "Waiting for 10 samples..."

        tvMotionProcessed.text =
            "--"

        tvMotionLatency.text =
            "-- ms"


        // --------------------------------------------------------
        // PRIOR SPEED
        // --------------------------------------------------------

        tvPriorSpeed.text =
            "0.00 km/h"
    }


    // ============================================================
    // SENSOR ACCURACY
    // ============================================================

    override fun onAccuracyChanged(
        sensor: Sensor?,
        accuracy: Int
    ) {

        // Not required for current inference test.
    }
}