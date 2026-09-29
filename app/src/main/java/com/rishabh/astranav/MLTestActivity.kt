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

import com.rishabh.astranav.navigation.eskf.Eskf
import com.rishabh.astranav.navigation.eskf.Vec3
import com.rishabh.astranav.navigation.zupt.ZuptDetector
import com.rishabh.astranav.navigation.zupt.ZuptResult

import com.rishabh.astranav.sensor.ImuSample

import java.util.Locale
import kotlin.math.atan2
import kotlin.math.sqrt


class MLTestActivity :
    AppCompatActivity(),
    SensorEventListener {

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
    // ASTRA-CORE ESKF
    // ============================================================

    private lateinit var eskf: Eskf

    private var latestPredictionAccepted = false

    private var latestZuptAccepted = false
    private var latestZaruAccepted = false
    private var latestNhcAccepted = false

    private var latestZuptNis = Double.NaN
    private var latestZaruNis = Double.NaN
    private var latestNhcNis = Double.NaN

    private var latestEskfReason: String? = null


    // ============================================================
    // SENSOR VALUES
    // ============================================================

    private var latestAccel =
        FloatArray(3)

    private var latestGyro =
        FloatArray(3)

    private var latestGravity =
        floatArrayOf(
            0f,
            0f,
            9.81f
        )


    // ============================================================
    // TIMING
    // ============================================================

    private var lastSensorTimestampNs =
        0L

    private var lastModelSampleNs =
        0L

    private var previousModelSampleNsForMotion =
        0L


    // ============================================================
    // TEMPORAL MODEL STATE
    // ============================================================

    private var previousTrustedSpeedKmh =
        0.0

    private var previousYawRate =
        0.0


    // ============================================================
    // COUNTERS
    // ============================================================

    private var sampleCount =
        0


    // ============================================================
    // LATEST MODEL OUTPUTS
    // ============================================================

    private var latestSpeedOutput:
            GruSpeedOutput? = null

    private var latestMotionOutput:
            AstraMotionOutput? = null

    private var latestSphmOutput:
            AstraSphmOutput? = null


    // ============================================================
    // DVFC
    // ============================================================

    private var dvfcTransform:
            Quat? = null


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
    // ASTRA-CORE UI
    // ============================================================

    private lateinit var tvEskfStatus: TextView
    private lateinit var tvEskfPosition: TextView
    private lateinit var tvEskfVelocity: TextView
    private lateinit var tvEskfHeading: TextView
    private lateinit var tvEskfBias: TextView
    private lateinit var tvEskfConstraints: TextView
    private lateinit var tvEskfHealth: TextView


    // ============================================================
    // ACTIVITY CREATE
    // ============================================================

    override fun onCreate(
        savedInstanceState: Bundle?
    ) {

        super.onCreate(
            savedInstanceState
        )

        setContentView(
            R.layout.activity_mltest
        )

        bindViews()


        // --------------------------------------------------------
        // SENSOR MANAGER
        // --------------------------------------------------------

        sensorManager =
            getSystemService(
                SENSOR_SERVICE
            ) as SensorManager

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
        // ASTRA-CORE
        // --------------------------------------------------------

        eskf =
            Eskf()

        tvEskfStatus.text =
            "● ESKF READY · WAITING FOR IMU"


        // --------------------------------------------------------
        // LOAD DVFC
        // --------------------------------------------------------

        dvfcTransform =
            DvfcCalibrationStore.load(
                this
            )

        tvDvfcStatus.text =
            if (dvfcTransform != null) {
                "DVFC: ACTIVE"
            } else {
                "DVFC: NOT CALIBRATED"
            }


        // --------------------------------------------------------
        // RESET
        // --------------------------------------------------------

        findViewById<Button>(
            R.id.btnResetMl
        ).setOnClickListener {

            resetModels()
        }


        // --------------------------------------------------------
        // CLOSE
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
            findViewById(
                R.id.tvMlSystemStatus
            )

        tvSensorStatus =
            findViewById(
                R.id.tvMlSensorStatus
            )

        tvDvfcStatus =
            findViewById(
                R.id.tvMlDvfcStatus
            )


        tvAccel =
            findViewById(
                R.id.tvMlAccel
            )

        tvGravity =
            findViewById(
                R.id.tvMlGravity
            )

        tvGyro =
            findViewById(
                R.id.tvMlGyro
            )

        tvRate =
            findViewById(
                R.id.tvMlRate
            )


        tvSpeedWindow =
            findViewById(
                R.id.tvSpeedWindow
            )

        tvSpeedRaw =
            findViewById(
                R.id.tvSpeedRaw
            )

        tvSpeedKmh =
            findViewById(
                R.id.tvSpeedKmh
            )

        tvSpeedMps =
            findViewById(
                R.id.tvSpeedMps
            )

        tvSpeedLatency =
            findViewById(
                R.id.tvSpeedLatency
            )


        tvMotionWindow =
            findViewById(
                R.id.tvMotionWindow
            )

        tvMotionRaw =
            findViewById(
                R.id.tvMotionRaw
            )

        tvMotionProcessed =
            findViewById(
                R.id.tvMotionProcessed
            )

        tvMotionLatency =
            findViewById(
                R.id.tvMotionLatency
            )


        tvSphmWindow =
            findViewById(
                R.id.tvSphmWindow
            )

        tvSphmRaw =
            findViewById(
                R.id.tvSphmRaw
            )

        tvSphmProcessed =
            findViewById(
                R.id.tvSphmProcessed
            )

        tvSphmLatency =
            findViewById(
                R.id.tvSphmLatency
            )


        tvPriorSpeed =
            findViewById(
                R.id.tvPriorSpeed
            )


        // --------------------------------------------------------
        // ASTRA-CORE
        // --------------------------------------------------------

        tvEskfStatus =
            findViewById(
                R.id.tvEskfStatus
            )

        tvEskfPosition =
            findViewById(
                R.id.tvEskfPosition
            )

        tvEskfVelocity =
            findViewById(
                R.id.tvEskfVelocity
            )

        tvEskfHeading =
            findViewById(
                R.id.tvEskfHeading
            )

        tvEskfBias =
            findViewById(
                R.id.tvEskfBias
            )

        tvEskfConstraints =
            findViewById(
                R.id.tvEskfConstraints
            )

        tvEskfHealth =
            findViewById(
                R.id.tvEskfHealth
            )
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

        sensorManager.unregisterListener(
            this
        )
    }


    // ============================================================
    // SENSOR CALLBACK
    // ============================================================

    override fun onSensorChanged(
        event: SensorEvent
    ) {

        when (
            event.sensor.type
        ) {

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
        // 10 HZ MODEL TIMELINE
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
    // MAIN PIPELINE
    // ============================================================

    private fun processModelSample(
        timestampNs: Long
    ) {

        sampleCount++


        // --------------------------------------------------------
        // RAW SENSOR VALUES
        // --------------------------------------------------------

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


        // --------------------------------------------------------
        // IMU SAMPLE
        // --------------------------------------------------------

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


        updateSensorUi(
            sample
        )


        // ========================================================
        // DEVICE → VEHICLE
        // ========================================================

        val deviceLinear =
            floatArrayOf(
                (ax - gravX).toFloat(),
                (ay - gravY).toFloat(),
                (az - gravZ).toFloat()
            )

        val deviceGyro =
            floatArrayOf(
                gx.toFloat(),
                gy.toFloat(),
                gz.toFloat()
            )

        val vehicleLinear =
            dvfcTransform?.rotate(
                deviceLinear
            ) ?: deviceLinear

        val vehicleGyro =
            dvfcTransform?.rotate(
                deviceGyro
            ) ?: deviceGyro


        // ========================================================
        // ASTRA-CORE ESKF PREDICTION
        // ========================================================

        try {

            val prediction =
                eskf.predict(

                    timestampNanos =
                        timestampNs,

                    accelerationBody =
                        Vec3(

                            vehicleLinear[0]
                                .toDouble(),

                            vehicleLinear[1]
                                .toDouble(),

                            vehicleLinear[2]
                                .toDouble()
                        ),

                    gyroBody =
                        Vec3(

                            vehicleGyro[0]
                                .toDouble(),

                            vehicleGyro[1]
                                .toDouble(),

                            vehicleGyro[2]
                                .toDouble()
                        )
                )

            latestPredictionAccepted =
                prediction.accepted

            latestEskfReason =
                prediction.reason

        } catch (e: Exception) {

            latestPredictionAccepted =
                false

            latestEskfReason =
                "Prediction: ${e.message}"
        }


        // ========================================================
        // ASTRA-SPEED
        // ========================================================

        try {

            val speedOutput =
                astraSpeed.add(

                    sample =
                        sample,

                    previousTrustedSpeedKmh =
                        previousTrustedSpeedKmh
                )

            if (
                speedOutput != null
            ) {

                latestSpeedOutput =
                    speedOutput

                if (
                    speedOutput.valid
                ) {

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

        var motionOutput:
                AstraMotionOutput? =
            null

        try {

            val aFwd =
                vehicleLinear[0]
                    .toDouble()

            val aLat =
                vehicleLinear[1]
                    .toDouble()

            val wYaw =
                vehicleGyro[2]
                    .toDouble()


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


            val yawAccel =
                (
                        wYaw -
                                previousYawRate
                        ) / dt

            previousYawRate =
                wYaw

            previousModelSampleNsForMotion =
                timestampNs


            motionOutput =
                astraMotion.add(

                    aFwd =
                        aFwd,

                    wYaw =
                        wYaw,

                    aLat =
                        aLat,

                    previousSpeedMps =
                        previousTrustedSpeedKmh /
                                3.6,

                    yawAccel =
                        yawAccel
                )


            if (
                motionOutput != null
            ) {

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

        try {

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
                        eskf.getVelocity().norm(),

                    zuptLogit =
                        motionOutput?.zuptScore
                )


            latestZuptResult =
                zuptResult


            // ====================================================
            // ESKF ZUPT / ZARU / NHC
            // ====================================================

            latestZuptAccepted =
                false

            latestZaruAccepted =
                false

            latestNhcAccepted =
                false

            latestZuptNis =
                Double.NaN

            latestZaruNis =
                Double.NaN

            latestNhcNis =
                Double.NaN


            if (
                zuptResult.active
            ) {

                // ------------------------------------------------
                // ZUPT
                // ------------------------------------------------

                try {

                    val result =
                        eskf.applyZupt()

                    latestZuptAccepted =
                        result.accepted

                    latestZuptNis =
                        result.nis

                    latestEskfReason =
                        result.reason

                } catch (e: Exception) {

                    latestEskfReason =
                        "ZUPT: ${e.message}"
                }


                // ------------------------------------------------
                // ZARU
                //
                // IMPORTANT:
                // raw gyro measurement
                // ------------------------------------------------

                try {

                    val result =
                        eskf.applyZaru(

                            Vec3(

                                vehicleGyro[0]
                                    .toDouble(),

                                vehicleGyro[1]
                                    .toDouble(),

                                vehicleGyro[2]
                                    .toDouble()
                            )
                        )

                    latestZaruAccepted =
                        result.accepted

                    latestZaruNis =
                        result.nis

                } catch (e: Exception) {

                    latestEskfReason =
                        "ZARU: ${e.message}"
                }

            } else {

                // ------------------------------------------------
                // NHC
                // ------------------------------------------------

                val speed =
                    eskf.getVelocity().norm()

                if (
                    speed >= 1.0 &&
                    speed <= 60.0
                ) {

                    try {

                        val result =
                            eskf.applyNhc()

                        latestNhcAccepted =
                            result.accepted

                        latestNhcNis =
                            result.nis

                        latestEskfReason =
                            result.reason

                    } catch (e: Exception) {

                        latestEskfReason =
                            "NHC: ${e.message}"
                    }
                }
            }


        } catch (e: Exception) {

            Log.e(
                "ASTRA_ZUPT",
                "ZUPT / ESKF constraint error",
                e
            )
        }


        // ========================================================
        // ASTRA-SPHM
        // ========================================================

        try {

            val sphmOutput =
                astraSphm.addSample(

                    sample =
                        sample,

                    initialSpeedMps =
                        eskf.getVelocity().norm()
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
        // UPDATE ASTRA-CORE UI
        // ========================================================

        renderEskf()


        // ========================================================
        // PRIOR SPEED
        // ========================================================

        tvPriorSpeed.text =
            String.format(

                Locale.US,

                "%.2f km/h",

                eskf.getVelocity().norm() *
                        3.6
            )


        // ========================================================
        // SENSOR STATUS
        // ========================================================

        tvSensorStatus.text =
            "● LIVE · sample $sampleCount"
    }


    // ============================================================
    // ASTRA-CORE UI
    // ============================================================

    private fun renderEskf() {

        val state =
            eskf.getState()

        val velocity =
            state.velocity

        val position =
            state.position

        val speed =
            velocity.norm()

        val horizontalSpeed =
            sqrt(

                velocity.x *
                        velocity.x +

                        velocity.y *
                        velocity.y
            )


        val heading =
            if (
                horizontalSpeed >
                0.20
            ) {

                (
                        Math.toDegrees(
                            atan2(
                                velocity.y,
                                velocity.x
                            )
                        ) + 360.0
                        ) % 360.0

            } else {

                Double.NaN
            }


        val diagnostics =
            eskf.diagnostics()


        tvEskfStatus.text =
            if (
                latestPredictionAccepted
            ) {

                "● RUNNING · ESKF PREDICTION ACCEPTED"

            } else {

                "● WAITING / PREDICTION REJECTED"
            }


        tvEskfPosition.text =
            String.format(

                Locale.US,

                "N %+.3f   E %+.3f   D %+.3f m",

                position.x,
                position.y,
                position.z
            )


        tvEskfVelocity.text =
            String.format(

                Locale.US,

                "Vn %+.3f   Ve %+.3f   Vd %+.3f m/s   |V| %.3f",

                velocity.x,
                velocity.y,
                velocity.z,

                speed
            )


        tvEskfHeading.text =
            if (
                heading.isFinite()
            ) {

                String.format(

                    Locale.US,

                    "HEADING %.2f°   H-SPEED %.3f m/s",

                    heading,
                    horizontalSpeed
                )

            } else {

                String.format(

                    Locale.US,

                    "HEADING --   H-SPEED %.3f m/s",

                    horizontalSpeed
                )
            }


        tvEskfBias.text =
            String.format(

                Locale.US,

                "GYRO BIAS [%+.5f, %+.5f, %+.5f]\n" +
                        "ACC BIAS  [%+.4f, %+.4f, %+.4f]",

                state.gyroBias.x,
                state.gyroBias.y,
                state.gyroBias.z,

                state.accelBias.x,
                state.accelBias.y,
                state.accelBias.z
            )


        val zuptText =
            when {

                latestZuptAccepted ->
                    "ACCEPT"

                latestZuptResult?.active == true ->
                    "REJECT"

                else ->
                    "OFF"
            }


        val zaruText =
            when {

                latestZaruAccepted ->
                    "ACCEPT"

                latestZuptResult?.active == true ->
                    "REJECT"

                else ->
                    "OFF"
            }


        val nhcText =
            if (
                latestNhcAccepted
            ) {

                "ACCEPT"

            } else {

                "OFF"
            }


        tvEskfConstraints.text =
            "ZUPT  $zuptText   NIS ${formatNis(latestZuptNis)}\n" +
                    "ZARU  $zaruText   NIS ${formatNis(latestZaruNis)}\n" +
                    "NHC   $nhcText   NIS ${formatNis(latestNhcNis)}"


        tvEskfHealth.text =
            "STATE finite=${diagnostics.stateFinite}\n" +
                    "P finite=${diagnostics.covarianceFinite}  " +
                    "diag=${diagnostics.covarianceDiagonalValid}\n" +
                    "Prediction accepted=" +
                    diagnostics.acceptedPredictionCount +
                    " rejected=" +
                    diagnostics.rejectedPredictionCount +
                    "\nSpeed=%.3f m/s".format(
                        Locale.US,
                        speed
                    ) +
                    (
                            latestEskfReason?.let {
                                "\nLast: $it"
                            } ?: ""
                            )
    }


    private fun formatNis(
        value: Double
    ): String {

        return if (
            value.isFinite()
        ) {

            String.format(
                Locale.US,
                "%.3f",
                value
            )

        } else {

            "--"
        }
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

                if (
                    zupt != null
                ) {

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

        tvSphmWindow.text =
            "${astraSphm.windowSize()} / 20"


        if (
            !output.valid
        ) {

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


        tvSphmProcessed.text =
            buildString {

                append(
                    "STATUS = "
                )

                append(

                    if (
                        output.valid
                    ) {

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


        tvSphmLatency.text =
            "${output.latencyMs} ms"
    }


    // ============================================================
    // RESET
    // ============================================================

    private fun resetModels() {

        try {

            eskf.reset()

            astraSpeed.reset()

            astraMotion.reset()

            astraSphm.clearWindow()

            zuptDetector.reset()

        } catch (_: Exception) {
        }


        previousTrustedSpeedKmh =
            0.0

        previousYawRate =
            0.0

        previousModelSampleNsForMotion =
            0L

        lastModelSampleNs =
            0L

        lastSensorTimestampNs =
            0L

        sampleCount =
            0


        latestSpeedOutput =
            null

        latestMotionOutput =
            null

        latestSphmOutput =
            null

        latestZuptResult =
            null


        latestPredictionAccepted =
            false

        latestZuptAccepted =
            false

        latestZaruAccepted =
            false

        latestNhcAccepted =
            false

        latestZuptNis =
            Double.NaN

        latestZaruNis =
            Double.NaN

        latestNhcNis =
            Double.NaN

        latestEskfReason =
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
        // ASTRA-CORE
        // --------------------------------------------------------

        tvEskfStatus.text =
            "● READY · WAITING FOR IMU"

        tvEskfPosition.text =
            "N +0.000   E +0.000   D +0.000 m"

        tvEskfVelocity.text =
            "Vn +0.000   Ve +0.000   Vd +0.000 m/s   |V| 0.000"

        tvEskfHeading.text =
            "HEADING --   H-SPEED 0.000 m/s"

        tvEskfBias.text =
            "GYRO BIAS [0.00000, 0.00000, 0.00000]\n" +
                    "ACC BIAS  [0.0000, 0.0000, 0.0000]"

        tvEskfConstraints.text =
            "ZUPT OFF   NIS --\n" +
                    "ZARU OFF   NIS --\n" +
                    "NHC OFF    NIS --"

        tvEskfHealth.text =
            "P finite=--  diag=--\n" +
                    "Prediction accepted=0 rejected=0\n" +
                    "State finite=--"


        tvPriorSpeed.text =
            "0.00 km/h"

        tvSensorStatus.text =
            "● RESET · WAITING"
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