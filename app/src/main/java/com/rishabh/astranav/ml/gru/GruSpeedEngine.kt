package com.rishabh.astranav.ml.gru

import android.content.Context
import android.util.Log
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import ai.onnxruntime.TensorInfo
import com.google.gson.Gson
import com.rishabh.astranav.ml.OnnxModelUtils
import com.rishabh.astranav.sensor.ImuSample
import java.util.ArrayDeque
import java.util.Locale
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * ASTRA-Speed
 *
 * GRU speed estimation model.
 *
 * Input:
 *      20 x 15
 *
 * Features:
 *
 *      0  acc_x
 *      1  acc_y
 *      2  acc_z
 *      3  lin_acc_x
 *      4  lin_acc_y
 *      5  lin_acc_z
 *      6  acc_mag
 *      7  lin_acc_mag
 *      8  gyro_yaw
 *      9  gyro_pitch
 *      10 gyro_roll
 *      11 gyro_mag
 *      12 pitch_rad
 *      13 roll_rad
 *      14 prior_speed
 */
class GruSpeedEngine(context: Context) {

    companion object {

        private const val TAG =
            "ASTRA_SPEED"

        private const val WINDOW_SIZE =
            20

        private const val FEATURE_COUNT =
            15

        private const val MAX_SPEED_KMH =
            180.0
    }

    private val env =
        OrtEnvironment.getEnvironment()

    private val session: OrtSession?

    private val window =
        ArrayDeque<FloatArray>()

    private val scaler:
            StandardScaler?

    private val targetScaler:
            StandardScaler?

    init {

        var tempSession:
                OrtSession? = null

        var tempScaler:
                StandardScaler? = null

        var tempTargetScaler:
                StandardScaler? = null

        try {

            val appContext =
                context.applicationContext

            // ---------------------------------------------------------
            // MODEL
            // ---------------------------------------------------------

            val modelBytes =
                appContext.assets
                    .open(
                        "gru/gru_speed.onnx"
                    )
                    .use {
                        it.readBytes()
                    }

            tempSession =
                env.createSession(
                    modelBytes,
                    OrtSession.SessionOptions()
                )

            val gson =
                Gson()

            // ---------------------------------------------------------
            // FEATURE SCALER
            // ---------------------------------------------------------

            tempScaler =
                gson.fromJson(
                    appContext.assets
                        .open(
                            "gru/scaler.json"
                        )
                        .bufferedReader()
                        .use {
                            it.readText()
                        },
                    StandardScaler::class.java
                )

            // ---------------------------------------------------------
            // TARGET SCALER
            // ---------------------------------------------------------

            tempTargetScaler =
                gson.fromJson(
                    appContext.assets
                        .open(
                            "gru/y_scaler.json"
                        )
                        .bufferedReader()
                        .use {
                            it.readText()
                        },
                    StandardScaler::class.java
                )

            // ---------------------------------------------------------
            // VALIDATE SCALER
            // ---------------------------------------------------------

            require(
                tempScaler.n_features ==
                        FEATURE_COUNT
            ) {
                "ASTRA-Speed scaler feature count=" +
                        tempScaler.n_features +
                        " expected=$FEATURE_COUNT"
            }

            require(
                tempScaler.mean.size ==
                        FEATURE_COUNT
            ) {
                "ASTRA-Speed scaler mean size=" +
                        tempScaler.mean.size
            }

            require(
                tempScaler.scale.size ==
                        FEATURE_COUNT
            ) {
                "ASTRA-Speed scaler scale size=" +
                        tempScaler.scale.size
            }

            require(
                tempTargetScaler.mean.isNotEmpty()
            ) {
                "ASTRA-Speed target scaler empty"
            }

            require(
                tempTargetScaler.scale.isNotEmpty()
            ) {
                "ASTRA-Speed target scaler empty"
            }

            logModelMetadata(
                tempSession
            )

            Log.i(
                TAG,
                "ASTRA-Speed initialization successful"
            )

        } catch (e: Exception) {

            Log.e(
                TAG,
                "Failed to initialize ASTRA-Speed",
                e
            )

            tempSession = null
            tempScaler = null
            tempTargetScaler = null
        }

        session =
            tempSession

        scaler =
            tempScaler

        targetScaler =
            tempTargetScaler
    }

    fun isLoaded(): Boolean {

        return session != null &&
                scaler != null &&
                targetScaler != null
    }

    fun reset() {

        window.clear()
    }

    fun windowSize(): Int {

        return window.size
    }

    /**
     * Add one approximately 10 Hz sample.
     */
    fun add(
        sample: ImuSample,
        previousTrustedSpeedKmh: Double
    ): GruSpeedOutput? {

        if (!isLoaded()) {
            return null
        }

        val feature =
            buildFeatures(
                sample,
                previousTrustedSpeedKmh
            )

        if (
            window.size >=
            WINDOW_SIZE
        ) {

            window.removeFirst()
        }

        window.addLast(
            feature
        )

        if (
            window.size <
            WINDOW_SIZE
        ) {

            return null
        }

        return runInference()
    }

    /**
     * Build exact 15-feature model input.
     */
    private fun buildFeatures(
        sample: ImuSample,
        previousTrustedSpeedKmh: Double
    ): FloatArray {

        val ax =
            sample.accelX

        val ay =
            sample.accelY

        val az =
            sample.accelZ

        // raw acceleration - gravity
        val lax =
            sample.accelX -
                    sample.gravityX

        val lay =
            sample.accelY -
                    sample.gravityY

        val laz =
            sample.accelZ -
                    sample.gravityZ

        val accMag =
            sqrt(
                ax * ax +
                        ay * ay +
                        az * az
            )

        val linAccMag =
            sqrt(
                lax * lax +
                        lay * lay +
                        laz * laz
            )

        val gx =
            sample.gyroX

        val gy =
            sample.gyroY

        val gz =
            sample.gyroZ

        val gyroMag =
            sqrt(
                gx * gx +
                        gy * gy +
                        gz * gz
            )

        /*
         * Gravity-derived pitch.
         */
        val pitch =
            atan2(
                -sample.gravityX,
                sqrt(
                    sample.gravityY *
                            sample.gravityY +
                            sample.gravityZ *
                            sample.gravityZ
                )
            )

        /*
         * Gravity-derived roll.
         */
        val roll =
            atan2(
                sample.gravityY,
                sample.gravityZ
            )

        /*
         * EXACT TRAINED ORDER.
         */
        return floatArrayOf(

            // 0-2
            ax.toFloat(),
            ay.toFloat(),
            az.toFloat(),

            // 3-5
            lax.toFloat(),
            lay.toFloat(),
            laz.toFloat(),

            // 6-7
            accMag.toFloat(),
            linAccMag.toFloat(),

            // 8-10
            gz.toFloat(),
            gy.toFloat(),
            gx.toFloat(),

            // 11
            gyroMag.toFloat(),

            // 12-13
            pitch.toFloat(),
            roll.toFloat(),

            // 14
            previousTrustedSpeedKmh
                .toFloat()
        )
    }

    private fun runInference():
            GruSpeedOutput? {

        val activeSession =
            session
                ?: return null

        val activeScaler =
            scaler
                ?: return null

        val activeTargetScaler =
            targetScaler
                ?: return null

        val start =
            System.nanoTime()

        try {

            // ---------------------------------------------------------
            // FLATTEN + SCALE
            // ---------------------------------------------------------

            val raw =
                FloatArray(
                    WINDOW_SIZE *
                            FEATURE_COUNT
                )

            var index =
                0

            for (row in window) {

                require(
                    row.size ==
                            FEATURE_COUNT
                ) {
                    "Invalid ASTRA-Speed row"
                }

                for (value in row) {

                    val featureIndex =
                        index %
                                FEATURE_COUNT

                    raw[index] =
                        activeScaler.transform(
                            featureIndex,
                            value
                        )

                    index++
                }
            }

            // ---------------------------------------------------------
            // SHAPE
            // ---------------------------------------------------------

            val inputShape =
                resolveInputShape(
                    activeSession
                )

            // ---------------------------------------------------------
            // TENSOR
            // ---------------------------------------------------------

            val input =
                OnnxModelUtils
                    .createFloatTensor(
                        env,
                        raw,
                        inputShape
                    )

            input.use {

                // -----------------------------------------------------
                // INFERENCE
                // -----------------------------------------------------

                activeSession.run(
                    mapOf(
                        activeSession
                            .inputNames
                            .first() to input
                    )
                ).use { outputs ->

                    if (
                        outputs.size() == 0
                    ) {

                        Log.e(
                            TAG,
                            "ASTRA-Speed returned zero outputs"
                        )

                        return null
                    }

                    val rawOutput =
                        OnnxModelUtils
                            .flattenOutput(
                                outputs[0].value
                            )

                    if (
                        rawOutput.isEmpty()
                    ) {

                        Log.e(
                            TAG,
                            "ASTRA-Speed returned empty output"
                        )

                        return null
                    }

                    val normalizedSpeed =
                        rawOutput[0]
                            .toDouble()

                    if (
                        !normalizedSpeed.isFinite()
                    ) {

                        Log.e(
                            TAG,
                            "ASTRA-Speed output is non-finite"
                        )

                        return null
                    }

                    // -------------------------------------------------
                    // INVERSE TARGET SCALER
                    // -------------------------------------------------

                    val speedKmh =
                        activeTargetScaler.inverse(
                            normalizedSpeed
                        )

                    val speedMps =
                        speedKmh /
                                3.6

                    // -------------------------------------------------
                    // PHYSICAL VALIDATION
                    // -------------------------------------------------

                    val valid =
                        speedKmh.isFinite() &&
                                speedMps.isFinite() &&
                                speedKmh >= 0.0 &&
                                speedKmh <=
                                MAX_SPEED_KMH

                    val latency =
                        (
                                System.nanoTime() -
                                        start
                                ) / 1_000_000L

                    Log.d(
                        TAG,
                        String.format(
                            Locale.US,
                            "RESULT raw=%.6f " +
                                    "speed=%.3f km/h " +
                                    "mps=%.3f " +
                                    "valid=%s " +
                                    "latency=%dms",
                            normalizedSpeed,
                            speedKmh,
                            speedMps,
                            valid,
                            latency
                        )
                    )

                    return GruSpeedOutput(

                        normalizedOutput =
                            normalizedSpeed,

                        speedKmh =
                            speedKmh,

                        speedMps =
                            speedMps,

                        valid =
                            valid,

                        inferenceMs =
                            latency,

                        samplesUsed =
                            WINDOW_SIZE
                    )
                }
            }

        } catch (e: Exception) {

            Log.e(
                TAG,
                "ASTRA-Speed inference failed",
                e
            )

            return null
        }
    }

    /**
     * Resolve:
     *
     *      [1,20,15]
     *
     * or:
     *
     *      [20,15]
     */
    private fun resolveInputShape(
        activeSession: OrtSession
    ): LongArray {

        val nodeInfo =
            activeSession
                .inputInfo
                .values
                .firstOrNull()
                ?: throw IllegalStateException(
                    "ASTRA-Speed has no input metadata"
                )

        val tensorInfo =
            nodeInfo.info as? TensorInfo
                ?: throw IllegalStateException(
                    "ASTRA-Speed input is not a tensor"
                )

        val shape =
            tensorInfo.shape

        Log.d(
            TAG,
            "Model input shape=" +
                    shape.contentToString()
        )

        return when (
            shape.size
        ) {

            3 -> {

                validateDimension(
                    shape[1],
                    WINDOW_SIZE.toLong(),
                    "timesteps"
                )

                validateDimension(
                    shape[2],
                    FEATURE_COUNT.toLong(),
                    "features"
                )

                longArrayOf(
                    1L,
                    WINDOW_SIZE.toLong(),
                    FEATURE_COUNT.toLong()
                )
            }

            2 -> {

                validateDimension(
                    shape[0],
                    WINDOW_SIZE.toLong(),
                    "timesteps"
                )

                validateDimension(
                    shape[1],
                    FEATURE_COUNT.toLong(),
                    "features"
                )

                longArrayOf(
                    WINDOW_SIZE.toLong(),
                    FEATURE_COUNT.toLong()
                )
            }

            else -> {

                throw IllegalStateException(
                    "Unexpected ASTRA-Speed input rank=" +
                            shape.size +
                            " shape=" +
                            shape.contentToString()
                )
            }
        }
    }

    private fun validateDimension(
        actual: Long,
        expected: Long,
        label: String
    ) {

        if (
            actual == -1L
        ) {
            return
        }

        require(
            actual == expected
        ) {

            "Unexpected ASTRA-Speed $label: " +
                    "expected=$expected " +
                    "actual=$actual"
        }
    }

    private fun logModelMetadata(
        activeSession: OrtSession
    ) {

        Log.i(
            TAG,
            "=========================================="
        )

        Log.i(
            TAG,
            "ASTRA-SPEED ONNX MODEL"
        )

        Log.i(
            TAG,
            "Inputs=${activeSession.inputNames}"
        )

        Log.i(
            TAG,
            "Outputs=${activeSession.outputNames}"
        )

        activeSession.inputInfo.forEach {
                (name, info) ->

            val tensorInfo =
                info.info as? TensorInfo

            Log.i(
                TAG,
                "INPUT name=$name " +
                        "shape=${
                            tensorInfo?.shape
                                ?.contentToString()
                        }"
            )
        }

        activeSession.outputInfo.forEach {
                (name, info) ->

            val tensorInfo =
                info.info as? TensorInfo

            Log.i(
                TAG,
                "OUTPUT name=$name " +
                        "shape=${
                            tensorInfo?.shape
                                ?.contentToString()
                        }"
            )
        }

        Log.i(
            TAG,
            "Expected window=$WINDOW_SIZE"
        )

        Log.i(
            TAG,
            "Expected features=$FEATURE_COUNT"
        )

        Log.i(
            TAG,
            "=========================================="
        )
    }

    /**
     * sklearn StandardScaler JSON representation.
     */
    data class StandardScaler(

        val type: String = "",

        val n_features: Int = 0,

        val mean:
        List<Double> =
            emptyList(),

        val scale:
        List<Double> =
            emptyList()
    ) {

        fun transform(
            index: Int,
            value: Float
        ): Float {

            require(
                index in mean.indices
            ) {
                "Missing scaler mean index=$index"
            }

            require(
                index in scale.indices
            ) {
                "Missing scaler scale index=$index"
            }

            val scaleValue =
                scale[index]

            require(
                scaleValue.isFinite() &&
                        scaleValue != 0.0
            ) {
                "Invalid scaler scale=$scaleValue"
            }

            return (
                    (
                            value.toDouble() -
                                    mean[index]
                            ) / scaleValue
                    ).toFloat()
        }

        fun inverse(
            value: Double
        ): Double {

            require(
                mean.isNotEmpty()
            ) {
                "Target scaler mean empty"
            }

            require(
                scale.isNotEmpty()
            ) {
                "Target scaler scale empty"
            }

            return (
                    value *
                            scale[0]
                    ) + mean[0]
        }
    }
}