package com.rishabh.astranav.ml.astramotion

import android.content.Context
import android.util.Log
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import ai.onnxruntime.TensorInfo
import com.google.gson.Gson
import com.rishabh.astranav.ml.OnnxModelUtils
import java.util.ArrayDeque
import java.util.Locale

/**
 * ASTRA-Motion
 *
 * V7 / Supreme Motion Model
 *
 * Input:
 *      10 timesteps x 6 features
 *
 * Features:
 *
 *      0  a_fwd
 *      1  w_yaw
 *      2  a_lat
 *      3  v_prev
 *      4  yaw_accel
 *      5  centripetal_residual
 *
 * Each row represents one 100 ms timestep.
 *
 * Actual ONNX outputs:
 *
 *      disp_pred
 *      ori_pred
 *      zupt_logits
 *      router_weights
 *
 * IMPORTANT:
 * Output tensors are decoded by their ONNX names,
 * NOT by assumed output indexes.
 */
class AstraMotionEngine(context: Context) {

    companion object {

        private const val TAG = "ASTRA_MOTION"

        private const val WINDOW_SIZE = 10
        private const val FEATURE_COUNT = 6

        // -------------------------------------------------------------
        // Actual ONNX output names
        // -------------------------------------------------------------

        private const val OUTPUT_DISPLACEMENT = "disp_pred"
        private const val OUTPUT_ORIENTATION = "ori_pred"
        private const val OUTPUT_ZUPT = "zupt_logits"
        private const val OUTPUT_ROUTER = "router_weights"
    }

    private val env =
        OrtEnvironment.getEnvironment()

    private val session: OrtSession?

    private val window =
        ArrayDeque<FloatArray>()

    private val scaler: InputScaler?

    private val displacementScaler: OutputScaler?

    private val orientationScaler: OutputScaler?

    init {

        var tempSession: OrtSession? = null
        var tempScaler: InputScaler? = null
        var tempDispScaler: OutputScaler? = null
        var tempOriScaler: OutputScaler? = null

        try {

            val appContext =
                context.applicationContext

            // ---------------------------------------------------------
            // MODEL
            // ---------------------------------------------------------

            val modelBytes =
                appContext.assets
                    .open(
                        "astra_motion/astra_motion.onnx"
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
            // INPUT SCALER
            // ---------------------------------------------------------

            tempScaler =
                gson.fromJson(
                    appContext.assets
                        .open(
                            "astra_motion/" +
                                    "astra_motion_input_scaler.json"
                        )
                        .bufferedReader()
                        .use {
                            it.readText()
                        },
                    InputScaler::class.java
                )

            // ---------------------------------------------------------
            // DISPLACEMENT SCALER
            // ---------------------------------------------------------

            tempDispScaler =
                gson.fromJson(
                    appContext.assets
                        .open(
                            "astra_motion/" +
                                    "astra_motion_disp_scaler.json"
                        )
                        .bufferedReader()
                        .use {
                            it.readText()
                        },
                    OutputScaler::class.java
                )

            // ---------------------------------------------------------
            // ORIENTATION SCALER
            // ---------------------------------------------------------

            tempOriScaler =
                gson.fromJson(
                    appContext.assets
                        .open(
                            "astra_motion/" +
                                    "astra_motion_ori_scaler.json"
                        )
                        .bufferedReader()
                        .use {
                            it.readText()
                        },
                    OutputScaler::class.java
                )

            // ---------------------------------------------------------
            // SCALER VALIDATION
            // ---------------------------------------------------------

            require(
                tempScaler.scale.isNotEmpty()
            ) {
                "ASTRA-Motion input scaler is empty"
            }

            require(
                tempScaler.min_offset.isNotEmpty()
            ) {
                "ASTRA-Motion input scaler offset is empty"
            }

            require(
                tempScaler.scale[0].size ==
                        FEATURE_COUNT
            ) {
                "ASTRA-Motion scaler feature count=" +
                        tempScaler.scale[0].size +
                        ", expected=$FEATURE_COUNT"
            }

            // ---------------------------------------------------------
            // OUTPUT SCALER VALIDATION
            // ---------------------------------------------------------

            require(
                tempDispScaler.scale.isFinite() &&
                        tempDispScaler.scale != 0.0
            ) {
                "Invalid ASTRA-Motion displacement scaler"
            }

            require(
                tempOriScaler.scale.isFinite() &&
                        tempOriScaler.scale != 0.0
            ) {
                "Invalid ASTRA-Motion orientation scaler"
            }

            // ---------------------------------------------------------
            // LOG MODEL METADATA
            // ---------------------------------------------------------

            logModelMetadata(tempSession)

            Log.i(
                TAG,
                "ASTRA-Motion initialization successful"
            )

        } catch (e: Exception) {

            Log.e(
                TAG,
                "Failed to initialize ASTRA-Motion",
                e
            )

            tempSession = null
            tempScaler = null
            tempDispScaler = null
            tempOriScaler = null
        }

        session =
            tempSession

        scaler =
            tempScaler

        displacementScaler =
            tempDispScaler

        orientationScaler =
            tempOriScaler
    }

    /**
     * Whether the ONNX model and preprocessing assets
     * loaded successfully.
     */
    fun isLoaded(): Boolean {

        return session != null &&
                scaler != null &&
                displacementScaler != null &&
                orientationScaler != null
    }

    /**
     * Clear the temporal model window.
     */
    fun reset() {

        window.clear()
    }

    /**
     * Current number of rows in the rolling window.
     */
    fun windowSize(): Int {

        return window.size
    }

    /**
     * Adds one 10 Hz / 100 ms motion timestep.
     */
    fun add(
        aFwd: Double,
        wYaw: Double,
        aLat: Double,
        previousSpeedMps: Double,
        yawAccel: Double
    ): AstraMotionOutput? {

        val activeSession =
            session
                ?: return null

        val activeScaler =
            scaler
                ?: return null

        val activeDispScaler =
            displacementScaler
                ?: return null

        val activeOriScaler =
            orientationScaler
                ?: return null

        // ---------------------------------------------------------
        // 6 FEATURES
        // ---------------------------------------------------------

        val centripetalResidual =
            aLat -
                    (
                            previousSpeedMps *
                                    wYaw
                            )

        val row =
            floatArrayOf(

                // 0
                aFwd.toFloat(),

                // 1
                wYaw.toFloat(),

                // 2
                aLat.toFloat(),

                // 3
                previousSpeedMps.toFloat(),

                // 4
                yawAccel.toFloat(),

                // 5
                centripetalResidual.toFloat()
            )

        // ---------------------------------------------------------
        // INPUT VALIDATION
        // ---------------------------------------------------------

        if (
            row.any {
                !it.isFinite()
            }
        ) {

            Log.w(
                TAG,
                "Rejected non-finite ASTRA-Motion input"
            )

            return null
        }

        // ---------------------------------------------------------
        // SCALE INPUT
        // ---------------------------------------------------------

        val scaled =
            activeScaler.transform(
                row
            )

        // ---------------------------------------------------------
        // ROLLING WINDOW
        // ---------------------------------------------------------

        if (
            window.size >= WINDOW_SIZE
        ) {
            window.removeFirst()
        }

        window.addLast(
            scaled
        )

        // ---------------------------------------------------------
        // WAIT UNTIL 10 SAMPLES
        // ---------------------------------------------------------

        if (
            window.size < WINDOW_SIZE
        ) {
            return null
        }

        return runInference(
            activeSession,
            activeDispScaler,
            activeOriScaler
        )
    }

    /**
     * Execute ONNX inference and decode the actual
     * ASTRA-Motion output contract.
     */
    private fun runInference(
        activeSession: OrtSession,
        activeDispScaler: OutputScaler,
        activeOriScaler: OutputScaler
    ): AstraMotionOutput? {

        val start =
            System.nanoTime()

        try {

            // -----------------------------------------------------
            // FLATTEN WINDOW
            // -----------------------------------------------------

            val flat =
                FloatArray(
                    WINDOW_SIZE *
                            FEATURE_COUNT
                )

            var index = 0

            for (row in window) {

                require(
                    row.size ==
                            FEATURE_COUNT
                ) {
                    "Invalid ASTRA-Motion row size=" +
                            row.size +
                            ", expected=$FEATURE_COUNT"
                }

                for (value in row) {

                    flat[index++] =
                        value
                }
            }

            // -----------------------------------------------------
            // INPUT SHAPE
            // -----------------------------------------------------

            val inputShape =
                resolveInputShape(
                    activeSession
                )

            // -----------------------------------------------------
            // CREATE TENSOR
            // -----------------------------------------------------

            val input =
                OnnxModelUtils.createFloatTensor(
                    env,
                    flat,
                    inputShape
                )

            input.use {

                // -------------------------------------------------
                // INFERENCE
                // -------------------------------------------------

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
                            "ASTRA-Motion returned zero outputs"
                        )

                        return null
                    }

                    // -------------------------------------------------
                    // RAW OUTPUTS BY NAME
                    // -------------------------------------------------

                    val rawOutputsByName =
                        linkedMapOf<String, FloatArray>()

                    outputs.forEach { output ->

                        val name =
                            output.key

                        val onnxValue =
                            output.value

                        val values =
                            OnnxModelUtils.flattenOutput(
                                onnxValue
                            )

                        rawOutputsByName[name] =
                            values

                        Log.d(
                            TAG,
                            "OUTPUT name=$name " +
                                    "size=${values.size} " +
                                    "values=${
                                        values
                                            .take(20)
                                            .joinToString(", ")
                                    }"
                        )
                    }

                    // -------------------------------------------------
                    // PRESERVE OUTPUT ORDER
                    // -------------------------------------------------

                    val rawOutputs =
                        activeSession
                            .outputNames
                            .mapNotNull { name ->
                                rawOutputsByName[name]
                            }

                    // -------------------------------------------------
                    // VALIDATION
                    // -------------------------------------------------

                    val valid =
                        rawOutputs.isNotEmpty() &&
                                rawOutputs.all { values ->

                                    values.isNotEmpty() &&
                                            values.all {
                                                it.isFinite()
                                            }
                                }

                    // -------------------------------------------------
                    // DECODE DISPLACEMENT
                    // -------------------------------------------------

                    val displacementScaled =
                        rawOutputsByName[
                            OUTPUT_DISPLACEMENT
                        ]?.firstOrNull()

                    val displacementM =
                        displacementScaled
                            ?.takeIf {
                                it.isFinite()
                            }
                            ?.toDouble()
                            ?.let {
                                activeDispScaler.inverse(
                                    it
                                )
                            }

                    // -------------------------------------------------
                    // DECODE ORIENTATION
                    // -------------------------------------------------

                    val orientationScaled =
                        rawOutputsByName[
                            OUTPUT_ORIENTATION
                        ]?.firstOrNull()

                    val orientationChangeRad =
                        orientationScaled
                            ?.takeIf {
                                it.isFinite()
                            }
                            ?.toDouble()
                            ?.let {
                                activeOriScaler.inverse(
                                    it
                                )
                            }

                    // -------------------------------------------------
                    // DECODE ZUPT LOGIT
                    // -------------------------------------------------

                    val zuptScore =
                        rawOutputsByName[
                            OUTPUT_ZUPT
                        ]?.firstOrNull()
                            ?.takeIf {
                                it.isFinite()
                            }
                            ?.toDouble()

                    // -------------------------------------------------
                    // ROUTER WEIGHTS
                    //
                    // Not decoded into AstraMotionOutput yet.
                    // They remain available through rawOutputs.
                    // -------------------------------------------------

                    val routerWeights =
                        rawOutputsByName[
                            OUTPUT_ROUTER
                        ]

                    // -------------------------------------------------
                    // LATENCY
                    // -------------------------------------------------

                    val latency =
                        (
                                System.nanoTime() -
                                        start
                                ) / 1_000_000L

                    // -------------------------------------------------
                    // DETAILED DEBUG LOG
                    // -------------------------------------------------

                    Log.i(
                        TAG,
                        String.format(
                            Locale.US,
                            "ASTRA-Motion decoded " +
                                    "valid=%s " +
                                    "disp=%.6f m " +
                                    "ori=%.6f rad " +
                                    "zuptLogit=%.6f " +
                                    "router=%s " +
                                    "latency=%dms",
                            valid,
                            displacementM ?: Double.NaN,
                            orientationChangeRad
                                ?: Double.NaN,
                            zuptScore ?: Double.NaN,
                            routerWeights
                                ?.joinToString(
                                    prefix = "[",
                                    postfix = "]"
                                )
                                ?: "null",
                            latency
                        )
                    )

                    // -------------------------------------------------
                    // FINAL OUTPUT
                    // -------------------------------------------------

                    return AstraMotionOutput(

                        displacementM =
                            displacementM,

                        orientationChangeRad =
                            orientationChangeRad,

                        zuptScore =
                            zuptScore,

                        rawOutputs =
                            rawOutputs,

                        valid =
                            valid &&
                                    displacementM != null &&
                                    orientationChangeRad != null,

                        inferenceMs =
                            latency
                    )
                }
            }

        } catch (e: Exception) {

            val latency =
                (
                        System.nanoTime() -
                                start
                        ) / 1_000_000L

            Log.e(
                TAG,
                "ASTRA-Motion inference failed " +
                        "after ${latency}ms",
                e
            )

            return null
        }
    }

    /**
     * Resolve actual ONNX tensor shape.
     *
     * Supports:
     *
     *      [1, 10, 6]
     *
     * and:
     *
     *      [10, 6]
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
                    "ASTRA-Motion has no input metadata"
                )

        val tensorInfo =
            nodeInfo.info as? TensorInfo
                ?: throw IllegalStateException(
                    "ASTRA-Motion input is not a tensor"
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
                    actual = shape[1],
                    expected =
                        WINDOW_SIZE.toLong(),
                    label = "timesteps"
                )

                validateDimension(
                    actual = shape[2],
                    expected =
                        FEATURE_COUNT.toLong(),
                    label = "features"
                )

                longArrayOf(
                    1L,
                    WINDOW_SIZE.toLong(),
                    FEATURE_COUNT.toLong()
                )
            }

            2 -> {

                validateDimension(
                    actual = shape[0],
                    expected =
                        WINDOW_SIZE.toLong(),
                    label = "timesteps"
                )

                validateDimension(
                    actual = shape[1],
                    expected =
                        FEATURE_COUNT.toLong(),
                    label = "features"
                )

                longArrayOf(
                    WINDOW_SIZE.toLong(),
                    FEATURE_COUNT.toLong()
                )
            }

            else -> {

                throw IllegalStateException(
                    "Unexpected ASTRA-Motion input rank=" +
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

        /*
         * -1 means dynamic dimension.
         */
        if (
            actual == -1L
        ) {
            return
        }

        require(
            actual == expected
        ) {

            "Unexpected ASTRA-Motion $label. " +
                    "Expected=$expected " +
                    "actual=$actual"
        }
    }

    /**
     * Logs exact ONNX input/output metadata.
     */
    private fun logModelMetadata(
        activeSession: OrtSession
    ) {

        Log.i(
            TAG,
            "=========================================="
        )

        Log.i(
            TAG,
            "ASTRA-MOTION ONNX MODEL"
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
                        } " +
                        "info=$info"
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
                        } " +
                        "info=$info"
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
     * MinMaxScaler exported from Python.
     *
     * scaled =
     *      raw * scale + min_offset
     */
    data class InputScaler(

        val data_min:
        List<List<Double>> =
            emptyList(),

        val data_max:
        List<List<Double>> =
            emptyList(),

        val scale:
        List<List<Double>> =
            emptyList(),

        val min_offset:
        List<List<Double>> =
            emptyList()
    ) {

        fun transform(
            row: FloatArray
        ): FloatArray {

            require(
                row.size ==
                        FEATURE_COUNT
            ) {
                "Expected $FEATURE_COUNT features"
            }

            require(
                scale.isNotEmpty()
            ) {
                "Input scaler scale is empty"
            }

            require(
                min_offset.isNotEmpty()
            ) {
                "Input scaler offset is empty"
            }

            require(
                scale[0].size ==
                        FEATURE_COUNT
            ) {
                "Input scaler has " +
                        "${scale[0].size} features"
            }

            return FloatArray(
                FEATURE_COUNT
            ) { i ->

                val scaled =
                    row[i].toDouble() *
                            scale[0][i] +
                            min_offset[0][i]

                scaled
                    .coerceIn(
                        0.0,
                        1.0
                    )
                    .toFloat()
            }
        }
    }

    /**
     * Output scaler.
     *
     * Inverse:
     *
     * raw =
     *      ((scaled - min_offset) / scale)
     *      + data_min
     */
    data class OutputScaler(

        val data_min:
        Double = 0.0,

        val data_max:
        Double = 1.0,

        val scale:
        Double = 1.0,

        val min_offset:
        Double = 0.0
    ) {

        fun inverse(
            scaled: Double
        ): Double {

            require(
                scale.isFinite() &&
                        scale != 0.0
            ) {
                "Invalid output scaler"
            }

            return (
                    (
                            scaled -
                                    min_offset
                            ) / scale
                    ) + data_min
        }
    }
}