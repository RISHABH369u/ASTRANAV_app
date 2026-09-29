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
 * V7 / Supreme motion model.
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
 * IMPORTANT:
 * Output semantics are intentionally NOT guessed.
 * Raw ONNX tensors are preserved until the actual
 * exported ONNX output contract is confirmed.
 */
class AstraMotionEngine(context: Context) {

    companion object {

        private const val TAG = "ASTRA_MOTION"

        private const val WINDOW_SIZE = 10
        private const val FEATURE_COUNT = 6
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

        if (
            displacementScaler == null ||
            orientationScaler == null
        ) {
            return null
        }

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
        // SCALE
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

        if (
            window.size < WINDOW_SIZE
        ) {
            return null
        }

        return runInference(
            activeSession
        )
    }

    /**
     * Execute ONNX inference.
     *
     * Semantic output decoding is deliberately disabled
     * until the actual ONNX output names/shapes are known.
     */
    private fun runInference(
        activeSession: OrtSession
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
                    // RAW OUTPUTS
                    // -------------------------------------------------

                    val rawOutputs =
                        outputs.map { output ->

                            OnnxModelUtils
                                .flattenOutput(
                                    output.value
                                )
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
                    // LOG EACH OUTPUT
                    // -------------------------------------------------

                    rawOutputs.forEachIndexed {
                            outputIndex,
                            values ->

                        val outputName =
                            activeSession
                                .outputNames
                                .elementAtOrNull(
                                    outputIndex
                                )
                                ?: "unknown"

                        Log.d(
                            TAG,
                            String.format(
                                Locale.US,
                                "OUTPUT[%d] " +
                                        "name=%s " +
                                        "size=%d " +
                                        "values=%s",
                                outputIndex,
                                outputName,
                                values.size,
                                values
                                    .take(12)
                                    .joinToString()
                            )
                        )
                    }

                    val latency =
                        (
                                System.nanoTime() -
                                        start
                                ) / 1_000_000L

                    Log.i(
                        TAG,
                        String.format(
                            Locale.US,
                            "INFERENCE valid=%s " +
                                    "outputs=%d " +
                                    "latency=%dms",
                            valid,
                            rawOutputs.size,
                            latency
                        )
                    )

                    /*
                     * IMPORTANT:
                     *
                     * We intentionally do NOT do:
                     *
                     * output[0] = displacement
                     * output[1] = orientation
                     *
                     * yet.
                     */

                    return AstraMotionOutput(

                        displacementM =
                            null,

                        orientationChangeRad =
                            null,

                        zuptScore =
                            null,

                        rawOutputs =
                            rawOutputs,

                        valid =
                            valid,

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
     * The important part here is:
     *
     *      ValueInfo -> TensorInfo -> shape
     *
     * ONNX Runtime exposes generic ValueInfo,
     * therefore we explicitly cast it to TensorInfo.
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
     * Kept for semantic decoding later.
     */
    data class OutputScaler(

        val data_min: Double = 0.0,

        val data_max: Double = 1.0,

        val scale: Double = 1.0,

        val min_offset: Double = 0.0
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