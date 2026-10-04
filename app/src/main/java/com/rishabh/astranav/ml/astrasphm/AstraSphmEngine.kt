package com.rishabh.astranav.ml.astrasphm

import android.content.Context
import android.util.Log
import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OnnxValue
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import com.rishabh.astranav.sensor.ImuSample
import kotlin.math.abs
import kotlin.math.exp

/**
 * Runtime engine for ASTRA-SPHM.
 *
// * ASTRA-SPHM is the renamed V8 state-conditioned dead-reckoning model.
 *
 * Responsibilities:
 *  - maintain the 20 x 6 IMU window
 *  - normalize model inputs
 *  - provide optional initial-speed state input
 *  - execute ONNX inference
 *  - decode all V8/ASTRA-SPHM output heads
 *  - denormalize model outputs
 *  - expose model uncertainty/log-variance
 *  - perform basic physical sanity validation
 *
 * This class does NOT:
 *  - modify ESKF state
 *  - perform ZUPT updates
 *  - perform map matching
 *  - override the navigation state
 *
 * Those responsibilities belong to the navigation/fusion layer.
 */
class AstraSphmEngine(
    private val context: Context
) : AutoCloseable {

    companion object {
        private const val TAG = "ASTRA_SPHM"

        private const val SAMPLE_RATE_HZ = 10L
        private const val MIN_SAMPLE_INTERVAL_NS =
            1_000_000_000L / SAMPLE_RATE_HZ

        private const val MAX_REASONABLE_SPEED_MPS = 80.0

        /*
         * Log-variance safety limits.
         *
         * sigma = exp(0.5 * logVariance)
         *
         * These limits prevent malformed model output from producing
         * Infinity/NaN and poisoning later fusion code.
         */
        private const val MIN_LOG_VARIANCE = -20.0
        private const val MAX_LOG_VARIANCE = 10.0
    }

    // ============================================================
    // ONNX RUNTIME
    // ============================================================

    private val environment =
        OrtEnvironment.getEnvironment()

    private var session: OrtSession? = null

    private var inputName: String? = null

    private var initialized = false

    // ============================================================
    // FEATURE PIPELINE
    // ============================================================

    private val featureBuilder =
        AstraSphmFeatureBuilder()

    private val preprocessor =
        AstraSphmPreprocessor()

    // ============================================================
    // TIMESTAMP PROTECTION
    // ============================================================

    /**
     * Last sample accepted by this model engine.
     *
     * SensorAdapter is still responsible for the actual 10 Hz
     * synchronization/resampling. This value only protects the model
     * from accidental duplicate or excessively fast calls.
     */
    private var lastAcceptedTimestampNs = 0L

    // ============================================================
    // INITIALIZATION
    // ============================================================

    fun initialize(): Result<Unit> {

        if (initialized) {
            return Result.success(Unit)
        }

        return try {

            Log.d(TAG, "Initializing ASTRA-SPHM...")

            // ----------------------------------------------------
            // Load ONNX model
            // ----------------------------------------------------

            val modelBytes =
                context.assets.open(
                    AstraSphmModelMetadata.MODEL_ASSET
                ).use {
                    it.readBytes()
                }

            require(modelBytes.isNotEmpty()) {
                "ASTRA-SPHM model asset is empty"
            }

            // ----------------------------------------------------
            // ORT session
            // ----------------------------------------------------

            val options =
                OrtSession.SessionOptions()

            session =
                environment.createSession(
                    modelBytes,
                    options
                )

            // ----------------------------------------------------
            // Input names
            // ----------------------------------------------------

            val inputNames =
                session
                    ?.inputNames
                    ?.toList()
                    ?: emptyList()

            require(inputNames.isNotEmpty()) {
                "ASTRA-SPHM ONNX model has no inputs"
            }

            Log.d(
                TAG,
                "Model input names=$inputNames"
            )

            val outputNames =
                session
                    ?.outputInfo
                    ?.keys
                    ?.toList()
                    ?: emptyList()

            Log.d(
                TAG,
                "Model output names=$outputNames"
            )

            inputName =
                inputNames.firstOrNull()

            require(
                !inputName.isNullOrBlank()
            ) {
                "ASTRA-SPHM ONNX input name not found"
            }

            initialized = true

            Log.i(
                TAG,
                "ASTRA-SPHM initialized successfully"
            )

            Result.success(Unit)

        } catch (e: Exception) {

            initialized = false
            session = null
            inputName = null

            Log.e(
                TAG,
                "ASTRA-SPHM initialization failed",
                e
            )

            Result.failure(e)
        }
    }

    // ============================================================
    // ADD IMU SAMPLE
    // ============================================================

    fun addSample(
        sample: ImuSample,
        initialSpeedMps: Double
    ): AstraSphmOutput {

        if (!initialized) {

            val result =
                initialize()

            if (result.isFailure) {

                return AstraSphmOutput.invalid(
                    result.exceptionOrNull()?.message
                        ?: "ASTRA-SPHM initialization failed"
                )
            }
        }

        // --------------------------------------------------------
        // Validate timestamp
        // --------------------------------------------------------

        val timestampNs =
            sample.timestampNanos

        if (timestampNs > 0L) {

            if (
                lastAcceptedTimestampNs != 0L &&
                timestampNs <= lastAcceptedTimestampNs
            ) {
                return AstraSphmOutput(
                    valid = false,
                    errorMessage =
                        "ASTRA-SPHM ignored duplicate/out-of-order sample"
                )
            }

            if (
                lastAcceptedTimestampNs != 0L &&
                timestampNs - lastAcceptedTimestampNs <
                MIN_SAMPLE_INTERVAL_NS
            ) {
                return AstraSphmOutput(
                    valid = false,
                    errorMessage =
                        "ASTRA-SPHM waiting for 10 Hz sample interval"
                )
            }

            lastAcceptedTimestampNs =
                timestampNs
        }

        // --------------------------------------------------------
        // Validate initial speed
        // --------------------------------------------------------

        if (!initialSpeedMps.isFinite()) {

            return AstraSphmOutput.invalid(
                "ASTRA-SPHM received non-finite initial speed"
            )
        }

        // --------------------------------------------------------
        // Add sample
        // --------------------------------------------------------

        featureBuilder.addSample(sample)

        // --------------------------------------------------------
        // Wait for 20 samples
        // --------------------------------------------------------

        if (!featureBuilder.isReady()) {

            return AstraSphmOutput(
                valid = false,
                errorMessage =
                    "Waiting for ASTRA-SPHM window: " +
                            "${featureBuilder.size()}/" +
                            AstraSphmModelMetadata.WINDOW_SIZE
            )
        }

        return infer(
            initialSpeedMps = initialSpeedMps
        )
    }

    // ============================================================
    // ADD RAW FEATURES
    // ============================================================

    fun addFeatures(
        features: DoubleArray,
        initialSpeedMps: Double
    ): AstraSphmOutput {

        if (!initialized) {

            val result =
                initialize()

            if (result.isFailure) {

                return AstraSphmOutput.invalid(
                    result.exceptionOrNull()?.message
                        ?: "ASTRA-SPHM initialization failed"
                )
            }
        }

        if (!initialSpeedMps.isFinite()) {

            return AstraSphmOutput.invalid(
                "ASTRA-SPHM received non-finite initial speed"
            )
        }

        // --------------------------------------------------------
        // Validate feature vector
        // --------------------------------------------------------

        if (
            features.size !=
            AstraSphmModelMetadata.FEATURE_COUNT
        ) {

            return AstraSphmOutput.invalid(
                "ASTRA-SPHM expected " +
                        "${AstraSphmModelMetadata.FEATURE_COUNT} features, " +
                        "received ${features.size}"
            )
        }

        if (features.any { !it.isFinite() }) {

            return AstraSphmOutput.invalid(
                "ASTRA-SPHM received non-finite feature values"
            )
        }

        // --------------------------------------------------------
        // Add raw features
        // --------------------------------------------------------

        featureBuilder.addFeatures(features)

        // --------------------------------------------------------
        // Wait for 20 samples
        // --------------------------------------------------------

        if (!featureBuilder.isReady()) {

            return AstraSphmOutput(
                valid = false,
                errorMessage =
                    "Waiting for ASTRA-SPHM window: " +
                            "${featureBuilder.size()}/" +
                            AstraSphmModelMetadata.WINDOW_SIZE
            )
        }

        return infer(
            initialSpeedMps = initialSpeedMps
        )
    }

    // ============================================================
    // INFERENCE
    // ============================================================

    private fun infer(
        initialSpeedMps: Double
    ): AstraSphmOutput {

        val started =
            System.nanoTime()

        val currentSession =
            session
                ?: return AstraSphmOutput.invalid(
                    "ASTRA-SPHM session is null"
                )

        val currentInputName =
            inputName
                ?: return AstraSphmOutput.invalid(
                    "ASTRA-SPHM input name is null"
                )

        var primaryTensor: OnnxTensor? = null
        var speedTensor: OnnxTensor? = null
        var result: OrtSession.Result? = null

        return try {

            // ====================================================
            // RAW WINDOW
            // ====================================================

            val rawWindow =
                featureBuilder.buildWindow()
                    ?: return AstraSphmOutput.invalid(
                        "ASTRA-SPHM window unavailable"
                    )

            if (
                rawWindow.size !=
                AstraSphmModelMetadata.WINDOW_SIZE
            ) {

                return AstraSphmOutput.invalid(
                    "ASTRA-SPHM invalid window size: " +
                            rawWindow.size
                )
            }

            // ====================================================
            // NORMALIZATION
            // ====================================================

            val normalizedWindow =
                preprocessor.normalizeImu(
                    rawWindow
                )

            // Validate normalized input.

            for (row in normalizedWindow) {

                if (
                    row.size !=
                    AstraSphmModelMetadata.FEATURE_COUNT
                ) {

                    return AstraSphmOutput.invalid(
                        "ASTRA-SPHM normalized row has invalid feature count"
                    )
                }

                if (row.any { !it.isFinite() }) {

                    return AstraSphmOutput.invalid(
                        "ASTRA-SPHM normalization produced non-finite values"
                    )
                }
            }

            val speedNormalized =
                preprocessor.normalizeInitialSpeed(
                    initialSpeedMps
                )

            if (!speedNormalized.isFinite()) {

                return AstraSphmOutput.invalid(
                    "ASTRA-SPHM speed normalization produced non-finite value"
                )
            }

            // ====================================================
            // MODEL INPUT
            //
            // Shape:
            // [1, 20, 6]
            // ====================================================

            val imuInput =
                Array(1) {

                    Array(
                        AstraSphmModelMetadata.WINDOW_SIZE
                    ) { row ->

                        normalizedWindow[row]
                    }
                }

            // ====================================================
            // MODEL INPUTS
            // ====================================================

            val inputNames =
                currentSession.inputNames.toList()

            val tensors =
                mutableMapOf<String, OnnxTensor>()

            primaryTensor =
                OnnxTensor.createTensor(
                    environment,
                    imuInput
                )

            tensors[currentInputName] =
                primaryTensor

            // ====================================================
            // OPTIONAL INITIAL SPEED INPUT
            // ====================================================

            if (inputNames.size >= 2) {

                val speedInputName =
                    inputNames.firstOrNull {
                        it != currentInputName
                    }

                if (speedInputName != null) {

                    speedTensor =
                        OnnxTensor.createTensor(
                            environment,
                            floatArrayOf(
                                speedNormalized.toFloat()
                            )
                        )

                    tensors[speedInputName] =
                        speedTensor

                    Log.d(
                        TAG,
                        "Initial speed input=$speedInputName " +
                                "normalized=$speedNormalized"
                    )
                }
            }

            // ====================================================
            // RUN MODEL
            // ====================================================

            result =
                currentSession.run(
                    tensors
                )

            // ====================================================
            // OUTPUT NAMES
            // ====================================================

            val outputNames =
                currentSession.outputInfo.keys.toList()

            Log.d(
                TAG,
                "Model output names=$outputNames"
            )

            // ====================================================
            // FIND OUTPUTS
            // ====================================================

            val speedOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_SPEED
                )

            val speedVarianceOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_SPEED_LOG_VARIANCE
                )

            val positionOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_POSITION
                )

            val positionVarianceOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_POSITION_LOG_VARIANCE
                )

            val headingOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_HEADING_DELTA
                )

            val headingVarianceOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_HEADING_DELTA_LOG_VARIANCE
                )

            val motionOutput =
                findOutput(
                    result,
                    outputNames,
                    AstraSphmModelMetadata.OUTPUT_MOTION_LOGITS
                )

            // ====================================================
            // FLATTEN OUTPUTS
            // ====================================================

            val speedValues =
                flattenOutput(speedOutput)

            val speedVarianceValues =
                flattenOutput(speedVarianceOutput)

            val positionValues =
                flattenOutput(positionOutput)

            val positionVarianceValues =
                flattenOutput(positionVarianceOutput)

            val headingValues =
                flattenOutput(headingOutput)

            val headingVarianceValues =
                flattenOutput(headingVarianceOutput)

            val motionValues =
                flattenOutput(motionOutput)

            // ====================================================
            // SPEED
            // ====================================================

            val speedMps =
                if (speedValues.isNotEmpty()) {

                    val rawSpeed =
                        preprocessor.denormalizeSpeed(
                            speedValues[0].toDouble()
                        )

                    if (rawSpeed.isFinite()) {
                        rawSpeed.coerceAtLeast(0.0)
                    } else {
                        null
                    }

                } else {
                    null
                }

            val speedKmh =
                speedMps?.times(3.6)

            val speedLogVariance =
                speedVarianceValues
                    .firstOrNull()
                    ?.toDouble()
                    ?.takeIf { it.isFinite() }

            // Convert uncertainty for diagnostics only.
            // The existing AstraSphmOutput contract remains unchanged.
            val speedSigmaMps =
                speedLogVariance?.let {
                    sigmaFromLogVariance(it)
                }

            // ====================================================
            // POSITION
            // ====================================================

            val position =
                if (positionValues.size >= 2) {

                    preprocessor.denormalizePosition(
                        positionValues[0].toDouble(),
                        positionValues[1].toDouble()
                    )

                } else {
                    null
                }

            val positionX =
                position?.first
                    ?.takeIf { it.isFinite() }

            val positionY =
                position?.second
                    ?.takeIf { it.isFinite() }

            val positionLogVarianceX =
                positionVarianceValues
                    .getOrNull(0)
                    ?.toDouble()
                    ?.takeIf { it.isFinite() }

            val positionLogVarianceY =
                positionVarianceValues
                    .getOrNull(1)
                    ?.toDouble()
                    ?.takeIf { it.isFinite() }

            val positionSigmaX =
                positionLogVarianceX?.let {
                    sigmaFromLogVariance(it)
                }

            val positionSigmaY =
                positionLogVarianceY?.let {
                    sigmaFromLogVariance(it)
                }

            // ====================================================
            // HEADING DELTA
            // ====================================================

            val headingDeltaRad =
                if (headingValues.isNotEmpty()) {

                    preprocessor.denormalizeHeadingDelta(
                        headingValues[0].toDouble()
                    )

                } else {
                    null
                }

            val safeHeadingDeltaRad =
                headingDeltaRad
                    ?.takeIf { it.isFinite() }

            val headingDeltaDeg =
                safeHeadingDeltaRad?.times(
                    180.0 / Math.PI
                )

            val headingDeltaLogVariance =
                headingVarianceValues
                    .firstOrNull()
                    ?.toDouble()
                    ?.takeIf { it.isFinite() }

            val headingSigmaRad =
                headingDeltaLogVariance?.let {
                    sigmaFromLogVariance(it)
                }

            // ====================================================
            // DIAGNOSTIC UNCERTAINTY LOG
            // ====================================================

            Log.d(
                TAG,
                "Prediction: " +
                        "speed=${speedMps?.let { "%.3f".format(it) } ?: "--"} m/s " +
                        "speedSigma=${speedSigmaMps?.let { "%.3f".format(it) } ?: "--"} " +
                        "position=(" +
                        "${positionX?.let { "%.3f".format(it) } ?: "--"}, " +
                        "${positionY?.let { "%.3f".format(it) } ?: "--"}) " +
                        "positionSigma=(" +
                        "${positionSigmaX?.let { "%.3f".format(it) } ?: "--"}, " +
                        "${positionSigmaY?.let { "%.3f".format(it) } ?: "--"}) " +
                        "headingDelta=" +
                        "${headingDeltaDeg?.let { "%.2f".format(it) } ?: "--"} deg " +
                        "headingSigma=" +
                        "${headingSigmaRad?.let { "%.4f".format(it) } ?: "--"} rad"
            )

            // ====================================================
            // LATENCY
            // ====================================================

            val latencyMs =
                (
                        System.nanoTime() -
                                started
                        ) / 1_000_000L

            // ====================================================
            // PHYSICAL VALIDATION
            // ====================================================

            val valid =
                isPhysicallyValid(
                    speedMps = speedMps,
                    positionX = positionX,
                    positionY = positionY,
                    headingDeltaRad = safeHeadingDeltaRad
                )

            if (!valid) {

                return AstraSphmOutput.invalid(
                    message =
                        "ASTRA-SPHM produced invalid physical output",
                    latencyMs = latencyMs
                )
            }

            // ====================================================
            // FINAL OUTPUT
            // ====================================================

            AstraSphmOutput(

                valid = true,

                speedMps = speedMps,

                speedKmh = speedKmh,

                speedLogVariance =
                    speedLogVariance,

                positionX =
                    positionX,

                positionY =
                    positionY,

                positionLogVarianceX =
                    positionLogVarianceX,

                positionLogVarianceY =
                    positionLogVarianceY,

                headingDeltaRad =
                    safeHeadingDeltaRad,

                headingDeltaDeg =
                    headingDeltaDeg,

                headingDeltaLogVariance =
                    headingDeltaLogVariance,

                motionLogits =
                    motionValues,

                latencyMs =
                    latencyMs
            )

        } catch (e: Exception) {

            val latencyMs =
                (
                        System.nanoTime() -
                                started
                        ) / 1_000_000L

            Log.e(
                TAG,
                "ASTRA-SPHM inference failed",
                e
            )

            AstraSphmOutput.invalid(
                message =
                    e.message
                        ?: "ASTRA-SPHM inference failed",

                latencyMs =
                    latencyMs
            )

        } finally {

            try {
                result?.close()
            } catch (_: Exception) {
            }

            try {
                primaryTensor?.close()
            } catch (_: Exception) {
            }

            try {
                speedTensor?.close()
            } catch (_: Exception) {
            }
        }
    }

    // ============================================================
    // FIND OUTPUT BY SEMANTIC NAME
    // ============================================================

    private fun findOutput(
        result: OrtSession.Result,
        outputNames: List<String>,
        semanticName: String
    ): OnnxValue? {

        val matchedIndex =
            outputNames.indexOfFirst { name ->

                name.equals(
                    semanticName,
                    ignoreCase = true
                ) ||

                        name.contains(
                            semanticName,
                            ignoreCase = true
                        )
            }

        if (matchedIndex >= 0) {

            return try {
                result[matchedIndex]
            } catch (e: Exception) {

                Log.w(
                    TAG,
                    "Unable to access output $semanticName",
                    e
                )

                null
            }
        }

        Log.w(
            TAG,
            "Output not found: $semanticName"
        )

        return null
    }

    // ============================================================
    // FLATTEN ONNX OUTPUT
    // ============================================================

    private fun flattenOutput(
        value: OnnxValue?
    ): FloatArray {

        if (value == null) {
            return FloatArray(0)
        }

        return try {

            if (value is OnnxTensor) {

                val buffer =
                    value.floatBuffer

                if (buffer != null) {

                    val duplicate =
                        buffer.duplicate()

                    val output =
                        FloatArray(
                            duplicate.remaining()
                        )

                    duplicate.get(output)

                    return output
                }

                return flattenJavaValue(
                    value.value
                )
            }

            flattenJavaValue(
                value.value
            )

        } catch (e: Exception) {

            Log.w(
                TAG,
                "Unable to flatten ONNX output",
                e
            )

            FloatArray(0)
        }
    }

    // ============================================================
    // FLATTEN JAVA ARRAYS
    // ============================================================

    private fun flattenJavaValue(
        value: Any?
    ): FloatArray {

        return when (value) {

            is FloatArray ->
                value.copyOf()

            is DoubleArray ->
                FloatArray(
                    value.size
                ) {
                    value[it].toFloat()
                }

            is Float ->
                floatArrayOf(value)

            is Double ->
                floatArrayOf(
                    value.toFloat()
                )

            is Int ->
                floatArrayOf(
                    value.toFloat()
                )

            is Long ->
                floatArrayOf(
                    value.toFloat()
                )

            is Array<*> -> {

                val output =
                    ArrayList<Float>()

                fun visit(item: Any?) {

                    when (item) {

                        is Float ->
                            output.add(item)

                        is Double ->
                            output.add(
                                item.toFloat()
                            )

                        is Int ->
                            output.add(
                                item.toFloat()
                            )

                        is Long ->
                            output.add(
                                item.toFloat()
                            )

                        is FloatArray ->
                            item.forEach {
                                output.add(it)
                            }

                        is DoubleArray ->
                            item.forEach {
                                output.add(
                                    it.toFloat()
                                )
                            }

                        is Array<*> ->
                            item.forEach(::visit)
                    }
                }

                value.forEach(::visit)

                output.toFloatArray()
            }

            else ->
                FloatArray(0)
        }
    }

    // ============================================================
    // LOG-VARIANCE -> STANDARD DEVIATION
    // ============================================================

    /**
     * Converts model log-variance into standard deviation.
     *
     * sigma = exp(0.5 * logVariance)
     *
     * This is used only for diagnostics in this engine.
     * It does NOT automatically become an ESKF covariance.
     */
    private fun sigmaFromLogVariance(
        logVariance: Double
    ): Double {

        if (!logVariance.isFinite()) {
            return 0.0
        }

        val safe =
            logVariance.coerceIn(
                MIN_LOG_VARIANCE,
                MAX_LOG_VARIANCE
            )

        val sigma =
            exp(0.5 * safe)

        return if (sigma.isFinite()) {
            sigma
        } else {
            0.0
        }
    }

    // ============================================================
    // PHYSICAL VALIDATION
    // ============================================================

    private fun isPhysicallyValid(
        speedMps: Double?,
        positionX: Double?,
        positionY: Double?,
        headingDeltaRad: Double?
    ): Boolean {

        // --------------------------------------------------------
        // Speed
        // --------------------------------------------------------

        if (speedMps != null) {

            if (
                !speedMps.isFinite() ||
                speedMps < 0.0 ||
                speedMps > MAX_REASONABLE_SPEED_MPS
            ) {
                return false
            }
        }

        // --------------------------------------------------------
        // Position
        // --------------------------------------------------------

        if (
            positionX != null &&
            !positionX.isFinite()
        ) {
            return false
        }

        if (
            positionY != null &&
            !positionY.isFinite()
        ) {
            return false
        }

        // --------------------------------------------------------
        // Heading
        // --------------------------------------------------------

        if (headingDeltaRad != null) {

            if (
                !headingDeltaRad.isFinite() ||
                abs(headingDeltaRad) >
                Math.PI * 4.0
            ) {
                return false
            }
        }

        // --------------------------------------------------------
        // At least one meaningful prediction
        // --------------------------------------------------------

        return (
                speedMps != null ||
                        positionX != null ||
                        positionY != null ||
                        headingDeltaRad != null
                )
    }

    // ============================================================
    // STATUS
    // ============================================================

    fun isReady(): Boolean {

        return initialized &&
                featureBuilder.isReady()
    }

    fun windowProgress(): Float {

        return featureBuilder.progress()
    }

    fun windowSize(): Int {

        return featureBuilder.size()
    }

    // ============================================================
    // CLEAR WINDOW
    // ============================================================

    fun clearWindow() {

        featureBuilder.clear()

        // Important: after clearing the model window, allow the
        // next sample to start a completely fresh sequence.
        lastAcceptedTimestampNs = 0L
    }

    // ============================================================
    // INPUT NAMES
    // ============================================================

    fun getInputNames(): List<String> {

        return session
            ?.inputInfo
            ?.keys
            ?.toList()
            ?: emptyList()
    }

    // ============================================================
    // OUTPUT NAMES
    // ============================================================

    fun getOutputNames(): List<String> {

        return session
            ?.outputInfo
            ?.keys
            ?.toList()
            ?: emptyList()
    }

    // ============================================================
    // CLOSE
    // ============================================================

    override fun close() {

        try {
            session?.close()
        } catch (_: Exception) {
        }

        session = null
        initialized = false
        inputName = null

        featureBuilder.clear()

        lastAcceptedTimestampNs = 0L
    }
}