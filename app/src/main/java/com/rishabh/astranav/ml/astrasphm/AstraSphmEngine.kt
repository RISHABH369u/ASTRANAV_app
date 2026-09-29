package com.rishabh.astranav.ml.astrasphm

import android.content.Context
import android.util.Log
import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OnnxValue
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import com.rishabh.astranav.sensor.ImuSample
import kotlin.math.abs


class AstraSphmEngine(
    private val context: Context
) {

    companion object {
        private const val TAG = "ASTRA_SPHM"
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

            Log.d(
                TAG,
                "Model input names=$inputNames"
            )

            Log.d(
                TAG,
                "Model output names=${session?.outputInfo?.keys?.toList()}"
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


        featureBuilder.addFeatures(features)


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


            // ====================================================
            // NORMALIZATION
            // ====================================================

            val normalizedWindow =
                preprocessor.normalizeImu(
                    rawWindow
                )


            val speedNormalized =
                preprocessor.normalizeInitialSpeed(
                    initialSpeedMps
                )


            // ====================================================
            // IMU INPUT
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
                                speedNormalized
                            )
                        )


                    tensors[speedInputName] =
                        speedTensor


                    Log.d(
                        TAG,
                        "Initial speed input=$speedInputName normalized=$speedNormalized"
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
            // VERIFIED OUTPUT NAMES
            // ====================================================

            val outputNames =
                currentSession.outputInfo.keys.toList()


            Log.d(
                TAG,
                "Model output names=$outputNames"
            )

            Log.d(
                TAG,
                "Model output count=${outputNames.size}"
            )


            // ====================================================
            // FIND OUTPUTS
            // ====================================================

            val speedOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_SPEED
                )


            val speedVarianceOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_SPEED_LOG_VARIANCE
                )


            val positionOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_POSITION
                )


            val positionVarianceOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_POSITION_LOG_VARIANCE
                )


            val headingOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_HEADING_DELTA
                )


            val headingVarianceOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
                        AstraSphmModelMetadata.OUTPUT_HEADING_DELTA_LOG_VARIANCE
                )


            val motionOutput =
                findOutput(
                    result = result,
                    outputNames = outputNames,
                    semanticName =
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
            // LOG RAW OUTPUTS
            // ====================================================

            Log.d(
                TAG,
                "speedValues=${speedValues.contentToString()}"
            )

            Log.d(
                TAG,
                "speedLogVarianceValues=${speedVarianceValues.contentToString()}"
            )

            Log.d(
                TAG,
                "positionValues=${positionValues.contentToString()}"
            )

            Log.d(
                TAG,
                "positionLogVarianceValues=${positionVarianceValues.contentToString()}"
            )

            Log.d(
                TAG,
                "headingValues=${headingValues.contentToString()}"
            )

            Log.d(
                TAG,
                "headingLogVarianceValues=${headingVarianceValues.contentToString()}"
            )

            Log.d(
                TAG,
                "motionValues=${motionValues.contentToString()}"
            )


            // ====================================================
            // SPEED
            // ====================================================

            val speedMps =
                if (speedValues.isNotEmpty()) {

                    preprocessor.denormalizeSpeed(
                        speedValues[0].toDouble()
                    )

                } else {
                    null
                }


            val speedKmh =
                speedMps?.times(3.6)


            val speedLogVariance =
                speedVarianceValues
                    .firstOrNull()
                    ?.toDouble()


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

            val positionY =
                position?.second


            val positionLogVarianceX =
                positionVarianceValues
                    .getOrNull(0)
                    ?.toDouble()


            val positionLogVarianceY =
                positionVarianceValues
                    .getOrNull(1)
                    ?.toDouble()


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


            val headingDeltaDeg =
                headingDeltaRad?.times(
                    180.0 / Math.PI
                )


            val headingDeltaLogVariance =
                headingVarianceValues
                    .firstOrNull()
                    ?.toDouble()


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
                    headingDeltaRad = headingDeltaRad
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
                    headingDeltaRad,

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


            Log.d(
                TAG,
                "INPUT names=${currentSession.inputInfo.keys}"
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
    // FIND OUTPUT BY ACTUAL ONNX NAME
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
            } catch (_: Exception) {
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

        if (
            speedMps != null &&
            (
                    !speedMps.isFinite() ||
                            speedMps < 0.0 ||
                            speedMps > 80.0
                    )
        ) {
            return false
        }


        // --------------------------------------------------------
        // Position X
        // --------------------------------------------------------

        if (
            positionX != null &&
            !positionX.isFinite()
        ) {
            return false
        }


        // --------------------------------------------------------
        // Position Y
        // --------------------------------------------------------

        if (
            positionY != null &&
            !positionY.isFinite()
        ) {
            return false
        }


        // --------------------------------------------------------
        // Heading
        // --------------------------------------------------------

        if (
            headingDeltaRad != null &&
            (
                    !headingDeltaRad.isFinite() ||
                            abs(headingDeltaRad) >
                            Math.PI * 4.0
                    )
        ) {
            return false
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

    fun close() {

        try {
            session?.close()
        } catch (_: Exception) {
        }


        session = null

        initialized = false

        inputName = null

        featureBuilder.clear()
    }
}