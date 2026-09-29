//package com.rishabh.astranav.ml.astramotion
//
//data class AstraMotionOutput(
//    val displacementM: Double?,
//    val orientationChangeRad: Double?,
//    val zuptScore: Double?,
//    val rawOutputs: List<FloatArray>,
//    val valid: Boolean,
//    val inferenceMs: Long
//)

package com.rishabh.astranav.ml.astramotion

data class AstraMotionOutput(
    val displacementM: Double?,
    val orientationChangeRad: Double?,
    val zuptScore: Double?,
    val rawOutputs: List<FloatArray>,
    val valid: Boolean,
    val inferenceMs: Long
)

