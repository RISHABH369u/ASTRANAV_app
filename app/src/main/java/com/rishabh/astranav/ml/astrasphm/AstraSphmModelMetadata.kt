package com.rishabh.astranav.ml.astrasphm

object AstraSphmModelMetadata {

    // ============================================================
    // ASSETS
    // ============================================================

    const val MODEL_ASSET =
        "astra_sphm/astra_sphm.onnx"

    const val MANIFEST_ASSET =
        "astra_sphm/astra_sphm_manifest.json"

    const val NORMALIZATION_ASSET =
        "astra_sphm/astra_sphm_normalization.json"


    // ============================================================
    // MODEL INPUT
    // ============================================================

    const val SAMPLE_RATE_HZ = 10

    const val WINDOW_SIZE = 20

    const val FEATURE_COUNT = 6

    const val WINDOW_DURATION_SECONDS =
        WINDOW_SIZE.toDouble() / SAMPLE_RATE_HZ.toDouble()


    // ============================================================
    // INPUT FEATURES
    // ============================================================

    const val FEATURE_ACCEL_X = 0
    const val FEATURE_ACCEL_Y = 1
    const val FEATURE_ACCEL_Z = 2

    const val FEATURE_GYRO_X = 3
    const val FEATURE_GYRO_Y = 4
    const val FEATURE_GYRO_Z = 5


    // ============================================================
    // VERIFIED ONNX OUTPUT NAMES
    // ============================================================

    const val OUTPUT_SPEED =
        "speed"

    const val OUTPUT_SPEED_LOG_VARIANCE =
        "speed_log_variance"

    const val OUTPUT_POSITION =
        "position"

    const val OUTPUT_POSITION_LOG_VARIANCE =
        "position_log_variance"

    const val OUTPUT_HEADING_DELTA =
        "heading_delta"

    const val OUTPUT_HEADING_DELTA_LOG_VARIANCE =
        "heading_delta_log_variance"

    const val OUTPUT_MOTION_LOGITS =
        "motion_logits"
}