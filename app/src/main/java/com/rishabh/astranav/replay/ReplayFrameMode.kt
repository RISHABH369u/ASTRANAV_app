package com.rishabh.astranav.replay

/**
 * Defines how an IO-VNBD replay sample is interpreted
 * before entering the ASTRANAV navigation core.
 *
 * LIVE navigation uses DVFC externally.
 *
 * IO-VNBD replay must NOT use the calibration of the
 * physical phone currently running the replay because
 * the recorded IMU data came from a different device/setup.
 */
enum class ReplayFrameMode {

    /**
     * Use the recorded IO-VNBD sensor frame directly.
     *
     * No current-device DVFC calibration is applied.
     */
    DATASET_FRAME,

    /**
     * Use the normal Device → Vehicle calibration path.
     *
     * This is available for controlled replay experiments,
     * but must NOT be the default IO-VNBD mode.
     */
    DVFC
}