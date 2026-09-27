package com.rishabh.astranav.dvfc.math

/**
 * Computes and locks R_device_to_vehicle — the rotation that maps a vector
 * expressed in the device (phone) frame into the vehicle frame. This is the
 * actual deliverable of the DVFC screen (spec §9): it does NOT compute
 * velocity or any nav output itself, only the transform, for the
 * ESKF/navigation pipeline to consume.
 *
 * Frames:
 *   qEarthToDevice   — device orientation relative to Earth (ENU: X=East,
 *                       Y=North, Z=Up — Android's TYPE_ROTATION_VECTOR
 *                       convention), from SensorFusion.
 *   qEarthToVehicle  — vehicle orientation relative to Earth. We only track
 *                       vehicle *heading* (yaw about Z/Up) here; pitch/roll
 *                       of the vehicle body is assumed level at calibration
 *                       time (matches how the phone is mounted while
 *                       parked — spec's calibration flow keeps the vehicle
 *                       stationary during capture).
 *
 *   R_device_to_vehicle = qEarthToVehicle⁻¹ ∘ qEarthToDevice     (spec §2, §5)
 */
class DeviceVehicleTransform {

    var lockedTransform: Quat? = null
        private set

    /**
     * Captures the transform (spec STEP 6) from the current fused device
     * orientation and a vehicle heading reference.
     *
     * @param vehicleHeadingDeg the vehicle's forward heading in the Earth
     *   frame, degrees. Per spec §5, don't treat magnetometer heading as
     *   perfect ground truth by itself — callers should prefer a stronger
     *   reference (e.g. GNSS course-over-ground captured while briefly
     *   moving in a straight line) and fall back to the device's own fused
     *   heading only when nothing better is available. This class just
     *   consumes whatever final number the caller decides on.
     */
    fun calibrate(qEarthToDevice: Quat, vehicleHeadingDeg: Float): Quat {
        val qEarthToVehicle = Quat.fromAxisAngleDegrees(0f, 0f, 1f, vehicleHeadingDeg)
        val result = (qEarthToVehicle.inverse() * qEarthToDevice).normalized()
        lockedTransform = result
        return result
    }

    fun reset() {
        lockedTransform = null
    }

    /** a_vehicle = R_device_to_vehicle × a_device (spec §8). Null until calibrated. */
    fun transformAcceleration(aDevice: FloatArray): FloatArray? =
        lockedTransform?.rotate(aDevice)

    /** Live relative yaw between the phone's current heading and the vehicle's, degrees, wrapped to [-180, 180]. */
    fun headingOffsetDegrees(qEarthToDevice: Quat, vehicleHeadingDeg: Float): Float {
        val deviceYaw = qEarthToDevice.toEulerDegrees()[2]
        var diff = deviceYaw - vehicleHeadingDeg
        while (diff > 180f) diff -= 360f
        while (diff < -180f) diff += 360f
        return diff
    }
}
