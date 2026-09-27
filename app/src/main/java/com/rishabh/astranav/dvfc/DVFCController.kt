package com.rishabh.astranav.dvfc

import android.content.Context
import com.rishabh.astranav.dvfc.math.DeviceVehicleTransform
import com.rishabh.astranav.dvfc.math.Quat
import com.rishabh.astranav.dvfc.sensor.SensorFusion
import com.rishabh.astranav.dvfc.sensor.StabilityDetector
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

data class DvfcUiState(
    val status: CalibrationStatus = CalibrationStatus.STABILIZING,
    val rollDeg: Float = 0f,
    val pitchDeg: Float = 0f,
    val yawDeg: Float = 0f,
    val headingOffsetDeg: Float = 0f,
    val currentQuaternion: Quat = Quat.IDENTITY,
    val sensorsAvailable: Boolean = true,
)

/**
 * Orchestrates spec STEP 1–9. Holds no rendering/UI code — DVFCActivity only
 * reads [state] and forwards `currentQuaternion` to DvfcSceneRenderer, and
 * reads [lockedDeviceToVehicleTransform] once COMPLETE to hand off to the
 * navigation pipeline (ESKF). No fake velocity or nav output is computed
 * here, per spec §9.
 */
class DVFCController(context: Context) {

    private val stability = StabilityDetector()
    private val transform = DeviceVehicleTransform()

    // TODO(spec §5): wire to GNSS course-over-ground while briefly moving in
    // a straight line, when available — that's a stronger heading reference
    // than the device's own fused (magnetometer-backed) heading alone. Until
    // then this falls back to treating the device's heading at capture time
    // as the vehicle-forward reference, which only holds if the phone is
    // known to be roughly forward-facing when calibration starts.
    private var vehicleHeadingDeg = 0f

    private var validatingSinceNs = 0L

    private val _state = MutableStateFlow(DvfcUiState())
    val state: StateFlow<DvfcUiState> = _state

    private val fusion = SensorFusion(context) { sample ->
        onSample(sample.quaternion, sample.angularVelocity, sample.timestampNs)
    }

    fun start() {
        _state.value = _state.value.copy(sensorsAvailable = fusion.isAvailable)
        fusion.start()
    }

    fun stop() = fusion.stop()

    private fun onSample(q: Quat, angularVelocity: FloatArray, timestampNs: Long) {
        val euler = q.toEulerDegrees()
        val headingOffset = transform.headingOffsetDegrees(q, vehicleHeadingDeg)
        val current = _state.value

        val nextStatus = when (current.status) {
            CalibrationStatus.STABILIZING -> {
                // STEP 3: hold still.
                if (stability.update(angularVelocity)) CalibrationStatus.ALIGNING else CalibrationStatus.STABILIZING
            }
            CalibrationStatus.ALIGNING -> {
                // STEP 4–6: estimate orientation, reference vehicle heading, capture the transform.
                vehicleHeadingDeg = euler[2] // capture-time device heading as the provisional vehicle-forward reference — see TODO above
                transform.calibrate(q, vehicleHeadingDeg)
                validatingSinceNs = timestampNs
                CalibrationStatus.VALIDATING
            }
            CalibrationStatus.VALIDATING -> {
                // STEP 7: hold the captured transform for a short confirmation window.
                val stillStable = stability.update(angularVelocity)
                val elapsedMs = (timestampNs - validatingSinceNs) / 1_000_000
                when {
                    !stillStable -> {
                        transform.reset()
                        CalibrationStatus.STABILIZING
                    }
                    elapsedMs > VALIDATION_HOLD_MS -> CalibrationStatus.COMPLETE // STEP 8: lock.
                    else -> CalibrationStatus.VALIDATING
                }
            }
            CalibrationStatus.COMPLETE -> CalibrationStatus.COMPLETE
        }

        _state.value = current.copy(
            status = nextStatus,
            rollDeg = euler[0],
            pitchDeg = euler[1],
            yawDeg = euler[2],
            headingOffsetDeg = headingOffset,
            currentQuaternion = q,
        )
    }

    /** STEP 9 hand-off point — read this once COMPLETE and pass it to the nav/ESKF pipeline. Null until locked. */
    fun lockedDeviceToVehicleTransform(): Quat? = transform.lockedTransform

    fun recalibrate() {
        transform.reset()
        stability.reset()
        _state.value = _state.value.copy(status = CalibrationStatus.STABILIZING)
    }

    companion object {
        private const val VALIDATION_HOLD_MS = 2000L
    }
}
