package com.rishabh.astranav.navigation

import com.rishabh.astranav.constraints.NonHolonomicConstraint
import com.rishabh.astranav.constraints.ZuptConstraint
import com.rishabh.astranav.gnss.GnssQualityMonitor
import com.rishabh.astranav.integrity.IntegrityMonitor
import com.rishabh.astranav.ml.MlMeasurement
import com.rishabh.astranav.sensor.GnssSample
import com.rishabh.astranav.sensor.ImuSample
import kotlin.math.atan2
import kotlin.math.sqrt

class NavigationEngine(
    private val mechanization: ImuMechanization = ImuMechanization(),
    private val ekf: ErrorStateEkf = ErrorStateEkf()
) {
    private val modes = NavigationModeManager()
    private val gnssQuality = GnssQualityMonitor()
    private val integrity = IntegrityMonitor()
    private val nhc = NonHolonomicConstraint()
    private val zupt = ZuptConstraint()
    private var lastState = NavigationState()
    private var stationarySamples = 0

    fun reset(yawRad: Double = 0.0) {
        mechanization.initialize(yawRad)
        lastState = NavigationState()
        stationarySamples = 0
    }

    fun update(imu: ImuSample, dtSeconds: Double, gnss: GnssSample?, ml: MlMeasurement?): NavigationState {
        mechanization.propagate(imu, dtSeconds)
        ekf.predict(dtSeconds)

        val speed = sqrt(mechanization.state.ve*mechanization.state.ve + mechanization.state.vn*mechanization.state.vn)
        val gyroMag = sqrt(imu.gyroX*imu.gyroX + imu.gyroY*imu.gyroY + imu.gyroZ*imu.gyroZ)
        val accelMag = sqrt(imu.accelX*imu.accelX + imu.accelY*imu.accelY + imu.accelZ*imu.accelZ)
        val stationary = speed < 0.35 && gyroMag < 0.10 && kotlin.math.abs(accelMag-9.81) < 0.8
        if(stationary) stationarySamples++ else stationarySamples=0

        val q = gnss?.let { gnssQuality.assess(it.accuracyM) }
        val mode = modes.update(gnss?.timestampMillis ?: System.currentTimeMillis(), q?.available == true, q?.accuracyM ?: 999.0)

        val mlAccepted = ml?.valid == true && ml.speedMps.isFinite()
        if(mlAccepted) ekf.correctSpeed(ml!!.speedMps, speed, ml.variance)
        if(stationarySamples >= 5 && zupt.active(speed, gyroMag, 0.02)) {
            mechanization.state.ve *= 0.15
            mechanization.state.vn *= 0.15
        }

        val gnssScore = q?.score ?: 0.0
        val mlScore = ml?.confidence?.coerceIn(0.0,1.0) ?: 0.0
        val physics = if(stationary || gyroMag < 1.0) 1.0 else 0.75
        val score = integrity.evaluate(mlScore,physics,0.0,gnssScore)

        val heading = Math.toDegrees(atan2(mechanization.state.ve, mechanization.state.vn)).let { (it+360.0)%360.0 }
        lastState = NavigationState(
            timestampMillis = gnss?.timestampMillis ?: System.currentTimeMillis(),
            mode = mode,
            eastMeters = mechanization.state.east,
            northMeters = mechanization.state.north,
            speedMps = if(mlAccepted) 0.5*speed+0.5*ml!!.speedMps else speed,
            headingDegrees = heading,
            positionSigmaMeters = ekf.positionSigma(),
            headingSigmaDegrees = 8.0,
            driftMeters = ekf.positionSigma(),
            confidencePercent = score.scorePercent,
            stationary = stationary,
            mlAvailable = mlAccepted,
            mapMatched = false
        )
        return lastState
    }

    fun state(): NavigationState = lastState
}
