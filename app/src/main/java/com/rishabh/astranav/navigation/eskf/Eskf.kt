package com.rishabh.astranav.navigation.eskf

/**
 * ASTRA-Core Error-State Kalman Filter.
 *
 * Central owner of:
 *
 *  1. Nominal navigation state
 *  2. Error-state covariance
 *  3. IMU / INS prediction
 *  4. Covariance prediction
 *  5. Measurement corrections
 *
 * Current supported corrections:
 *
 *  - ZUPT
 *  - NHC
 *  - ZARU
 *
 * Future corrections:
 *
 *  - GNSS
 *  - ASTRA-Speed
 *  - ASTRA-Motion
 *  - ASTRA-SPHM
 *  - Map constraint
 *
 * Error-state ordering:
 *
 *  [δp, δv, δθ, δbg, δba]
 *
 *  0..2    position
 *  3..5    velocity
 *  6..8    attitude
 *  9..11   gyro bias
 *  12..14  accelerometer bias
 */
class Eskf(
    private val config: EskfConfig = EskfConfig()
) {

    companion object {
        const val STATE_SIZE = 15
    }

    /*
     * ------------------------------------------------------------------
     * Core state
     * ------------------------------------------------------------------
     */

    private val navigationState =
        NavigationState()

    private val covariance =
        EskfCovariance.fromConfig(config)

    private val prediction =
        EskfPrediction(config)

    private val covariancePrediction =
        EskfCovariancePrediction(config)

    private val zuptUpdate =
        EskfZuptUpdate(config)

    private val nhcUpdate =
        EskfNhcUpdate(
            lateralStdMps = 0.20,
            verticalStdMps = 0.30,
            nisGate = 9.21
        )

    private val zaruUpdate =
        EskfZaruUpdate(
            gyroStdRadPerSec = 0.05,
            nisGate = 11.34
        )

    private val gnssUpdate =
        EskfGnssUpdate(
            positionStdM = 5.0,
            velocityStdMps = 1.5,
            positionNisGate = 16.27,
            velocityNisGate = 16.27
        )


    private val learnedSpeedUpdate =
        EskfLearnedSpeedUpdate()

    /*
     * ------------------------------------------------------------------
     * Diagnostics counters
     * ------------------------------------------------------------------
     */

    private var acceptedPredictionCount = 0L
    private var rejectedPredictionCount = 0L

    private var acceptedZuptCount = 0L
    private var rejectedZuptCount = 0L

    private var acceptedNhcCount = 0L
    private var rejectedNhcCount = 0L

    private var acceptedZaruCount = 0L
    private var rejectedZaruCount = 0L

    private var acceptedGnssPositionCount = 0L
    private var rejectedGnssPositionCount = 0L

    private var acceptedGnssVelocityCount = 0L
    private var rejectedGnssVelocityCount = 0L

    private var acceptedLearnedSpeedCount = 0L
    private var rejectedLearnedSpeedCount = 0L

    private var lastPredictionResult:
            EskfPrediction.PredictionResult? = null

    private var lastZuptResult:
            EskfZuptUpdate.UpdateResult? = null

    private var lastNhcResult:
            EskfNhcUpdate.UpdateResult? = null

    private var lastZaruResult:
            EskfZaruUpdate.UpdateResult? = null

    private var lastGnssPositionResult:
            EskfGnssUpdate.PositionUpdateResult? = null

    private var lastGnssVelocityResult:
            EskfGnssUpdate.VelocityUpdateResult? = null

    private var lastLearnedSpeedResult:
            EskfLearnedSpeedUpdate.UpdateResult? = null

    /*
     * ------------------------------------------------------------------
     * Prediction output
     * ------------------------------------------------------------------
     */

    data class PredictionOutput(
        val accepted: Boolean,
        val deltaTimeSeconds: Double,
        val state: NavigationState,
        val covariance: Array<DoubleArray>,
        val specificForceBody: Vec3,
        val specificForceNavigation: Vec3,
        val gyroRateBody: Vec3,
        val accelerationNavigation: Vec3,
        val acceptedPredictionCount: Long,
        val rejectedPredictionCount: Long,
        val reason: String? = null
    )

    /*
     * ------------------------------------------------------------------
     * ZUPT output
     * ------------------------------------------------------------------
     */

    data class ZuptOutput(
        val accepted: Boolean,
        val innovation: Vec3,
        val innovationNormMps: Double,
        val nis: Double,
        val velocityCorrection: Vec3,
        val positionCorrection: Vec3,
        val attitudeCorrectionRad: Vec3,
        val gyroBiasCorrection: Vec3,
        val accelBiasCorrection: Vec3,
        val acceptedZuptCount: Long,
        val rejectedZuptCount: Long,
        val reason: String? = null
    )

    /*
     * ------------------------------------------------------------------
     * IMU prediction
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun predict(
        timestampNanos: Long,
        accelerationBody: Vec3,
        gyroBody: Vec3
    ): PredictionOutput {

        val result =
            prediction.predict(
                state = navigationState,
                timestampNanos = timestampNanos,
                accelerationBody = accelerationBody,
                gyroBody = gyroBody
            )

        if (!result.accepted) {

            rejectedPredictionCount++

            lastPredictionResult =
                result

            return buildPredictionOutput(
                result
            )
        }

        /*
         * First sample establishes the ESKF clock.
         */
        if (result.deltaTimeSeconds > 0.0) {

            covariancePrediction.propagate(
                covariance = covariance,
                state = navigationState,
                specificForceBody =
                    result.specificForceBody,
                gyroRateBody =
                    result.gyroRateBody,
                dtSeconds =
                    result.deltaTimeSeconds
            )
        }

        acceptedPredictionCount++

        lastPredictionResult =
            result

        return buildPredictionOutput(
            result
        )
    }

    /*
     * ------------------------------------------------------------------
     * NHC correction
     * ------------------------------------------------------------------
     *
     * Non-Holonomic Constraint:
     *
     *     body lateral velocity  ~= 0
     *     body vertical velocity ~= 0
     *
     * Forward velocity is NOT constrained.
     *
     * The caller decides whether NHC is currently eligible.
     */
    @Synchronized
    fun applyNhc(): EskfNhcUpdate.UpdateResult {

        val result =
            nhcUpdate.update(
                state = navigationState,
                covariance = covariance
            )

        lastNhcResult =
            result

        if (result.accepted) {
            acceptedNhcCount++
        } else {
            rejectedNhcCount++
        }

        return result
    }

    /*
     * ------------------------------------------------------------------
     * ZARU correction
     * ------------------------------------------------------------------
     *
     * Zero Angular Rate Update.
     *
     * The caller must establish that the vehicle is stationary
     * before invoking this method.
     *
     * The actual gyro measurement is passed into ZARU.
     *
     * Measurement:
     *
     *     gyro ~= gyroBias
     *
     * Main correction:
     *
     *     gyro bias
     */
    @Synchronized
    fun applyZaru(
        gyroBody: Vec3
    ): EskfZaruUpdate.UpdateResult {

        val result =
            zaruUpdate.update(
                state = navigationState,
                covariance = covariance,
                gyroMeasurement = gyroBody
            )

        lastZaruResult =
            result

        if (result.accepted) {
            acceptedZaruCount++
        } else {
            rejectedZaruCount++
        }

        return result
    }



    /*
 * ------------------------------------------------------------------
 * GNSS correction
 * ------------------------------------------------------------------
 *
 * GNSS is a measurement, NOT a direct state overwrite.
 *
 * Position:
 *   local NED metres
 *
 * Velocity:
 *   local NED m/s
 *
 * Latitude/longitude -> local NED conversion belongs outside ESKF.
 */

    data class GnssPositionOutput(
        val accepted: Boolean,
        val measurement: Vec3,
        val innovation: Vec3,
        val innovationNormM: Double,
        val nis: Double,
        val acceptedGnssPositionCount: Long,
        val rejectedGnssPositionCount: Long,
        val reason: String? = null
    )

    data class GnssVelocityOutput(
        val accepted: Boolean,
        val measurement: Vec3,
        val innovation: Vec3,
        val innovationNormMps: Double,
        val nis: Double,
        val acceptedGnssVelocityCount: Long,
        val rejectedGnssVelocityCount: Long,
        val reason: String? = null
    )

    /**
     * Applies a local-NED GNSS position measurement.
     *
     * The GNSS position must already be expressed in the same local
     * navigation frame as NavigationState.position.
     */
    @Synchronized
    fun applyGnssPosition(
        gnssPositionNed: Vec3,
        horizontalAccuracyM: Double? = null,
        verticalAccuracyM: Double? = null
    ): GnssPositionOutput {

        val result =
            gnssUpdate.updatePosition(
                state = navigationState,
                covariance = covariance,
                gnssPositionNed = gnssPositionNed,
                horizontalAccuracyM = horizontalAccuracyM,
                verticalAccuracyM = verticalAccuracyM
            )

        lastGnssPositionResult = result

        if (result.accepted) {
            acceptedGnssPositionCount++
        } else {
            rejectedGnssPositionCount++
        }

        return GnssPositionOutput(
            accepted = result.accepted,
            measurement = result.measurement,
            innovation = result.innovation,
            innovationNormM = result.innovationNormM,
            nis = result.nis,
            acceptedGnssPositionCount = acceptedGnssPositionCount,
            rejectedGnssPositionCount = rejectedGnssPositionCount,
            reason = result.reason ?: "unknown"
        )
    }

    /**
     * Applies a local-NED GNSS velocity measurement.
     *
     * GNSS velocity must be supplied in m/s.
     */
    @Synchronized
    fun applyGnssVelocity(
        gnssVelocityNed: Vec3,
        speedAccuracyMps: Double? = null
    ): GnssVelocityOutput {

        val result =
            gnssUpdate.updateVelocity(
                state = navigationState,
                covariance = covariance,
                gnssVelocityNed = gnssVelocityNed,
                speedAccuracyMps = speedAccuracyMps
            )

        lastGnssVelocityResult = result

        if (result.accepted) {
            acceptedGnssVelocityCount++
        } else {
            rejectedGnssVelocityCount++
        }

        return GnssVelocityOutput(
            accepted = result.accepted,
            measurement = result.measurement,
            innovation = result.innovation,
            innovationNormMps = result.innovationNormMps,
            nis = result.nis,
            acceptedGnssVelocityCount = acceptedGnssVelocityCount,
            rejectedGnssVelocityCount = rejectedGnssVelocityCount,
            reason = result.reason ?: "unknown"
        )
    }


    /*
 * ------------------------------------------------------------------
 * ASTRA-Speed learned speed correction
 * ------------------------------------------------------------------
 *
 * ASTRA-Speed provides a learned scalar vehicle speed measurement.
 *
 * IMPORTANT:
 *
 * The learned speed does NOT overwrite ESKF velocity.
 *
 * It is fused as an ordinary ESKF measurement:
 *
 *     z = learned speed
 *     h(x) = |v|
 *
 * This allows the ESKF to use covariance, innovation and NIS
 * gating instead of blindly trusting the neural network.
 */
    @Synchronized
    fun applyLearnedSpeed(
        learnedSpeedMps: Double,
        speedStdMps: Double? = null
    ): EskfLearnedSpeedUpdate.UpdateResult {

        val result =
            learnedSpeedUpdate.update(
                state = navigationState,
                covariance = covariance,
                learnedSpeedMps = learnedSpeedMps,
                speedStdMps = speedStdMps
            )

        lastLearnedSpeedResult =
            result

        if (result.accepted) {
            acceptedLearnedSpeedCount++
        } else {
            rejectedLearnedSpeedCount++
        }

        return result
    }

    /*
     * ------------------------------------------------------------------
     * Convenience prediction API
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun predict(
        timestampNanos: Long,
        accelX: Double,
        accelY: Double,
        accelZ: Double,
        gyroX: Double,
        gyroY: Double,
        gyroZ: Double
    ): PredictionOutput {

        return predict(
            timestampNanos = timestampNanos,

            accelerationBody = Vec3(
                x = accelX,
                y = accelY,
                z = accelZ
            ),

            gyroBody = Vec3(
                x = gyroX,
                y = gyroY,
                z = gyroZ
            )
        )
    }

    /*
     * ------------------------------------------------------------------
     * ZUPT correction
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun applyZupt(): ZuptOutput {

        val result =
            zuptUpdate.update(
                state = navigationState,
                covariance = covariance
            )

        lastZuptResult =
            result

        if (result.accepted) {
            acceptedZuptCount++
        } else {
            rejectedZuptCount++
        }

        return ZuptOutput(
            accepted =
                result.accepted,

            innovation =
                result.innovation,

            innovationNormMps =
                result.innovationNormMps,

            nis =
                result.nis,

            velocityCorrection =
                result.velocityCorrection,

            positionCorrection =
                result.positionCorrection,

            attitudeCorrectionRad =
                result.attitudeCorrectionRad,

            gyroBiasCorrection =
                result.gyroBiasCorrection,

            accelBiasCorrection =
                result.accelBiasCorrection,

            acceptedZuptCount =
                acceptedZuptCount,

            rejectedZuptCount =
                rejectedZuptCount,

            reason =
                result.reason
        )
    }

    /*
     * ------------------------------------------------------------------
     * State access
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getState(): NavigationState {
        return navigationState.copyState()
    }

    @Synchronized
    fun getCovariance(): Array<DoubleArray> {
        return covariance.copyMatrix()
    }

    @Synchronized
    fun getPosition(): Vec3 {
        return navigationState.position.copy()
    }

    @Synchronized
    fun getVelocity(): Vec3 {
        return navigationState.velocity.copy()
    }

    @Synchronized
    fun getAttitude(): Quaternion {
        return navigationState.attitude.copy()
    }

    @Synchronized
    fun getGyroBias(): Vec3 {
        return navigationState.gyroBias.copy()
    }

    @Synchronized
    fun getAccelBias(): Vec3 {
        return navigationState.accelBias.copy()
    }

    @Synchronized
    fun getTimestampNanos(): Long {
        return navigationState.timestampNanos
    }

    /*
     * ------------------------------------------------------------------
     * Prediction diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getAcceptedPredictionCount(): Long {
        return acceptedPredictionCount
    }

    @Synchronized
    fun getRejectedPredictionCount(): Long {
        return rejectedPredictionCount
    }

    @Synchronized
    fun getLastPredictionResult():
            EskfPrediction.PredictionResult? {

        return lastPredictionResult
    }

    /*
     * ------------------------------------------------------------------
     * ZUPT diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getAcceptedZuptCount(): Long {
        return acceptedZuptCount
    }

    @Synchronized
    fun getRejectedZuptCount(): Long {
        return rejectedZuptCount
    }

    @Synchronized
    fun getLastZuptResult():
            EskfZuptUpdate.UpdateResult? {

        return lastZuptResult
    }

    /*
     * ------------------------------------------------------------------
     * NHC diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getAcceptedNhcCount(): Long {
        return acceptedNhcCount
    }

    @Synchronized
    fun getRejectedNhcCount(): Long {
        return rejectedNhcCount
    }

    @Synchronized
    fun getLastNhcResult():
            EskfNhcUpdate.UpdateResult? {

        return lastNhcResult
    }

    /*
     * ------------------------------------------------------------------
     * ZARU diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getAcceptedZaruCount(): Long {
        return acceptedZaruCount
    }

    @Synchronized
    fun getRejectedZaruCount(): Long {
        return rejectedZaruCount
    }

    @Synchronized
    fun getLastZaruResult():
            EskfZaruUpdate.UpdateResult? {

        return lastZaruResult
    }

    /*
 * ------------------------------------------------------------------
 * GNSS diagnostics
 * ------------------------------------------------------------------
 */

    @Synchronized
    fun getAcceptedGnssPositionCount(): Long {
        return acceptedGnssPositionCount
    }

    @Synchronized
    fun getRejectedGnssPositionCount(): Long {
        return rejectedGnssPositionCount
    }

    @Synchronized
    fun getAcceptedGnssVelocityCount(): Long {
        return acceptedGnssVelocityCount
    }

    @Synchronized
    fun getRejectedGnssVelocityCount(): Long {
        return rejectedGnssVelocityCount
    }

    @Synchronized
    fun getLastGnssPositionResult():
            EskfGnssUpdate.PositionUpdateResult? {

        return lastGnssPositionResult
    }

    @Synchronized
    fun getLastGnssVelocityResult():
            EskfGnssUpdate.VelocityUpdateResult? {

        return lastGnssVelocityResult
    }

    /*
 * ------------------------------------------------------------------
 * ASTRA-Speed diagnostics
 * ------------------------------------------------------------------
 */

    @Synchronized
    fun getAcceptedLearnedSpeedCount(): Long {
        return acceptedLearnedSpeedCount
    }

    @Synchronized
    fun getRejectedLearnedSpeedCount(): Long {
        return rejectedLearnedSpeedCount
    }

    @Synchronized
    fun getLastLearnedSpeedResult():
            EskfLearnedSpeedUpdate.UpdateResult? {

        return lastLearnedSpeedResult
    }

    /*
     * ------------------------------------------------------------------
     * Numerical validity
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun isStateFinite(): Boolean {

        return navigationState.position.isFinite() &&
                navigationState.velocity.isFinite() &&
                navigationState.attitude.isFinite() &&
                navigationState.gyroBias.isFinite() &&
                navigationState.accelBias.isFinite()
    }

    @Synchronized
    fun isCovarianceValid(): Boolean {

        return covariance.isFinite() &&
                covariance.hasValidDiagonal()
    }

    @Synchronized
    fun covarianceDiagnostics(): String {
        return covariance.diagnosticSummary()
    }

    /*
     * ------------------------------------------------------------------
     * Complete diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun diagnostics(): EskfDiagnostics {

        return EskfDiagnostics(

            timestampNanos =
                navigationState.timestampNanos,

            position =
                navigationState.position.copy(),

            velocity =
                navigationState.velocity.copy(),

            speedMps =
                navigationState.velocity.norm(),

            attitude =
                navigationState.attitude.copy(),

            gyroBias =
                navigationState.gyroBias.copy(),

            accelBias =
                navigationState.accelBias.copy(),

            acceptedPredictionCount =
                acceptedPredictionCount,

            rejectedPredictionCount =
                rejectedPredictionCount,

            acceptedZuptCount =
                acceptedZuptCount,

            rejectedZuptCount =
                rejectedZuptCount,

            acceptedNhcCount =
                acceptedNhcCount,

            rejectedNhcCount =
                rejectedNhcCount,

            acceptedZaruCount =
                acceptedZaruCount,

            rejectedZaruCount =
                rejectedZaruCount,

            acceptedGnssPositionCount =
                acceptedGnssPositionCount,

            rejectedGnssPositionCount =
                rejectedGnssPositionCount,

            acceptedGnssVelocityCount =
                acceptedGnssVelocityCount,

            rejectedGnssVelocityCount =
                rejectedGnssVelocityCount,

            acceptedLearnedSpeedCount =
                acceptedLearnedSpeedCount,

            rejectedLearnedSpeedCount =
                rejectedLearnedSpeedCount,


            stateFinite =
                isStateFinite(),

            covarianceFinite =
                covariance.isFinite(),

            covarianceDiagonalValid =
                covariance.hasValidDiagonal()
        )
    }

    /*
     * ------------------------------------------------------------------
     * Reset
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun reset() {

        navigationState.reset()

        val freshCovariance =
            EskfCovariance.fromConfig(
                config
            )

        covariance.setFrom(
            freshCovariance
        )

        prediction.resetTimestamp(
            navigationState
        )

        acceptedPredictionCount = 0L
        rejectedPredictionCount = 0L

        acceptedZuptCount = 0L
        rejectedZuptCount = 0L

        acceptedNhcCount = 0L
        rejectedNhcCount = 0L

        acceptedZaruCount = 0L
        rejectedZaruCount = 0L

        acceptedGnssPositionCount = 0L
        rejectedGnssPositionCount = 0L

        acceptedGnssVelocityCount = 0L
        rejectedGnssVelocityCount = 0L

        acceptedLearnedSpeedCount = 0L
        rejectedLearnedSpeedCount = 0L


        lastPredictionResult = null
        lastZuptResult = null
        lastNhcResult = null
        lastZaruResult = null
        lastGnssPositionResult = null
        lastGnssVelocityResult = null
        lastLearnedSpeedResult = null
    }


    /*
     * NOTE: this previously modified getState(), which returns a
     * COPY, so it silently did nothing. It now writes the real
     * navigation state.
     */
    @Synchronized
    fun initializeReplayAttitude(
        attitude: Quaternion,
        timestampNanos: Long
    ) {
        navigationState.attitude =
            attitude.normalized()

        navigationState.velocity =
            Vec3.ZERO

        navigationState.position =
            Vec3.ZERO

        navigationState.timestampNanos =
            timestampNanos
    }

    /*
     * ------------------------------------------------------------------
     * Divergence recovery
     * ------------------------------------------------------------------
     */

    /**
     * Replaces the nominal velocity with an externally trusted
     * value (e.g. the learned-speed estimate) and widens the
     * velocity covariance so subsequent measurements can refine it.
     *
     * Use this when the inertial prediction has diverged
     * (repeated "velocity exceeded safety limit" rejections),
     * instead of freezing the filter forever.
     */
    @Synchronized
    fun reseedVelocity(
        velocity: Vec3,
        velocityStdMps: Double
    ) {
        require(velocity.isFinite()) {
            "Reseed velocity is non-finite"
        }

        require(
            velocityStdMps.isFinite() &&
                    velocityStdMps > 0.0
        ) {
            "velocityStdMps must be finite and > 0"
        }

        navigationState.velocity =
            velocity.copy()

        val variance =
            velocityStdMps * velocityStdMps

        // Velocity states occupy indices 3..5.
        for (i in 3..5) {
            for (j in 0 until STATE_SIZE) {
                if (j != i) {
                    covariance[i, j] = 0.0
                    covariance[j, i] = 0.0
                }
            }
            covariance.setDiagonal(i, variance)
        }
    }

    /**
     * Gravity-based roll/pitch correction.
     *
     * Attitude is otherwise pure gyro integration; any gyro bias
     * or axis error accumulates into a tilt that leaks gravity
     * into the horizontal plane. When the specific-force magnitude
     * is close to g (little linear acceleration), pull the
     * predicted "up" direction toward the measured one.
     *
     * Body = FRD, Navigation = NED. At rest f_b = R^T * [0, 0, -g].
     *
     * Small-angle correction applied as a body-frame increment:
     *
     *     q <- q (x) dq(k * (u_meas x u_pred))
     *
     * Returns true if a correction was applied.
     */
    @Synchronized
    fun applyGravityTilt(
        specificForceBody: Vec3,
        gain: Double,
        expectedLinearAccelBody: Vec3 = Vec3.ZERO,
        maxMagnitudeDeviationMps2: Double = 0.15,
        dtSeconds: Double = 0.0,
        biasGain: Double = 0.0
    ): Boolean {

        if (
            !specificForceBody.isFinite() ||
            !expectedLinearAccelBody.isFinite() ||
            !gain.isFinite() ||
            gain <= 0.0
        ) {
            return false
        }

        /*
         * f_b = a_b - g_b, so the gravity-only part is
         * f_b - a_b. In a steady turn the dominant kinematic
         * term is the centripetal acceleration [0, v*wz, 0]
         * (FRD, +wz = turning right); callers pass it in so the
         * correction does not cancel real cornering.
         */
        val gravityPart =
            specificForceBody - expectedLinearAccelBody

        val magnitude =
            gravityPart.norm()

        if (
            !magnitude.isFinite() ||
            magnitude < 1e-6 ||
            kotlin.math.abs(
                magnitude - config.gravityMps2
            ) > maxMagnitudeDeviationMps2
        ) {
            return false
        }

        val measuredUp =
            gravityPart / magnitude

        val predictedUp =
            navigationState.attitude
                .conjugate()
                .rotate(
                    Vec3(0.0, 0.0, -1.0)
                )

        val angleError =
            measuredUp.cross(predictedUp)

        val correction =
            angleError * gain

        /*
         * Integral path (complementary filter): a persistent
         * tilt error means the gyro bias estimate is wrong.
         * Gyro bias is subtracted in predict(), so a positive
         * residual drift (negative angleError) must INCREASE
         * the bias estimate:
         *
         *     b <- b - ki * dt * e
         *
         * Without this, tilt aiding only bounds the error at
         * (bias / loop gain), e.g. ~5 deg for a 0.003 rad/s bias.
         */
        if (
            biasGain > 0.0 &&
            dtSeconds > 0.0 &&
            dtSeconds.isFinite()
        ) {

            val updated =
                navigationState.gyroBias -
                        angleError * (biasGain * dtSeconds)

            // Never let this path wander beyond ~3 deg/s.
            val limit = 0.05

            navigationState.gyroBias =
                Vec3(
                    updated.x.coerceIn(-limit, limit),
                    updated.y.coerceIn(-limit, limit),
                    updated.z.coerceIn(-limit, limit)
                )
        }

        navigationState.attitude =
            (
                    navigationState.attitude *
                            Quaternion.fromRotationVector(
                                correction
                            )
                    ).normalized()

        return true
    }

    /*
     * ------------------------------------------------------------------
     * Sensor timeline reset
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun resetSensorTimeline() {

        prediction.resetTimestamp(
            navigationState
        )
    }

    /*
     * ------------------------------------------------------------------
     * Initial state
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun setInitialState(
        position: Vec3 = Vec3.ZERO,
        velocity: Vec3 = Vec3.ZERO,
        attitude: Quaternion =
            Quaternion.IDENTITY,
        gyroBias: Vec3 = Vec3.ZERO,
        accelBias: Vec3 = Vec3.ZERO,
        timestampNanos: Long = 0L
    ) {

        require(position.isFinite()) {
            "Initial position is non-finite"
        }

        require(velocity.isFinite()) {
            "Initial velocity is non-finite"
        }

        require(attitude.isFinite()) {
            "Initial attitude is non-finite"
        }

        require(gyroBias.isFinite()) {
            "Initial gyro bias is non-finite"
        }

        require(accelBias.isFinite()) {
            "Initial accelerometer bias is non-finite"
        }

        require(timestampNanos >= 0L) {
            "Initial timestamp cannot be negative"
        }

        navigationState.position =
            position.copy()

        navigationState.velocity =
            velocity.copy()

        navigationState.attitude =
            attitude.normalized()

        navigationState.gyroBias =
            gyroBias.copy()

        navigationState.accelBias =
            accelBias.copy()

        navigationState.timestampNanos =
            timestampNanos

        lastPredictionResult = null
        lastZuptResult = null
        lastNhcResult = null
        lastZaruResult = null
        lastGnssPositionResult = null
        lastGnssVelocityResult = null
    }

    /*
     * ------------------------------------------------------------------
     * Internal prediction output builder
     * ------------------------------------------------------------------
     */

    private fun buildPredictionOutput(
        result:
        EskfPrediction.PredictionResult
    ): PredictionOutput {

        return PredictionOutput(

            accepted =
                result.accepted,

            deltaTimeSeconds =
                result.deltaTimeSeconds,

            state =
                result.state,

            covariance =
                covariance.copyMatrix(),

            specificForceBody =
                result.specificForceBody,

            specificForceNavigation =
                result.specificForceNavigation,

            gyroRateBody =
                result.gyroRateBody,

            accelerationNavigation =
                result.accelerationNavigation,

            acceptedPredictionCount =
                acceptedPredictionCount,

            rejectedPredictionCount =
                rejectedPredictionCount,

            reason =
                result.reason
        )
    }
}


/**
 * Complete ESKF diagnostic snapshot.
 */
data class EskfDiagnostics(

    val timestampNanos: Long,

    val position: Vec3,

    val velocity: Vec3,

    val speedMps: Double,

    val attitude: Quaternion,

    val gyroBias: Vec3,

    val accelBias: Vec3,

    val acceptedPredictionCount: Long,

    val rejectedPredictionCount: Long,

    val acceptedZuptCount: Long,

    val rejectedZuptCount: Long,

    val acceptedNhcCount: Long,

    val rejectedNhcCount: Long,

    val acceptedZaruCount: Long,

    val rejectedZaruCount: Long,

    val acceptedGnssPositionCount: Long,

    val rejectedGnssPositionCount: Long,

    val acceptedGnssVelocityCount: Long,

    val rejectedGnssVelocityCount: Long,

    val acceptedLearnedSpeedCount: Long,

    val rejectedLearnedSpeedCount: Long,

    val stateFinite: Boolean,

    val covarianceFinite: Boolean,

    val covarianceDiagonalValid: Boolean
)