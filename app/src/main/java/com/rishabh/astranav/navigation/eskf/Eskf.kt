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
 *
 * Future corrections:
 *
 *  - ZARU
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

    private var lastPredictionResult:
            EskfPrediction.PredictionResult? = null

    private var lastZuptResult:
            EskfZuptUpdate.UpdateResult? = null

    private var lastNhcResult:
            EskfNhcUpdate.UpdateResult? = null

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
     *
     * Main ESKF prediction API.
     *
     * Flow:
     *
     * IMU
     *  ↓
     * nominal INS prediction
     *  ↓
     * covariance prediction
     *  ↓
     * updated ESKF state
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

        /*
         * Prediction rejected.
         */
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
         *
         * No covariance propagation is necessary
         * when dt = 0.
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
     * IMPORTANT:
     *
     * This method does not decide whether NHC should be used.
     * The runtime/navigation layer must decide whether the vehicle
     * is in a valid moving-ground-vehicle condition.
     *
     * Runtime flow:
     *
     *     vehicle moving
     *          ↓
     *     NHC eligible
     *          ↓
     *     Eskf.applyNhc()
     *          ↓
     *     generic measurement update
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
     *
     * IMPORTANT:
     *
     * ZuptDetector decides whether the vehicle is stationary.
     *
     * This method only performs the actual ESKF measurement update.
     *
     * Runtime flow:
     *
     * ZuptDetector
     *      ↓
     * stationary == true
     *      ↓
     * Eskf.applyZupt()
     *      ↓
     * z = [0,0,0] m/s
     *      ↓
     * Kalman correction
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

    /**
     * Returns a copy of the current nominal navigation state.
     */
    @Synchronized
    fun getState(): NavigationState {
        return navigationState.copyState()
    }

    /**
     * Returns a copy of the current covariance matrix.
     */
    @Synchronized
    fun getCovariance(): Array<DoubleArray> {
        return covariance.copyMatrix()
    }

    /**
     * Current position.
     */
    @Synchronized
    fun getPosition(): Vec3 {
        return navigationState.position.copy()
    }

    /**
     * Current velocity.
     */
    @Synchronized
    fun getVelocity(): Vec3 {
        return navigationState.velocity.copy()
    }

    /**
     * Current attitude.
     */
    @Synchronized
    fun getAttitude(): Quaternion {
        return navigationState.attitude.copy()
    }

    /**
     * Current gyro bias.
     */
    @Synchronized
    fun getGyroBias(): Vec3 {
        return navigationState.gyroBias.copy()
    }

    /**
     * Current accelerometer bias.
     */
    @Synchronized
    fun getAccelBias(): Vec3 {
        return navigationState.accelBias.copy()
    }

    /**
     * Current state timestamp.
     */
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
     * Prediction result diagnostics
     * ------------------------------------------------------------------
     */

    @Synchronized
    fun getLastPredictionResult():
            EskfPrediction.PredictionResult? {

        return lastPredictionResult
    }

    /*
     * ------------------------------------------------------------------
     * Numerical validity
     * ------------------------------------------------------------------
     */

    /**
     * Checks whether the nominal state contains
     * only finite values.
     */
    @Synchronized
    fun isStateFinite(): Boolean {

        return navigationState.position.isFinite() &&
                navigationState.velocity.isFinite() &&
                navigationState.attitude.isFinite() &&
                navigationState.gyroBias.isFinite() &&
                navigationState.accelBias.isFinite()
    }

    /**
     * Checks whether covariance is numerically valid.
     */
    @Synchronized
    fun isCovarianceValid(): Boolean {

        return covariance.isFinite() &&
                covariance.hasValidDiagonal()
    }

    /**
     * Covariance diagnostic summary.
     */
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
     *
     * Complete ESKF reset.
     *
     * Resets:
     *
     *  - position
     *  - velocity
     *  - attitude
     *  - gyro bias
     *  - accelerometer bias
     *  - timestamp
     *  - covariance
     *  - diagnostics
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

        lastPredictionResult = null
        lastZuptResult = null
        lastNhcResult = null
    }

    /*
     * ------------------------------------------------------------------
     * Sensor timeline reset
     * ------------------------------------------------------------------
     *
     * Resets only the IMU timeline.
     *
     * State and covariance are retained.
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
     *
     * Sets the initial nominal navigation state.
     *
     * Covariance remains unchanged.
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

    val stateFinite: Boolean,

    val covarianceFinite: Boolean,

    val covarianceDiagonalValid: Boolean
)