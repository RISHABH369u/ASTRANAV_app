package com.rishabh.astranav.navigation

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import androidx.core.content.ContextCompat
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.rishabh.astranav.constraints.ZaruConstraint
import com.rishabh.astranav.constraints.ZuptConstraint
import com.rishabh.astranav.dvfc.DvfcCalibrationStore
import com.rishabh.astranav.ml.gru.GruSpeedEngine
import com.rishabh.astranav.ml.astramotion.AstraMotionEngine
import com.rishabh.astranav.sensor.GnssSample
import com.rishabh.astranav.sensor.ImuSample
import com.rishabh.astranav.sensor.SensorAdapter
import kotlin.math.abs
import kotlin.math.sqrt
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/** What the vehicle is doing right now — derived from NavigationState.stationary + live yaw rate. */
enum class MotionActivity { STATIONARY, DRIVING, TURNING }

/** Everything HomeActivity needs to render, in one live snapshot. */
data class HomeDashboardState(
    val nav: NavigationState = NavigationState(),
    val motion: MotionActivity = MotionActivity.STATIONARY,
    val zuptActive: Boolean = false,
    val zaruActive: Boolean = false,
    val imuAvailable: Boolean = false,
    val imuHz: Double? = null,
    val gnssAvailable: Boolean = false,
    val gnssAccuracyM: Double? = null,
    val aiAvailable: Boolean = false,
    val aiSpeedMps: Double? = null,
    val dvfcCalibrated: Boolean = false,
    val dvfcYawOffsetDeg: Double? = null,
    val gnssLatitude: Double? = null,
    val gnssLongitude: Double? = null,
    // Rolling, downsampled traces for the sparkline tiles — newest value last.
    val aiSpeedHistory: List<Float> = emptyList(),
    val motionEnergyHistory: List<Float> = emptyList(),
)

/**
 * The first real bridge from live Android sensors/GNSS to the existing
 * NavigationEngine. Everything in navigation/, constraints/, gnss/ and ml/
 * was pure, already-correct logic with no Android sensor wiring anywhere in
 * the app yet — MainActivity's own comment says as much ("no sensor,
 * location, or permission APIs are used"). This is that wiring, scoped to
 * what HomeActivity needs to show live numbers instead of mock ones.
 *
 * A process-wide singleton for now, started/stopped from HomeActivity's
 * onResume/onPause — not a foreground Service. The manifest already
 * requests FOREGROUND_SERVICE / FOREGROUND_SERVICE_LOCATION, which is the
 * natural next step once navigation needs to keep running with the screen
 * off or the app backgrounded; that's out of scope for just the Home screen.
 */
object NavigationSessionController : SensorEventListener {

    private const val TURN_RATE_THRESHOLD_RAD_S = 0.12 // ≈ 7°/s
    private const val ACCEL_VARIANCE_WINDOW = 30

    private val engine = NavigationEngine()
    private val adapter = SensorAdapter()
    private val zupt = ZuptConstraint()
    private val zaru = ZaruConstraint()
    private var astraSpeed: GruSpeedEngine? = null
    private var astraMotion: AstraMotionEngine? = null

    private var previousTrustedSpeedKmh = 0.0
    private var previousYawRate = 0.0
    private var previousModelTimestampNs: Long? = null

    private var sensorManager: SensorManager? = null
    private var fusedLocationClient: FusedLocationProviderClient? = null
    private var locationCallback: LocationCallback? = null
    private var started = false

    private var latestGravity = doubleArrayOf(0.0, 0.0, 9.81)
    private var latestGyro = doubleArrayOf(0.0, 0.0, 0.0)
    private var latestGnss: GnssSample? = null

    // Raw last-known GNSS coordinates, kept purely for display (e.g. a "current position" card).
    // NavigationEngine itself works in a local East/North metre frame, not lat/lon.
    private var lastKnownLat: Double? = null
    private var lastKnownLon: Double? = null

    // IMU Hz — rolling average of inter-sample periods (wall clock, not sensor timestamp,
    // since this is a UI-facing "is the sensor actually delivering samples" readout).
    private var hzSampleCount = 0
    private var hzPeriodSumMs = 0.0
    private var lastSampleUptimeMs = 0L

    // Rolling accel-magnitude variance, purely for a slightly more honest ZUPT *display*
    // than the constant NavigationEngine passes internally (0.02) — doesn't change engine behavior.
    private val accelWindow = ArrayDeque<Double>()

    // Sparkline traces for the Vehicle Signal card — pushed at ~5 Hz (every SPARKLINE_STRIDE-th
    // IMU sample), not every sample, so the bars read as a trend rather than noise.
    private const val SPARKLINE_STRIDE = 20
    private const val SPARKLINE_LENGTH = 24
    private var sparklineTick = 0
    private val aiSpeedHistory = ArrayDeque<Float>()
    private val motionEnergyHistory = ArrayDeque<Float>()

    private val _state = MutableStateFlow(HomeDashboardState())
    val state: StateFlow<HomeDashboardState> = _state

    fun start(context: Context) {
        if (started) return
        started = true

        val appContext = context.applicationContext
        engine.reset()

        val dvfc = DvfcCalibrationStore.load(appContext)
        _state.value = _state.value.copy(
            dvfcCalibrated = dvfc != null,
            dvfcYawOffsetDeg = dvfc?.toEulerDegrees()?.get(2)?.toDouble(),
        )

        astraSpeed = runCatching {
            GruSpeedEngine(appContext)
        }.onFailure {
            android.util.Log.e(
                "ASTRANAV_ML",
                "Failed to load ASTRA-Speed",
                it
            )
        }.getOrNull()

        astraMotion = runCatching {
            AstraMotionEngine(appContext)
        }.onFailure {
            android.util.Log.e(
                "ASTRANAV_ML",
                "Failed to load ASTRA-Motion",
                it
            )
        }.getOrNull()

        previousTrustedSpeedKmh = 0.0
        previousYawRate = 0.0
        previousModelTimestampNs = null

        sensorManager = (appContext.getSystemService(Context.SENSOR_SERVICE) as? SensorManager)?.also { sm ->
            sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
            sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
            sm.getDefaultSensor(Sensor.TYPE_GRAVITY)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        }

        val hasFineLocation = ContextCompat.checkSelfPermission(
            appContext,
            Manifest.permission.ACCESS_FINE_LOCATION,
        ) == PackageManager.PERMISSION_GRANTED
        if (hasFineLocation) startLocationUpdates(appContext)
    }

    /** Call once permission is granted after start() already ran without it. */
    fun onLocationPermissionGranted(context: Context) {
        if (fusedLocationClient == null) startLocationUpdates(context.applicationContext)
    }

    fun stop() {
        started = false
        sensorManager?.unregisterListener(this)
        locationCallback?.let { fusedLocationClient?.removeLocationUpdates(it) }
        locationCallback = null
        fusedLocationClient = null
    }

    private fun startLocationUpdates(context: Context) {
        val client = LocationServices.getFusedLocationProviderClient(context)
        fusedLocationClient = client
        val request = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L).build()
        val callback = object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                val loc = result.lastLocation ?: return
                lastKnownLat = loc.latitude
                lastKnownLon = loc.longitude
                latestGnss = GnssSample(
                    timestampMillis = loc.time,
                    latitudeDeg = loc.latitude,
                    longitudeDeg = loc.longitude,
                    altitudeM = loc.altitude,
                    speedMps = if (loc.hasSpeed()) loc.speed.toDouble() else 0.0,
                    bearingDeg = if (loc.hasBearing()) loc.bearing.toDouble() else 0.0,
                    accuracyM = if (loc.hasAccuracy()) loc.accuracy.toDouble() else 99.0,
                )
            }
        }
        locationCallback = callback
        try {
            @Suppress("MissingPermission")
            client.requestLocationUpdates(request, callback, null)
        } catch (_: SecurityException) {
            // Permission revoked between the check in start() and here — GNSS just stays null,
            // NavigationEngine already handles gnss == null as a dead-reckoning input.
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_GRAVITY -> latestGravity = doubleArrayOf(
                event.values[0].toDouble(), event.values[1].toDouble(), event.values[2].toDouble(),
            )
            Sensor.TYPE_GYROSCOPE -> latestGyro = doubleArrayOf(
                event.values[0].toDouble(), event.values[1].toDouble(), event.values[2].toDouble(),
            )
            Sensor.TYPE_ACCELEROMETER -> onAccelerometer(event)
        }
    }

    private fun onAccelerometer(event: SensorEvent) {
        val imu = ImuSample(
            timestampNanos = event.timestamp,
            accelX = event.values[0].toDouble(), accelY = event.values[1].toDouble(), accelZ = event.values[2].toDouble(),
            gyroX = latestGyro[0], gyroY = latestGyro[1], gyroZ = latestGyro[2],
            gravityX = latestGravity[0], gravityY = latestGravity[1], gravityZ = latestGravity[2],
        )
        val dt = adapter.dtSeconds(imu) ?: return
        trackImuHz()

        val accelMag = sqrt(imu.accelX * imu.accelX + imu.accelY * imu.accelY + imu.accelZ * imu.accelZ)
        accelWindow.addLast(accelMag)
        if (accelWindow.size > ACCEL_VARIANCE_WINDOW) accelWindow.removeFirst()

//        val ml = tcn?.add(imu)
//        val nav = engine.update(imu, dt, latestGnss, ml)
        val speedOutput =
            astraSpeed?.add(
                sample = imu,
                previousTrustedSpeedKmh = previousTrustedSpeedKmh
            )

        if (speedOutput?.valid == true) {

            previousTrustedSpeedKmh =
                speedOutput.speedKmh

            android.util.Log.i(
                "ASTRANAV_ML",
                "ASTRA-Speed REAL OUTPUT = " +
                        "${"%.2f".format(speedOutput.speedKmh)} km/h"
            )
        }
        val ml =
            speedOutput?.takeIf { it.valid }?.let {

                com.rishabh.astranav.ml.MlMeasurement(
                    speedMps = it.speedMps,

                    // Conservative temporary fallback.
                    // This is NOT claimed as calibrated model uncertainty.
                    variance = 9.0,

                    confidence = 0.5,

                    valid = true
                )
            }

        val nav =
            engine.update(
                imu,
                dt,
                latestGnss,
                ml
            )




        val gyroMag = sqrt(imu.gyroX * imu.gyroX + imu.gyroY * imu.gyroY + imu.gyroZ * imu.gyroZ)
        val motion = when {
            nav.stationary -> MotionActivity.STATIONARY
            abs(imu.gyroZ) > TURN_RATE_THRESHOLD_RAD_S -> MotionActivity.TURNING
            else -> MotionActivity.DRIVING
        }

        sparklineTick++
        if (sparklineTick % SPARKLINE_STRIDE == 0) {
            val aiKmh = ml?.takeIf { it.valid }?.speedMps?.times(3.6)?.toFloat() ?: 0f
            pushSparkline(aiSpeedHistory, aiKmh)
            // A simple 0..1 "how much is happening" trace: normalized gyro magnitude,
            // capped so one sharp turn doesn't flatten the rest of the trace.
            val energy = (gyroMag / 2.0).coerceIn(0.0, 1.0).toFloat()
            pushSparkline(motionEnergyHistory, energy)
        }

        _state.value = _state.value.copy(
            nav = nav,
            motion = motion,
            zuptActive = zupt.active(nav.speedMps, gyroMag, variance(accelWindow)),
            zaruActive = zaru.active(gyroMag, nav.stationary),
            imuAvailable = true,
            imuHz = currentImuHz(),
            gnssAvailable = latestGnss != null,
            gnssAccuracyM = latestGnss?.accuracyM,
            aiAvailable = ml?.valid == true,
            aiSpeedMps = ml?.takeIf { it.valid }?.speedMps,
            gnssLatitude = lastKnownLat,
            gnssLongitude = lastKnownLon,
            aiSpeedHistory = aiSpeedHistory.toList(),
            motionEnergyHistory = motionEnergyHistory.toList(),
        )
    }

    private fun pushSparkline(buffer: ArrayDeque<Float>, value: Float) {
        buffer.addLast(value)
        if (buffer.size > SPARKLINE_LENGTH) buffer.removeFirst()
    }

    private fun trackImuHz() {
        val now = SystemClock.elapsedRealtime()
        if (lastSampleUptimeMs != 0L) {
            val periodMs = (now - lastSampleUptimeMs).toDouble()
            if (periodMs in 0.5..500.0) {
                hzPeriodSumMs += periodMs
                hzSampleCount++
                if (hzSampleCount > 200) { // keep it a rolling-ish average, not an unbounded sum
                    hzPeriodSumMs *= 0.5
                    hzSampleCount /= 2
                }
            }
        }
        lastSampleUptimeMs = now
    }

    private fun currentImuHz(): Double? {
        if (hzSampleCount == 0) return null
        val avgPeriodMs = hzPeriodSumMs / hzSampleCount
        return if (avgPeriodMs > 0) 1000.0 / avgPeriodMs else null
    }

    private fun variance(values: ArrayDeque<Double>): Double {
        if (values.isEmpty()) return 0.0
        val mean = values.average()
        return values.sumOf { (it - mean) * (it - mean) } / values.size
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
