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
import com.rishabh.astranav.integrity.IntegrityMonitor
import com.rishabh.astranav.integrity.IntegritySnapshot
import com.rishabh.astranav.ml.TcnMotionEngine
import com.rishabh.astranav.sensor.GnssSample
import com.rishabh.astranav.sensor.ImuSample
import com.rishabh.astranav.sensor.SensorAdapter
import kotlin.math.abs
import kotlin.math.sqrt
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/** What the vehicle is doing right now — derived from NavigationState.stationary + live yaw rate. */
enum class MotionActivity { STATIONARY, DRIVING, TURNING }

/** Everything HomeActivity / MapActivity need to render, in one live snapshot. */
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
    // Rolling, downsampled traces for the Home sparkline tiles — newest value last.
    val aiSpeedHistory: List<Float> = emptyList(),
    val motionEnergyHistory: List<Float> = emptyList(),
    // Map-screen additions
    val tripDistanceMeters: Double = 0.0,
    val integrity: IntegritySnapshot? = null,
    /** The last lat/lon we had a real GNSS fix at — the "last reliable GNSS point" marker while in dead reckoning. */
    val lastGnssFixLatitude: Double? = null,
    val lastGnssFixLongitude: Double? = null,
)

/**
 * The bridge from live Android sensors/GNSS to the existing NavigationEngine.
 * Everything in navigation/, constraints/, gnss/, ml/ and integrity/ was pure,
 * already-correct logic with no Android sensor wiring anywhere in the app —
 * MainActivity's own comment says as much ("no sensor, location, or
 * permission APIs are used"). This is that wiring.
 *
 * A process-wide singleton, started/stopped from each consuming Activity's
 * onResume/onPause (Home, Map) — not a foreground Service. The manifest
 * already requests FOREGROUND_SERVICE / FOREGROUND_SERVICE_LOCATION, which
 * is the natural next step once navigation needs to keep running with the
 * screen off or the app backgrounded; out of scope for now.
 */
object NavigationSessionController : SensorEventListener {

    private const val TURN_RATE_THRESHOLD_RAD_S = 0.12 // ≈ 7°/s
    private const val ACCEL_VARIANCE_WINDOW = 30
    private const val SPARKLINE_STRIDE = 20
    private const val SPARKLINE_LENGTH = 24

    private val engine = NavigationEngine()
    private val adapter = SensorAdapter()
    private val zupt = ZuptConstraint()
    private val zaru = ZaruConstraint()
    private val integrityMonitor = IntegrityMonitor()
    private var tcn: TcnMotionEngine? = null

    private var sensorManager: SensorManager? = null
    private var fusedLocationClient: FusedLocationProviderClient? = null
    private var locationCallback: LocationCallback? = null
    private var started = false

    private var latestGravity = doubleArrayOf(0.0, 0.0, 9.81)
    private var latestGyro = doubleArrayOf(0.0, 0.0, 0.0)
    private var latestGnss: GnssSample? = null

    // Raw last-known GNSS coordinates, kept purely for display (e.g. a "current position" card,
    // or the map's "last reliable fix" marker during dead reckoning).
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

    // Sparkline traces for Home's Vehicle Signal card — pushed at ~5 Hz (every
    // SPARKLINE_STRIDE-th IMU sample), not every sample, so the bars read as a trend.
    private var sparklineTick = 0
    private val aiSpeedHistory = ArrayDeque<Float>()
    private val motionEnergyHistory = ArrayDeque<Float>()

    // Cumulative trip distance — trapezoidal integration of fused speed. Resettable.
    private var tripDistanceMeters = 0.0

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

        if (tcn == null) tcn = runCatching { TcnMotionEngine(appContext) }.getOrNull()

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

    /** Zeroes the cumulative trip odometer — call when the user starts a new trip. */
    fun resetTrip() {
        tripDistanceMeters = 0.0
        _state.value = _state.value.copy(tripDistanceMeters = 0.0)
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

        val ml = tcn?.add(imu)
        val nav = engine.update(imu, dt, latestGnss, ml)

        tripDistanceMeters += nav.speedMps * dt

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
            val energy = (gyroMag / 2.0).coerceIn(0.0, 1.0).toFloat()
            pushSparkline(motionEnergyHistory, energy)
        }

        val gnssAvailableNow = latestGnss != null
        val gnssAccuracy = latestGnss?.accuracyM
        val aiAvailableNow = ml?.valid == true

        // Integrity inputs — three of these are real signals already produced elsewhere in the
        // app; "physics" is an honest placeholder (1.0) until a real physics-consistency check
        // (e.g. NHC residual) exists to drive it — see IntegrityMonitor's doc in the README.
        val mlScore = if (aiAvailableNow) 1.0 else 0.4
        val physicsScore = 1.0
        val mapScore = if (nav.mapMatched) 1.0 else 0.6
        val gnssScore = if (gnssAvailableNow && gnssAccuracy != null) (1.0 - (gnssAccuracy / 30.0)).coerceIn(0.0, 1.0) else 0.0
        val integrity = integrityMonitor.evaluate(mlScore, physicsScore, mapScore, gnssScore)

        _state.value = _state.value.copy(
            nav = nav,
            motion = motion,
            zuptActive = zupt.active(nav.speedMps, gyroMag, variance(accelWindow)),
            zaruActive = zaru.active(gyroMag, nav.stationary),
            imuAvailable = true,
            imuHz = currentImuHz(),
            gnssAvailable = gnssAvailableNow,
            gnssAccuracyM = gnssAccuracy,
            aiAvailable = aiAvailableNow,
            aiSpeedMps = ml?.takeIf { it.valid }?.speedMps,
            gnssLatitude = lastKnownLat,
            gnssLongitude = lastKnownLon,
            aiSpeedHistory = aiSpeedHistory.toList(),
            motionEnergyHistory = motionEnergyHistory.toList(),
            tripDistanceMeters = tripDistanceMeters,
            integrity = integrity,
            lastGnssFixLatitude = if (gnssAvailableNow) lastKnownLat else _state.value.lastGnssFixLatitude,
            lastGnssFixLongitude = if (gnssAvailableNow) lastKnownLon else _state.value.lastGnssFixLongitude,
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
                if (hzSampleCount > 200) {
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
