package com.rishabh.astranav

import android.graphics.Bitmap
import android.graphics.Canvas
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast

import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat

import com.mapbox.geojson.Point
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.CoordinateBounds
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.mapbox.maps.extension.style.layers.properties.generated.IconAnchor
import com.mapbox.maps.plugin.annotation.annotations
import com.mapbox.maps.plugin.annotation.generated.PointAnnotation
import com.mapbox.maps.plugin.annotation.generated.PointAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.PointAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotation
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.PolylineAnnotationOptions
import com.mapbox.maps.plugin.annotation.generated.createPointAnnotationManager
import com.mapbox.maps.plugin.annotation.generated.createPolylineAnnotationManager

import com.rishabh.astranav.navigation.AstraNavigationEngine
import com.rishabh.astranav.replay.IoVnbdReplaySession
import com.rishabh.astranav.replay.ReplayDataset
import com.rishabh.astranav.replay.ReplayFrame

import java.util.Locale
import kotlin.math.cos
import kotlin.math.roundToInt


/**
 * =============================================================
 * ASTRANAV — IO-VNBD TRIP REPLAY
 * =============================================================
 *
 * Real replay pipeline:
 *
 * S-*.csv
 *     ↓
 * IoVnbdReplaySession
 *     ↓
 * same ASTRANAV navigation runtime
 *     ↓
 * DVFC
 *     ↓
 * ASTRA-Core ESKF
 *     ↓
 * ZUPT / ZARU / NHC
 *     ↓
 * NavigationSolution
 *     ↓
 * compare with V-*.csv reference
 *
 * Mapbox is used ONLY as the visualization layer.
 *
 * No synthetic navigation metrics are generated here.
 */
class TripReplayActivity : AppCompatActivity() {

    // ---------------------------------------------------------
    // MAP
    // ---------------------------------------------------------

    private lateinit var mapView: MapView

    private var styleReady = false

    private var pointAnnotationManager: PointAnnotationManager? = null

    private var trajectoryAnnotationManager:
            PolylineAnnotationManager? = null

    private var vehicleAnnotation: PointAnnotation? = null

    private var estimatedTrajectoryAnnotation:
            PolylineAnnotation? = null

    private var vehicleIcon: Bitmap? = null


    // ---------------------------------------------------------
    // HEADER
    // ---------------------------------------------------------

    private lateinit var tvDatasetName: TextView
    private lateinit var tvDatasetStatus: TextView
    private lateinit var tvHz: TextView


    // ---------------------------------------------------------
    // PRIMARY TELEMETRY
    // ---------------------------------------------------------

    private lateinit var tvSpeed: TextView
    private lateinit var tvHeading: TextView
    private lateinit var tvPositionError: TextView
    private lateinit var tvHeadingError: TextView


    // ---------------------------------------------------------
    // METRICS
    // ---------------------------------------------------------

    private lateinit var tvRmse: TextView
    private lateinit var tvP95: TextView
    private lateinit var tvFinalError: TextView
    private lateinit var tvSpeedMae: TextView


    // ---------------------------------------------------------
    // SYSTEM HEALTH
    // ---------------------------------------------------------

    private lateinit var chipSensor: TextView
    private lateinit var chipRate: TextView
    private lateinit var chipEskf: TextView
    private lateinit var chipNhc: TextView
    private lateinit var chipZupt: TextView
    private lateinit var chipZaru: TextView
    private lateinit var chipDvfc: TextView


    // ---------------------------------------------------------
    // TIMELINE
    // ---------------------------------------------------------

    private lateinit var replaySeekBar: SeekBar
    private lateinit var tvTimeElapsed: TextView
    private lateinit var tvTimeTotal: TextView


    // ---------------------------------------------------------
    // PLAYBACK
    // ---------------------------------------------------------

    private lateinit var btnPlayPause: View
    private lateinit var tvPlayPause: TextView
    private lateinit var tvPlaybackSpeed: TextView


    // ---------------------------------------------------------
    // FILE BUTTONS
    // ---------------------------------------------------------

    private lateinit var btnLoadPhone: TextView
    private lateinit var btnLoadVehicle: TextView


    // ---------------------------------------------------------
    // STATE
    // ---------------------------------------------------------

    private var phoneUri: Uri? = null
    private var vehicleUri: Uri? = null

    private var dataset: ReplayDataset? = null

    private lateinit var navigationEngine: AstraNavigationEngine

    private lateinit var replaySession: IoVnbdReplaySession


    // ---------------------------------------------------------
    // PLAYBACK
    // ---------------------------------------------------------

    private var currentTimeSeconds = 0.0

    private var playbackSpeed = 1.0

    private var playing = false

    private var seeking = false

    private var lastTickNanos = 0L


    // ---------------------------------------------------------
    // TRAJECTORY
    // ---------------------------------------------------------

    private val estimatedPoints =
        ArrayList<Point>()

    private var trajectoryOriginLat = Double.NaN
    private var trajectoryOriginLon = Double.NaN


    // ---------------------------------------------------------
    // HANDLER
    // ---------------------------------------------------------

    private val handler =
        Handler(Looper.getMainLooper())


    private val tickRunnable =
        object : Runnable {

            override fun run() {

                if (playing) {

                    val now =
                        System.nanoTime()

                    val dt =
                        if (lastTickNanos == 0L) {
                            0.0
                        } else {
                            (
                                    now -
                                            lastTickNanos
                                    ) /
                                    1_000_000_000.0
                        }

                    lastTickNanos = now

                    val duration =
                        datasetDurationSeconds()

                    if (duration > 0.0) {

                        currentTimeSeconds +=
                            dt *
                                    playbackSpeed

                        if (
                            currentTimeSeconds >=
                            duration
                        ) {

                            currentTimeSeconds =
                                duration

                            playing = false

                            updatePlayPause()
                        }

                        renderReplay()
                    }

                } else {

                    lastTickNanos = 0L
                }

                handler.postDelayed(
                    this,
                    33L
                )
            }
        }


    // ---------------------------------------------------------
    // FILE PICKERS
    // ---------------------------------------------------------

    private val phoneFilePicker =
        registerForActivityResult(
            ActivityResultContracts.OpenDocument()
        ) { uri ->

            if (uri != null) {

                phoneUri = uri

                tryTakePersistablePermission(uri)

                btnLoadPhone.text =
                    "S-CSV  ✓"

                tvDatasetStatus.text =
                    "SMARTPHONE DATA READY"

                attemptLoadReplay()
            }
        }


    private val vehicleFilePicker =
        registerForActivityResult(
            ActivityResultContracts.OpenDocument()
        ) { uri ->

            if (uri != null) {

                vehicleUri = uri

                tryTakePersistablePermission(uri)

                btnLoadVehicle.text =
                    "V-CSV  ✓"

                attemptLoadReplay()
            }
        }


    // =========================================================
    // ACTIVITY
    // =========================================================

    override fun onCreate(
        savedInstanceState: Bundle?
    ) {

        super.onCreate(
            savedInstanceState
        )

        setContentView(
            R.layout.activity_trip_replay
        )


        // -----------------------------------------------------
        // ENGINE
        // -----------------------------------------------------

        navigationEngine =
            AstraNavigationEngine(
                applicationContext
            )

        replaySession =
            IoVnbdReplaySession(
                applicationContext,
                navigationEngine
            )


        // -----------------------------------------------------
        // BIND VIEWS
        // -----------------------------------------------------

        bindViews()


        // -----------------------------------------------------
        // UI
        // -----------------------------------------------------

        setupButtons()

        setupSeekBar()

        setupPlaybackControls()

        updateEmptyState()


        // -----------------------------------------------------
        // MAP
        // -----------------------------------------------------

        setupMap()
    }


    // =========================================================
    // VIEW BINDING
    // =========================================================

    private fun bindViews() {

        mapView =
            findViewById(
                R.id.replayMapView
            )

        tvDatasetName =
            findViewById(
                R.id.tvDatasetName
            )

        tvDatasetStatus =
            findViewById(
                R.id.tvDatasetStatus
            )

        tvHz =
            findViewById(
                R.id.tvHz
            )


        tvSpeed =
            findViewById(
                R.id.tvSpeed
            )

        tvHeading =
            findViewById(
                R.id.tvHeading
            )

        tvPositionError =
            findViewById(
                R.id.tvPositionError
            )

        tvHeadingError =
            findViewById(
                R.id.tvHeadingError
            )


        tvRmse =
            findViewById(
                R.id.tvRmse
            )

        tvP95 =
            findViewById(
                R.id.tvP95
            )

        tvFinalError =
            findViewById(
                R.id.tvFinalError
            )

        tvSpeedMae =
            findViewById(
                R.id.tvSpeedMae
            )


        chipSensor =
            findViewById(
                R.id.chipSensor
            )

        chipRate =
            findViewById(
                R.id.chipRate
            )

        chipEskf =
            findViewById(
                R.id.chipEskf
            )

        chipNhc =
            findViewById(
                R.id.chipNhc
            )

        chipZupt =
            findViewById(
                R.id.chipZupt
            )

        chipZaru =
            findViewById(
                R.id.chipZaru
            )

        chipDvfc =
            findViewById(
                R.id.chipDvfc
            )


        replaySeekBar =
            findViewById(
                R.id.replaySeekBar
            )

        tvTimeElapsed =
            findViewById(
                R.id.tvTimeElapsed
            )

        tvTimeTotal =
            findViewById(
                R.id.tvTimeTotal
            )


        btnPlayPause =
            findViewById(
                R.id.btnPlayPause
            )

        tvPlayPause =
            findViewById(
                R.id.tvPlayPause
            )

        tvPlaybackSpeed =
            findViewById(
                R.id.tvPlaybackSpeed
            )


        btnLoadPhone =
            findViewById(
                R.id.btnLoadPhone
            )

        btnLoadVehicle =
            findViewById(
                R.id.btnLoadVehicle
            )
    }


    // =========================================================
    // BUTTONS
    // =========================================================

    private fun setupButtons() {

        findViewById<View>(
            R.id.btnCloseReplay
        ).setOnClickListener {

            finish()
        }


        btnLoadPhone.setOnClickListener {

            phoneFilePicker.launch(
                arrayOf(
                    "text/csv",
                    "text/*",
                    "*/*"
                )
            )
        }


        btnLoadVehicle.setOnClickListener {

            vehicleFilePicker.launch(
                arrayOf(
                    "text/csv",
                    "text/*",
                    "*/*"
                )
            )
        }


        findViewById<View>(
            R.id.btnBack
        ).setOnClickListener {

            currentTimeSeconds =
                (
                        currentTimeSeconds -
                                5.0
                        )
                    .coerceAtLeast(0.0)

            renderReplay()
        }


        findViewById<View>(
            R.id.btnForward
        ).setOnClickListener {

            currentTimeSeconds =
                (
                        currentTimeSeconds +
                                5.0
                        )
                    .coerceAtMost(
                        datasetDurationSeconds()
                    )

            renderReplay()
        }


        findViewById<View>(
            R.id.btnSpeed
        ).setOnClickListener {

            playbackSpeed =
                when (playbackSpeed) {

                    1.0 -> 2.0

                    2.0 -> 5.0

                    else -> 1.0
                }

            tvPlaybackSpeed.text =
                "${formatSpeed(playbackSpeed)}×"
        }
    }


    // =========================================================
    // SEEK BAR
    // =========================================================

    private fun setupSeekBar() {

        replaySeekBar.max =
            1000

        replaySeekBar.progress =
            0


        replaySeekBar.setOnSeekBarChangeListener(

            object :
                SeekBar.OnSeekBarChangeListener {

                override fun onProgressChanged(
                    seekBar: SeekBar,
                    progress: Int,
                    fromUser: Boolean
                ) {

                    if (!fromUser) {
                        return
                    }

                    val duration =
                        datasetDurationSeconds()

                    if (duration <= 0.0) {
                        return
                    }

                    currentTimeSeconds =
                        duration *
                                (
                                        progress /
                                                1000.0
                                        )

                    renderReplay()
                }


                override fun onStartTrackingTouch(
                    seekBar: SeekBar
                ) {

                    seeking = true

                    playing = false

                    updatePlayPause()
                }


                override fun onStopTrackingTouch(
                    seekBar: SeekBar
                ) {

                    seeking = false
                }
            }
        )
    }


    // =========================================================
    // PLAYBACK
    // =========================================================

    private fun setupPlaybackControls() {

        btnPlayPause.setOnClickListener {

            if (dataset == null) {

                Toast.makeText(
                    this,
                    "Load S-CSV first",
                    Toast.LENGTH_SHORT
                ).show()

                return@setOnClickListener
            }


            val duration =
                datasetDurationSeconds()

            if (duration <= 0.0) {
                return@setOnClickListener
            }


            if (
                currentTimeSeconds >=
                duration
            ) {

                currentTimeSeconds =
                    0.0

                resetVisualTrajectory()
            }


            playing =
                !playing

            lastTickNanos =
                0L

            updatePlayPause()
        }
    }


    private fun updatePlayPause() {

        tvPlayPause.text =
            if (playing) {
                "Ⅱ"
            } else {
                "▶"
            }
    }


    // =========================================================
    // MAP SETUP
    // =========================================================

    private fun setupMap() {

        mapView.mapboxMap.loadStyle(
            Style.STANDARD_SATELLITE
        ) {

            styleReady = true

            vehicleIcon =
                vectorToBitmap(
                    R.drawable.ic_vehicle_marker
                )

            pointAnnotationManager =
                mapView.annotations
                    .createPointAnnotationManager()

            trajectoryAnnotationManager =
                mapView.annotations
                    .createPolylineAnnotationManager()


            renderDatasetRoute()

            renderReplay()
        }
    }


    // =========================================================
    // DATA LOADING
    // =========================================================

    private fun attemptLoadReplay() {

        val phone =
            phoneUri

        if (phone == null) {
            return
        }


        try {

            playing = false

            currentTimeSeconds =
                0.0

            resetVisualTrajectory()


            dataset =
                replaySession.load(
                    phoneUri = phone,
                    vehicleUri = vehicleUri,
                    id = datasetNameFromUri(
                        phone
                    )
                )


            val d =
                dataset
                    ?: return


            tvDatasetName.text =
                d.id


            tvDatasetStatus.text =
                if (d.vehicle.isNotEmpty()) {
                    "S + V REFERENCE READY"
                } else {
                    "S-CSV READY · V-CSV OPTIONAL"
                }


            tvHz.text =
                estimateDatasetHz(d)


            tvTimeTotal.text =
                fmtTime(
                    datasetDurationSeconds()
                )


            chipSensor.text =
                "SENSOR  ✓"

            chipRate.text =
                "${estimateDatasetHz(d)}  ✓"

            chipEskf.text =
                "ESKF  READY"

            chipDvfc.text =
                "DVFC  —"


            if (d.vehicle.isNotEmpty()) {

                trajectoryOriginLat =
                    d.vehicle
                        .first()
                        .latitude

                trajectoryOriginLon =
                    d.vehicle
                        .first()
                        .longitude

            } else if (
                d.phone.firstOrNull()?.latitude != null &&
                d.phone.firstOrNull()?.longitude != null
            ) {

                trajectoryOriginLat =
                    d.phone
                        .first()
                        .latitude
                        ?: Double.NaN

                trajectoryOriginLon =
                    d.phone
                        .first()
                        .longitude
                        ?: Double.NaN
            }


            renderDatasetRoute()

            renderReplay()


            Toast.makeText(
                this,
                if (d.vehicle.isNotEmpty()) {
                    "IO-VNBD S + V loaded"
                } else {
                    "IO-VNBD S loaded"
                },
                Toast.LENGTH_SHORT
            ).show()


        } catch (e: Exception) {

            val fullError =
                buildString {
                    appendLine("Replay load failed")
                    appendLine()
                    appendLine("Exception: ${e::class.java.name}")
                    appendLine()
                    appendLine("Message:")
                    appendLine(e.message ?: "No exception message")
                    appendLine()
                    appendLine("Cause:")
                    appendLine(e.cause?.message ?: "None")
                }

            android.util.Log.e(
                "ASTRA_REPLAY",
                fullError,
                e
            )

            android.app.AlertDialog.Builder(this)
                .setTitle("Replay Load Failed")
                .setMessage(fullError)
                .setPositiveButton("OK", null)
                .setNeutralButton("Copy") { _, _ ->

                    val clipboard =
                        getSystemService(android.content.Context.CLIPBOARD_SERVICE)
                                as android.content.ClipboardManager

                    clipboard.setPrimaryClip(
                        android.content.ClipData.newPlainText(
                            "ASTRA Replay Error",
                            fullError
                        )
                    )

                    android.widget.Toast.makeText(
                        this,
                        "Error copied",
                        android.widget.Toast.LENGTH_SHORT
                    ).show()
                }
                .show()



            tvDatasetStatus.text =
                "LOAD ERROR"
        }
    }


    // =========================================================
    // RENDER REPLAY
    // =========================================================

    private fun renderReplay() {

        val d =
            dataset
                ?: run {
                    updateEmptyState()
                    return
                }


        val duration =
            datasetDurationSeconds()

        if (duration <= 0.0) {
            return
        }


        val progress =
            (
                    currentTimeSeconds /
                            duration
                    )
                .coerceIn(
                    0.0,
                    1.0
                )
                .toFloat()


        val frame =
            try {

                replaySession.sampleAt(
                    progress
                )

            } catch (e: Exception) {

                tvDatasetStatus.text =
                    "REPLAY ERROR"

                return
            }


        if (frame == null) {
            return
        }


        // -----------------------------------------------------
        // TIMELINE
        // -----------------------------------------------------

        if (!seeking) {

            replaySeekBar.progress =
                (
                        progress *
                                1000
                        )
                    .roundToInt()
        }


        tvTimeElapsed.text =
            fmtTime(
                currentTimeSeconds
            )


        tvTimeTotal.text =
            fmtTime(
                duration
            )


        // -----------------------------------------------------
        // NAVIGATION TELEMETRY
        // -----------------------------------------------------

        val solution =
            frame.solution


        tvSpeed.text =
            String.format(
                Locale.US,
                "%.1f km/h",
                solution.speedMps * 3.6
            )


        tvHeading.text =
            headingLabel(
                solution.headingDegrees
            )


        tvPositionError.text =
            frame.metrics?.let {

                String.format(
                    Locale.US,
                    "%.2f m",
                    it.positionErrorM
                )

            } ?: "—"


        tvHeadingError.text =
            frame.metrics?.let {

                String.format(
                    Locale.US,
                    "%.2f°",
                    it.headingErrorDeg
                )

            } ?: "—"


        // -----------------------------------------------------
        // METRICS
        // -----------------------------------------------------

        frame.metrics?.let {

            tvRmse.text =
                String.format(
                    Locale.US,
                    "%.2f m",
                    it.positionRmseM
                )

            tvP95.text =
                String.format(
                    Locale.US,
                    "%.2f m",
                    it.position95M
                )

            tvFinalError.text =
                String.format(
                    Locale.US,
                    "%.2f m",
                    it.finalPositionErrorM
                )

            tvSpeedMae.text =
                String.format(
                    Locale.US,
                    "%.2f km/h",
                    it.speedMaeKmh
                )
        }


        // -----------------------------------------------------
        // SYSTEM STATUS
        // -----------------------------------------------------

        chipEskf.text =
            if (solution.predictionAccepted) {
                "ESKF  ✓"
            } else {
                "ESKF  !"
            }


        chipNhc.text =
            if (solution.nhcAccepted) {
                "NHC  ✓"
            } else {
                "NHC  —"
            }


        chipZupt.text =
            if (solution.zuptAccepted) {
                "ZUPT  ✓"
            } else {
                "ZUPT  —"
            }


        chipZaru.text =
            if (solution.zaruAccepted) {
                "ZARU  ✓"
            } else {
                "ZARU  —"
            }


        chipDvfc.text =
            if (solution.dvfcActive) {
                "DVFC  ✓"
            } else {
                "DVFC  —"
            }


        // -----------------------------------------------------
        // MAP
        // -----------------------------------------------------

        updateMapFrame(
            frame
        )
    }


    // =========================================================
    // MAP — REFERENCE ROUTE
    // =========================================================

    private fun renderDatasetRoute() {

        if (!styleReady) {
            return
        }


        val d =
            dataset
                ?: return


        if (d.vehicle.size < 2) {
            return
        }


        val manager =
            trajectoryAnnotationManager
                ?: return


        val points =
            d.vehicle.map {

                Point.fromLngLat(
                    it.longitude,
                    it.latitude
                )
            }


        manager.deleteAll()


        manager.create(

            PolylineAnnotationOptions()
                .withPoints(points)
                .withLineColor(
                    "#FFFFFF"
                )
                .withLineOpacity(
                    0.70
                )
                .withLineWidth(
                    3.0
                )
        )


        fitCameraToPoints(
            points
        )
    }


    // =========================================================
    // MAP — LIVE FRAME
    // =========================================================

    private fun updateMapFrame(
        frame: ReplayFrame
    ) {

        if (!styleReady) {
            return
        }


        val manager =
            pointAnnotationManager
                ?: return


        val icon =
            vehicleIcon
                ?: return


        val estimatedPoint =
            solutionToPoint(
                frame
            )


        if (estimatedPoint != null) {

            // -------------------------------------------------
            // TRAJECTORY
            // -------------------------------------------------

            if (
                estimatedPoints.isEmpty() ||
                distanceBetweenPoints(
                    estimatedPoints.last(),
                    estimatedPoint
                ) > 0.5
            ) {

                estimatedPoints +=
                    estimatedPoint

                updateEstimatedTrajectory()
            }


            // -------------------------------------------------
            // VEHICLE MARKER
            // -------------------------------------------------

            val annotation =
                vehicleAnnotation


            if (annotation == null) {

                vehicleAnnotation =
                    manager.create(

                        PointAnnotationOptions()
                            .withPoint(
                                estimatedPoint
                            )
                            .withIconImage(
                                icon
                            )
                            .withIconRotate(
                                frame.solution
                                    .headingDegrees
                            )
                            .withIconAnchor(
                                IconAnchor.CENTER
                            )
                    )

            } else {

                annotation.point =
                    estimatedPoint

                annotation.iconRotate =
                    frame.solution
                        .headingDegrees

                manager.update(
                    annotation
                )
            }
        }
    }


    // =========================================================
    // MAP — ESTIMATED TRAJECTORY
    // =========================================================

    private fun updateEstimatedTrajectory() {

        if (!styleReady) {
            return
        }


        if (estimatedPoints.size < 2) {
            return
        }


        val manager =
            trajectoryAnnotationManager
                ?: return


        val existing =
            estimatedTrajectoryAnnotation


        if (existing == null) {

            estimatedTrajectoryAnnotation =
                manager.create(

                    PolylineAnnotationOptions()
                        .withPoints(
                            estimatedPoints.toList()
                        )
                        .withLineColor(
                            "#00E5FF"
                        )
                        .withLineOpacity(
                            0.95
                        )
                        .withLineWidth(
                            5.0
                        )
                )

        } else {

            existing.points =
                estimatedPoints.toList()

            manager.update(
                existing
            )
        }
    }


    // =========================================================
    // RESET VISUAL TRAJECTORY
    // =========================================================

    private fun resetVisualTrajectory() {

        estimatedPoints.clear()

        estimatedTrajectoryAnnotation
            ?.let { annotation ->

                trajectoryAnnotationManager
                    ?.delete(
                        annotation
                    )
            }

        estimatedTrajectoryAnnotation =
            null


        vehicleAnnotation
            ?.let { annotation ->

                pointAnnotationManager
                    ?.delete(
                        annotation
                    )
            }

        vehicleAnnotation =
            null
    }


    // =========================================================
    // SOLUTION → GEO POINT
    // =========================================================

    private fun solutionToPoint(
        frame: ReplayFrame
    ): Point? {

        if (
            !trajectoryOriginLat.isFinite() ||
            !trajectoryOriginLon.isFinite()
        ) {
            return null
        }


        val north =
            frame.solution
                .position
                .x


        val east =
            frame.solution
                .position
                .y


        val earthRadius =
            6_371_000.0


        val lat =
            trajectoryOriginLat +
                    Math.toDegrees(
                        north /
                                earthRadius
                    )


        val cosLat =
            cos(
                Math.toRadians(
                    trajectoryOriginLat
                )
            )
                .coerceAtLeast(
                    1e-8
                )


        val lon =
            trajectoryOriginLon +
                    Math.toDegrees(
                        east /
                                (
                                        earthRadius *
                                                cosLat
                                        )
                    )


        if (
            !lat.isFinite() ||
            !lon.isFinite()
        ) {
            return null
        }


        return Point.fromLngLat(
            lon,
            lat
        )
    }


    // =========================================================
    // CAMERA
    // =========================================================

    private fun fitCameraToPoints(
        points: List<Point>
    ) {

        if (points.isEmpty()) {
            return
        }


        if (points.size == 1) {

            mapView.mapboxMap.setCamera(

                CameraOptions.Builder()
                    .center(points.first())
                    .zoom(15.0)
                    .build()
            )

            return
        }


        var minLat =
            points.first().latitude()

        var maxLat =
            minLat

        var minLon =
            points.first().longitude()

        var maxLon =
            minLon


        points.forEach {

            minLat =
                minOf(
                    minLat,
                    it.latitude()
                )

            maxLat =
                maxOf(
                    maxLat,
                    it.latitude()
                )

            minLon =
                minOf(
                    minLon,
                    it.longitude()
                )

            maxLon =
                maxOf(
                    maxLon,
                    it.longitude()
                )
        }


        val bounds =
            CoordinateBounds(

                Point.fromLngLat(
                    minLon,
                    minLat
                ),

                Point.fromLngLat(
                    maxLon,
                    maxLat
                )
            )


        val camera =
            mapView.mapboxMap
                .cameraForCoordinateBounds(
                    bounds,
                    com.mapbox.maps.EdgeInsets(
                        100.0,
                        60.0,
                        420.0,
                        60.0
                    )
                )


        mapView.mapboxMap.setCamera(
            camera
        )
    }


    // =========================================================
    // EMPTY STATE
    // =========================================================

    private fun updateEmptyState() {

        tvDatasetName.text =
            "NO DATASET"

        tvDatasetStatus.text =
            "LOAD IO-VNBD S-CSV"

        tvHz.text =
            "10 Hz"

        tvSpeed.text =
            "—"

        tvHeading.text =
            "—"

        tvPositionError.text =
            "—"

        tvHeadingError.text =
            "—"

        tvRmse.text =
            "—"

        tvP95.text =
            "—"

        tvFinalError.text =
            "—"

        tvSpeedMae.text =
            "—"


        chipSensor.text =
            "SENSOR  —"

        chipRate.text =
            "10 Hz  —"

        chipEskf.text =
            "ESKF  —"

        chipNhc.text =
            "NHC  —"

        chipZupt.text =
            "ZUPT  —"

        chipZaru.text =
            "ZARU  —"

        chipDvfc.text =
            "DVFC  —"


        tvTimeElapsed.text =
            "00:00"

        tvTimeTotal.text =
            "00:00"


        replaySeekBar.progress =
            0
    }


    // =========================================================
    // HELPERS
    // =========================================================

    private fun datasetDurationSeconds():
            Double {

        return dataset
            ?.durationMs
            ?.div(
                1000.0
            )
            ?: 0.0
    }


    private fun estimateDatasetHz(
        d: ReplayDataset
    ): String {

        if (d.phone.size < 2) {
            return "—"
        }


        val duration =
            d.durationMs /
                    1000.0


        if (duration <= 0.0) {
            return "—"
        }


        val hz =
            (
                    d.phone.size - 1
                    ) /
                    duration


        return String.format(
            Locale.US,
            "%.1f Hz",
            hz
        )
    }


    private fun datasetNameFromUri(
        uri: Uri
    ): String {

        return try {

            contentResolver
                .query(
                    uri,
                    arrayOf(
                        android.provider.OpenableColumns.DISPLAY_NAME
                    ),
                    null,
                    null,
                    null
                )
                ?.use { cursor ->

                    if (
                        cursor.moveToFirst()
                    ) {

                        cursor.getString(
                            0
                        )
                    } else {
                        "IO-VNBD"
                    }
                }
                ?: "IO-VNBD"

        } catch (_: Exception) {

            "IO-VNBD"
        }
    }


    private fun tryTakePersistablePermission(
        uri: Uri
    ) {

        try {

            contentResolver
                .takePersistableUriPermission(
                    uri,
                    android.content.Intent.FLAG_GRANT_READ_URI_PERMISSION
                )

        } catch (_: Exception) {
            // Some document providers do not
            // expose persistable permissions.
        }
    }


    private fun distanceBetweenPoints(
        a: Point,
        b: Point
    ): Double {

        val lat =
            Math.toRadians(
                (
                        a.latitude() +
                                b.latitude()
                        ) /
                        2.0
            )


        val dx =
            (
                    b.longitude() -
                            a.longitude()
                    ) *
                    111_320.0 *
                    cos(lat)


        val dy =
            (
                    b.latitude() -
                            a.latitude()
                    ) *
                    110_540.0


        return kotlin.math.sqrt(
            dx * dx +
                    dy * dy
        )
    }


    private fun vectorToBitmap(
        resId: Int
    ): Bitmap {

        val drawable =
            ContextCompat.getDrawable(
                this,
                resId
            )
                ?: throw IllegalStateException(
                    "Vehicle marker drawable missing"
                )


        val width =
            drawable.intrinsicWidth
                .coerceAtLeast(1)

        val height =
            drawable.intrinsicHeight
                .coerceAtLeast(1)


        val bitmap =
            Bitmap.createBitmap(
                width,
                height,
                Bitmap.Config.ARGB_8888
            )


        val canvas =
            Canvas(bitmap)


        drawable.setBounds(
            0,
            0,
            canvas.width,
            canvas.height
        )


        drawable.draw(
            canvas
        )


        return bitmap
    }


    private fun headingLabel(
        degrees: Double
    ): String {

        val normalized =
            (
                    degrees % 360.0 +
                            360.0
                    ) %
                    360.0


        val headings =
            arrayOf(
                "N",
                "NE",
                "E",
                "SE",
                "S",
                "SW",
                "W",
                "NW"
            )


        val index =
            (
                    (
                            normalized /
                                    45.0
                            )
                        .roundToInt()
                        .mod(8)
                    )


        return String.format(
            Locale.US,
            "%s · %03d°",
            headings[index],
            normalized.roundToInt()
        )
    }


    private fun fmtTime(
        seconds: Double
    ): String {

        val s =
            seconds
                .roundToInt()
                .coerceAtLeast(0)


        return String.format(
            Locale.US,
            "%02d:%02d",
            s / 60,
            s % 60
        )
    }


    private fun formatSpeed(
        speed: Double
    ): String {

        return if (
            speed == speed.toInt().toDouble()
        ) {

            speed.toInt().toString()

        } else {

            String.format(
                Locale.US,
                "%.1f",
                speed
            )
        }
    }


    // =========================================================
    // LIFECYCLE
    // =========================================================

    override fun onResume() {

        super.onResume()

        lastTickNanos =
            0L

        handler.post(
            tickRunnable
        )
    }


    override fun onPause() {

        super.onPause()

        handler.removeCallbacks(
            tickRunnable
        )

        lastTickNanos =
            0L
    }


    override fun onDestroy() {

        handler.removeCallbacks(
            tickRunnable
        )

        navigationEngine.stop()

        mapView.onDestroy()

        super.onDestroy()
    }
}