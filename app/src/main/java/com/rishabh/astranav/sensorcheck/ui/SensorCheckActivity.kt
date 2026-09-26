package com.rishabh.astranav.sensorcheck.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.rishabh.astranav.R
import com.rishabh.astranav.sensorcheck.data.GnssMonitor
import com.rishabh.astranav.sensorcheck.data.SensorCapabilityProbe
import com.rishabh.astranav.sensorcheck.model.Health
import com.rishabh.astranav.sensorcheck.model.RowState
import com.rishabh.astranav.sensorcheck.model.toHealth
import com.rishabh.astranav.sensorcheck.model.toRowState
import com.rishabh.astranav.sensorcheck.ui.adapter.SensorRowAdapter
import com.rishabh.astranav.sensorcheck.ui.view.PhoneScanView
import com.rishabh.astranav.sensorcheck.ui.view.StatusDotView

/**
 * Kotlin/View port of the React `SensorCheck` screen. The Handler-based
 * schedule below is a 1:1 translation of the original `useEffect`'s
 * setTimeout sequence: sensors resolve one at a time, then a hard 6s
 * failsafe force-resolves anything left pending/checking. All timers are
 * tracked and cancelled in onDestroy, same as the effect's cleanup fn.
 *
 * Sensor list now comes from [SensorCapabilityProbe] — real IMU Hz read off
 * SensorManager, plus a GNSS row backed by [GnssMonitor] for a live
 * used/visible satellite count once a fix comes in.
 */
class SensorCheckActivity : AppCompatActivity() {

    private val handler = Handler(Looper.getMainLooper())
    private val pendingRunnables = mutableListOf<Runnable>()

    private val sensors by lazy { SensorCapabilityProbe.probe(this) }
    private val states by lazy { MutableList(sensors.size) { RowState.PENDING } }
    private val liveDetails by lazy { MutableList(sensors.size) { sensors[it].detail } }
    private var done = false

    private var gnssMonitor: GnssMonitor? = null
    private val gnssIndex by lazy { sensors.indexOfFirst { it.key == "gnss" } }

    private lateinit var adapter: SensorRowAdapter
    private lateinit var phoneScan: PhoneScanView
    private lateinit var summaryCard: LinearLayout
    private lateinit var summarySpinner: ProgressBar
    private lateinit var summaryDot: StatusDotView
    private lateinit var summaryText: TextView
    private lateinit var actionContainer: LinearLayout
    private lateinit var btnContinue: Button
    private lateinit var btnDiagnostics: Button

    private val locationPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) {
                startGnssMonitor()
            } else if (gnssIndex >= 0) {
                liveDetails[gnssIndex] = "Permission needed"
                adapter.notifyItemChanged(gnssIndex)
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_sensor_check)

        phoneScan = findViewById(R.id.phoneScan)
        summaryCard = findViewById(R.id.summaryCard)
        summarySpinner = findViewById(R.id.summarySpinner)
        summaryDot = findViewById(R.id.summaryDot)
        summaryText = findViewById(R.id.summaryText)
        actionContainer = findViewById(R.id.actionContainer)
        btnContinue = findViewById(R.id.btnContinue)
        btnDiagnostics = findViewById(R.id.btnDiagnostics)

        val list = findViewById<RecyclerView>(R.id.sensorList)
        adapter = SensorRowAdapter(sensors, states, liveDetails) { onDiagnostics() }
        list.layoutManager = LinearLayoutManager(this)
        list.adapter = adapter

        btnContinue.setOnClickListener { onContinue() }
        btnDiagnostics.setOnClickListener { onDiagnostics() }

        phoneScan.setScanning(true)
        refreshSummary()
        updateActionButtons()
        scheduleSequence()
        requestGnssPermissionIfNeeded()
    }

    private fun schedule(delayMs: Long, action: () -> Unit) {
        val runnable = Runnable(action)
        pendingRunnables += runnable
        handler.postDelayed(runnable, delayMs)
    }

    /** Mirrors the deterministic setTimeout chain built inside the original useEffect. */
    private fun scheduleSequence() {
        var t = 400L
        sensors.forEachIndexed { i, def ->
            schedule(t) { setRowState(i, RowState.CHECKING) }
            schedule(t + 260) { setRowState(i, def.result.toRowState()) }
            t += 520
        }
        schedule(t + 200) { complete(force = false) }

        // Guaranteed upper bound — the screen can never sit checking longer than this.
        schedule(HARD_TIMEOUT_MS) { complete(force = true) }
    }

    private fun setRowState(index: Int, newState: RowState) {
        states[index] = newState
        adapter.notifyItemChanged(index)
        refreshSummary()
    }

    private fun complete(force: Boolean) {
        if (done) return
        if (force) {
            states.forEachIndexed { i, s ->
                if (s == RowState.PENDING || s == RowState.CHECKING) {
                    states[i] = sensors[i].result.toRowState()
                }
            }
            adapter.notifyDataSetChanged()
        }
        done = true
        phoneScan.setScanning(false)
        refreshSummary()
        updateActionButtons()
    }

    // ---- GNSS live updates -------------------------------------------------

    private fun requestGnssPermissionIfNeeded() {
        if (gnssIndex < 0 || Build.VERSION.SDK_INT < Build.VERSION_CODES.N) return
        val granted = ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION,
        ) == PackageManager.PERMISSION_GRANTED
        if (granted) startGnssMonitor() else locationPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
    }

    private fun startGnssMonitor() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) return
        gnssMonitor = GnssMonitor(this) { used, total -> onGnssUpdate(used, total) }.also { it.start() }
    }

    /** Only applied once the row's own reveal animation has finished, so it never fights the checking spinner. */
    private fun onGnssUpdate(used: Int, total: Int) {
        if (gnssIndex < 0) return
        val current = states[gnssIndex]
        if (current == RowState.PENDING || current == RowState.CHECKING) return

        states[gnssIndex] = if (used > 0) RowState.AVAILABLE else RowState.DEGRADED
        liveDetails[gnssIndex] = if (total == 0) "Searching…" else "$used/$total sats"
        adapter.notifyItemChanged(gnssIndex)
        refreshSummary()
        updateActionButtons()
    }

    // ---- Summary / actions --------------------------------------------------

    private fun isBlocked() =
        sensors.indices.any { i -> sensors[i].required && states[i] == RowState.UNAVAILABLE }

    private fun anyLimited() =
        states.any { it == RowState.DEGRADED } ||
            sensors.indices.any { i -> !sensors[i].required && states[i] == RowState.UNAVAILABLE }

    private fun refreshSummary() {
        val blocked = isBlocked()
        val limited = anyLimited()

        summaryCard.setBackgroundResource(
            when {
                blocked -> R.drawable.bg_summary_crit
                limited -> R.drawable.bg_summary_warn
                else -> R.drawable.bg_summary_default
            },
        )

        summarySpinner.visibility = if (!done) View.VISIBLE else View.GONE
        summaryDot.visibility = if (done) View.VISIBLE else View.GONE
        if (done) {
            summaryDot.health = if (blocked) Health.CRIT else if (limited) Health.WARN else Health.GOOD
            summaryDot.setPulse(!blocked)
        }

        summaryText.text = when {
            !done -> "Checking sensors…"
            blocked -> "A required sensor is unavailable"
            limited -> "Ready with limited accuracy"
            else -> "All systems ready"
        }
    }

    private fun updateActionButtons() {
        actionContainer.animate().alpha(if (done) 1f else 0.5f).setDuration(300).start()
        setViewsEnabled(actionContainer, done)

        val blocked = isBlocked()
        btnContinue.text = if (blocked) "Continue with limited accuracy" else "Continue"
        btnContinue.setCompoundDrawablesRelativeWithIntrinsicBounds(
            ContextCompat.getDrawable(this, if (blocked) R.drawable.ic_check else R.drawable.ic_chevron_right),
            null, null, null,
        )
        btnDiagnostics.visibility = if (blocked) View.VISIBLE else View.GONE
    }

    private fun setViewsEnabled(view: View, enabled: Boolean) {
        view.isEnabled = enabled
        if (view is ViewGroup) {
            for (i in 0 until view.childCount) setViewsEnabled(view.getChildAt(i), enabled)
        }
    }

    private fun onContinue() {
        android.util.Log.d(TAG, "Sensor check complete → opening alignment")
        // TODO: navigate to the alignment screen, e.g.:
        // startActivity(Intent(this, AlignmentActivity::class.java))
    }

    private fun onDiagnostics() {
        // TODO: navigate to the diagnostics screen.
    }

    override fun onDestroy() {
        super.onDestroy()
        pendingRunnables.forEach(handler::removeCallbacks)
        pendingRunnables.clear()
        gnssMonitor?.stop()
    }

    companion object {
        private const val TAG = "SensorCheck"
        private const val HARD_TIMEOUT_MS = 6000L
    }
}
