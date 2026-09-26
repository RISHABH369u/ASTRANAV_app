package com.rishabh.astranav.sensorcheck.ui.adapter

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ProgressBar
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.recyclerview.widget.RecyclerView
import com.rishabh.astranav.R
import com.rishabh.astranav.sensorcheck.model.AvailabilitySensor
import com.rishabh.astranav.sensorcheck.model.RowState
import com.rishabh.astranav.sensorcheck.model.colorRes
import com.rishabh.astranav.sensorcheck.model.toHealth
import com.rishabh.astranav.sensorcheck.ui.view.StatusDotView

/**
 * Backs the sensor-row list. `states` is read live off the Activity/ViewModel —
 * call notifyItemChanged(i) / notifyDataSetChanged() when it mutates, exactly
 * like the React re-render driven by `setStates`.
 *
 * `liveDetails` is separate from `sensors[i].detail` because some rows (GNSS)
 * keep updating their detail text after they first resolve — e.g.
 * "Searching…" → "7/12 sats" as GnssMonitor reports fixes. IMU rows just
 * keep their initial probed "xxHz" value here.
 */
class SensorRowAdapter(
    private val sensors: List<AvailabilitySensor>,
    private val states: List<RowState>,
    private val liveDetails: List<String?> = sensors.map { it.detail },
    private val onDiagnosticsClick: () -> Unit,
) : RecyclerView.Adapter<SensorRowAdapter.RowHolder>() {

    class RowHolder(view: View) : RecyclerView.ViewHolder(view) {
        val root: LinearLayout = view.findViewById(R.id.rowRoot)
        val icon: ImageView = view.findViewById(R.id.sensorIcon)
        val label: TextView = view.findViewById(R.id.sensorLabel)
        val detail: TextView = view.findViewById(R.id.sensorDetail)
        val sub: TextView = view.findViewById(R.id.sensorSub)
        val spinner: ProgressBar = view.findViewById(R.id.rowSpinner)
        val check: ImageView = view.findViewById(R.id.rowCheck)
        val dot: StatusDotView = view.findViewById(R.id.rowDot)
        val statusText: TextView = view.findViewById(R.id.rowStatusText)
        val alertPanel: LinearLayout = view.findViewById(R.id.alertPanel)
        val alertMessage: TextView = view.findViewById(R.id.alertMessage)
        val alertDiagnosticsLink: TextView = view.findViewById(R.id.alertDiagnosticsLink)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): RowHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.item_sensor_row, parent, false)
        return RowHolder(view)
    }

    override fun getItemCount() = sensors.size

    override fun onBindViewHolder(holder: RowHolder, position: Int) {
        val def = sensors[position]
        val state = states[position]
        val ctx = holder.itemView.context
        val resolved = state == RowState.AVAILABLE || state == RowState.DEGRADED || state == RowState.UNAVAILABLE
        val showAlert = state == RowState.UNAVAILABLE

        holder.icon.setImageResource(def.icon)
        holder.label.text = def.label
        holder.sub.text = def.sub

        val detailText = liveDetails.getOrNull(position)
        holder.detail.visibility =
            if (detailText != null && resolved && state != RowState.UNAVAILABLE) View.VISIBLE else View.GONE
        holder.detail.text = detailText

        holder.root.alpha = if (state == RowState.PENDING) 0.4f else 1f
        holder.root.setBackgroundResource(
            when {
                showAlert && def.required -> R.drawable.bg_row_alert_crit
                showAlert && !def.required -> R.drawable.bg_row_alert_warn
                else -> R.drawable.bg_row_default
            },
        )

        holder.spinner.visibility = if (state == RowState.CHECKING) View.VISIBLE else View.GONE
        holder.check.visibility = if (resolved && state == RowState.AVAILABLE) View.VISIBLE else View.GONE
        holder.dot.visibility = if (resolved && state != RowState.AVAILABLE) View.VISIBLE else View.GONE
        if (resolved && state != RowState.AVAILABLE) {
            holder.dot.health = state.toHealth()
        }

        val statusLabel = when (state) {
            RowState.PENDING -> ""
            RowState.CHECKING -> "Checking"
            RowState.AVAILABLE -> "Ready"
            RowState.DEGRADED -> "Limited"
            RowState.UNAVAILABLE -> "Unavailable"
        }
        holder.statusText.text = statusLabel
        holder.statusText.visibility = if (statusLabel.isEmpty()) View.INVISIBLE else View.VISIBLE
        holder.statusText.setTextColor(
            ContextCompat.getColor(
                ctx,
                if (state == RowState.CHECKING) R.color.fg_dim else state.toHealth().colorRes(),
            ),
        )

        holder.alertPanel.visibility = if (showAlert) View.VISIBLE else View.GONE
        if (showAlert) {
            holder.alertPanel.setBackgroundResource(
                if (def.required) R.drawable.bg_alert_panel_crit else R.drawable.bg_alert_panel_warn,
            )
            holder.alertMessage.text = if (def.required) {
                "Your device may not support the required motion sensing. Navigation will run with reduced accuracy."
            } else {
                "${def.label} unavailable — heading confidence may be reduced."
            }
            holder.alertDiagnosticsLink.visibility = if (def.required) View.VISIBLE else View.GONE
            holder.alertDiagnosticsLink.setOnClickListener { onDiagnosticsClick() }
        }
    }
}
