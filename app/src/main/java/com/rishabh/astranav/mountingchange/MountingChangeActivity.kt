package com.rishabh.astranav.mountingchange

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.rishabh.astranav.DVFCActivity
import com.rishabh.astranav.R
import java.util.Locale

/**
 * Kotlin/XML port of the MountingChange.tsx warning screen shown to
 * returning users when the mounting orientation looks different from the
 * last locked DVFC transform.
 *
 * This screen does not itself detect the change or run any calibration —
 * it's a decision prompt. Whoever launches it (e.g. a startup check that
 * compares the newly-fused device heading against the last saved
 * `DeviceVehicleTransform.lockedTransform`) passes the two offsets in via
 * Intent extras.
 */
class MountingChangeActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_mounting_change)

        val savedOffsetDeg = intent.getFloatExtra(EXTRA_SAVED_OFFSET_DEG, 0f)
        val detectedOffsetDeg = intent.getFloatExtra(EXTRA_DETECTED_OFFSET_DEG, savedOffsetDeg)

        findViewById<TextView>(R.id.savedOffsetValue).text = formatDeg(savedOffsetDeg)
        findViewById<TextView>(R.id.detectedOffsetValue).text = formatDeg(detectedOffsetDeg)

        findViewById<android.widget.Button>(R.id.btnRecalibrateNow).setOnClickListener {
            startActivity(Intent(this, DVFCActivity::class.java))
            finish()
        }
        findViewById<android.widget.Button>(R.id.btnContinueAnyway).setOnClickListener {
            finish()
        }
    }

    private fun formatDeg(v: Float) = String.format(Locale.US, "%.1f°", v)

    companion object {
        private const val EXTRA_SAVED_OFFSET_DEG = "saved_offset_deg"
        private const val EXTRA_DETECTED_OFFSET_DEG = "detected_offset_deg"

        fun intent(context: Context, savedOffsetDeg: Float, detectedOffsetDeg: Float): Intent =
            Intent(context, MountingChangeActivity::class.java)
                .putExtra(EXTRA_SAVED_OFFSET_DEG, savedOffsetDeg)
                .putExtra(EXTRA_DETECTED_OFFSET_DEG, detectedOffsetDeg)
    }
}
