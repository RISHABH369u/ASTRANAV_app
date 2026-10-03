package com.rishabh.astranav

import android.content.Intent
import android.content.SharedPreferences
import android.os.Bundle
import android.view.View
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import com.rishabh.astranav.dvfc.DvfcCalibrationStore

/**
 * Settings, wired to real state instead of the mock `settingsSections` data
 * the design reference used: every switch here is a genuine persisted user
 * preference, and every info row states an actual fact about this build
 * (model names, calibration status, app version) rather than an invented
 * number. Rows for capabilities that aren't wired into the app yet — offline
 * maps, external IMU — say so instead of showing a toggle that would quietly
 * do nothing.
 */
class SettingsActivity : AppCompatActivity() {

    companion object {
        const val PREFS_NAME = "astranav_settings"
        const val KEY_NHC = "pref_nhc_enabled"
        const val KEY_MAP_MATCHING = "pref_map_matching_enabled"
    }

    private lateinit var prefs: SharedPreferences

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_settings)

        prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)

        findViewById<View>(R.id.btnBack).setOnClickListener { finish() }

        val switchNhc = findViewById<SwitchCompat>(R.id.switchNhc)
        switchNhc.isChecked = prefs.getBoolean(KEY_NHC, true) // NonHolonomicConstraint is on by default in NavigationEngine
        switchNhc.setOnCheckedChangeListener { _, checked ->
            prefs.edit().putBoolean(KEY_NHC, checked).apply()
        }

        val switchMapMatching = findViewById<SwitchCompat>(R.id.switchMapMatching)
        switchMapMatching.isChecked = prefs.getBoolean(KEY_MAP_MATCHING, false)
        switchMapMatching.setOnCheckedChangeListener { _, checked ->
            prefs.edit().putBoolean(KEY_MAP_MATCHING, checked).apply()
        }

        // External IMU: disabled switch, no listener — there's no BLE/external
        // sensor pairing implemented anywhere in this codebase yet.
        findViewById<SwitchCompat>(R.id.switchExternalImu).isChecked = false

        findViewById<View>(R.id.rowCalibration).setOnClickListener {
            startActivity(Intent(this, DVFCActivity::class.java))
        }

        findViewById<TextView>(R.id.rowVersion).text = versionLabel()
    }

    override fun onResume() {
        super.onResume()
        // Calibration status can change in DVFCActivity between visits, so
        // refresh this row every time Settings comes back to the foreground.
        val calibrated = DvfcCalibrationStore.load(this) != null
        findViewById<TextView>(R.id.rowCalibrationCaption).text =
            if (calibrated) "Calibrated — tap to redo" else "Not calibrated yet — tap to run"
    }

    private fun versionLabel(): String {
        return try {
            val info = packageManager.getPackageInfo(packageName, 0)
            "${info.versionName}"
        } catch (_: Exception) {
            "—"
        }
    }
}
