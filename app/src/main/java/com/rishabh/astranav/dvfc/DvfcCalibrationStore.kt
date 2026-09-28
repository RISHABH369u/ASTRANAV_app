package com.rishabh.astranav.dvfc

import android.content.Context
import com.rishabh.astranav.dvfc.math.Quat

/**
 * Persists the locked Device→Vehicle transform across app restarts.
 *
 * DVFCController (and the math it wraps) only ever lived in memory for the
 * lifetime of DVFCActivity — nothing wrote the result anywhere durable, so
 * there was no way for another screen (like Home) to know "is this device
 * actually calibrated" without re-running DVFC. This is a minimal
 * SharedPreferences-backed store for exactly that one fact.
 *
 * DVFCController.kt needs one addition to use this — see
 * PATCH_INSTRUCTIONS.md for the exact 3-line diff (save on COMPLETE).
 */
object DvfcCalibrationStore {
    private const val PREFS_NAME = "astranav_dvfc_calibration"
    private const val KEY_X = "locked_x"
    private const val KEY_Y = "locked_y"
    private const val KEY_Z = "locked_z"
    private const val KEY_W = "locked_w"
    private const val KEY_LOCKED_AT_MS = "locked_at_ms"

    fun save(context: Context, transform: Quat) {
        prefs(context).edit()
            .putFloat(KEY_X, transform.x)
            .putFloat(KEY_Y, transform.y)
            .putFloat(KEY_Z, transform.z)
            .putFloat(KEY_W, transform.w)
            .putLong(KEY_LOCKED_AT_MS, System.currentTimeMillis())
            .apply()
    }

    /** Null if the device has never completed DVFC calibration. */
    fun load(context: Context): Quat? {
        val p = prefs(context)
        if (!p.contains(KEY_W)) return null
        return Quat(
            x = p.getFloat(KEY_X, 0f),
            y = p.getFloat(KEY_Y, 0f),
            z = p.getFloat(KEY_Z, 0f),
            w = p.getFloat(KEY_W, 1f),
        )
    }

    fun lockedAtMillis(context: Context): Long? {
        val p = prefs(context)
        return if (p.contains(KEY_LOCKED_AT_MS)) p.getLong(KEY_LOCKED_AT_MS, 0L) else null
    }

    fun clear(context: Context) {
        prefs(context).edit().clear().apply()
    }

    private fun prefs(context: Context) =
        context.applicationContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
}
