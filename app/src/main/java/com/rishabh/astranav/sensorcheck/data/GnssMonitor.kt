package com.rishabh.astranav.sensorcheck.data

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.GnssStatus
import android.location.LocationManager
import android.os.Build
import androidx.annotation.RequiresApi
import androidx.core.content.ContextCompat

/**
 * Live GNSS satellite tracking, used to upgrade the GNSS row from
 * "Searching…" to a real "used/visible satellites" reading once a fix comes
 * in — mirrors what a real nav app means by "GNSS availability".
 *
 * Requires ACCESS_FINE_LOCATION at runtime and API 24+ (GnssStatus.Callback).
 * Call [start] once the permission is granted; always call [stop] from
 * onDestroy to avoid leaking the callback.
 */
@RequiresApi(Build.VERSION_CODES.N)
class GnssMonitor(
    private val context: Context,
    private val onUpdate: (satellitesUsed: Int, satellitesInView: Int) -> Unit,
) {
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private var callback: GnssStatus.Callback? = null

    fun hasPermission(): Boolean = ContextCompat.checkSelfPermission(
        context,
        Manifest.permission.ACCESS_FINE_LOCATION,
    ) == PackageManager.PERMISSION_GRANTED

    fun start() {
        if (!hasPermission()) return
        stop()
        val cb = object : GnssStatus.Callback() {
            override fun onSatelliteStatusChanged(status: GnssStatus) {
                var used = 0
                val total = status.satelliteCount
                for (i in 0 until total) {
                    if (status.usedInFix(i)) used++
                }
                onUpdate(used, total)
            }
        }
        callback = cb
        try {
            locationManager.registerGnssStatusCallback(cb, null)
        } catch (_: SecurityException) {
            // Permission was revoked between the check above and registering.
        }
    }

    fun stop() {
        callback?.let { locationManager.unregisterGnssStatusCallback(it) }
        callback = null
    }
}
