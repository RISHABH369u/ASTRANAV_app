package com.rishabh.astranav.gnss

import com.rishabh.astranav.navigation.ImuMechanization
import com.rishabh.astranav.sensor.GnssSample
import kotlin.math.cos

class GnssFusion {
    private var originLat: Double? = null
    private var originLon: Double? = null

    fun setOrigin(latDeg: Double, lonDeg: Double) {
        originLat = latDeg
        originLon = lonDeg
    }

    fun correction(mech: ImuMechanization, gnss: GnssSample): Boolean {
        if (gnss.accuracyM > 30.0) return false
        if (originLat == null) setOrigin(gnss.latitudeDeg, gnss.longitudeDeg)
        val lat0 = Math.toRadians(originLat!!)
        val dLat = Math.toRadians(gnss.latitudeDeg - originLat!!)
        val dLon = Math.toRadians(gnss.longitudeDeg - originLon!!)
        val earth = 6378137.0
        val north = dLat * earth
        val east = dLon * earth * cos(lat0)

        val alpha = (20.0 / (20.0 + gnss.accuracyM)).coerceIn(0.05, 0.8)
        mech.state.east += (east - mech.state.east) * alpha
        mech.state.north += (north - mech.state.north) * alpha

        if (gnss.speedMps.isFinite() && gnss.speedMps >= 0.0) {
            val bearing = Math.toRadians(gnss.bearingDeg)
            val targetVe = gnss.speedMps * kotlin.math.sin(bearing)
            val targetVn = gnss.speedMps * cos(bearing)
            mech.state.ve += (targetVe - mech.state.ve) * alpha
            mech.state.vn += (targetVn - mech.state.vn) * alpha
        }
        return true
    }
}
