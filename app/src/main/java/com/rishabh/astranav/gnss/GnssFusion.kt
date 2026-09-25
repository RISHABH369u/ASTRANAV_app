package com.rishabh.astranav.gnss
import com.rishabh.astranav.navigation.ImuMechanization
import com.rishabh.astranav.sensor.GnssSample
class GnssFusion {
    fun correction(mech:ImuMechanization,gnss:GnssSample):Boolean {
        if(gnss.accuracyM>30.0) return false
        mech.state.east = mech.state.east
        mech.state.north = mech.state.north
        return true
    }
}
