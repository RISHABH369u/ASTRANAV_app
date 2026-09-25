package com.rishabh.astranav

import com.rishabh.astranav.calibration.VehicleAlignmentCalibrator
import com.rishabh.astranav.integrity.NisMonitor
import com.rishabh.astranav.navigation.NavigationModeManager
import org.junit.Assert.*
import org.junit.Test

class NavigationCoreTest {
    @Test fun alignmentConverges() {
        val c=VehicleAlignmentCalibrator()
        repeat(20){ c.add(10.0,40.0,10.0,3.0) }
        assertEquals(30.0,c.result().yawOffsetDeg,1.0)
    }
    @Test fun innovationRejectsLargeResidual() {
        assertFalse(NisMonitor().scalar(20.0,1.0).accepted)
    }
    @Test fun modeTransitionsToDenied() {
        val m=NavigationModeManager()
        m.update(0,true,5.0)
        assertEquals(com.rishabh.astranav.navigation.NavigationMode.GNSS_DENIED,m.update(5000,false,999.0))
    }
}
