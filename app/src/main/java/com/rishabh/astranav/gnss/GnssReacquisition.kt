package com.rishabh.astranav.gnss
class GnssReacquisition {
    fun shouldFuse(accuracyM:Double, innovationM:Double):Boolean = accuracyM<20.0 && innovationM<50.0
}
