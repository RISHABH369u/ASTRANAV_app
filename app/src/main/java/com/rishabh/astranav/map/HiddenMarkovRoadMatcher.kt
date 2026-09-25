package com.rishabh.astranav.map
class HiddenMarkovRoadMatcher {
    fun score(distanceM:Double,headingErrorDeg:Double,speedErrorMps:Double):Double =
        -distanceM.coerceAtLeast(0.0)/15.0 - kotlin.math.abs(headingErrorDeg)/30.0 - kotlin.math.abs(speedErrorMps)/5.0
}
