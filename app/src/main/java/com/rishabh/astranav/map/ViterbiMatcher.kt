package com.rishabh.astranav.map
class ViterbiMatcher {
    fun best(candidates:List<RoadCandidate>):RoadCandidate? = candidates.maxByOrNull{it.score}
}
