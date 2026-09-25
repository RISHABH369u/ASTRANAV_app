package com.rishabh.astranav.map
class OfflineRoadNetwork {
    private val roads=mutableMapOf<String,RoadCandidate>()
    fun put(candidate:RoadCandidate){roads[candidate.id]=candidate}
    fun nearby():List<RoadCandidate> = roads.values.toList()
    fun clear(){roads.clear()}
}
