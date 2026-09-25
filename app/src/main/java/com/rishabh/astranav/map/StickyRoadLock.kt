package com.rishabh.astranav.map
class StickyRoadLock(private val switchMargin:Double=1.5) {
    private var locked:RoadCandidate?=null
    fun update(candidates:List<RoadCandidate>):RoadCandidate? {
        val best=candidates.maxByOrNull{it.score} ?: return locked
        val old=locked
        locked=if(old==null || best.id==old.id || best.score>old.score+switchMargin) best else old
        return locked
    }
}
