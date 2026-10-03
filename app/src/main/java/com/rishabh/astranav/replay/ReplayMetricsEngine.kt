package com.rishabh.astranav.replay
import kotlin.math.*
data class ReplayMetricSnapshot(
 val positionErrorM:Double=0.0,val speedErrorKmh:Double=0.0,val headingErrorDeg:Double=0.0,
 val samples:Int=0,val positionRmseM:Double=0.0,val positionMaeM:Double=0.0,val position95M:Double=0.0,
 val finalPositionErrorM:Double=0.0,val speedMaeKmh:Double=0.0,val headingMaeDeg:Double=0.0)
class ReplayMetricsEngine{
 private var lat0=Double.NaN;private var lon0=Double.NaN
 private val pe=ArrayList<Double>();private val se=ArrayList<Double>();private val he=ArrayList<Double>()
 fun reset(){lat0=Double.NaN;lon0=Double.NaN;pe.clear();se.clear();he.clear()}
 fun update(n:Double,e:Double,speedMps:Double,heading:Double,t:IovnbdVehicleSample):ReplayMetricSnapshot{
  if(!lat0.isFinite()){lat0=t.latitude;lon0=t.longitude}
  val (tn,te)=local(t.latitude,t.longitude);val p=hypot(n-tn,e-te)
  val s=abs(speedMps*3.6-t.velocityKmh);val h=ang(heading,t.headingDeg);pe+=p;se+=s;he+=h
  return ReplayMetricSnapshot(p,s,h,pe.size,sqrt(pe.map{it*it}.average()),pe.average(),pct(pe,.95),p,se.average(),he.average())
 }
 private fun local(lat:Double,lon:Double):Pair<Double,Double>{val r=6371000.0;val dl=Math.toRadians(lat-lat0);val dn=Math.toRadians(lon-lon0);val m=Math.toRadians((lat+lat0)/2);return r*dl to r*cos(m)*dn}
 private fun ang(a:Double,b:Double):Double{var d=(a-b)%360.0;if(d>180)d-=360.0;if(d< -180)d+=360.0;return abs(d)}
 private fun pct(v:List<Double>,p:Double):Double{if(v.isEmpty())return 0.0;return v[((v.size-1)*p).roundToInt().coerceIn(0,v.lastIndex)]}
}