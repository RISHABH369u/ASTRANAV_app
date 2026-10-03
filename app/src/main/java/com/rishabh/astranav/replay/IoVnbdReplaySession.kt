package com.rishabh.astranav.replay
import android.content.Context
import android.net.Uri
import com.rishabh.astranav.dvfc.math.Quat
import com.rishabh.astranav.dvfc.sensor.DeviceSensorSample
import com.rishabh.astranav.navigation.AstraNavigationEngine
import com.rishabh.astranav.navigation.NavigationSolution
import kotlin.math.sqrt

data class ReplayDataset(val id:String,val phone:List<IovnbdPhoneSample>,val vehicle:List<IovnbdVehicleSample>){
 val durationMs:Long get()=if(phone.size<2)0L else phone.last().timeMs-phone.first().timeMs
}
data class ReplayFrame(val phone:IovnbdPhoneSample,val vehicle:IovnbdVehicleSample?,val solution:NavigationSolution,val metrics:ReplayMetricSnapshot?)

class IoVnbdReplaySession(context:Context,private val engine:AstraNavigationEngine){
 private val parser=IovnbdCsvParser(context);val metrics=ReplayMetricsEngine()
 private var ds:ReplayDataset?=null;private var index=-1;private var truthIndex=0;private var metricIndex=-1
 fun load(phoneUri:Uri,vehicleUri:Uri?,id:String="IO-VNBD"):ReplayDataset{
  val p=parser.readPhone(phoneUri);require(p.isNotEmpty()){"No valid smartphone samples"}
  val v=vehicleUri?.let{parser.readVehicle(it)}?:emptyList()
  ds=ReplayDataset(id,p,v);reset();return ds!!
 }
 fun reset(){index=-1;truthIndex=0;metricIndex=-1;metrics.reset();engine.resetForReplay()}
 fun sampleAt(progress:Float):ReplayFrame?{
  val d=ds?:return null;if(d.phone.isEmpty())return null
  val start=d.phone.first().timeMs;val target=start+(d.durationMs*progress.coerceIn(0f,1f)).toLong()
  val targetIdx=d.phone.indexOfLast{it.timeMs<=target}.coerceAtLeast(0)
  if(targetIdx<index)reset()
  while(index<targetIdx){index++;engine.processReplaySample(toDevice(d.phone[index]))}
  if(index<0){index=0;engine.processReplaySample(toDevice(d.phone[0]))}
  val p=d.phone[index]
  while(truthIndex+1<d.vehicle.size && relMs(d.vehicle[truthIndex+1],d.vehicle)<=p.timeMs-start)truthIndex++
  val truth=d.vehicle.getOrNull(truthIndex);val sol=engine.state()
  val met=if(truth!=null && index!=metricIndex){metricIndex=index;metrics.update(sol.position.x,sol.position.y,sol.speedMps,sol.headingDegrees,truth)}else null
  return ReplayFrame(p,truth,sol,met)
 }
 private fun relMs(v:IovnbdVehicleSample):Long{val d=ds?:return 0L;return if(d.vehicle.isEmpty())0L else ((v.timeSeconds-d.vehicle.first().timeSeconds)*1000.0).toLong()}
 private fun toDevice(p:IovnbdPhoneSample):DeviceSensorSample{
  val g=floatArrayOf(p.gravityX.toFloat(),p.gravityY.toFloat(),p.gravityZ.toFloat())
  val a=floatArrayOf(p.accelX.toFloat(),p.accelY.toFloat(),p.accelZ.toFloat())
  val lin=floatArrayOf((p.accelX-p.gravityX).toFloat(),(p.accelY-p.gravityY).toFloat(),(p.accelZ-p.gravityZ).toFloat())
  return DeviceSensorSample(
   timestampNs=p.timeMs*1_000_000L,acceleration=a,
   angularVelocity=floatArrayOf(p.gyroYaw.toFloat(),p.gyroPitch.toFloat(),p.gyroRoll.toFloat()),
   gravity=g,linearAcceleration=lin,quaternion=Quat.IDENTITY,
   gravityMagnitude=sqrt(p.gravityX*p.gravityX+p.gravityY*p.gravityY+p.gravityZ*p.gravityZ).toFloat(),
   gravityStable=true,gravityLevelRollDeg=p.orientationRollDeg?.toFloat()?:0f,gravityLevelPitchDeg=p.orientationPitchDeg?.toFloat()?:0f,
   estimatedSampleHz=10f,timestampJitterMs=0f,dataGapCount=0,maxGapMs=0f,duplicateTimestampCount=0,
   resamplingActive=false,resamplingRateHz=10f,accelerationAvailable=true,gyroscopeAvailable=true,gravityAvailable=true,rotationVectorAvailable=false,rotationAccuracy=0)
 }
 fun dataset():ReplayDataset?=ds
}