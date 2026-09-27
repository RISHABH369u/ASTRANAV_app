package com.rishabh.astranav.dvfc.render

import androidx.lifecycle.LifecycleCoroutineScope
import com.rishabh.astranav.dvfc.math.ModelFrameCorrection
import com.rishabh.astranav.dvfc.math.Quat
import dev.romainguy.kotlin.math.Quaternion
import io.github.sceneview.SceneView
import io.github.sceneview.math.Position
import io.github.sceneview.math.Scale
import io.github.sceneview.math.Transform
import io.github.sceneview.node.ModelNode
import kotlinx.coroutines.launch

/**
 * Owns the GLB phone node inside the SceneView declared in activity_dvfc.xml
 * and applies the live device→vehicle quaternion to it every frame — this
 * is the load-bearing requirement in the spec (§1, §3, §14): the 3D phone
 * IS the sensor state, not a decorative animation.
 *
 * Scoping note on the "vehicle chassis + axis" gizmos the spec describes
 * (§6, §10): those are implemented as a flat 2D overlay
 * (view_frame_axes.xml, wired up in DVFCActivity) rather than hand-built raw
 * Filament geometry. Hand-rolling VertexBuffer/IndexBuffer primitives is
 * fragile and version-specific, and isn't where the engineering value of
 * this screen is — the phone's real orientation is. If you want a literal
 * 3D chassis, the robust way is a second small static GLB
 * (e.g. `vehicle_chassis.glb`) loaded as a second ModelNode exactly like
 * the phone below, kept at identity rotation as the fixed reference frame —
 * not raw Filament calls.
 */
class DvfcSceneRenderer(
    private val sceneView: SceneView,
    private val lifecycleScope: LifecycleCoroutineScope,
) {
    private var modelNode: ModelNode? = null
    private var basePosition: Position = Position(x = 0f, y = 0f, z = -1.4f)
    private var baseScale: Scale = Scale(1f, 1f, 1f)

    /**
     * VERSION NOTE — this is the one part of the file worth checking against
     * your installed `io.github.sceneview:sceneview` version. The shape
     * (get a ModelInstance from `sceneView.modelLoader`, construct
     * `ModelNode(modelInstance = ...)`, `sceneView.addChildNode(node)`) is
     * the current documented pattern, but SceneView's exact method names on
     * ModelLoader have moved across releases. If this doesn't compile
     * as-is, check `io.github.sceneview.loaders.ModelLoader` in your
     * version's sources — the two-step shape below stays the same even
     * when a method gets renamed. Nothing downstream of `modelNode`
     * (updateOrientation, all the DVFC math) is affected by that.
     */
    fun loadPhoneModel(onReady: () -> Unit = {}, onError: (Throwable) -> Unit = {}) {
        lifecycleScope.launch {
            try {
                sceneView.modelLoader.loadModelInstanceAsync(
                    fileLocation = MODEL_ASSET_PATH,
                    onResult = { modelInstance ->
                        if (modelInstance == null) {
                            onError(IllegalStateException("Failed to load $MODEL_ASSET_PATH"))
                            return@loadModelInstanceAsync
                        }
                        val node = ModelNode(modelInstance = modelInstance, autoAnimate = false, scaleToUnits = 1.0f)
                        modelNode = node
                        basePosition = node.position
                        baseScale = node.scale
                        sceneView.addChildNode(node)
                        onReady()
                    },
                )
            } catch (t: Throwable) {
                onError(t)
            }
        }
    }

    /**
     * Applies the current device→vehicle rotation to the phone node.
     *
     * We rebuild the whole Transform from cached position/scale rather than
     * assigning `node.quaternion = ...` directly: Node's quaternion setter
     * decomposes the live Filament matrix to read back position/scale on
     * every call, and at sensor-fusion update rates that decomposition
     * slowly skews the mesh via floating-point drift
     * (github.com/sceneview/sceneview/issues/2187 — the fix recommended
     * there is exactly this: compose Transform from clean, cached values).
     */
    fun updateOrientation(deviceToVehicle: Quat) {
        val node = modelNode ?: return
        val corrected = deviceToVehicle * ModelFrameCorrection.CORRECTION
        node.transform = Transform(
            basePosition,
            Quaternion(corrected.x, corrected.y, corrected.z, corrected.w),
            baseScale,
        )
    }

    companion object {
        private const val MODEL_ASSET_PATH = "models/low_poly_mobile_phone.glb"
    }
}
