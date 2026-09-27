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
 * ---- Why the model wasn't showing / looked huge ----
 * Inspected the actual low_poly_mobile_phone.glb you uploaded: its root
 * scale is ~0.01, but several sub-parts (case, camera, buttons) each carry
 * their own large internal translations (tens of units, before that 0.01
 * scale is applied) relative to the mesh origin. Two consequences:
 *   1. Without an explicit `scaleToUnits`, the model's on-screen size is
 *      whatever the artist's raw scale happens to produce — not something
 *      you should trust for any GLB.
 *   2. Without an explicit `centerOrigin`, the model's bounding box isn't
 *      centered on the node's own local origin, so it can end up partly or
 *      fully outside wherever the camera is pointed, even at a "sensible"
 *      scale — this was almost certainly the actual "not appearing" cause,
 *      more than pure size.
 * Both are fixed below, plus the camera is now pointed at the model
 * explicitly instead of relying on SceneView's default camera pose (which
 * isn't guaranteed to frame wherever this particular node ends up).
 *
 * Scoping note on the "vehicle chassis + axis" gizmos the spec describes
 * (§6, §10): those are implemented as a flat 2D overlay
 * (the device/vehicle-frame card + HeadingCompassView in activity_dvfc.xml)
 * rather than hand-built raw Filament geometry — see HeadingCompassView.kt
 * for the arrow/angle indicator. If you want a literal 3D chassis, the
 * robust way is a second small static GLB (e.g. `vehicle_chassis.glb`)
 * loaded as a second ModelNode exactly like the phone below, kept at
 * identity rotation as the fixed reference frame — not raw Filament calls.
 */
class DvfcSceneRenderer(
    private val sceneView: SceneView,
    private val lifecycleScope: LifecycleCoroutineScope,
) {
    private var modelNode: ModelNode? = null
    private var baseScale: Scale = Scale(1f, 1f, 1f)

    // The model always sits at the scene origin — we frame it by moving the
    // camera (frameCamera()), not by nudging the model's own position.
    private val basePosition = Position(x = 0f, y = 0f, z = 0f)

    /**
     * VERSION NOTE — `sceneView.modelLoader.loadModelInstanceAsync(...)` and
     * `sceneView.cameraNode` / `lookAt(...)` below are the current
     * documented shapes, but SceneView's exact method/property names shift
     * across releases (it's now pushing a Compose-first v4.x, while this
     * spec explicitly wants XML). If something here doesn't compile,
     * search your installed version's `io.github.sceneview.loaders.ModelLoader`
     * and `io.github.sceneview.node.CameraNode` sources — the two-step shape
     * (load a ModelInstance → construct ModelNode; compute a look-at
     * transform → assign it to the camera node) stays the same even when a
     * method gets renamed.
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
                        val node = ModelNode(
                            modelInstance = modelInstance,
                            autoAnimate = false,
                            // Normalizes whatever raw scale the asset was authored at to a
                            // fixed, predictable size. This is the actual fix for "too big" —
                            // tune TARGET_SIZE_METERS below, never the GLB itself.
                            scaleToUnits = TARGET_SIZE_METERS,
                            // Recenters the bounding box on the node's own local origin,
                            // undoing the asset's internal per-part offsets described above.
                            // This is the fix for "doesn't fit / not appearing".
                            centerOrigin = Position(x = 0f, y = 0f, z = 0f),
                        )
                        modelNode = node
                        baseScale = node.scale
                        node.position = basePosition
                        sceneView.addChildNode(node)
                        frameCamera()
                        onReady()
                    },
                )
            } catch (t: Throwable) {
                onError(t)
            }
        }
    }

    /**
     * Explicitly points the camera at the model instead of trusting
     * SceneView's default pose — see the class doc comment for why this
     * matters as much as scale/centering did.
     */
    private fun frameCamera() {
        sceneView.cameraNode.position = Position(x = 0f, y = 0f, z = CAMERA_DISTANCE_METERS)
        sceneView.cameraNode.lookAt(targetWorldPosition = basePosition)
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

        // Longest edge of the phone after normalization, in meters. This is
        // what actually controls "how big the phone looks" — tune THIS, not
        // the raw GLB. 0.22 sits a bit above a real phone's long edge
        // (~0.15m) so it still reads clearly inside a ~260dp container.
        private const val TARGET_SIZE_METERS = 0.22f

        // How far back the camera sits. Kept well beyond TARGET_SIZE_METERS
        // so the model doesn't clip the near plane or overflow the frame as
        // it rotates through every orientation (a rotating object's on-screen
        // footprint is at most its diagonal, not just its longest edge).
        private const val CAMERA_DISTANCE_METERS = TARGET_SIZE_METERS * 3.2f
    }
}
