package com.rishabh.astranav.dvfc.math

/**
 * Fixed correction between low_poly_mobile_phone.glb's own local axes and
 * Android's sensor coordinate frame (device X = right along the screen,
 * Y = up along the screen, Z = out of the screen towards the user) — spec §4:
 *
 *   q_model = q_sensor ∘ q_modelCorrection
 *
 * DO NOT derive this by nudging numbers until it "looks right" on screen —
 * spec §4 explicitly forbids that. Derive it once, properly, against the
 * actual asset:
 *   1. Physically place the phone flat, screen up, top edge pointing in the
 *      vehicle's forward direction. In that pose the phone's own
 *      TYPE_ROTATION_VECTOR is close to a known reference orientation.
 *   2. With CORRECTION = identity, note which way the *rendered GLB* is
 *      facing/up in that same pose.
 *   3. Solve for the single fixed rotation that reconciles the two — i.e.
 *      the rotation from the model's authored "front/up" to Android's
 *      sensor-frame "front/up". Most GLB phone assets are authored with
 *      the model's own -Z as "screen forward" and +Y as "up the screen",
 *      which rarely matches Android's sensor convention out of the box.
 *   4. Hardcode that single fixed quaternion here. It never changes at
 *      runtime, which is why it lives here and not in
 *      DeviceVehicleTransform (that class only holds things that get
 *      recalibrated per-session).
 *
 * IDENTITY below is a placeholder — we don't have the actual
 * low_poly_mobile_phone.glb to inspect, so this MUST be verified against
 * your real asset before trusting the on-screen orientation.
 */
object ModelFrameCorrection {
    // TODO: replace with the real correction derived from low_poly_mobile_phone.glb (see steps above).
    val CORRECTION: Quat = Quat.IDENTITY
}
