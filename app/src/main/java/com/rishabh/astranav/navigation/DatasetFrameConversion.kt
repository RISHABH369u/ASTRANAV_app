package com.rishabh.astranav.navigation

/**
 * IO-VNBD (DATASET_FRAME) -> ESKF body frame conversion.
 *
 * The ESKF (EskfPrediction) is written for:
 *
 *     Navigation = NED  (X north, Y east, Z down)
 *     Body       = FRD  (X forward, Y right, Z down)
 *     a_n        = R_nb * f_b + [0, 0, +g]
 *
 * In that convention a level, stationary sensor measures
 * specific force f_b = [0, 0, -g].
 *
 * The recorded IO-VNBD phone channels are Android-style
 * (accelerometer reads +g on Z at rest, i.e. Z points UP).
 *
 * Previously the replay fed those channels to the ESKF unchanged
 * and compensated with a 180 degree initial attitude flip. That
 * is self-consistent for mechanization, but:
 *
 *   - the gyro columns were assigned to the wrong axes
 *     ([Yaw, Pitch, Roll] -> X, Y, Z), so vehicle yaw was
 *     integrated about a horizontal body axis and the attitude
 *     tilted ~90 degrees within a minute (gravity leaked into
 *     the horizontal plane -> |a_nav| ~ 14 m/s^2), and
 *   - NHC / ZARU assume a true FRD body (lateral = Y,
 *     vertical = Z, yaw = about Z).
 *
 * ASSUMED dataset sensor frame (Android device frame, phone
 * lying flat in the vehicle):
 *
 *     X = right, Y = forward, Z = up
 *
 * which maps to FRD with the proper rotation (det = +1):
 *
 *     FRD = [ Y, X, -Z ]
 *
 * If the first replay shows the wrong travel direction, only
 * [specificForceToFrd] / [gyroToFrd] need to change.
 *
 * Gyro columns: the replay session delivers
 * [gyroYaw, gyroPitch, gyroRoll]. The dataset's "Pitch"
 * column is the rotation about the vertical axis (see the
 * note in IoVnbdReplaySession.toDeviceSample), so for the
 * ESKF the sensor-frame rate vector is
 *
 *     sensor = [ Roll, Yaw, Pitch ]  (about X, Y, Z-up)
 *
 * and after the FRD permutation
 *
 *     FRD = [ Yaw, Roll, -Pitch ]
 *
 * The ML models keep receiving the original arrays; only the
 * ESKF mechanization uses these converted values.
 */
object DatasetFrameConversion {

    /** Android-style specific force (Z up) -> FRD specific force. */
    fun specificForceToFrd(a: FloatArray): FloatArray =
        floatArrayOf(a[1], a[0], -a[2])

    /**
     * Replay gyro [yawCol, pitchCol, rollCol] -> FRD angular rate.
     */
    fun gyroToFrd(g: FloatArray): FloatArray =
        floatArrayOf(g[0], g[2], -g[1])

    /**
     * Unit "down" direction in the FRD body frame, from the
     * recorded gravity channel (which, like the accelerometer,
     * reads +g on Z-up at rest).
     *
     * Returns null when the gravity magnitude is unusable.
     */
    fun downInBodyFrd(gravity: FloatArray): DoubleArray? {
        val f = specificForceToFrd(gravity)
        val n = Math.sqrt(
            f[0].toDouble() * f[0] +
                    f[1].toDouble() * f[1] +
                    f[2].toDouble() * f[2]
        )
        if (!n.isFinite() || n < 1e-6) return null
        // down = -f/|f|
        return doubleArrayOf(-f[0] / n, -f[1] / n, -f[2] / n)
    }
}