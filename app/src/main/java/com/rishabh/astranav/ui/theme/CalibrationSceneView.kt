package com.astranav.ui

import android.content.Context
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.MotionEvent
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.max

/** Real OpenGL ES geometry used by the XML alignment scene; phone axes rotate with the phone, car and grid do not. */
class CalibrationSceneView(context: Context) : GLSurfaceView(context) {
    private val scene = Renderer();
    private var downX = 0f;
    private var downY = 0f

    init {
        setEGLContextClientVersion(2); setRenderer(scene); renderMode = RENDERMODE_CONTINUOUSLY
    }

    fun setAxis(axis: Int, value: Float) {
        queueEvent {
            when (axis) {
                0 -> scene.yaw = value; 1 -> scene.pitch = value; else -> scene.roll = value
            }
        }
    }

    fun reset() {
        queueEvent { scene.yaw = -22f; scene.pitch = 8f; scene.roll = -5f }
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.action) {
            MotionEvent.ACTION_DOWN -> {
                downX = e.x; downY = e.y
            }; MotionEvent.ACTION_MOVE -> {
            scene.yaw += (e.x - downX) * .42f; scene.pitch -= (e.y - downY) * .42f; downX =
                e.x; downY = e.y
        }
        }; return true
    }

    private class Renderer : GLSurfaceView.Renderer {
        var yaw = -22f;
        var pitch = 8f;
        var roll = -5f
        private val projection = FloatArray(16);
        private val view = FloatArray(16);
        private val model = FloatArray(16);
        private lateinit var solid: Solid
        override fun onSurfaceCreated(
            gl: GL10?,
            config: EGLConfig?
        ) {
            GLES20.glEnable(GLES20.GL_DEPTH_TEST); GLES20.glClearColor(
                .027f,
                .043f,
                .071f,
                1f
            ); solid = Solid()
        }

        override fun onSurfaceChanged(
            g: javax.microedition.khronos.opengles.GL10?,
            w: Int,
            h: Int
        ) {
            GLES20.glViewport(0, 0, w, h); Matrix.perspectiveM(
                projection,
                0,
                42f,
                w.toFloat() / h,
                1f,
                40f
            ); Matrix.setLookAtM(view, 0, 4.3f, 3.1f, 8.5f, 0f, .2f, 0f, 0f, 1f, 0f)
        }

        override fun onDrawFrame(g: javax.microedition.khronos.opengles.GL10?) {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
            // Ground grid: real 3D parallel lines, fixed vehicle reference.
            for (i in -5..5) {
                solid.line(
                    floatArrayOf(i.toFloat(), -1.8f, -4f),
                    floatArrayOf(i.toFloat(), -1.8f, 4f),
                    .07f,
                    .14f,
                    .2f,
                    projection,
                    view
                ); solid.line(
                    floatArrayOf(-5f, -1.8f, i.toFloat()),
                    floatArrayOf(5f, -1.8f, i.toFloat()),
                    .07f,
                    .14f,
                    .2f,
                    projection,
                    view
                )
            }
            Matrix.setIdentityM(model, 0); Matrix.translateM(
                model,
                0,
                0f,
                -1.25f,
                -1.0f
            ); solid.cube(
                model,
                2.2f,
                .42f,
                3.6f,
                .12f,
                .18f,
                .28f,
                projection,
                view
            ); Matrix.translateM(model, 0, 0f, .48f, .2f); solid.cube(
                model,
                1.5f,
                .62f,
                1.7f,
                .16f,
                .23f,
                .35f,
                projection,
                view
            ) // car body + roof
            // phone body, glass, camera module and device frame are all actual cubes and transform as one object.
            Matrix.setIdentityM(model, 0); Matrix.translateM(
                model,
                0,
                0f,
                1.1f,
                0f
            ); Matrix.rotateM(model, 0, yaw, 0f, 1f, 0f); Matrix.rotateM(
                model,
                0,
                -pitch,
                1f,
                0f,
                0f
            ); Matrix.rotateM(model, 0, roll, 0f, 0f, 1f)
            solid.cube(
                model,
                1.35f,
                2.7f,
                .22f,
                .12f,
                .16f,
                .23f,
                projection,
                view
            ); Matrix.translateM(model, 0, 0f, 0f, .125f); solid.cube(
                model,
                1.16f,
                2.42f,
                .03f,
                .03f,
                .13f,
                .17f,
                projection,
                view
            ); Matrix.translateM(model, 0, -.35f, .82f, .03f); solid.cube(
                model,
                .38f,
                .44f,
                .04f,
                .08f,
                .1f,
                .14f,
                projection,
                view
            )
            solid.axis(model, 1.8f, 1f, .36f, .42f, projection, view); solid.axis(
                model,
                1.8f,
                .2f,
                .85f,
                .54f,
                projection,
                view
            ); solid.axis(model, 1.8f, .31f, .62f, 1f, projection, view)
        }
    }
}
