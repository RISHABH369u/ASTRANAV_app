package com.astranav.ui

import android.opengl.GLES20
import android.opengl.Matrix
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/** Tiny deterministic solid-color geometry helper; deliberately avoids textures, sensors and network data. */
class Solid {
    private val program: Int
    private val cube = floatArrayOf(
        -1f, -1f, 1f, 1f, -1f, 1f, 1f, 1f, 1f, -1f, -1f, 1f, 1f, 1f, 1f, -1f, 1f, 1f,
        -1f, -1f, -1f, -1f, 1f, -1f, 1f, 1f, -1f, -1f, -1f, -1f, 1f, 1f, -1f, 1f, -1f, -1f,
        -1f, -1f, -1f, -1f, -1f, 1f, -1f, 1f, 1f, -1f, -1f, -1f, -1f, 1f, 1f, -1f, 1f, -1f,
        1f, -1f, -1f, 1f, 1f, -1f, 1f, 1f, 1f, 1f, -1f, -1f, 1f, 1f, 1f, 1f, -1f, 1f,
        -1f, 1f, -1f, -1f, 1f, 1f, 1f, 1f, 1f, -1f, 1f, -1f, 1f, 1f, 1f, 1f, 1f, -1f,
        -1f, -1f, -1f, 1f, -1f, -1f, 1f, -1f, 1f, -1f, -1f, -1f, 1f, -1f, 1f, -1f, -1f, 1f
    )

    init {
        program = GLES20.glCreateProgram().also { p ->
            val vs = compile(
                GLES20.GL_VERTEX_SHADER,
                "attribute vec3 p; uniform mat4 m; void main(){gl_Position=m*vec4(p,1.0);}"
            )
            val fs = compile(
                GLES20.GL_FRAGMENT_SHADER,
                "precision mediump float; uniform vec4 c; void main(){gl_FragColor=c;}"
            )
            GLES20.glAttachShader(p, vs); GLES20.glAttachShader(p, fs); GLES20.glLinkProgram(p)
        }
    }

    private fun compile(type: Int, src: String) = GLES20.glCreateShader(type)
        .also { GLES20.glShaderSource(it, src); GLES20.glCompileShader(it) }

    private fun data(a: FloatArray): FloatBuffer =
        ByteBuffer.allocateDirect(a.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
            .apply { put(a); position(0) }

    private fun draw(
        vertices: FloatArray,
        mode: Int,
        matrix: FloatArray,
        r: Float,
        g: Float,
        b: Float,
        p: FloatArray,
        v: FloatArray
    ) {
        val m = FloatArray(16); Matrix.multiplyMM(m, 0, v, 0, matrix, 0); Matrix.multiplyMM(
            m,
            0,
            p,
            0,
            m,
            0
        ); GLES20.glUseProgram(program);
        val pos = GLES20.glGetAttribLocation(
            program,
            "p"
        ); GLES20.glEnableVertexAttribArray(pos); GLES20.glVertexAttribPointer(
            pos,
            3,
            GLES20.GL_FLOAT,
            false,
            0,
            data(vertices)
        ); GLES20.glUniformMatrix4fv(
            GLES20.glGetUniformLocation(program, "m"),
            1,
            false,
            m,
            0
        ); GLES20.glUniform4f(
            GLES20.glGetUniformLocation(program, "c"),
            r,
            g,
            b,
            1f
        ); GLES20.glDrawArrays(mode, 0, vertices.size / 3)
    }

    fun cube(
        m: FloatArray,
        w: Float,
        h: Float,
        d: Float,
        r: Float,
        g: Float,
        b: Float,
        p: FloatArray,
        v: FloatArray
    ) {
        val local = m.copyOf(); Matrix.scaleM(local, 0, w / 2, h / 2, d / 2); draw(
            cube,
            GLES20.GL_TRIANGLES,
            local,
            r,
            g,
            b,
            p,
            v
        )
    }

    fun line(
        a: FloatArray,
        b: FloatArray,
        r: Float,
        g: Float,
        bl: Float,
        p: FloatArray,
        v: FloatArray
    ) {
        val m = FloatArray(16); Matrix.setIdentityM(
            m,
            0
        ); GLES20.glLineWidth(1f); draw(floatArrayOf(*a, *b), GLES20.GL_LINES, m, r, g, bl, p, v)
    }

    fun axis(
        m: FloatArray,
        len: Float,
        r: Float,
        g: Float,
        b: Float,
        p: FloatArray,
        v: FloatArray
    ) {
        val a = floatArrayOf(0f, 0f, 0f);
        val end = floatArrayOf(len, 0f, 0f);
        val mm = m.copyOf(); fun tx(q: FloatArray): FloatArray {
            val o = FloatArray(4); Matrix.multiplyMV(
                o,
                0,
                mm,
                0,
                floatArrayOf(q[0], q[1], q[2], 1f),
                0
            ); return floatArrayOf(o[0], o[1], o[2])
        }; line(tx(a), tx(end), r, g, b, p, v)
    }
}
