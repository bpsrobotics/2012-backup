package com.team2898.engine.math

import org.apache.commons.math3.complex.Quaternion

fun quaternionFromEuler(pitch: Double, roll: Double, yaw: Double): Quaternion {
    val cy = Math.cos(yaw * 0.5)
    val sy = Math.sin(yaw * 0.5)
    val cr = Math.cos(roll * 0.5)
    val sr = Math.sin(roll * 0.5)
    val cp = Math.cos(pitch * 0.5)
    val sp = Math.sin(pitch * 0.5)
    val w = cy * cr * cp + sy * sr * sp
    val x = cy * sr * cp - sy * cr * sp
    val y = cy * cr * sp + sy * sr * cp
    val z = sy * cr * cp - cy * sr * sp
    return Quaternion(w, x, y, z)
}