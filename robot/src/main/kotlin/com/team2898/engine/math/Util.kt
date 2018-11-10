package com.team2898.engine.math

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.apache.commons.math3.linear.RealMatrix
import kotlin.math.absoluteValue

val kEpsilon: Double = 1.0E-8


fun Double.square() = Math.pow(this, 2.0)
fun Double.sqrt() = Math.sqrt(this)
fun Double.exp() = Math.exp(this)
fun Int.square() = this * this

fun clamp(value: Double, magnitude: Double) = clamp(value, -magnitude, magnitude)
fun clamp(value: Double, min: Double, max: Double) = Math.min(max, Math.max(min, value))
fun clamp(value: Int, min: Int, max: Int) = Math.min(max, Math.max(min, value))


fun avg(vararg numbers: Double): Double {
    var total = 0.0
    numbers.forEach { total += it }
    return total / numbers.size
}

fun sum(vararg numbers: Double): Double {
    var total = 0.0
    numbers.forEach { total += it }
    return total
}

fun prod(vararg numbers: Double): Double {
    var total = 0.0
    numbers.forEach { total *= it }
    return total
}

fun l1(vararg numbers: Double): Double =
        sum(*numbers.map { it.absoluteValue }.toDoubleArray())

fun l2(vararg numbers: Double): Double =
        avg(*numbers.map { it.square() }.toDoubleArray()).sqrt()
