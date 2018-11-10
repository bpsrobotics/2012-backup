package com.team2898.engine.kinematics

import com.team2898.engine.extensions.Vector2D.l2
import com.team2898.engine.extensions.Vector2D.plus
import com.team2898.engine.extensions.Vector2D.unaryMinus
import com.team2898.engine.math.linear.rotateVector2D
import com.team2898.engine.types.Interpolable
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import java.text.DecimalFormat


// Represents a 2D translation on the x, y plane
class Translation2d : Interpolable<Translation2d> {
    var position = Vector2D(0.0, 0.0)

    var x: Double
        get() = position.x
        set(value) {
            position = Vector2D(value, y)
        }

    var y: Double
        get() = position.y
        set(value) {
            position = Vector2D(x, value)
        }

    constructor()

    constructor(x: Double, y: Double) {
        position = Vector2D(x, y)
    }

    constructor(toCopy: Translation2d) {
        position = toCopy.position
    }

    constructor(vector: Vector2D) {
        position = vector
    }

    /**
     * Normalized distance (distance to origin)
     */
    val norm: Double
        get() = position.l2

    val normalized: Vector2D
        get() = position.normalize()

    infix fun translateBy(other: Translation2d): Translation2d = translateBy(other.position)
    infix fun translateBy(other: Vector2D): Translation2d =
            Translation2d(position + other)

    infix fun rotateByOrigin(rotation: Rotation2d): Translation2d =
            Translation2d(rotateVector2D(position, rotation.rotation))

    fun inverse(): Translation2d = Translation2d(-position)

    override fun interpolate(upperVal: Translation2d, interpolatePoint: Double): Translation2d =
            when {
                (interpolatePoint <= 0) -> Translation2d(this)
                (interpolatePoint >= 1) -> Translation2d(upperVal)
                else -> extrapolate(upperVal, interpolatePoint)
            }

    fun extrapolate(slopePoint: Translation2d, extrapolatePoint: Double): Translation2d =
            Translation2d(
                    extrapolatePoint * (slopePoint.x - position.x) + position.x,
                    extrapolatePoint * (slopePoint.y - position.y) + position.y
            )

    override fun toString(): String =
            "[x: ${"%.3f".format(position.x)}, y: ${"%.3f".format(position.y)}]"
}
