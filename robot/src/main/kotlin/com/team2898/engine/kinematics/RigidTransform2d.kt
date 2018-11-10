package com.team2898.engine.kinematics

import com.team2898.engine.math.kEpsilon
import com.team2898.engine.math.linear.rotateVector2D
import com.team2898.engine.types.Interpolable
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.RealMatrix

// Represents 2D pose (position, orientation) on the x, y plane
class RigidTransform2d : Interpolable<RigidTransform2d> {
    companion object {

        fun fromTranslation(translation: Translation2d): RigidTransform2d {
            return RigidTransform2d(translation, Rotation2d())
        }

        fun fromRotation(rotation: Rotation2d): RigidTransform2d {
            return RigidTransform2d(Translation2d(), rotation)
        }

        /**
         * Creates a new RigidTransform2d given a velocity + curvature
         */
        fun fromDelta(twist: Twist2d): RigidTransform2d {
            val sinTheta: Double = Math.sin(twist.dtheta)
            val cosTheta: Double = Math.cos(twist.dtheta)
            val sin: Double
            val cos: Double

            if (Math.abs(twist.dtheta) < kEpsilon) {
                sin = 1.0 - (1.0 / 6.0) * Math.pow(twist.dtheta, 2.0)
                cos = 0.5 * twist.dtheta
            } else {
                sin = sinTheta / twist.dtheta
                cos = (1.0 - cosTheta) / twist.dtheta
            }

            val rotated = rotateVector2D(
                    // yes, the rotation's sin and cos are flipped. This is intentional
                    source = Vector2D(twist.dx, twist.dy), rotation = Vector2D(sin, cos)
            )

            return RigidTransform2d(
                    Translation2d(rotated.x, rotated.y), Rotation2d(cosTheta, sinTheta)
            )
        }
    }


    var translation: Translation2d = Translation2d()
    var rotation: Rotation2d = Rotation2d()

    val x: Double
        get() = translation.x
    val y: Double
        get() = translation.y
    val cos: Double
        get() = rotation.cos
    val sin: Double
        get() = rotation.sin
    val theta: Double
        get() = rotation.theta

    constructor()

    constructor(translation: Translation2d, rotation: Rotation2d) {
        this.translation = translation
        this.rotation = rotation
    }

    constructor(toCopy: RigidTransform2d) {
        translation = Translation2d(toCopy.translation)
        rotation = Rotation2d(toCopy.rotation)
    }


    /** To transform a 2D pose by another, we first translate by another translation then rotate by the other rotation
     */
    fun transformBy(other: RigidTransform2d): RigidTransform2d {
        return RigidTransform2d(
                translation.translateBy(other.translation.rotateByOrigin(rotation)),
                rotation.rotateBy(other.rotation)
        )
    }

    /** Inverse of the transformation undoes effect of translating by the own transform
     * (if that makes any sense)
     */

    val inverse: RigidTransform2d
        get() =
            RigidTransform2d(translation.inverse().rotateByOrigin(rotation.inverse), rotation.inverse)

    /** Linear interpolation (sucks)
     * TODO: Update to constant curvature interpolation
     */
    override fun interpolate(upperVal: RigidTransform2d, interpolatePoint: Double): RigidTransform2d =
            when {
                (interpolatePoint <= 0) -> RigidTransform2d(this)
                (interpolatePoint >= 1) -> RigidTransform2d(upperVal)
                else -> RigidTransform2d(
                        translation.interpolate(upperVal.translation, interpolatePoint),
                        rotation.interpolate(upperVal.rotation, interpolatePoint)
                )
            }


    override fun toString(): String {
        return "T: ${translation.toString()}, R: ${rotation.toString()}"
    }
}

