package com.team2898.engine.kinematics

import com.team2898.engine.extensions.Vector2D.atan2
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

/** Stores a "twist" of our 2d position (2d linear + 2d angular component)
 * Modeled off of the ROS type
 * @param dx delta position, x component
 * @param dy delta position, y component
 * @param dtheta delta rotation, in radians
 */
data class Twist2d(var dx: Double, var dy: Double, var dtheta: Double) {
    constructor(linear: Vector2D, angular: Vector2D) : this(linear.x, linear.y, angular.atan2)
    constructor (linear: Translation2d, angular: Rotation2d) : this(linear.x, linear.y, angular.radians)
}
