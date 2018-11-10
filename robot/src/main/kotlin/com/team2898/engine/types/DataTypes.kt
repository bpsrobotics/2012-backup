package com.team2898.engine.types

import edu.wpi.first.wpilibj.Timer

/** Stores a time along with a generic stamp
 * @param time time
 * @param stamp value associated with the time
 */
data class Timestamp<out T>(val time: Double = Timer.getFPGATimestamp(), val stamp: T)


/** Simply stores a minimum value, maximum value, and a default nominal value
 * @param min minimum value
 * @param max maximum value
 * @param norm default value
 */
data class MinMax(val min: Double, val max: Double, val norm: Double = (max + min) / 2)


data class Quadruple<out A, out B, out C, out D>(val first: A, val second: B, val third: C, val fourth: D)
