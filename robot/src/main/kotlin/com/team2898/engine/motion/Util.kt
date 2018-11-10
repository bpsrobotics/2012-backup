package com.team2898.engine.motion

/** Stores left + ripackage com.team2898.engine.motion

import com.ctre.CANTalon

/** Stores left + right motor speed + brake value
 * @param left left power/speed
 * @param right right power/speed
 * @param brake brake mode or not
 */
data class DriveSignal(val left: Double = 0.0, val right: Double = 0.0, val brake: Boolean = false) {
    companion object {
        val NEUTRAL: DriveSignal
            get() = DriveSignal(0.0, 0.0, false)
        val BRAKE: DriveSignal
            get() = DriveSignal(0.0, 0.0, true)
    }

    operator fun times(mul: Int): DriveSignal {
        return DriveSignal(this.left * mul, this.right * mul, this.brake)
    }
}

/** Generated motion profile for built in profiling, Talon profiling compatible
 * @param position position in rotations
 * @param velocity velocity in rpm
 * @param duration duration in ms
 */
data class TalonPoint(val position: Double, val velocity: Double, val duration: Double)


ght motor speed + brake value
 * @param left left power/speed
 * @param right right power/speed
 * @param brake brake mode or not
 */
data class DriveSignal(val left: Double = 0.0, val right: Double = 0.0, val brake: Boolean = false) {
    companion object {
        val NEUTRAL: DriveSignal
            get() = DriveSignal(0.0, 0.0, false)
        val BRAKE: DriveSignal
            get() = DriveSignal(0.0, 0.0, true)
    }

    operator fun times(mul: Int): DriveSignal {
        return DriveSignal(this.left * mul, this.right * mul, this.brake)
    }
}

/** Generated motion profile for built in profiling, Talon profiling compatible
 * @param position position in rotations
 * @param velocity velocity in rpm
 * @param duration duration in ms
 */
data class TalonPoint(val position: Double, val velocity: Double, val duration: Double)