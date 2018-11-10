package com.team2898.engine.controlLoops.classicControl

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.team2898.engine.math.clamp
import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs

/** 3DOF PID Controller for simultanious control of position, velocity, and acceleration
 * @param Kpp
 *  Position: Proportional gain
 * @param Kpi
 *  Position: Integral gain
 * @param Kvp
 *  Velocity: Proportional gain
 * @param Kvf
 *  Velocity: Feed-forward gain
 * @param Kaf
 *  Acceleration: Feed-forward gain
 *
 */

class PVAPID(val Kpp: Double, val Kpi: Double,
             val Kvp: Double, val Kvf: (Double) -> Double, // (target speed) -> -1 to 1
             val Kaf: Double, val Kpf: () -> Double,
             val integratorMax: Double = Double.MAX_VALUE,
             val integratorMin: Double = Double.MIN_VALUE,
             val minOutput: Double = -1.0,
             val maxOutput: Double = 1.0,
             val maxInput: Double = 1.0,
             val minInput: Double = -1.0) {

    var prevPosError = Double.NaN
    var prevVelError = Double.NaN

    var integrator: Double = 0.0

    var lastTime = Double.NaN

    fun update(position: Double, velocity: Double, targetPos: Double, targetVel: Double, targetAcc: Double, currentTime: Double = Timer.getFPGATimestamp()): Double {
        if ((lastTime) != Double.NaN)
            lastTime = currentTime
        val dt = currentTime - lastTime

        val posError = position - targetPos
        val velError = velocity - targetVel

        prevPosError = posError
        prevVelError = velError

        integrator += posError * dt

        var output = Kpf() + Kpp * posError + Kvp * velError + Kvf(targetVel) + Kaf * targetAcc

        if (minOutput <= output && output <= maxOutput) {
            integrator += posError * dt
            output += Kpi * integrator
        } else integrator = 0.0


        integrator = clamp(integratorMin, integratorMax)

        output = clamp(output, minOutput, maxOutput)


        return output
    }


    fun onTarget(allowablePosError: Double, allowableVelError: Double) =
            abs(prevPosError) < abs(allowablePosError) && abs(prevVelError) < abs(allowableVelError)

    fun reset() {
        integrator = 0.0
        lastTime = Double.NaN
    }
}