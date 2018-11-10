package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer

class VelocityRamp {
    var accelRate: Double = 0.0
    var deaccelRate: Double = 0.0

    var setpoint: Double = 0.0
    var setpointTimestamp: Double = 0.0

    var currentSpeed: Double = 0.0

    constructor(accelRate: Double, deaccelRate: Double = accelRate, currentSpeed: Double = 0.0, currentTime: Double = Timer.getFPGATimestamp()) {
        this.accelRate = accelRate
        this.deaccelRate = deaccelRate
        this.currentSpeed = currentSpeed
        setpointTimestamp = currentTime
    }

    fun setSetpoint(setpoint: Double, currentTime: Double = Timer.getFPGATimestamp()) {
        this.setpoint = setpoint
        this.setpointTimestamp = currentTime
    }

    fun getRampedSpeed(currentTime: Double = Timer.getFPGATimestamp()): Double {
        val dt = currentTime - setpointTimestamp

        when {
            (currentSpeed > setpoint) -> {
                if (currentSpeed - dt*deaccelRate <= setpoint)
                    currentSpeed = setpoint
                else
                    currentSpeed -= dt*deaccelRate
            }
            (currentSpeed < setpoint) -> {
                if (currentSpeed + dt*accelRate >= setpoint)
                    currentSpeed = setpoint
                else
                    currentSpeed += dt*accelRate
            }
        }

        return currentSpeed
    }

}
