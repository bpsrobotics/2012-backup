package com.team2898.robot.subsystem

import com.team2898.engine.async.AsyncLooper
import edu.wpi.first.wpilibj.Victor

object Flywheel {
    private val PORT = 1
    private val flywheelMotor = Victor(PORT)

    private var x0 = 0.0
    private var y = 0.0

    // Generated with python-control script
    private val A = 0.9874840920748863
    private val B = 1.9598542385687778
    private val C = 1.0
    private val D = 0.0

    val sim = AsyncLooper(1/0.02) {
        x0 = A*x0 + B*inputVoltage
        y = C*x0 + D*inputVoltage
    }

    private val kP = 0.0
    private val kF = 12.0/(18730.0/10.0)

    val PFController = AsyncLooper(1/0.05) {
        val error = targetSpeedRPM - flywheelRPM
        val p = error*kP
        val f = targetSpeedRPM*kF
        inputVoltage = p+f
    }

    init {
        sim.start()
        PFController.start()
    }

    var targetSpeedRPM = 0.0

    private var inputVoltage: Double
        get() = inputVoltage
        set(value) {
            flywheelMotor.set(inputVoltage/12)
        }

    val flywheelRPM
        get() = y
}