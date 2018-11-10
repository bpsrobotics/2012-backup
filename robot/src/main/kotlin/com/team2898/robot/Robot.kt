package com.team2898.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.TalonWrapper
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.abs


class Robot : TimedRobot() {

    val driver = Joystick(0)
//    val operator = Joystick(1)
    val operator = XboxController(1)

    val lDt = Jaguar(4)
    val rDt = Jaguar(5)
    val intake = Talon(8)
    val elevator = Victor(9)
    val feeder = Jaguar(7)
//    val flywheel = Jaguar(6)
    val flywheel = TalonWrapper(7)

    val MAX_VEL = 2000
    val MAX_ACC = MAX_VEL / 2
//    val TARGET_VEL = -100000.0
    val TARGET_VEL = -25000.0

    val encVelocity
        get() = flywheel.getSelectedSensorVelocity(0).toDouble()

    init {
        flywheel.apply {
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0, 10)
            setMagEncoder()
            setSensorPhase(true)
            setPID(0.15, 0.04, 0.03, 0.001, 0, 0)
            configMotionAcceleration(MAX_ACC, 10)
            configMotionCruiseVelocity(MAX_VEL, 10)
            setNeutralMode(NeutralMode.Coast)
            configReverseSoftLimitEnable(false, 10)
            configForwardSoftLimitEnable(false, 10)
        }

        AsyncLooper(100.0) {
            SmartDashboard.putNumber("Vel", encVelocity)
            SmartDashboard.putNumber("target", TARGET_VEL)
            SmartDashboard.putBoolean("key", encVelocity < TARGET_VEL)
        }.start()

    }


    fun deadzone(value: Double): Double {
        if (Math.abs(value) < 0.2) return 0.0
        return value
    }

    fun cube(value: Double): Double = Math.pow(value, 3.0)
    fun square(value: Double): Double = Math.pow(value, 2.0) * if (value > 0) 1 else -1

    fun process(value: Double, deadzone: Boolean = true, cube: Boolean = false, square: Boolean = false): Double {
        var localValue = value
        if (deadzone)
            localValue = deadzone(value)
        if (cube) {
            localValue = cube(localValue)
        } else if (square) {
            localValue = square(localValue)
        }
        return localValue
    }

    fun turn() = process(driver.getRawAxis(4), square = true) * 0.8

    val throttle
        get() = process(driver.getRawAxis(1), square = true)
    val turn
        get() = turn()
    val quickTurn: Boolean
        get() = process(Math.max(driver.getRawAxis(2), driver.getRawAxis(3))) != 0.0
    val leftTrigger
        get() = process(driver.getRawAxis(2) * 0.8, square = true)
    val rightTrigger
        get() = process(driver.getRawAxis(3) * 0.8, square = true)

    override fun teleopPeriodic() {
        CheesyDrive.updateQuickTurn(quickTurn)
        val pow = CheesyDrive.updateCheesy(
                (if (!quickTurn) turn else -leftTrigger + rightTrigger),
                -throttle,
                quickTurn,
                true
        )
        lDt.set(pow.left)
        rDt.set(pow.right)
        intake.set(process(operator.getRawAxis(2)))
        elevator.set(operator.getRawAxis(3))

        // TODO change this value
        val setpoint = 24000.0

        if (operator.getRawButton(6)) feeder.set(-0.3) else feeder.set(0.0)
        if (abs(operator.getRawAxis(1)) > 0.5) flywheel.set(ControlMode.Velocity, TARGET_VEL) else flywheel.set(ControlMode.PercentOutput, 0.0)
    }
}