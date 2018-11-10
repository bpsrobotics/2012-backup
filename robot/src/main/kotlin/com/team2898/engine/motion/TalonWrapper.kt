package com.team2898.engine.motion

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.team2898.engine.types.MinMax
import kotlin.math.roundToInt

class TalonWrapper(
        deviceID: Int,
        val stateMachineHz: Double = 10.0,
        val frameSpeed: MinMax = MinMax(min = 10.0, max = 100.0, norm = 10.0),
        val frameSpeedHysteresisHz: Double = 5.0,
        var useAdaptiveFramerate: Boolean = true
) : TalonSRX(deviceID) {

    companion object {
        fun createSlave(deviceID: Int, other: TalonSRX): TalonWrapper {
            //return TalonSRXWrapper(deviceID) slaveTo other
            return TalonWrapper(1)
        }
    }

    var lastPidProfile = -1

    init {
    }

    var lastSpeed = Double.NaN
    var lastControlMode = controlMode ?: ControlMode.PercentOutput

    val pwmPos
        get() = sensorCollection.pulseWidthPosition and 0xFFF

    @Synchronized
    fun set(value: Double) {
        if (lastSpeed != value) {
            lastSpeed = value
            super.set(lastControlMode, value)
        }
    }

    @Synchronized
    fun changeControlMode(mode: ControlMode) {
        if (mode != lastControlMode) {
            lastControlMode = mode
            super.set(mode, lastSpeed)
        }
    }

    @Synchronized
    fun setOpenLoop(value: Double) {
        setOpenLoop()
        set(value)
    }

    @Synchronized
    fun setVelocityControl(value: Double) {
        setVelocityControl()
        set(value)
    }

    @Synchronized
    fun setPositionControl(value: Double) {
        setPositionControl()
        set(value)
    }


    @Synchronized
    fun setOpenLoop() = changeControlMode(ControlMode.PercentOutput)

    @Synchronized
    fun setPositionControl() = changeControlMode(ControlMode.Position)

    @Synchronized
    fun setVelocityControl() = changeControlMode(ControlMode.Velocity)

    @Synchronized
    fun setMagEncoder() {
        configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
    }

    @Synchronized
    fun setSlave(master: TalonSRX) = apply {
        changeControlMode(ControlMode.Follower)
        set(master.deviceID.toDouble())
    }

    @Synchronized
    infix fun slaveTo(master: TalonSRX) = setSlave(master)

    @Synchronized
    fun setControlFrameHz(map: Map<ControlFrame, Int>) {
        useAdaptiveFramerate = false
        map.forEach {
            val frame = it.key
            val periodMs = (1000.0 / it.value.toDouble()).roundToInt()
            setControlFramePeriod(frame, periodMs)
        }
    }

    @Synchronized
    fun setFeedbackFrameHz(map: Map<StatusFrameEnhanced, Int>) {
        useAdaptiveFramerate = false
        map.forEach {
            val frame = it.key
            val periodMs = (1000.0 / it.value.toDouble()).roundToInt()
            setStatusFramePeriod(frame, periodMs, 0)
        }
    }

    @Synchronized
    fun setPID(Kp: Double, Ki: Double, Kd: Double, Kf: Double = 0.0, slot: Int = 0, iZone: Int = 0) {
        config_IntegralZone(slot, iZone, 0)
        config_kP(slot, Kp, 0)
        config_kI(slot, Ki, 0)
        config_kD(slot, Kd, 0)
        config_kF(slot, Kf, 0)
    }
}