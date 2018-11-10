package com.team2898.engine.motion

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.LoopManager
import edu.wpi.first.wpilibj.PWMSpeedController

class PWMSlave(val slave: PWMSpeedController, val master: TalonSRX, val hz: Double, val modifyMasterStatusFrameHz: Boolean = false): ILooper {
    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)
    init {
        LoopManager.register(this)
        if (modifyMasterStatusFrameHz) master.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, (1000/hz).toInt(), 0)
    }
    override val loop = AsyncLooper(hz) {
        slave.set(master.motorOutputPercent)
    }
}
