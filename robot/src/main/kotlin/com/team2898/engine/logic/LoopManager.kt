package com.team2898.engine.logic

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation

object LoopManager {
    val loops = mutableListOf<ILooper>()

    @Synchronized
    fun register(loop: ILooper) {
        Logger.logInfo(reflectLocation(), LogLevel.DEBUG, "Registering loop $loop")
        loops.add(loop)
    }

    @Synchronized
    fun onDisable() = setPeriod(GamePeriods.DISABLE)

    @Synchronized
    fun onAutonomous() = setPeriod(GamePeriods.AUTO)

    @Synchronized
    fun onTeleop() = setPeriod(GamePeriods.TELEOP)

    @Synchronized
    fun setPeriod(period: GamePeriods) {
        loops.forEach { iloop ->
            if (iloop.enableTimes.contains(period)) {
                Logger.logInfo(reflectLocation(), LogLevel.DEBUG, "Starting loop $iloop")
                iloop.loop.start()
            } else iloop.loop.stop()
        }
    }
}