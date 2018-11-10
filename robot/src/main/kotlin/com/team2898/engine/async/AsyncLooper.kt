package com.team2898.engine.async

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.TimeBombAsync
import com.team2898.engine.logging.reflectLocation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.experimental.*

class AsyncLooper(
        var hz: Double,
        val logOnCantKeepUp: Boolean = true,
        val startStopTimeout: Double = 1.0,
        val ioPool: Boolean = false,
        val func: () -> Unit
) {
    var job: Job = init()

    private fun init(): Job {
        return launch(CommonPool, CoroutineStart.LAZY) {
            while (isActive) {
                val startTime = Timer.getFPGATimestamp()
                func()
                val deltaTime = Timer.getFPGATimestamp() - startTime
                if (deltaTime < 1000 / hz) delay((1000 / hz - deltaTime).toLong())
                else if (logOnCantKeepUp) logCantKeepUp(deltaTime)
            }
        }
    }

    fun start(): Job {
        job = init()
        return TimeBombAsync(startStopTimeout) {
            job.start()
        }.start()
    }

    fun stop(): Job {
        return TimeBombAsync(startStopTimeout) {
            job.cancel()
            job.join()
        }.start()
    }

    private fun logCantKeepUp(lastTime: Double) {
        Logger.logInfo(reflectLocation(), LogLevel.WARNING, "AsyncLooper cannot keep up! Running at $hz hz and previous loop took ${lastTime * 1000} ms")
    }

}
