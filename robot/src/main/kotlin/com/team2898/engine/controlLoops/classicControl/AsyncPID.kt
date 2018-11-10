package com.team2898.engine.controlLoops

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.IAsyncLoop
import com.team2898.engine.extensions.blockJoin
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.experimental.async

/* import edu.wpi.first.wpilibj.util.BoundaryException */

/**
 * Asynchronous PID control loop wrapper
 * Inherits a standard synchronous PID loop as well as Kotlin's concurrent Job system to provide "true" concurrency (none of that notifier trash)
 * @author Solomon
 */

class AsyncPID(
        Kp: Double = 0.0,
        Ki: Double = 0.0,
        Kd: Double = 0.0,
        Kf: Double = 0.0,
        integratorMax: Double = Double.POSITIVE_INFINITY,
        currentTime: Double = Timer.getFPGATimestamp(),
        execHz: Double
) : StandardPID(Kp, Ki, Kd, Kf, integratorMax, currentTime), IAsyncLoop {

    var sensorValue: Double = 0.0 // Current sensor value
    var sensorOutput: Double = 0.0

    var getSensorInput: () -> Double = {
        throw NotImplementedError("Warning: getSensorInput() -> Double not implemented")
    }
        @Synchronized get
        @Synchronized set

    var useControllerOutput: (Double) -> Unit = {
        throw NotImplementedError("Warning: useControllerOutput (Double) -> Unit not implemented")
    }
        @Synchronized set
        @Synchronized get

    var asyncLooper: AsyncLooper = AsyncLooper(execHz) {
        sensorValue = getSensorInput()
        sensorOutput = update(sensorOutput, currentTime = Timer.getFPGATimestamp())
        useControllerOutput(sensorOutput)
    }

    override fun getLoop(): AsyncLooper = asyncLooper

    @Synchronized
    override fun start() = asyncLooper.start().blockJoin()

    @Synchronized
    override fun stop() = asyncLooper.stop().blockJoin()


    override fun setTargetHz(hz: Double) {
        asyncLooper.hz = hz
    }
}
