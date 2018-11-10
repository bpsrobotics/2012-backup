package com.team2898.engine.logging

import com.team2898.engine.async.util.go
import com.team2898.engine.async.util.goLazy
import kotlinx.coroutines.experimental.*

/**
 * TimeBomb is meant for watching asynchronous processes and dumping a stack trace if they take more than a set amount of time.
 * @param time time in seconds until bomb blows
 * @param startArmed whether or not the bomb should automatically arm upon init
 * @param throwException whether or not the bomb should throw an exception upon blowing
 * @param printLogger whether or not the bomb should log upon blowing
 * @constructor creates a default timebomb
 */
class TimeBomb(val time: Double, val startArmed: Boolean = true, val throwException: Boolean = false, val printLogger: Boolean = true) {

    private var onBlow: () -> Unit = {}
    private var useAsync = false

    var blown = false

    init {
        if (startArmed) fuse()
    }

    private var job: Job = goLazy {
        delay(time.toLong() * 1000)
        err()
        blown = true
    }

    /**
     * Starts TimeBomb
     * Only necessary if startArmed is specified to be false
     */
    fun fuse() {
        job.start()
    }

    /**
     * Stops TimeBomb
     * Should be called when the process TimeBomb is watching exits correctly
     */
    fun defuse() {
        if (!blown)
            job.cancel()
    }

    /**
     * Sets function to run if it blows
     * @param block code to run
     * @param useAsync whether or not we should dispatch a coroutine to run it
     */
    fun setOnBlow(useAsync: Boolean = false, block: () -> Unit) {
        onBlow = block
        this.useAsync = useAsync
    }

    private fun err() {
        if (throwException) {
            throw Exception("TimeBomb not defused in time")
        }
        if (printLogger) {
            Logger.logInfo(reflectLocation(), LogLevel.WARNING, "TimeBomb timer expired")
        }

        if (!this.useAsync) onBlow()
        else go { onBlow }
    }
}
