package com.team2898.engine.logging

import com.team2898.engine.async.util.go
import kotlinx.coroutines.experimental.Job

class TimeBombAsync {

    var time = 0.0
    var throwException = false
    var printLogger = false
    var runOnEnd: () -> Unit = {}
    var block: suspend () -> Unit = {}

    constructor (time: Double, throwException: Boolean = true, printLogger: Boolean = true, runOnEnd: () -> Unit = {}, block: suspend () -> Unit) {
        this.time = time
        this.throwException = throwException
        this.printLogger = printLogger
        this.runOnEnd = runOnEnd
        this.block = block
    }


    val tb: TimeBomb = TimeBomb(time = time, startArmed = false, throwException = throwException, printLogger = printLogger).apply {
        setOnBlow(false, runOnEnd)
    }

    fun start(): Job =
            go {
                tb.fuse()
                block()
                if (!tb.blown) tb.defuse()
            }
}
