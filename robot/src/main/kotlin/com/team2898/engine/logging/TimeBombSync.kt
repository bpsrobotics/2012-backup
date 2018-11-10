package com.team2898.engine.logging

class TimeBombSync(time: Double, throwException: Boolean = true, printLogger: Boolean = true, runOnEnd: () -> Unit = {}, block: () -> Unit) {
    val tb: TimeBomb = TimeBomb(time=time, startArmed=false, throwException=throwException, printLogger=printLogger).apply {
        setOnBlow(false, runOnEnd)
    }

    init {
        tb.fuse()
        block()
        if (!tb.blown) tb.defuse()
    }
}
