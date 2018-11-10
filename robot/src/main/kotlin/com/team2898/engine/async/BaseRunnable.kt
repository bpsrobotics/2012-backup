package com.team2898.engine.async

import java.lang.Runnable;

abstract class BaseRunnable() : Runnable {
    override fun run() {
        runLoop()
    }

    abstract fun runLoop(): Unit
}
