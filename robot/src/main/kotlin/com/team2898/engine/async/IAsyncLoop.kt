package com.team2898.engine.async

interface IAsyncLoop {
    fun start()
    fun stop()
    fun setTargetHz(hz: Double)
    // fun logCantKeepUp(lagTimeMs: Double)
    fun getLoop(): AsyncLooper
}
