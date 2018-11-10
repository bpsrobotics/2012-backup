package com.team2898.engine.signalProcessing

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.IAsyncLoop
import com.team2898.engine.extensions.blockJoin
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation

class MovingAverageFilterAsync(averagePeriod: Int, updateHz: Double, input: () -> Double) : MovingAverageFilter(averagePeriod), IAsyncLoop {

    val m_looper: AsyncLooper = AsyncLooper(updateHz) {
        addValue(input())
    }

    @Synchronized
    override fun setTargetHz(hz: Double) {
        m_looper.hz = hz
    }

    @Synchronized
    override fun stop() = m_looper.stop().blockJoin()

    @Synchronized
    override fun start() = m_looper.start().blockJoin()

    override fun getLoop() = m_looper

    override fun getAverage(): Double {
        return average()
    }
}