package com.team2898.engine.logic

import com.team2898.engine.async.AsyncLooper

interface ILooper {
    val enableTimes: List<GamePeriods>
    val loop: AsyncLooper
    fun onStart(){}
    fun onStop(){}
}
