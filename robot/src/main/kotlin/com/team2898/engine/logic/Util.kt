package com.team2898.engine.logic

enum class GamePeriods {
    AUTO, TELEOP, DISABLE
}

class RunEvery(val number: Int) {
    var counter = 0

    fun shouldRun(): Boolean {
        if (counter == number) {
            counter = 0
            return true
        }
        else {
            counter++
            return false
        }
    }
}