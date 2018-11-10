package com.team2898.engine.OI

import edu.wpi.first.wpilibj.Timer

class ToggleDebounce(val debounceTime: Int = 50, val onFall: () -> Unit = {}, val onRise: () -> Unit = {}) {
    var state = false
    var lastTime = Timer.getFPGATimestamp()

    fun buttonPressed(newState: Boolean): Boolean {
        //if (newState && state) return

        if (Timer.getFPGATimestamp() - lastTime > debounceTime) {
            if (!state && newState) onRise() // rising edge
            else if (state && !newState) onFall()

            state = newState
            lastTime = Timer.getFPGATimestamp()
        }
        return state
    }
}