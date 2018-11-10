package com.team2898.engine.logic

import com.team2898.engine.logging.reflectLocation

class StateMachine {
    private var toActions = mutableMapOf<String, () -> Unit>()
    private var fromActions = mutableMapOf<String, () -> Unit>()
    private var whileActions = mutableMapOf<String, () -> Unit>()

    var currentState: String = ""
        private set

    @Synchronized
    infix fun changeStateTo(instanceOfEnum: Any) {
        transitionFrom(currentState)
        currentState = instanceOfEnum.toString()
        transitionTo(currentState)
    }

    @Synchronized
    fun update() {
        //println("Current state machine mode: $currentState\n" +
        //        "Lambda for state: ${whileActions[currentState] ?: "null"}")
        whileActions[currentState]?.invoke()
    }


    @Synchronized
    fun registerTo(instanceOfEnum: Any, block: () -> Unit) = toActions.put(instanceOfEnum.toString(), block)

    @Synchronized
    fun registerFrom(instanceOfEnum: Any, block: () -> Unit) = fromActions.put(instanceOfEnum.toString(), block)

    @Synchronized
    fun registerWhile(instanceOfEnum: Any, block: () -> Unit) = whileActions.put(instanceOfEnum.toString(), block)

    private fun transitionFrom(start: String) {
        toActions[start]?.invoke()
    }

    private fun transitionTo(end: String) {
        fromActions[end]?.invoke()
    }
}
