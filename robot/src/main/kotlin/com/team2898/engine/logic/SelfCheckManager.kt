package com.team2898.engine.logic

object SelfCheckManager {
    val subsystems = mutableListOf<Subsystem>()

    @Synchronized
    fun register(subsystem: Subsystem) = subsystems.add(subsystem)

    private fun all(action: ISelfCheck.() -> Boolean): Map<Subsystem, Boolean> {
        val resultMap = mutableMapOf<Subsystem, Boolean>()
        subsystems.forEach { subsystem ->
            resultMap.put(subsystem, subsystem.action())
        }
        return resultMap
    }

    @Synchronized
    fun checkAll() = all { selfCheckup() }

    @Synchronized
    fun testAll() = all { selfTest() }

    @Synchronized
    fun check(subsystem: Subsystem) = subsystem.selfCheckup()

    @Synchronized
    fun test(subsystem: Subsystem) = subsystem.selfTest()
}