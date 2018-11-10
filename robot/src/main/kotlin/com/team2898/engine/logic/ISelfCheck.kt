package com.team2898.engine.logic

interface ISelfCheck {
    /** selfCheckup is a quick test that simply ensures sane software values
     * @return whether or not check returned healthy
     */
    fun selfCheckup(): Boolean = true

    /** selfTest runs a thorough self test, possibly involving motor movement and manipulation
     * @return whether or not check returned healthy
     */
    fun selfTest(): Boolean = true
}