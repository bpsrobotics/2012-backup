package com.team2898.engine.types

/**
* Denotes something as loggable, or able to have logging data extracted
*/

interface Loggable {
    fun getLog(): String
}
