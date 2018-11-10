package com.team2898.engine.types

/*
* Interface used to denote a class or value able to be intepolated between two values
*/

interface Interpolable<T> {

    /*
    * Interpolates by given paramter (x) 
    * If interpolatePoint == 0, method should return lower value (normal)
    * If interpolatePoint == 1, method should return upper value (upperVal)
    * If 0 < interpolatePoint < 1, method should return an interpolation between the lower and higher values
    */

    fun interpolate(upperVal: T, interpolatePoint: Double): T
}
