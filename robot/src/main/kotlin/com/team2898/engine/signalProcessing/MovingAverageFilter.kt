package com.team2898.engine.signalProcessing

import com.team2898.engine.types.CircularArray

/** Moving average filter
 * Useful for smoothing out noise in a system.
 * @param averagePeriod period over which it will be averaged --- values older than that will be deleted
 * @param weightingEquation equation that will be used to find the weighting of the values based on index. Default is standard equal weighting
 */
open class MovingAverageFilter (var averagePeriod: Int) {
    var mainArray: CircularArray<Double> = CircularArray<Double>(averagePeriod)

    private var weightingEquation: (Int, Double) -> Double = {_, value -> value}

    /**
     * Set the internal weighting equation to equal weighting (everything is weighted the same)
     */
    fun setEqualWeighting() {
        weightingEquation = {_, value -> value}
    }

    /**
     * Set the internal weighting equation to linear weighting (newer are weighted more with a linear falloff)
     */
    fun setLinearWeighting() {
        throw NotImplementedError("do a linear weighting dumbass")
        // weightingEquation = {index, value -> value * index/mainArray.size}
    }

    // fun setExpDecay() = setWeightingEquation({time, value -> value * (period - time)/period})

    /** Sets the internal weighting equation to a custom one
     * @param eq lambda in the form of (Index, Value -> WeightedValue)
     */
    fun setWeightingEquation(eq: (Int, Double) -> Double) {
        weightingEquation = eq
    }

    protected fun average(): Double {
        var avg: Double = 0.0
        val size: Int = mainArray.size
        for (i in (size-1 downTo 0)) {
            avg += weightingEquation(i, mainArray[i])
        }
        return avg / size
    }

    /**
     * Returns average of stored vals
     * @return average
     */
    open fun getAverage(): Double {
        return average()
    }

    /**
     * Adds value, then returns average of stored vals
     * @param value value to add before averaging
     * @return average
     */
    open fun getAverage(value: Double): Double {
        mainArray.add(value)
        return average()
    }

    /**
     * Adds value to internal ring
     * @param value value to add
     */
    fun addValue(value: Double) = mainArray.add(value)
}
