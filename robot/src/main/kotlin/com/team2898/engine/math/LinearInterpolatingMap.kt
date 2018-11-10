package com.team2898.engine.math

import java.util.Map
import java.util.TreeMap

class LinearInterpolatingMap {
    // TODO make samples(at least 10?)
    //val sampleMap: MutableMap<Double, Double> = mutableMapOf()
    private val sampleMap: TreeMap<Double, Double> = TreeMap()

    constructor() {}

    constructor(vararg samples: Pair<Double, Double>) {
        for (sample in samples) {
            sampleMap[sample.first] = sample.second
        }
    }

    operator fun set(key: Double, value: Double) {
        sampleMap[key] = value
    }

    operator fun get(key: Double): Double {
        return interpolate(key)
    }

    fun interpolate(interpPos: Double): Double {
        var interpPos = interpPos
        if (interpPos >= sampleMap.lastKey() && sampleMap.lastEntry() != null)
            return sampleMap.lastEntry().value
        else if (interpPos <= sampleMap.firstKey() && sampleMap.firstEntry() != null)
            return sampleMap.firstEntry().value

        if (sampleMap.containsKey(interpPos) && sampleMap[interpPos] != null) {
            return sampleMap.getValue(interpPos)
        }

        val higherEntry: Pair<Double, Double> = sampleMap.higherEntry(interpPos).toPair()
        val lowerEntry: Pair<Double, Double> = sampleMap.lowerEntry(interpPos).toPair()
        var higherKey = higherEntry.first
        var lowerKey = lowerEntry.first
        val higherValue = higherEntry.second
        val lowerValue = lowerEntry.second

        lowerKey -= lowerKey
        higherKey -= lowerKey
        interpPos -= lowerKey

        // lowerKey is 0 anyways.....
        higherKey /= higherKey
        interpPos /= higherKey

        return (1 - interpPos) * lowerValue + interpPos * higherValue

//        (0, higherKey - lowerKey) kinda?
//        also, interpPos - lowKey
//         1-pointOfInterest * lowerValue + pointOfInterest * higherValue
//         soooooo loewrKey - lowekey, higherkey - lowerkey???
//         if (sampleMap.containsKey(interpPos)) return sampleMap.getValue(interpPos)
//        🤔 thonking time
    }


}

