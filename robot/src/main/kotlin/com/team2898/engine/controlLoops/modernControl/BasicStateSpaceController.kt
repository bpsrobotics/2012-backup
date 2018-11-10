package com.team2898.engine.controlLoops.modernControl

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.minus
import com.team2898.engine.math.linear.times
import org.apache.commons.math3.linear.RealMatrix

class BasicStateSpaceController<E : Enum<*>>(numInputs: Int, numOutputs: Int, numStates: Int, schedule: E, gains: (E) -> StateSpaceGains)
    : StateSpaceController<E>(numInputs, numOutputs, numStates, schedule, gains
) {

    init {
    }

    override fun update(r: RealMatrix, x: RealMatrix): RealMatrix =
            r - (K * x)
}