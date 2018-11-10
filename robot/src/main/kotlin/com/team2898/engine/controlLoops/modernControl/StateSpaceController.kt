package com.team2898.engine.controlLoops.modernControl

import com.team2898.engine.async.util.go
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.math.clamp
import com.team2898.engine.math.linear.*
import kotlinx.coroutines.experimental.runBlocking
import kotlinx.coroutines.experimental.sync.Mutex
import kotlinx.coroutines.experimental.sync.withLock
import org.apache.commons.math3.linear.RealMatrix

data class StateSpaceGains(val A: RealMatrix, val B: RealMatrix, val C: RealMatrix, val D: RealMatrix,
                           val K: RealMatrix, val Umax: RealMatrix, val Umin: RealMatrix)


// State space controller without kalman filter, using optimal feedback law
open class StateSpaceController<E : Enum<*>>(
        val numInputs: Int,
        val numOutputs: Int,
        val numStates: Int,
        schedule: E,
        val gains: (E) -> StateSpaceGains
) {

    var gainSchedule = schedule
        set(new) {
            field = new
            runBlocking {
                scheduleMutex.withLock {
                    updateGains()
                }
            }
        }

    val scheduleMutex = Mutex()

    protected var A: RealMatrix = Matrix(numStates, numStates)
    protected var B: RealMatrix = Matrix(numStates, numOutputs)
    protected var C: RealMatrix = Matrix(numOutputs, numStates)
    protected var D: RealMatrix = Matrix(numOutputs, numOutputs)
    protected var K: RealMatrix = Matrix(numOutputs, numStates)
    protected var Umin: RealMatrix = Matrix(numOutputs, 1)
    protected var Umax: RealMatrix = Matrix(numOutputs, 1)

    protected var U: RealMatrix = Matrix(numOutputs, 1)
    protected var Uuncapped: RealMatrix = Matrix(numOutputs, 1)

    init {
        updateGains()
    }

    fun updateGains() {
        val newGains = gains(synchronized(gainSchedule) { gainSchedule })
        try {
            assert(A.dim() == newGains.A.dim())
            assert(B.dim() == newGains.B.dim())
            assert(C.dim() == newGains.C.dim())
            assert(D.dim() == newGains.D.dim())
            assert(K.dim() == newGains.K.dim())
            assert(Umax.dim() == newGains.Umax.dim())
            assert(Umin.dim() == newGains.Umin.dim())
            A = newGains.A.copy()
            B = newGains.B.copy()
            C = newGains.C.copy()
            D = newGains.D.copy()
            K = newGains.K.copy()
            Umax = newGains.Umax.copy()
            Umin = newGains.Umin.copy()
        } catch (e: AssertionError) {
            Logger.logInfo(reflectLocation(), LogLevel.ERROR, "Warning, updated gain matrices wrong dimension")
            return
        }
    }

    /**
     * @param R Reference (setpoint) vector
     * @param X Current state vector
     */
    open fun update(r: RealMatrix, X: RealMatrix): RealMatrix =
            Matrix(0, 0)

    protected fun capU() {
        for (i in 0..numOutputs - 1) {
            val u_i = U[i, 1]
            val u_max = Umax[i, 1]
            val u_min = Umin[i, 1]
            U[i, 1] = clamp(value = u_i, min = u_min, max = u_max)
        }
    }
}