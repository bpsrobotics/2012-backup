package com.team2898.engine.controlLoops

import com.team2898.engine.math.clamp
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.BoundaryException

/**
 * Standard PID control loop
 * Everything is synchronous, which means that you have to manually call the update()  function
 * Adapted with modifications from Team 254's library
 * @author Solomon
 * TODO {Replace importdoc with the actual things from IPID interface (then delete ipid)}
 */

open class StandardPID {
    var Kp: Double = 0.0
    var Ki: Double = 0.0
    var Kd: Double = 0.0
    var Kf: Double = 0.0

    var integrator: Double = 0.0
    var derivator: Double = 0.0

    var integratorMax: Double = Double.POSITIVE_INFINITY

    var maxOut: Double = Double.POSITIVE_INFINITY
    var minOut: Double = Double.NEGATIVE_INFINITY

    var maxIn: Double = Double.POSITIVE_INFINITY
    var minIn: Double = Double.NEGATIVE_INFINITY

    var continuous: Boolean = false

    var prevError: Double = 0.0

    var setpoint: Double = 0.0
        set(value) {
                field = clamp(value,min=minIn,max=maxIn)
        } // Bound value to min and max input values

    var error: Double = 0.0

    var output: Double = 0.0

    var deadband: Double = 0.0

    var input: Double = 0.0

    var lastInput: Double = 0.0

    var lastTime: Double = 0.0

    var updated = false // this will prevent extraneous calling of ontarget and stuff

    var tolerance = 0.0

    val onTarget: Boolean
        get() = updated && Math.abs(lastInput - setpoint) < tolerance

    constructor () {}

    /** Creates PID controller with PIDF coefficients given
     * @param Kp
     *  proportonal coefficient
     * @param Ki
     *  integral coefficient
     * @param Kd
     *  derivative coefficient
     * @param Kf
     *  feed-forward coefficient
     */
    constructor (Kp: Double = 0.0, Ki: Double = 0.0, Kd: Double = 0.0, Kf: Double = 0.0, integratorMax: Double = Double.POSITIVE_INFINITY, currentTime: Double = Timer.getFPGATimestamp()) {
        this.Kp = Kp
        this.Ki = Ki
        this.Kd = Kd
        this.Kf = Kf
        this.integratorMax = integratorMax
        lastTime = currentTime
    }

    /**
     *  {@inheritDoc}
     */
    fun update(inputVal: Double, currentTime: Double = Timer.getFPGATimestamp()): Double {
        var dt = currentTime - lastTime
        lastTime = currentTime

        lastInput = inputVal
        input = inputVal
        error = setpoint - input

        // Wraparound error if sensor is continuous
        if (continuous && (Math.abs(error) > (maxIn - minIn) / 2)) {
            error = if (error > 0) error - maxIn + minIn else error + maxIn + minIn
        }

        // Update integral
        if (minOut < error * Kp && error * Kp < maxOut)
            integrator += error
        else integrator = 0.0

        // Update derivative
        derivator = (error - prevError) * dt

        // We want to keep error as it is, but we don't want to do proportional control if error is less than deadband
        val kpError: Double = if (Math.abs(error) < deadband) 0.0 else error

        output = Kp * kpError + Ki * integrator + Kd * derivator

        prevError = error

        // Clamp output
        output = if (output > maxOut) maxOut else if (output < minOut) minOut else output

        updated = true

        return output
    }

    /**
     * {@inheritDoc}
     */
    fun setPID(Kp: Double, Ki: Double, Kd: Double, Kf: Double) { // Defaults are set in IPID interface
        this.Kp = Kp
        this.Ki = Ki
        this.Kd = Kd
        this.Kf = Kf
    }


    /**
     * {@inheritDoc}
     */
    fun setInputRange(minIn: Double, maxIn: Double) {
        if (minIn > maxIn) throw BoundaryException("Lower input bound is greater than upper bound")
        this.minIn = minIn
        this.maxIn = maxIn
        setpoint = setpoint // So that the setpoint gets bounded to the new range
    }

    /**
     * {@inheritDoc}
     */
    fun setOutputRange(minOut: Double, maxOut: Double) {
        if (minOut > maxOut) throw BoundaryException("Lower output bound is greater than upper bound")

        this.minOut = minOut
        this.maxOut = maxOut
    }


    /**
     * {@inheritDoc}
     */
    fun reset() {
        prevError = 0.0
        lastInput = 0.0
        derivator = 0.0
        integrator = 0.0
        error = 0.0
        output = 0.0
        setpoint = 0.0
        updated = false
    }
}
