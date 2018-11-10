package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs
import kotlin.math.max


class TrapezoidProfile(
        var maxAcc: Double, // in FSTU
        var maxVel: Double // in FSTU
) {
    data class PVAData(
            val currentPos: Double,
            val currentVel: Double,
            val targetPos: Double,
            val targetVel: Double,
            val targetAcc: Double,
            val time: Double = 0.0
    )

    /**
     * REST: resting
     * ACC: Accelerating
     * CONST: Const velocity
     * DEC: Decelerating
     */
    private enum class CurrentState { REST, ACC, CONST, DEC }

    val currentTime = { Timer.getFPGATimestamp() }

    var currentPos = 0.0
    var currentVel = 0.0

    private var OFFSET = 0.0

    private var currentState = CurrentState.REST

    private var lastTime = 0.0
    private var startTime = 0.0

    private var lastPos = 0.0
    private var lastVel = 0.0

    var targetPos = 0.0

    var lastProfile = PVAData(0.0, 0.0, 0.0, 0.0, 0.0)

    private var isFinished = false
    private var dec = false
    private var sign = kotlin.math.sign(targetPos)

    init {
        startTime = currentTime()
        updateProfile()
    }

    fun updateProfile() {
        lastPos = lastProfile.currentPos
        lastVel = lastProfile.currentVel
        currentPos = lastProfile.targetPos
        currentVel = lastProfile.targetVel
        lastTime = currentTime()
        OFFSET = targetPos * 0.02
    }

    fun updateTarget(target: Double) {
        isFinished = false
        startTime = currentTime()
        lastTime = currentTime()
        targetPos = target
        currentState = CurrentState.ACC
        updateProfile()
        sign = kotlin.math.sign(targetPos)
    }

    fun hasToDec(): Boolean {
        val time = abs(currentVel) / maxAcc
        val offSet = targetPos - currentPos
        return abs(offSet) <= abs(currentVel * time + sign * (-maxAcc / 2) * Math.pow(time, 2.0))
//        return math.fabs(offSet - self.__OFFSET) <= math.fabs(self.__currentVel * time + self.__sign * (-self.__maxAcc / 2) * math.pow(time, 2))
    }

    fun updateCurrentState() {
        currentState = if (abs(targetPos - currentPos) < targetPos / OFFSET && abs(currentVel) < maxVel / OFFSET) {
            isFinished = true
            CurrentState.REST
        } else if (hasToDec() || dec) {
            dec = true
            CurrentState.DEC
        } else if (abs(maxVel) <= abs(currentVel)) {
            CurrentState.CONST
        } else {
            CurrentState.ACC // This should never happen
        }
    }

    fun update(): PVAData {
        val deltaTime = currentTime() - lastTime
        lastTime = currentTime()
        if (isFinished) {
            lastProfile = PVAData(
                    currentPos = currentPos,
                    currentVel = currentVel,
                    targetPos = currentPos,
                    targetVel = 0.0,
                    targetAcc = 0.0,
                    time = lastTime
            )
            return lastProfile
        }
        updateProfile()
        updateCurrentState()
        lastProfile = when (currentState) {
            CurrentState.ACC -> {
                PVAData(
                        currentPos = currentPos,
                        currentVel = currentVel,
                        targetPos = currentPos + deltaTime * currentVel + sign * maxAcc/2 * Math.pow(deltaTime, 2.0),
                        targetVel = currentVel + sign * maxAcc * deltaTime,
                        targetAcc = sign * maxAcc,
                        time = lastTime
                )
            }
            CurrentState.DEC -> {
                PVAData(
                        currentPos = currentPos,
                        currentVel = currentVel,
                        targetPos = currentPos + deltaTime * currentVel + sign + -maxAcc/2 * Math.pow(deltaTime, 2.0),
                        targetVel = currentVel + sign * -maxAcc * deltaTime,
                        targetAcc = sign * -maxAcc,
                        time = lastTime
                )
            }
            CurrentState.CONST -> {
                PVAData(
                        currentPos = currentPos,
                        currentVel = currentVel,
                        targetPos = currentPos + deltaTime * currentVel,
                        targetVel = sign * maxVel,
                        targetAcc = 0.0,
                        time = lastTime
                )
            }
            CurrentState.REST -> {
                PVAData(
                        currentPos = currentPos,
                        currentVel = currentVel,
                        targetPos =  currentPos,
                        targetVel = 0.0,
                        targetAcc = 0.0,
                        time = lastTime
                )
            }
        }
        //println("vel: ${lastProfile.targetVel}, pos: ${lastProfile.targetPos}, finished: ${isFinished}")
        return lastProfile
    }

    fun isFinished(): Boolean {
        return isFinished
    }
}

