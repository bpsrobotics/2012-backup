package com.team2898.engine.motion

import com.team2898.engine.math.linear.Matrix
import org.apache.commons.math3.stat.correlation.Covariance

class KalmanFilter(
        val stateTransitionMatrix: Matrix,
        val observationMatrix: Matrix,
        val processNoiseCovariance: Matrix,
        val observationNoiseCovariance: Matrix,
        val controlInputModel: Matrix
)