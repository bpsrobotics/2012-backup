package com.team2898.engine.util

import edu.wpi.first.wpilibj.Timer

fun millis(): Double = Timer.getFPGATimestamp()*1000
fun seconds(): Double = Timer.getFPGATimestamp()
