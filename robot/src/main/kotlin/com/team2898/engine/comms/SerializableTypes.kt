package com.team2898.engine.comms.SerializableTypes

//import kotlinx.serialization.Serializable

// Basically just a serializable stuff to make it easier to work with sending and recieving stuff from ROS

//@Serializable
data class SerializableVector3(val x: Double, val y: Double, val z: Double)

//@Serializable
data class SerializablePoint(val x: Double, val y: Double, val z: Double)

//@Serializable
data class SerializableQuaternion(val x: Double, val y: Double, val z: Double, val w: Double)

//@Serializable
data class SerializableTwist(val linear: SerializableVector3, val angular: SerializableVector3)

//@Serializable
data class SerializablePose(val position: SerializablePoint, val orientation: SerializableQuaternion)

