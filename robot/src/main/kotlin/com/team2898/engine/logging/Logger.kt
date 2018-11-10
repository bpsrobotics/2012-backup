package com.team2898.engine.logging

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.util.WaitGroup
import com.team2898.engine.async.util.go
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.experimental.runBlocking
import java.io.File
import java.util.UUID.randomUUID
import kotlin.system.measureTimeMillis

enum class LogLevel {
    INFO, WARNING, ERROR, FATAL, DEBUG
}


object Logger {
    val uuid = randomUUID()?.toString() ?: "UUID_GEN_FAILURE"

    val folderPath: String = "/home/lvuser/logs"
    val infoLogName = "$folderPath/infolog-$uuid.log"
    val dataLogName = "$folderPath/datalog-$uuid.log"

    var flushHz: Double = 1.0

    val infoLogBuffer = mutableListOf<InfoLog>()
    val dataLogBuffer = mutableListOf<DataLog>()
    val infoLogFileRef: File by lazy { File(infoLogName) }
    val dataLogFileRef: File by lazy { File(dataLogName) }

    init {
        logInfo(reflectLocation(), LogLevel.INFO, "Logger successfully initiated with a UUID of $uuid")
        infoLogFileRef.writeText("")
        dataLogFileRef.writeText("")
        flushBuffers()
        SmartDashboard.putString("Session UUID", uuid)
    }

    @Synchronized
    fun start() = flushLooper.start()

    @Synchronized
    fun stop() = flushLooper.stop()

    val flushLooper: AsyncLooper = AsyncLooper(flushHz) {
        val timeTaken = flushBuffers()
        logData(javaClass.name, "buffer_flush_ms", timeTaken)
    }

    /*
    @Synchronized
    fun flushBuffers() {
        WaitGroup()
                .add {
                    println("0a")
                    synchronized(infoLogBuffer) {
                        println("1a")
                        if (infoLogBuffer.size > 0) {
                            println("2a")
                            var tmp: String = ""
                            println("3a")
                            infoLogBuffer.forEach { log -> tmp += log.toString() + "\n"}
                            println("4a")
                            infoLogFileRef.appendText(tmp)
                            println("5a")
                            infoLogBuffer.clear()
                        }
                    }
                }
                .add {
                    println("0b")
                    synchronized(dataLogBuffer) {
                        println("1b")
                        if (dataLogBuffer.size > 0) {
                            println("2b")
                            var tmp: String = ""
                            println("3b")
                            dataLogBuffer.forEach { log -> tmp += log.toString() }
                            println("4b")
                            dataLogFileRef.appendText(tmp)
                            println("5b")
                            dataLogBuffer.clear()
                        }
                    }
                }
                .wait()
    }
    */
    @Synchronized
    fun flushBuffers() {
        if (infoLogBuffer.size > 0) {
            var tmp: String = ""
            infoLogBuffer.forEach { log -> tmp += log.toString() + "\n" }
            infoLogFileRef.appendText(tmp)
            infoLogBuffer.clear()
        }
        if (dataLogBuffer.size > 0) {
            var tmp: String = ""
            dataLogBuffer.forEach { log -> tmp += log.toString() + "\n"}
            dataLogFileRef.appendText(tmp)
            dataLogBuffer.clear()
        }
    }


    fun logInfo(line: InfoLog) = synchronized(infoLogBuffer) {
        println("${line.toString()}\n")
        infoLogBuffer.add(line)
    }
    fun logData(line: DataLog) = synchronized(dataLogBuffer) {
        dataLogBuffer.add(line)
    }

    fun logInfo(source: String, level: LogLevel, message: String) = logInfo(InfoLog(source, level, message))
    fun logData(source: String, name: String, data: Any) = logData(DataLog(source, name, data))

    // Dumb ancillary ease of use stuff

    // fun logInfo(name: String, level: LogLevel, data: String) = log(LogLine(name, level, data))
    // fun logInfo(name: String, level: String, data: String) = logInfo(name, LogLevel.valueOf(level.toUpperCase()), data)
    // fun logInfo(name: String, level: LogLevel, data: Throwable) = logInfo(name, level, getStackTraceString(data))
    // fun logInfo(name: String, level: String, data: Throwable) = logInfo(name, level, getStackTraceString(data))
}
