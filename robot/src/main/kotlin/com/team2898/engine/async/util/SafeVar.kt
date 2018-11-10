package com.team2898.engine.async.util

import kotlinx.coroutines.experimental.runBlocking
import kotlinx.coroutines.experimental.sync.Mutex
import kotlinx.coroutines.experimental.sync.withLock

class SafeVar<T>(var value: T) {

    fun get(): T = synchronized(this) { return value }

    fun set(new: T) = synchronized(this) {
        value = new
    }

    fun runSafe(block: () -> Unit) = synchronized(this) {
        block()
    }


}
