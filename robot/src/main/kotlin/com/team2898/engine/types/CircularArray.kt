package com.team2898.engine.types

/** This implements a simple circular array
 * Essentially it'll just delete the oldest entry whenever the array gets longer than its specified max size
 * Index 0 is newest element, as index goes up elements get lower
 * DISCUSS {If performance is a concern, use a circular pointer instead of add/remove}
 * @param size size of circular array
 */
class CircularArray<T>(size: Int) {
    var size = size

    val queue: MutableList<T> = mutableListOf()

    /** Get value at index
     * @param index index to get value from
     * @return chosen value
     */
    operator fun get(index: Int): T = queue[index]

    /** Self explanitory. Adds value.
     * @param value value to add
     */
    fun add(value: T) {
        queue.add(0, value)
        prune()
    }

    /** Returns entire array as a list
     * @return List<T> of array
     */
    fun getAll(): List<T> = queue.toList()

    private fun prune() {
        while (queue.size > size) {
            queue.removeAt(size)
        }
    }

    // Allows normal use of [] indexing to get values
    // operator fun <T> CircularArray<T>.get(index: Int): T = this.get(index)
}

