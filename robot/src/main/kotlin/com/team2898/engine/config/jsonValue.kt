package com.team2898.engine.config

import com.google.gson.*
import java.io.*

fun getValueFromJson(path: String): String {
    val element: MutableList<String> = path.split(".").toMutableList()
    val file = element[0] + ".json"

    element.remove(element[0])

    element.toList()

    val json = File(file).readText()
    val root = JsonParser().parse(json)

    var value = root

    for (i in element){
        value = value.getAsJsonObject().get(i)
    }

    return value.asString
}