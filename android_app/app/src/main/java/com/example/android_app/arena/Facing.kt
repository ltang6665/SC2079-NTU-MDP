package com.example.android_app.arena

/** Compass direction mapped to string representation for the Raspberry Pi. */
enum class Facing(val value: String) {
    NORTH("NORTH"),
    EAST("EAST"),
    SOUTH("SOUTH"),
    WEST("WEST");

    fun turnLeft(): Facing = entries[(ordinal + 3) % 4]
    fun turnRight(): Facing = entries[(ordinal + 1) % 4]

    companion object {
        fun fromValue(value: String?): Facing? = entries.firstOrNull { it.value.equals(value, ignoreCase = true) }
    }
}