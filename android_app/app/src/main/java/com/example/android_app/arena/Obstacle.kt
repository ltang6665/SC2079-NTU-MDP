package com.example.android_app.arena

/**
 * Placed obstacle on the arena.
 *
 * @param id      supervisor-visible number (1..n)
 * @param cellX   column, 0-indexed, 0 = leftmost
 * @param cellY   row,    0-indexed, 0 = bottom (arena convention)
 * @param face    which side of the block holds the target image (null = unset)
 * @param targetId numeric target ID once recognised by RPi image module (null = unrecognised)
 */
data class Obstacle(
    val id: Int,
    var cellX: Int,
    var cellY: Int,
    var face: Facing? = null,
    var targetId: Int? = null,
)