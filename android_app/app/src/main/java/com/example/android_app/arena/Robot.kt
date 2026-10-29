package com.example.android_app.arena

/**
 * Robot state on the arena.
 *
 * The robot occupies a 3x3 footprint. [cellX] and [cellY] are the CENTRE cell.
 * Facing is the direction the front of the robot points.
 */
data class Robot(
    var cellX: Int = 1,
    var cellY: Int = 1,
    var facing: Facing = Facing.NORTH,
)