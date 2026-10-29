package com.example.android_app.bluetooth

import com.example.android_app.arena.Facing

/**
 * String formats exchanged with the RPi over RFCOMM.
 *
 * INBOUND (RPi -> Tablet):
 *   TARGET,<obstacle_number>,<target_id>          -> C.9  set target ID on obstacle
 *   ROBOT,<x>,<y>,<direction>                     -> C.10 update robot pose
 * OUTBOUND (Tablet -> RPi):
 *   OBSTACLE,<num>,<x>,<y>                        -> C.6  obstacle placed / moved
 *   OBSTACLE_DEL,<num>                            -> C.6  obstacle dragged off map
 *   FACE,<num>,<direction>                        -> C.7  target face selected
 *   ROBOT_MOVE,<F|B|L|R|S>                        -> C.3  manual movement
 *   START
 */
object Protocol {

    // --- outbound ---
    fun obstacle(id: Int, x: Int, y: Int): String = "OBSTACLE,$id,$x,$y"
    fun obstacleDeleted(id: Int, x: Int, y: Int): String = "CLEAR,$id,$x,$y,-1"
    fun face(id: Int, x: Int, y: Int, dir: Facing): String = "OBSTACLE,$id,$x,$y,${dir.value}"
    fun move(cmd: MoveCmd): String = "ROBOT_MOVE,${cmd.token}"
    fun start(): String = "BEGIN"
    fun calc(): String = "CALCULATE"

    fun reset(): String = "RESET"

    enum class MoveCmd(val token: Char) {
        FORWARD('F'), BACKWARD('B'), LEFT('L'), RIGHT('R'), STOP('S')
    }

    // --- inbound ---
    sealed interface Inbound {
        data class Target(val obstacleId: Int, val targetId: Int) : Inbound
        data class Robot(val x: Int, val y: Int, val facing: Facing) : Inbound
        data object Ready : Inbound
        data class Unknown(val raw: String) : Inbound
    }

    /**
     * Parse a single line from the RPi.
     * Whitespace around fields is tolerated. Returns [Inbound.Unknown] for
     * anything we don't recognise so the caller can still log it.
     */
    fun parse(line: String): Inbound {
        val trimmed = line.trim()
        val parts = line.split(",").map { it.trim() }
        return when {
            trimmed.equals("PATHREADY", ignoreCase = true) -> Inbound.Ready

            parts.size == 3 && parts[0].equals("TARGET", ignoreCase = true) -> {
                val obs = parts[1].toIntOrNull()
                val tgt = parts[2].toIntOrNull()
                if (obs != null && tgt != null) Inbound.Target(obs, tgt)
                else Inbound.Unknown(line)
            }
            parts.size == 4 && parts[0].equals("ROBOT", ignoreCase = true) -> {
                val x = parts[1].toIntOrNull()
                val y = parts[2].toIntOrNull()
                val dir = Facing.fromValue(parts[3])
                if (x != null && y != null && dir != null) Inbound.Robot(x, y, dir)
                else Inbound.Unknown(line)
            }
            else -> Inbound.Unknown(line)
        }
    }
}