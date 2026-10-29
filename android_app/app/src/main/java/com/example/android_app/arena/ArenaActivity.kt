package com.example.android_app.arena

import android.os.Bundle
import android.text.format.DateFormat
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.activity.addCallback
import android.view.View
import android.widget.ScrollView
import androidx.core.widget.NestedScrollView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.example.android_app.R
import com.example.android_app.bluetooth.BluetoothService
import com.example.android_app.bluetooth.Protocol
import com.example.android_app.others.setupConnectionStatusBar
import kotlinx.coroutines.launch

class ArenaActivity : AppCompatActivity() {

    private lateinit var arena: ArenaView
    private lateinit var status: TextView
    private lateinit var statusScroll: NestedScrollView // for auto scrolling of status log

    var compiledObstacles = mutableListOf<Obstacle>()

    companion object {
        private const val TAG = "ArenaActivity12345"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_arena)
        setupConnectionStatusBar()

        arena = findViewById(R.id.arenaView)
        status = findViewById(R.id.statusText)
        statusScroll = findViewById(R.id.statusScroll)

        wireArenaCallbacks()
        wireControls()
        onBackPressedDispatcher.addCallback(this) { confirmLeaveArena() }
        observeMessages()

        appendStatus(getString(R.string.status_ready))
    }

    private fun wireArenaCallbacks() {
        arena.onObstaclePlaced = { obs ->
            Log.d(TAG, "Callback [onObstaclePlaced]")
            // BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
            appendStatus("Placed obstacle ${obs.id} at (${obs.cellX},${obs.cellY})")
            refreshArenaReadout()
        }

        /*arena.onObstacleMoved = { obs ->
            BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
            appendStatus("Moved obstacle ${obs.id} to (${obs.cellX},${obs.cellY})")
        }*/
        arena.onObstacleDropped = { obs ->
            Log.d(TAG, "Callback [onObstacleDropped]")
            // BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
            addToCompiled(obs, "dropped")
            appendStatus("Dropped obstacle ${obs.id} at (${obs.cellX},${obs.cellY})")
        }
        arena.onObstacleRemoved = { obs ->
            Log.d(TAG, "Callback [onObstacleRemoved]")
            // BluetoothService.send(Protocol.obstacleDeleted(obs.id, obs.cellX, obs.cellY))
            addToCompiled(obs, "remove")
            appendStatus("Removed obstacle ${obs.id}")
            refreshArenaReadout()
        }
        arena.onObstacleLongPress = { obs ->
            Log.d(TAG, "Callback [onObstacleLongPress]")
            showFaceDialog(obs)
        }

        // new btn to send compiled list of obstacles
    }

    private fun addToCompiled(obs: Obstacle, instruction: String) {
        if (instruction == "longPress" || instruction == "dropped") {
            // add or update obstacle in compiled list
            compiledObstacles.removeAll { it.id == obs.id }
            compiledObstacles.add(obs)

            /* if order of obstacles matter:
                val index = compiledObstacles.indexOfFirst { it.id == obs.id }
                if (index >= 0) compiledObstacles[index] = obs else compiledObstacles.add(obs)
             */
        } else if (instruction == "remove") {
            // remove obstacle from list
            compiledObstacles.removeAll { it.id == obs.id }
        }
    }

    private fun wireControls() {
        findViewById<Button>(R.id.btnArenaHelp).setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle(R.string.heist_help_title)
                .setMessage(R.string.heist_help_body)
                .setPositiveButton(R.string.heist_got_it, null)
                .show()
        }
        findViewById<Button>(R.id.btnForward).setOnClickListener {
            sendMove(Protocol.MoveCmd.FORWARD, "Forward")
        }
        findViewById<Button>(R.id.btnBackward).setOnClickListener {
            sendMove(Protocol.MoveCmd.BACKWARD, "Backward")
        }
        findViewById<Button>(R.id.btnLeft).setOnClickListener {
            sendMove(Protocol.MoveCmd.LEFT, "Turn left")
        }
        findViewById<Button>(R.id.btnRight).setOnClickListener {
            sendMove(Protocol.MoveCmd.RIGHT, "Turn right")
        }
        findViewById<Button>(R.id.btnStop).setOnClickListener {
            sendMove(Protocol.MoveCmd.STOP, "Stop")
        }
        findViewById<Button>(R.id.btnReady).setOnClickListener {
            compiledObstacles.forEach { obs ->
                val face = obs.face
                if (face != null) {
                    BluetoothService.send(Protocol.face(obs.id, obs.cellX, obs.cellY, face))
                } else {
                    BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
                }
            }

            if (BluetoothService.send(Protocol.calc())) {
                appendStatus("mdp28!")

                Log.d(TAG, "Attempting to send obstacles individually...")

                /*Log.d(TAG, "Obstacles sent (${compiledObstacles.size}): " +
                        compiledObstacles.joinToString { "id=${it.id} (${it.cellX},${it.cellY}) face=${it.face}" })*/

            } else {
                appendStatus("CALCULATE failed — not connected")
                // appendStatus("mdp28!") For vid demo

                Log.d(TAG, "Going to send the list of obstacles")

                Log.d(TAG, "Attempting to send obstacles individually...")

                compiledObstacles.forEach { obs ->
                    val face = obs.face
                    if (face != null) {
                        BluetoothService.send(Protocol.face(obs.id, obs.cellX, obs.cellY, face))
                    } else {
                        BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
                    }
                }
                Log.d(TAG, "Obstacles sent (${compiledObstacles.size}): " +
                        compiledObstacles.joinToString { "id=${it.id} (${it.cellX},${it.cellY}) face=${it.face}" })

            }
        }
        findViewById<Button>(R.id.btnStart).setOnClickListener {
            if (BluetoothService.send(Protocol.start())) {
                appendStatus("Sent START")
            } else {
                appendStatus("START failed — not connected")

                /* For vid demo
                lifecycleScope.launch {
                    handleInbound("TARGET, 1, 18")
                    kotlinx.coroutines.delay(1000)
                    handleInbound("TARGET, 3, 29")
                    kotlinx.coroutines.delay(2500)
                    handleInbound("TARGET, 2, 36")
                } */
            }
        }
        findViewById<Button>(R.id.btnReset).setOnClickListener {
            arena.reset()
            compiledObstacles.clear()
            refreshArenaReadout()

            if (BluetoothService.send(Protocol.reset())) {
                appendStatus("Arena reset")
            } else {
                appendStatus("RESET failed - not connected")
                // appendStatus("Arena reset")
            }
        }
    }

    private fun sendMove(cmd: Protocol.MoveCmd, label: String) {
        if (BluetoothService.send(Protocol.move(cmd))) {
            appendStatus(label)
            when (cmd) {
                Protocol.MoveCmd.LEFT ->
                    arena.setRobot(arena.robot.cellX, arena.robot.cellY, arena.robot.facing.turnLeft())
                Protocol.MoveCmd.RIGHT ->
                    arena.setRobot(arena.robot.cellX, arena.robot.cellY, arena.robot.facing.turnRight())
                else -> Unit
            }
        } else {
            appendStatus("$label failed — not connected")
        }
    }

    private fun showFaceDialog(obs: Obstacle) {
        val labels = arrayOf(
            getString(R.string.face_north),
            getString(R.string.face_east),
            getString(R.string.face_south),
            getString(R.string.face_west),
            getString(R.string.face_none),
        )
        AlertDialog.Builder(this)
            .setTitle(getString(R.string.label_face_dialog, obs.id))
            .setItems(labels) { _, which ->
                val face = when (which) {
                    0 -> Facing.NORTH
                    1 -> Facing.EAST
                    2 -> Facing.SOUTH
                    3 -> Facing.WEST
                    else -> null
                }
                arena.setObstacleFace(obs.id, face)
                if (face != null) {
                    // BluetoothService.send(Protocol.obstacle(obs.id, obs.cellX, obs.cellY))
                    // BluetoothService.send(Protocol.face(obs.id, obs.cellX, obs.cellY, face))
                    addToCompiled(obs, "longPress")
                    appendStatus("Obstacle ${obs.id} face -> ${face.value}")
                    Log.d(TAG, "Protocol.face(obs.id, face")
                } else {
                    appendStatus("Obstacle ${obs.id} face cleared")
                }
            }
            .show()
    }

    private fun handleInbound(line: String) {
        when (val msg = Protocol.parse(line)) {
            is Protocol.Inbound.Target -> {
                arena.setTargetId(msg.obstacleId, msg.targetId)
                appendStatus("Obstacle ${msg.obstacleId} -> target ${msg.targetId}")
                refreshArenaReadout()
            }
            is Protocol.Inbound.Robot -> {
                arena.setRobot(msg.x, msg.y, msg.facing)
                // C.4 note: selective log — pose updates are important events, not every raw stream byte.
                appendStatus("Robot @ (${msg.x},${msg.y}) ${msg.facing.value}")
            }
            is Protocol.Inbound.Ready -> {
                // appendStatus("Path calculation ready!")
                appendStatus("TASK 1!!!!!!!!!!!!!!!!!!!!!!!")
            }
            is Protocol.Inbound.Unknown -> {
                // Deliberately NOT logged to the visible status
                appendStatus("RPi sent ${msg}")
            }
        }
    }

    private fun appendStatus(msg: String) {
        val time = DateFormat.format("HH:mm:ss", System.currentTimeMillis())
        val current = status.text?.toString().orEmpty()
        val trimmed = if (current.length > 4000) current.takeLast(2000) else current
        val line = "[$time] $msg"
        status.text = if (trimmed.isEmpty()) line else "$trimmed\n$line"

        statusScroll.post {
            statusScroll.smoothScrollTo(0, status.bottom)
        }
    }

    private fun observeMessages() {
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                BluetoothService.messages.collect { line ->
                    handleInbound(line)
                }
            }
        }
    }

    private fun confirmLeaveArena() {
        if (arena.obstacles().isEmpty()) {
            finish()
        } else {
            AlertDialog.Builder(this)
                .setTitle(R.string.heist_leave_title)
                .setMessage(R.string.heist_leave_body)
                .setNegativeButton(R.string.heist_cancel, null)
                .setPositiveButton(R.string.heist_leave) { _, _ -> finish() }
                .show()
        }
    }

    private fun refreshArenaReadout() {
        val obstacles = arena.obstacles()
        val summaryView = findViewById<TextView>(R.id.arenaSummary)
        if (summaryView != null) {
            summaryView.text = getString(
                R.string.heist_obstacles,
                obstacles.size,
                obstacles.count { it.targetId != null }
            )
        }
    }
}