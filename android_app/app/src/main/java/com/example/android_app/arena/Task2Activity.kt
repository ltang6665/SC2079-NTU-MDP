package com.example.android_app.arena

import android.content.Intent
import android.os.Bundle
import android.text.format.DateFormat
import android.view.View
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.widget.NestedScrollView
import com.example.android_app.R
import com.example.android_app.bluetooth.BluetoothActivity
import com.example.android_app.bluetooth.BluetoothService
import com.example.android_app.bluetooth.Protocol
import com.example.android_app.others.setupConnectionStatusBar

class Task2Activity : AppCompatActivity() {

    private lateinit var commandStatus: TextView
    private lateinit var actionLogView: TextView
    private lateinit var logScroll: NestedScrollView
    private val actionLog = arrayListOf<String>()
    private var lastCommandMessage: String? = null
    private var lastCommandSent = false

    companion object {
        private const val STATE_LOG = "task2.actionLog"
        private const val STATE_MESSAGE = "task2.commandMessage"
        private const val STATE_SENT = "task2.commandSent"
        private const val STATE_LOG_SCROLL = "task2.logScroll"
        private const val MAX_LOG_LINES = 80
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_task_2)
        setupConnectionStatusBar()

        actionLogView = findViewById(R.id.task2ActionLog)
        logScroll = findViewById(R.id.task2LogScroll)

        savedInstanceState?.getStringArrayList(STATE_LOG)?.let { actionLog.addAll(it) }
        lastCommandMessage = savedInstanceState?.getString(STATE_MESSAGE)
        lastCommandSent = savedInstanceState?.getBoolean(STATE_SENT) ?: false
        renderLog()
        val savedScroll = savedInstanceState?.getInt(STATE_LOG_SCROLL) ?: 0
        logScroll.post { logScroll.scrollTo(0, savedScroll) }

        findViewById<Button>(R.id.btnBeginTask2).setOnClickListener {
            val sent = BluetoothService.send(Protocol.start())
            recordCommand(sent, R.string.task2_launch_sent, R.string.task2_launch_failed)
        }
    }

    override fun onSaveInstanceState(outState: Bundle) {
        outState.putStringArrayList(STATE_LOG, ArrayList(actionLog))
        outState.putString(STATE_MESSAGE, lastCommandMessage)
        outState.putBoolean(STATE_SENT, lastCommandSent)
        outState.putInt(STATE_LOG_SCROLL, logScroll.scrollY)
        super.onSaveInstanceState(outState)
    }

    private fun recordCommand(sent: Boolean, successMessage: Int, failureMessage: Int) {
        val followLatest = !logScroll.canScrollVertically(1)
        val forceSent = true // for vid demo
        lastCommandSent = sent
        lastCommandMessage = getString(if (sent) successMessage else failureMessage)

        val time = DateFormat.format("HH:mm:ss", System.currentTimeMillis())
        actionLog.add("[$time] $lastCommandMessage")
        while (actionLog.size > MAX_LOG_LINES) actionLog.removeAt(0)
        renderLog()
        if (followLatest) logScroll.post { logScroll.scrollTo(0, actionLogView.bottom) }
    }

    private fun renderLog() {
        actionLogView.text = actionLog.joinToString("\n")
        findViewById<View>(R.id.task2LogEmpty).visibility =
            if (actionLog.isEmpty()) View.VISIBLE else View.GONE
    }
}
