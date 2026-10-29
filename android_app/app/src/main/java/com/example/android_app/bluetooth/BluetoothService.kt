package com.example.android_app.bluetooth

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStream
import java.util.UUID

object BluetoothService {

    private const val TAG = "BluetoothService"
    // Standard SPP UUID used by RPi rfcomm channels
    private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private const val RECONNECT_DELAY_MS = 2500L
    private const val CONNECT_TIMEOUT_MS = 30_000L // 30 seconds connection timeout

    enum class State { DISCONNECTED, CONNECTING, CONNECTED, RECONNECTING }

    private val scope: CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.Main)
    private var connectionJob: Job? = null
    private var readerJob: Job? = null
    private var reconnectJob: Job? = null

    private var socket: BluetoothSocket? = null
    private var output: OutputStream? = null

    private var pendingDeviceAddress: String? = null

    private val _state = MutableStateFlow(State.DISCONNECTED)
    // private val _state = MutableStateFlow(State.CONNECTED) // For vid demo
    val state: StateFlow<State> = _state.asStateFlow()

    private val _connectedDevice = MutableStateFlow<String?>(null)
    // private val _connectedDevice = MutableStateFlow<String?>("MDPGrp28") // For vid demo
    val connectedDevice: StateFlow<String?> = _connectedDevice.asStateFlow()

    private val _messages = MutableSharedFlow<String>(
        replay = 0,
        extraBufferCapacity = 256
    )
    val messages: SharedFlow<String> = _messages.asSharedFlow()

    /**
     * Connect to [device]. Any current connection is closed first.
     * Remembers the address so we can auto-reconnect on disconnect (C.8).
     */
    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        // Cancel any pending tasks before attempting a fresh connection
        cancelConnectionJobs()
        pendingDeviceAddress = device.address

        connectionJob = scope.launch {
            doConnect(device)
        }
    }

    @SuppressLint("MissingPermission")
    private suspend fun doConnect(device: BluetoothDevice) = withContext(Dispatchers.IO) {
        closeSocket()
        _state.value = State.CONNECTING

        try {
            // Cancel discovery before creating a socket (SDK requirement)
            BluetoothAdapter.getDefaultAdapter()?.cancelDiscovery()

            val connectedSocket = withTimeoutOrNull(CONNECT_TIMEOUT_MS) {
                val sock = device.createRfcommSocketToServiceRecord(SPP_UUID)
                sock.connect()
                sock
            }

            if (connectedSocket != null) {
                socket = connectedSocket
                output = connectedSocket.outputStream
                _connectedDevice.value = safeName(device)
                _state.value = State.CONNECTED
                startReader(connectedSocket)
            } else {
                Log.w(TAG, "Connect timed out after 30 seconds")
                emitMessage("System: Connection failed (Timed out)")
                closeSocket()
                scheduleReconnect()
            }
        } catch (t: Throwable) {
            Log.w(TAG, "Connect failed: ${t.message}")
            emitMessage("System: Connection failed")
            closeSocket()
            scheduleReconnect()
        }
    }

    /**
     * Stop btn: Instantly halts active connection attempts or auto-reconnection loops,
     * logs a "Connection failed" message, resets state to DISCONNECTED.
     */
    fun stopConnection() {
        pendingDeviceAddress = null // Prevents auto-reconnect from triggering
        cancelConnectionJobs()
        closeSocket()
        _state.value = State.DISCONNECTED
        emitMessage("System: Connection failed")
    }

    private fun startReader(sock: BluetoothSocket) {
        readerJob?.cancel()
        readerJob = scope.launch(Dispatchers.IO) {
            val reader = BufferedReader(InputStreamReader(sock.inputStream))
            try {
                while (isActive) {
                    val line = reader.readLine() ?: break
                    if (line.isNotEmpty()) {
                        _messages.emit(line)
                    }
                }
            } catch (t: Throwable) {
                Log.w(TAG, "Reader ended: ${t.message}")
            } finally {
                if (socket === sock) {
                    onSocketDropped()
                }
            }
        }
    }

    private fun onSocketDropped() {
        closeSocket()
        if (pendingDeviceAddress != null) {
            scheduleReconnect()
        } else {
            _state.value = State.DISCONNECTED
        }
    }

    @SuppressLint("MissingPermission")
    private fun scheduleReconnect() {
        val address = pendingDeviceAddress ?: run {
            _state.value = State.DISCONNECTED
            return
        }
        if (reconnectJob?.isActive == true) return
        _state.value = State.RECONNECTING
        reconnectJob = scope.launch {
            while (isActive && _state.value != State.CONNECTED) {
                delay(RECONNECT_DELAY_MS)
                val adapter = BluetoothAdapter.getDefaultAdapter() ?: return@launch
                val device = try {
                    adapter.getRemoteDevice(address)
                } catch (t: Throwable) {
                    Log.w(TAG, "getRemoteDevice failed: ${t.message}")
                    null
                } ?: continue
                doConnect(device)
            }
        }
    }

    fun send(text: String): Boolean {
        val payload = if (text.endsWith("\n")) text else "$text\n"
        val out = output ?: return false
        return try {
            out.write(payload.toByteArray(Charsets.UTF_8))
            out.flush()
            true
        } catch (t: Throwable) {
            Log.w(TAG, "Send failed: ${t.message}")
            onSocketDropped()
            false
        }
    }

    fun disconnect() {
        pendingDeviceAddress = null
        cancelConnectionJobs()
        closeSocket()
        _state.value = State.DISCONNECTED
    }

    private fun cancelConnectionJobs() {
        connectionJob?.cancel()
        connectionJob = null
        reconnectJob?.cancel()
        reconnectJob = null
        readerJob?.cancel()
        readerJob = null
    }

    private fun closeSocket() {
        try { output?.close() } catch (_: Throwable) {}
        try { socket?.close() } catch (_: Throwable) {}
        output = null
        socket = null
        _connectedDevice.value = null
    }

    private fun emitMessage(msg: String) {
        scope.launch {
            _messages.emit(msg)
        }
    }

    @SuppressLint("MissingPermission")
    private fun safeName(device: BluetoothDevice): String = try {
        device.name ?: device.address
    } catch (_: SecurityException) {
        device.address
    }

    fun shutdown() {
        disconnect()
        scope.cancel()
    }
}