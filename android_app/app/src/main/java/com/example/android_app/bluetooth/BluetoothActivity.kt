package com.example.android_app.bluetooth

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.widget.NestedScrollView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.android_app.R
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch

class BluetoothActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_KEEP_LANDSCAPE = "com.example.android_app.bluetooth.KEEP_LANDSCAPE"
    }

    private lateinit var pairedAdapter: DeviceListAdapter
    private lateinit var discoveredAdapter: DeviceListAdapter
    private lateinit var statusView: TextView
    private lateinit var inboundLog: TextView
    private lateinit var logScroll: NestedScrollView
    private lateinit var sendField: EditText

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        (getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }

    private val discoveryReceiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                BluetoothDevice.ACTION_FOUND -> {
                    val device: BluetoothDevice? =
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                            intent.getParcelableExtra(
                                BluetoothDevice.EXTRA_DEVICE,
                                BluetoothDevice::class.java,
                            )
                        } else {
                            @Suppress("DEPRECATION")
                            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                        }
                    device?.let { discoveredAdapter.addIfMissing(it) }
                }
                BluetoothAdapter.ACTION_DISCOVERY_STARTED -> setScanning(hasScanPermissions())
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED ->
                    setScanning(hasScanPermissions() && bluetoothAdapter?.isDiscovering == true)
            }
        }
    }

    private val enableBtLauncher =
        registerForActivityResult(ActivityResultContracts.StartActivityForResult()) {
            /* nothing — we re-check on scan button press */
        }

    private enum class PendingAction { NONE, LOAD_PAIRED, START_SCAN }
    private var pendingAction: PendingAction = PendingAction.NONE

    private val permissionsLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { grants ->
            val allGranted = grants.values.all { it }
            val action = pendingAction
            pendingAction = PendingAction.NONE
            if (!allGranted) {
                toast("Bluetooth permissions required")
                return@registerForActivityResult
            }
            when (action) {
                PendingAction.LOAD_PAIRED -> loadPairedDevices()
                PendingAction.START_SCAN -> startDiscovery()
                PendingAction.NONE -> Unit
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        if (intent.getBooleanExtra(EXTRA_KEEP_LANDSCAPE, false)) {
            requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        }
        setContentView(R.layout.activity_bluetooth)

        statusView = findViewById(R.id.statusView)
        inboundLog = findViewById(R.id.inboundLog)
        logScroll = findViewById(R.id.logScroll)
        sendField = findViewById(R.id.sendField)

        pairedAdapter = DeviceListAdapter(::onDeviceSelected)
        discoveredAdapter = DeviceListAdapter(::onDeviceSelected)

        findViewById<RecyclerView>(R.id.pairedList).apply {
            layoutManager = LinearLayoutManager(this@BluetoothActivity)
            adapter = pairedAdapter
        }
        findViewById<RecyclerView>(R.id.discoveredList).apply {
            layoutManager = LinearLayoutManager(this@BluetoothActivity)
            adapter = discoveredAdapter
        }

        observeDeviceList(pairedAdapter, R.id.pairedList, R.id.pairedEmpty, R.id.pairedCount)
        observeDeviceList(discoveredAdapter, R.id.discoveredList, R.id.discoveredEmpty, R.id.discoveredCount)

        findViewById<Button>(R.id.btnScan).setOnClickListener { onScanClicked() }
        findViewById<Button>(R.id.btnDisconnect).setOnClickListener {
            BluetoothService.disconnect()
        }
        findViewById<Button>(R.id.btnStopConnection).setOnClickListener {
            BluetoothService.stopConnection()
        }
        findViewById<Button>(R.id.btnSend).setOnClickListener {
            val text = sendField.text.toString().trim()
            if (text.isEmpty()) return@setOnClickListener
            if (BluetoothService.send(text)) {
                sendField.text.clear()
            } else {
                toast("Not connected")
            }
        }

        registerReceiver(discoveryReceiver, IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        })

        observeService()
        loadPairedDevices()
    }

    override fun onDestroy() {
        super.onDestroy()
        try {
            unregisterReceiver(discoveryReceiver)
        } catch (_: IllegalArgumentException) {}
        cancelDiscovery()
    }

    private fun observeService() {
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    BluetoothService.state.onEach { state ->
                        val name = BluetoothService.connectedDevice.value
                        statusView.text = when (state) {
                            BluetoothService.State.DISCONNECTED ->
                                getString(R.string.status_disconnected)
                            BluetoothService.State.CONNECTING ->
                                getString(R.string.status_connecting)
                            BluetoothService.State.CONNECTED ->
                                getString(R.string.status_connected, name ?: "device")
                            BluetoothService.State.RECONNECTING ->
                                getString(R.string.status_reconnecting)
                        }
                        val statusColor = when (state) {
                            BluetoothService.State.CONNECTED -> R.color.status_connected
                            BluetoothService.State.CONNECTING,
                            BluetoothService.State.RECONNECTING -> R.color.status_connecting
                            BluetoothService.State.DISCONNECTED -> R.color.status_disconnected
                        }
                        statusView.setTextColor(ContextCompat.getColor(this@BluetoothActivity, statusColor))
                    }.launchIn(this)
                }
                launch {
                    BluetoothService.messages.onEach { line ->
                        appendInbound(line)
                    }.launchIn(this)
                }
            }
        }
    }

    private fun appendInbound(line: String) {
        // Follow new messages only while the operator is already reading the latest entry.
        val followLatest = !logScroll.canScrollVertically(1)
        val current = inboundLog.text?.toString().orEmpty()
        val trimmed = if (current.length > 4000) current.takeLast(2000) else current
        inboundLog.text = if (trimmed.isEmpty()) line else "$trimmed\n$line"
        findViewById<View>(R.id.logEmpty).visibility = View.GONE
        if (followLatest) logScroll.post { logScroll.scrollTo(0, inboundLog.bottom) }
    }

    private fun observeDeviceList(
        adapter: DeviceListAdapter,
        listId: Int,
        emptyId: Int,
        countId: Int,
    ) {
        val list = findViewById<RecyclerView>(listId)
        val empty = findViewById<TextView>(emptyId)
        val count = findViewById<TextView>(countId)
        fun render() {
            val size = adapter.itemCount
            list.visibility = if (size == 0) View.INVISIBLE else View.VISIBLE
            empty.visibility = if (size == 0) View.VISIBLE else View.GONE
            count.text = getString(R.string.comms_device_count, size)
            count.contentDescription = resources.getQuantityString(
                R.plurals.comms_device_count_accessibility, size, size,
            )
        }
        adapter.registerAdapterDataObserver(object : RecyclerView.AdapterDataObserver() {
            override fun onChanged() = render()
            override fun onItemRangeInserted(positionStart: Int, itemCount: Int) = render()
            override fun onItemRangeRemoved(positionStart: Int, itemCount: Int) = render()
        })
        render()
    }

    private fun setScanning(scanning: Boolean) {
        findViewById<Button>(R.id.btnScan).apply {
            isEnabled = !scanning
            setText(if (scanning) R.string.comms_scanning else R.string.comms_scan)
        }
        findViewById<TextView>(R.id.discoveredEmpty).setText(
            if (scanning) R.string.comms_discovered_scanning else R.string.comms_discovered_empty,
        )
    }

    private fun onDeviceSelected(device: BluetoothDevice) {
        cancelDiscovery()
        BluetoothService.connect(device)
    }

    private fun onScanClicked() {
        if (!requireBtEnabled()) return
        if (!hasScanPermissions()) {
            pendingAction = PendingAction.START_SCAN
            permissionsLauncher.launch(neededScanPermissions())
            return
        }
        startDiscovery()
    }

    @SuppressLint("MissingPermission")
    private fun startDiscovery() {
        val adapter = bluetoothAdapter ?: run {
            toast("No Bluetooth adapter")
            return
        }
        discoveredAdapter.submit(emptyList())
        adapter.cancelDiscovery()
        setScanning(adapter.startDiscovery())
    }

    @SuppressLint("MissingPermission")
    private fun cancelDiscovery() {
        bluetoothAdapter?.cancelDiscovery()
    }

    @SuppressLint("MissingPermission")
    private fun loadPairedDevices() {
        if (!hasConnectPermission()) {
            pendingAction = PendingAction.LOAD_PAIRED
            permissionsLauncher.launch(neededScanPermissions())
            return
        }
        val adapter = bluetoothAdapter ?: return
        pairedAdapter.submit(adapter.bondedDevices.toList())
    }

    private fun requireBtEnabled(): Boolean {
        val adapter = bluetoothAdapter ?: run {
            toast("No Bluetooth adapter")
            return false
        }
        if (!adapter.isEnabled) {
            enableBtLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
            return false
        }
        return true
    }

    private fun hasScanPermissions(): Boolean = neededScanPermissions().all { perm ->
        ContextCompat.checkSelfPermission(this, perm) == PackageManager.PERMISSION_GRANTED
    }

    private fun hasConnectPermission(): Boolean =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) ==
                    PackageManager.PERMISSION_GRANTED
        } else true

    private fun neededScanPermissions(): Array<String> =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
            )
        }

    private fun toast(msg: String) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
    }
}
