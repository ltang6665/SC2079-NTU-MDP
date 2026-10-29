package com.example.android_app.others

import android.content.res.ColorStateList
import android.widget.ImageView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.example.android_app.bluetooth.BluetoothService
import com.example.android_app.R
import kotlinx.coroutines.launch

fun AppCompatActivity.setupConnectionStatusBar() {
    val connectionBadge = findViewById<TextView>(R.id.connectionBadge) ?: return
    val statusDot = findViewById<ImageView>(R.id.statusDot) ?: return

    lifecycleScope.launch {
        repeatOnLifecycle(Lifecycle.State.STARTED) {
            BluetoothService.state.collect { state ->
                val deviceName = BluetoothService.connectedDevice.value

                val (textRes, colorRes) = when (state) {
                    BluetoothService.State.DISCONNECTED ->
                        R.string.status_disconnected to R.color.status_disconnected
                    BluetoothService.State.CONNECTING ->
                        R.string.status_connecting to R.color.status_connecting
                    BluetoothService.State.CONNECTED ->
                        R.string.status_connected to R.color.status_connected
                    BluetoothService.State.RECONNECTING ->
                        R.string.status_reconnecting to R.color.status_connecting
                }

                connectionBadge.text = if (state == BluetoothService.State.CONNECTED) {
                    getString(textRes, deviceName ?: "device")
                } else {
                    getString(textRes)
                }

                // Update dot color based on connection status
                val color = ContextCompat.getColor(this@setupConnectionStatusBar, colorRes)
                statusDot.imageTintList = ColorStateList.valueOf(color)
            }
        }
    }

    /* For vid demo
    val fakeDeviceName = "MDPGrp28"
    connectionBadge.text = getString(R.string.status_connected, fakeDeviceName)
    val color = ContextCompat.getColor(this, R.color.status_connected)
    statusDot.imageTintList = ColorStateList.valueOf(color)*/
}