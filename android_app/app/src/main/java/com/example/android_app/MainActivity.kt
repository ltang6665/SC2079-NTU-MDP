package com.example.android_app

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity

// import features
import android.widget.Button
import android.content.Intent

// import sub-packages
import com.example.android_app.bluetooth.BluetoothActivity
import com.example.android_app.arena.ArenaActivity
import com.example.android_app.arena.Task2Activity
import com.example.android_app.others.setupConnectionStatusBar
class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        setupConnectionStatusBar()

        val startBtn = findViewById<Button>(R.id.startButton)
        val task2Btn = findViewById<Button>(R.id.task2Button)
        val connectBtn = findViewById<Button>(R.id.connectBluetoothButton)

        connectBtn.setOnClickListener {
            val intent = Intent(this, BluetoothActivity::class.java)
            startActivity(intent)
        }

        startBtn.setOnClickListener {
            startActivity(Intent(this, ArenaActivity::class.java))
        }

        task2Btn.setOnClickListener {
            startActivity(Intent(this, Task2Activity::class.java))
        }

    }
}
