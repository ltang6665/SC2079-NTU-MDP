from typing import Optional
import logging
import serial
import os
from threading import Lock
from time import monotonic

from dotenv import load_dotenv
load_dotenv()

SERIAL_PORT = os.getenv("SERIAL_PORT")
BAUD_RATE = int(os.getenv("BAUD_RATE"))

class STM:
    def __init__(self):
        super().__init__()
        self.serial = None
        self.received = []
        self._rx_buffer = ""
        self._send_lock = Lock()

    def connect(self):
        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self._rx_buffer = ""
        logging.info(
            "Opened STM serial port %s at %d baud; waiting for firmware reply (raw RX logging enabled).",
            SERIAL_PORT, BAUD_RATE,
        )

    def disconnect(self):
        self.serial.close()
        self.serial = None
        print("Disconnected from STM32")

    def send(self, message: str) -> None:
        with self._send_lock:
            self.serial.write(bytes(message, "utf-8"))
        print("Sent to STM32:", str(message).rstrip())

    def wait_receive(self, ticks=5000, timeout_s=None) -> Optional[str]:
        deadline = None if timeout_s is None else monotonic() + timeout_s
        while True:
            message = self.receive()
            if message is not None:
                return message

            if deadline is not None and monotonic() >= deadline:
                return None

            data = self.serial.read(self.serial.in_waiting or 1)
            if data:
                logging.info("STM raw RX: %r", data)
                self._rx_buffer += data.decode("utf-8", errors="replace")

    def receive(self) -> Optional[str]:
        if "\n" in self._rx_buffer:
            message, self._rx_buffer = self._rx_buffer.split("\n", 1)
            return message + "\n"
        if self.serial.in_waiting > 0:
            self._rx_buffer += self.serial.read_all().decode("utf-8", errors="replace")
            if "\n" in self._rx_buffer:
                message, self._rx_buffer = self._rx_buffer.split("\n", 1)
                return message + "\n"
        return None

    def send_cmd(self, flag, speed, angle, val):
        command_text = flag
        if flag not in ["S", "D", "M"]:
            command_text += f"{speed}|{round(angle, 2)}|{round(val, 2)}"
        command_text += "\n"
        self.send(command_text)
