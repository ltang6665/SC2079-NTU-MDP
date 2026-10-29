import os
import sys
import logging
import fcntl
from typing import Optional
from threading import Thread, Event
from time import sleep

logging.basicConfig(level=logging.INFO)

from dotenv import load_dotenv
load_dotenv()

from bluetooth import (
    BluetoothSocket,
    BluetoothError,
    RFCOMM,
    SERIAL_PORT_CLASS,
    SERIAL_PORT_PROFILE,
    advertise_service,
)
from socket import timeout as SocketTimeout
import errno

LOCK_PATH = "/var/run/mdp-task1.lock"
os.makedirs(os.path.dirname(LOCK_PATH), exist_ok=True)
_fd = open(LOCK_PATH, "w")
try:
    fcntl.flock(_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    _fd.write(str(os.getpid()))
    _fd.flush()
except BlockingIOError:
    print("Another instance is already running; exiting.")
    sys.exit(0)


class Android:


    def __init__(self):
        self.client_socket: Optional[BluetoothSocket] = None
        self.server_socket: Optional[BluetoothSocket] = None
        self.connected = False
        self._rx_buffer = ""
        self._accept_thread: Optional[Thread] = None
        self._stop_event = Event()
        self.accept_backoff_s = 0.5


        self.uuid = "00001801-0000-1000-8000-00805f9b34fb"


        self.host = os.getenv("RPI_HOST", "")
        self.port = 1


        self.recv_timeout_s = 1.0


    def start(self):

        self.start_server()
        if self._accept_thread and self._accept_thread.is_alive():
            return
        self._stop_event.clear()
        self._accept_thread = Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def stop(self):

        self._stop_event.set()
        self._close_client()
        self.stop_server()

    def start_server(self):

        if self.server_socket:
            logging.info("Server already started.")
            return

        try:

            os.system("sudo hciconfig hci0 piscan")

            self.server_socket = BluetoothSocket(RFCOMM)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)

            advertise_service(
                self.server_socket,
                "MDPGroup20 RPi",
                service_id=self.uuid,
                service_classes=[self.uuid, SERIAL_PORT_CLASS],
                profiles=[SERIAL_PORT_PROFILE],
            )

            logging.info(f"SPP server started on RFCOMM channel {self.port} (advertised).")

        except Exception as e:
            logging.error(f"Failed to start server: {e}")
            self.stop_server()
            raise

    def stop_server(self):

        try:
            self._close_client()
        finally:
            if self.server_socket:
                try:
                    self.server_socket.close()
                    logging.info("Server socket closed.")
                except Exception as e:
                    logging.error(f"Error closing server socket: {e}")
            self.server_socket = None

    def accept_client(self):

        if not self.server_socket:
            self.start_server()

        logging.info(f"Waiting for connection on RFCOMM channel {self.port} ...")
        try:
            self.client_socket, client_info = self.server_socket.accept()

            self.client_socket.settimeout(self.recv_timeout_s)
            self.connected = True
            logging.info(f"Accepted connection from {client_info}")
        except Exception as e:
            logging.error(f"Accept failed: {e}")
            self._close_client()
            self.connected = False
            raise

    def _accept_loop(self):

        while not self._stop_event.is_set():
            if self.connected:
                sleep(0.1)
                continue
            try:

                if not self.server_socket:
                    self.start_server()

                logging.info(f"Waiting for connection on RFCOMM channel {self.port} ...")
                client_sock, client_info = self.server_socket.accept()
                client_sock.settimeout(3.0)
                self.client_socket = client_sock
                self.connected = True
                logging.info(f"Accepted connection from {client_info}")

            except Exception as e:
                if self._stop_event.is_set():
                    break
                logging.warning(f"Accept failed: {e}. Retrying soon...")
                sleep(self.accept_backoff_s)


    def _close_client(self):
        if self.client_socket:
            try:
                self.client_socket.close()
                logging.info("Client socket closed.")
            except Exception as e:
                logging.error(f"Error closing client socket: {e}")
        self.client_socket = None
        self.connected = False

    def _ensure_connected(self):

        if not self.connected or not self.client_socket:
            self._close_client()
            self.accept_client()


    def disconnect(self):

        if not self.connected:
            logging.info("Not connected to any Android device.")
            return
        self._close_client()
        logging.info("Disconnected from Android device (server still listening).")

    def send(self, message) -> None:
        if not self.connected or not self.client_socket:
            raise ConnectionError("Not connected.")
        payload = (message + "\n").encode("utf-8")
        try:
            self.client_socket.send(payload)
            logging.info(f"Sent: {message}")
        except (BluetoothError, OSError) as e:
            logging.warning(f"Send failed ({e}). Marking disconnected.")
            self._close_client()
            raise

    def receive(self) -> Optional[str]:
        if not self.connected or not self.client_socket:
            return None

        try:

            if "\n" in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                message_text = line.strip()
                if message_text:
                    logging.info(f"Received: {message_text}")
                    return message_text
            data = self.client_socket.recv(1024)
            if not data:
                logging.info("Client disconnected (0 bytes).")
                self._close_client()
                return None
            self._rx_buffer += data.decode("utf-8")


            if "\n" in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                message_text = line.strip()
                if message_text:
                    logging.info(f"Received: {message_text}")
                    return message_text
            return None

        except SocketTimeout:
            return None

        except BluetoothError as e:
            if e.args and str(e.args[0]).lower() == "timed out":
                return None
            logging.warning(f"Receive BluetoothError ({e}).")
            self._close_client()
            return None

        except OSError as e:
            if getattr(e, "errno", None) in (errno.ECONNRESET, errno.ENOTCONN, errno.EPIPE):
                logging.warning(f"Receive OSError ({e}).")
                self._close_client()
                return None
            logging.error(f"Unexpected OSError in receive: {e}")
            self._close_client()


    def wait_receive(self) -> str:

        while True:
            message_text = self.receive()
            if message_text is not None:
                return message_text
