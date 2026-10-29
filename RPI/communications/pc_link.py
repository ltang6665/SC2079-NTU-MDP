import socket
import logging
import sys
import os
from typing import Optional

from dotenv import load_dotenv
load_dotenv()
HOST = os.getenv("RPI_HOST")
PORT = int(os.getenv("RPI_PORT"))

logging.basicConfig(level=logging.INFO)

class PC:
    def __init__(self):
        self.host = HOST
        self.port = PORT
        self.server_socket = None
        self.client_socket = None

    def connect(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        logging.info(f"Attempting to connect to {self.host}:{self.port}")

        try:
            self.server_socket.bind((self.host, self.port))
            logging.info(f"Server bound to {self.host}:{self.port}")
        except socket.error as e:
            logging.error(f"Error binding to {self.host}:{self.port} - {e}")
            self.server_socket.close()
            sys.exit(1)

        logging.info("Waiting for a connection...")
        try:
            self.server_socket.listen(128)
            self.client_socket, addr = self.server_socket.accept()
            logging.info(f"Connection established with {addr}")
        except socket.error as e:
            logging.error(f"Error accepting connection - {e}")
            self.server_socket.close()
            sys.exit(1)

    def disconnect(self):
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                logging.info("Client socket closed.")
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None
                logging.info("Server socket closed.")
            logging.info("Disconnected successfully.")
        except Exception as e:
            logging.error(f"Error during disconnection - {e}")
            sys.exit(1)

    def send(self, message: str) -> None:
        if not self.client_socket:

            return

        try:
            self.client_socket.sendall(message.encode('utf-8'))
            logging.info(f"PC: Sent to pc: {message}")
        except socket.error as e:
            logging.error(f"PC: Error sending message - {e}")
            self.disconnect()

    def receive(self) -> Optional[str]:
        if not self.client_socket:

            return None

        try:
            data = self.client_socket.recv(2048)
            if not data:
                logging.info("PC: No data received, client may have disconnected.")
                self.disconnect()
                return None
            message = data.decode('utf-8')

            return message
        except socket.error as e:
            logging.error(f"PC: Error receiving message - {e}")
            self.disconnect()
            return None

    def wait_receive(self) -> str:

        message = None
        while message is None:
            message = self.receive()

        return message
