import socket
import threading
import os
import time
import io
import struct
import logging
logging.basicConfig(level=logging.INFO)

from dotenv import load_dotenv
load_dotenv()

import picamera

class StreamServer():
    def __init__(self):
        self.BUFFER_SIZE = int(os.getenv("BUFFER_SIZE"))
        self.HOST = os.getenv("RPI_HOST")
        self.PORT = int(os.getenv("STREAM_PORT"))
        self.REQ_STREAM = os.getenv("REQ_STREAM")
        self.STOP_STREAM = os.getenv("STOP_STREAM")
        self.PING_STREAM = os.getenv("PING_STREAM")

        self.server_socket = None
        self.client_socket = None

        self._server_stop = threading.Event()
        self._client_lock = threading.Lock()

    def connect(self):

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.BUFFER_SIZE)

        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)
        logging.info(f"Stream server listening on {self.HOST}:{self.PORT}")

        try:
            while not self._server_stop.is_set():
                cs, addr = self.server_socket.accept()
                logging.info(f"Stream client connected: {addr}")

                with self._client_lock:

                    if self.client_socket:
                        try:
                            self.client_socket.shutdown(socket.SHUT_RDWR)
                        except Exception:
                            pass
                        try:
                            self.client_socket.close()
                        except Exception:
                            pass
                    self.client_socket = cs

                th = threading.Thread(target=self._handle_client, args=(cs, addr), daemon=True)
                th.start()
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def _handle_client(self, cs: socket.socket, addr):


        session_alive = threading.Event()
        session_alive.set()
        streaming_on = threading.Event()

        def _recv_commands():
            try:
                f = cs.makefile("rb")
                while session_alive.is_set():
                    line = f.readline()
                    if not line:

                        break
                    command_text = line.strip().decode("utf-8", "ignore")
                    if command_text == self.REQ_STREAM:
                        streaming_on.set()
                        try: cs.sendall(b"OK STREAMING\n")
                        except Exception: pass
                    elif command_text == self.STOP_STREAM:
                        try: cs.sendall(b"OK STOPPING\n")
                        except Exception: pass
                        break
                    elif command_text == self.PING_STREAM:
                        try: cs.sendall(b"PONG\n")
                        except Exception: pass
                    else:
                        try: cs.sendall(b"ERR UNKNOWN\n")
                        except Exception: pass
            except Exception:

                pass
            finally:
                session_alive.clear()
                try: f.close()
                except Exception: pass


        try:
            cs.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            cs.settimeout(None)
        except Exception:
            pass
        ctrl = threading.Thread(target=_recv_commands, daemon=True)
        ctrl.start()

        try:
            with picamera.PiCamera(resolution=(640, 480), framerate=20) as cam:
                cam.rotation = 180
                time.sleep(0.2)
                stream = io.BytesIO()

                for _ in cam.capture_continuous(stream, format="jpeg", quality=45, use_video_port=True):
                    if not session_alive.is_set():
                        break

                    if not streaming_on.is_set():

                        stream.seek(0); stream.truncate()
                        time.sleep(0.02)
                        continue

                    size = stream.tell()
                    stream.seek(0)
                    try:
                        cs.sendall(struct.pack("!I", size))
                        cs.sendall(stream.read(size))
                    except (BrokenPipeError, ConnectionResetError, OSError):

                        break
                    finally:
                        stream.seek(0); stream.truncate()
        finally:

            session_alive.clear()
            streaming_on.clear()
            try:
                cs.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                cs.close()
            except Exception:
                pass
            with self._client_lock:
                if self.client_socket is cs:
                    self.client_socket = None
            print(f"Client {addr} closed.")

    def close(self):
        self._server_stop.set()
        try:
            if self.client_socket:
                self.client_socket.close()
        except Exception:
            pass
        try:
            if self.server_socket:
                self.server_socket.close()
        except Exception:
            pass
