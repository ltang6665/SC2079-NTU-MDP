import socket
import struct
import cv2
import numpy as np
import os
import inspect
import logging
import threading

from dotenv import load_dotenv
load_dotenv()

from ultralytics import YOLO

class StreamListener:
    def __init__(self, weights):
        self.HOST = os.getenv("RPI_HOST")
        self.PORT = int(os.getenv("STREAM_PORT", "5001"))

        req_stream = os.getenv("REQ_STREAM", "")
        stop_token = os.getenv("STOP_STREAM", "")
        ping_token = os.getenv("PING_STREAM", "")

        self.REQ_STREAM = bytes(req_stream + "\n", "utf-8")
        self.STOP_STREAM = bytes(stop_token + "\n", "utf-8")
        self.PING_STREAM = bytes(ping_token + "\n", "utf-8")

        self.model = YOLO(weights)

        self.sock = None

        self.stop_event = threading.Event()

        self.sock_lock = threading.Lock()

    def _connect(self):
        logging.info(
            "Connecting to RPI stream at %s:%s",
            self.HOST,
            self.PORT
        )

        sock = socket.create_connection(
            (self.HOST, self.PORT),
            timeout=5
        )

        try:
            sock.setsockopt(
                socket.IPPROTO_TCP,
                socket.TCP_NODELAY,
                1
            )
        except Exception:
            pass

        sock.settimeout(None)

        with self.sock_lock:
            self.sock = sock

        logging.info("RPI stream socket connected.")

    def _get_socket(self):
        with self.sock_lock:
            return self.sock

    def _readline(self, maxlen=256):
        sock = self._get_socket()

        if sock is None:
            logging.warning("_readline called but socket is None.")
            return None

        receive_buffer = bytearray()

        while len(receive_buffer) < maxlen:
            try:
                ch = sock.recv(1)

            except (OSError, AttributeError) as e:
                logging.warning(
                    "Stream socket recv failed while reading line: %s",
                    e
                )
                return None

            if not ch:
                return None

            receive_buffer += ch

            if ch == b"\n":
                break

        return bytes(receive_buffer)

    def _recv_exact(self, n):
        sock = self._get_socket()

        if sock is None:
            logging.warning(
                "_recv_exact(%d) called but socket is None.",
                n
            )
            return None

        data = bytearray()

        while len(data) < n:
            if self.stop_event.is_set():
                return None

            try:
                chunk = sock.recv(n - len(data))

            except (OSError, AttributeError) as e:
                if self.stop_event.is_set():
                    logging.info(
                        "Stream socket closed during shutdown."
                    )
                else:
                    logging.warning(
                        "Stream socket recv failed: %s",
                        e
                    )

                return None

            if not chunk:
                logging.info(
                    "RPI stream socket closed by remote host."
                )
                return None

            data.extend(chunk)

        return bytes(data)

    def _invoke_on_result(self, cb, res, annotated, raw, frame_context=None):
        if cb is None:
            return

        try:
            sig = inspect.signature(cb)
            params = len(sig.parameters)

        except Exception:
            params = 3

        try:
            if params >= 4:
                cb(res, annotated, raw, frame_context)
            elif params >= 3:
                cb(res, annotated, raw)

            else:
                cb(
                    res,
                    annotated if annotated is not None else raw
                )

        except Exception as e:
            logging.exception(
                "[StreamListener] on_result callback failed: %s",
                e
            )

    def req_stream(self):
        if self._get_socket() is None:
            self._connect()

        sock = self._get_socket()

        if sock is None:
            raise RuntimeError(
                "Cannot request stream: socket is None."
            )

        try:
            sock.sendall(self.REQ_STREAM)

        except OSError as e:
            raise RuntimeError(
                f"Failed to send stream request: {e}"
            ) from e

        header = self._readline()

        header_txt = (
            header.decode(
                "utf-8",
                errors="ignore"
            ).strip()
            if header
            else ""
        )

        if header_txt != "OK STREAMING":
            raise RuntimeError(
                f"Stream handshake failed: "
                f"'{header_txt or 'NO HEADER'}'. "
                f"Check REQ_STREAM value on PC and RPi."
            )

        logging.info("Stream handshake OK.")

    def start_stream_read(
        self,
        on_result,
        on_disconnect,
        conf_threshold=0.7,
        show_video=True,
        get_result_context=None,
    ):
        self.stop_event.clear()

        logging.info(
            "Starting stream listener. RPI=%s:%s",
            self.HOST,
            self.PORT
        )

        try:
            self.req_stream()

            while not self.stop_event.is_set():
                frame_context = get_result_context() if get_result_context else None

                hdr = self._recv_exact(4)

                if hdr is None:
                    logging.info(
                        "No frame header received; "
                        "ending stream loop."
                    )
                    break

                if len(hdr) != 4:
                    logging.warning(
                        "Invalid frame header length: %d",
                        len(hdr)
                    )
                    break

                try:
                    size = struct.unpack("!I", hdr)[0]

                except struct.error as e:
                    logging.warning(
                        "Failed to unpack frame header: %s",
                        e
                    )
                    break

                if size <= 0 or size > 50_000_000:
                    logging.warning(
                        "Invalid JPEG frame size: %d bytes",
                        size
                    )
                    break

                jpg = self._recv_exact(size)

                if jpg is None:
                    logging.info(
                        "JPEG payload not received; "
                        "ending stream loop."
                    )
                    break

                frame = cv2.imdecode(
                    np.frombuffer(
                        jpg,
                        dtype=np.uint8
                    ),
                    cv2.IMREAD_COLOR
                )

                if frame is None:
                    logging.warning(
                        "Failed to decode JPEG frame."
                    )

                    self._invoke_on_result(
                        on_result,
                        None,
                        None,
                        None,
                        frame_context
                    )

                    continue

                try:
                    res = self.model.predict(
                        frame,
                        save=False,
                        imgsz=frame.shape[1],
                        conf=conf_threshold,
                        verbose=False
                    )[0]

                except Exception as e:
                    logging.exception(
                        "YOLO inference failed: %s",
                        e
                    )

                    self._invoke_on_result(
                        on_result,
                        None,
                        None,
                        frame,
                        frame_context
                    )

                    continue

                if len(res.boxes) > 0:
                    annotated = res.plot()

                    self._invoke_on_result(
                        on_result,
                        res,
                        annotated,
                        frame,
                        frame_context
                    )

                    disp = annotated

                else:
                    self._invoke_on_result(
                        on_result,
                        None,
                        None,
                        frame,
                        frame_context
                    )

                    disp = frame

                if show_video:
                    cv2.imshow(
                        "Stream",
                        disp
                    )

                    key = cv2.waitKey(1) & 0xFF

                    if key == 27:
                        logging.info(
                            "ESC pressed; stopping stream."
                        )

                        self.close()
                        break

        except Exception as e:
            logging.exception(
                "Stream read loop failed: %s",
                e
            )

        finally:
            logging.info(
                "Stream listener cleaning up."
            )

            sock = self._get_socket()

            if sock is not None:
                try:
                    sock.shutdown(
                        socket.SHUT_RDWR
                    )

                except OSError:
                    pass

                try:
                    sock.close()

                except OSError:
                    pass

            with self.sock_lock:
                self.sock = None

            if show_video:
                try:
                    cv2.destroyAllWindows()

                except Exception:
                    pass

            if on_disconnect:
                try:
                    on_disconnect()

                except Exception as e:
                    logging.exception(
                        "on_disconnect callback failed: %s",
                        e
                    )

            logging.info(
                "Stream listener stopped."
            )

    def close(self):
        logging.info(
            "Stopping StreamListener..."
        )

        self.stop_event.set()

        sock = self._get_socket()

        if sock is None:
            return

        try:
            sock.sendall(
                self.STOP_STREAM
            )

        except Exception:
            pass

        try:
            sock.shutdown(
                socket.SHUT_RDWR
            )

        except Exception:
            pass

        try:
            sock.close()

        except Exception:
            pass
