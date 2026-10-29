from pathlib import Path
import socket
import os
import json
import threading
from time import time_ns, sleep
import logging

logging.basicConfig(level=logging.INFO)

from image_stitching import add_to_stitching_dict, stitch_images, add_to_stitching_dict_2, stitch_images_2
from stream_listener import StreamListener

PI_IP = os.getenv("RPI_HOST", "192.168.28.1")
PORT = int(os.getenv("RPI_PORT", "5000"))

class Task2:
    def __init__(self, host=PI_IP, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.exit = False
        self.timeout = 2

        self.detection_gate = threading.Event()
        self.detection_gate.clear()
        self.obstacle_lock = threading.RLock()
        self.stitch_lock = threading.RLock()

        self.cooldown_s = 0.0
        self.debounce_hits = 2

        self.obstacle_id = 1
        self.current_image_id = None
        self.consecutive_hits = 0

        self.prev_detected_id = None

        self.IMG_BLACKLIST = ["45"]
        self.conf_threshold = 0.7

        self.stitching_img_dict = {}

        self.model = str(Path(__file__).resolve().parent / "bestv8n.pt")
        self.filename = "stitches/task2"

        self.pc_receive_thread = None
        self.stream_listener = None

        self.LEFT_ARROW_ID = "39"
        self.RIGHT_ARROW_ID = "38"

        self.last_image = None

    def _start_cooldown_and_advance(self):
        self.detection_gate.set()
        with self.obstacle_lock:
            self.obstacle_id += 1
            self.current_image_id = None
            self.consecutive_hits = 0
            self.prev_detected_id = None
            logging.info(f"Advanced to obstacle {self.obstacle_id} (cooldown {self.cooldown_s}s).")

        t = threading.Timer(self.cooldown_s, self.detection_gate.clear)
        t.daemon = True
        t.start()

    def _get_vision_stage(self):
        with self.obstacle_lock:
            return self.obstacle_id

    def on_result(self, result, frame, raw_frame=None, frame_obstacle_id=None):
        if self.detection_gate.is_set():
            return
        with self.obstacle_lock:
            if frame_obstacle_id is None:
                frame_obstacle_id = self.obstacle_id
            if frame_obstacle_id != self.obstacle_id:
                return
        if result is None:
            with self.obstacle_lock:
                if frame_obstacle_id == self.obstacle_id:
                    self.consecutive_hits = 0
                    self.prev_detected_id = None
            return

        message_content = None
        detected_image_id = None

        if result is not None:
            nearest_arrow = None
            for box in result.boxes:
                class_index = int(box.cls[0].item())
                image_id = str(result.names[class_index])
                confidence = float(box.conf[0].item())
                if (image_id in self.IMG_BLACKLIST
                        or image_id not in (self.LEFT_ARROW_ID, self.RIGHT_ARROW_ID)):
                    continue
                if confidence < self.conf_threshold:
                    continue
                width = float(box.xywh[0][2].item())
                height = float(box.xywh[0][3].item())
                area = width * height
                if nearest_arrow is None or area > nearest_arrow[0]:
                    nearest_arrow = (area, image_id, confidence)

            if nearest_arrow is None:
                with self.obstacle_lock:
                    if frame_obstacle_id == self.obstacle_id:
                        self.consecutive_hits = 0
                        self.prev_detected_id = None
                return
            _, detected_image_id, detection_confidence = nearest_arrow

            with self.obstacle_lock:
                if (self.detection_gate.is_set()
                        or frame_obstacle_id != self.obstacle_id):
                    return
                if detected_image_id == self.prev_detected_id:
                    self.consecutive_hits += 1
                else:
                    self.consecutive_hits = 1
                self.prev_detected_id = detected_image_id
                if self.consecutive_hits < self.debounce_hits:
                    return
                obstacle_number = frame_obstacle_id

            if self.current_image_id is None:
                message_content = f"{obstacle_number},{detection_confidence},{detected_image_id}\n"
                self.current_image_id = detected_image_id

            with self.stitch_lock:
                if obstacle_number not in self.stitching_img_dict:
                    self.stitching_img_dict[obstacle_number] = {}
                add_to_stitching_dict_2(
                    self.stitching_img_dict,
                    obstacle_number,
                    detected_image_id,
                    detection_confidence,
                    frame
                )

            self.last_image = detected_image_id

        if message_content is not None and self.sock:
            try:
                self.sock.sendall(message_content.encode("utf-8"))
            except OSError as e:
                logging.error(f"Send error: {e}")
            finally:
                self.current_image_id = None

    def _handle_control_message(self, message_text):
        if message_text == "SEEN":
            self._start_cooldown_and_advance()
        elif message_text == "STITCH":
            with self.stitch_lock:
                keys = sorted(self.stitching_img_dict.keys())[-2:] or [1, 2]
                stitch_images_2(keys, self.stitching_img_dict, self.filename, ncols=2, show=False)

    def pc_receive(self) -> None:
        self.connect()
        pending_text = ""
        while not self.exit:
            try:
                chunk = self.sock.recv(1024).decode("utf-8")
                if not chunk:
                    break
                pending_text += chunk
                while "\n" in pending_text or pending_text in {"SEEN", "STITCH"}:
                    if "\n" in pending_text:
                        message, pending_text = pending_text.split("\n", 1)
                    else:
                        message, pending_text = pending_text, ""
                    message = message.strip()
                    if message:
                        self._handle_control_message(message)
            except OSError as exc:
                logging.error("Error receiving control message: %s", exc)
                break

    def start_stream(self):
        self.detection_gate.clear()
        self.stream_listener = StreamListener(weights=self.model)
        self.stream_listener.start_stream_read(
            on_result=self.on_result,
            on_disconnect=self.disconnect,
            conf_threshold=self.conf_threshold,
            show_video=False,
            get_result_context=self._get_vision_stage,
        )
        logging.info("Stream Started")

    def connect(self, retries: int = 10, delay: float = 1.0):
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                logging.info(f"Connecting to {self.host}:{self.port} (attempt {attempt}/{retries})")
                self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(None)
                logging.info("Connected.")
                return
            except OSError as e:
                last_err = e
                logging.warning(f"Connect failed: {e}; retrying in {delay}s")
                sleep(delay)
        raise ConnectionError(f"Unable to connect to {self.host}:{self.port}") from last_err

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None
        self.exit = True

    def start_task_2(self):
        self.pc_receive_thread = threading.Thread(target=self.pc_receive, daemon=True)
        print("Starting Task 2 server...")
        stream_thread = threading.Thread(target=self.start_stream, daemon=True)
        stream_thread.start()
        self.pc_receive_thread.start()
        self.pc_receive_thread.join()

if __name__ == "__main__":
    server = Task2()
    server.start_task_2()
