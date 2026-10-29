from pathlib import Path
import socket
import json
import threading
import os
from time import time_ns, sleep
import logging

logging.basicConfig(level=logging.INFO)

from pathfinding.pathfinding import pathfinding
from image_stitching import add_to_stitching_dict, stitch_images
from stream_listener import StreamListener
from pathfinding.settings import Direction

PI_IP = "192.168.28.1"
PORT = 5000

class Task1:
    def __init__(self, host=PI_IP, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.exit = False
        self.timeout = 2
        self.big_turn = 0

        self.IMG_BLACKLIST = ["45"]
        self.conf_threshold = 0.7
        self.lead = 2.0e9
        self.lag = 1.5e9
        self.max_match_age_ns = int(5.5e9)
        self.allow_recent_fallback = os.getenv("DETECT_ALLOW_RECENT_FALLBACK", "0").strip().lower() in {
            "1", "true", "yes", "on"
        }
        self.img_time_dict = {}
        self.pending_obstacles = []
        self.stitching_img_dict = {}
        self.ids_to_stitch = []
        self.should_stitch = False
        self.stitch_len = 0

        self.matched_img_ids = set()

        self.model = str(Path(__file__).resolve().parent / "bestv8n.pt")
        self.filename = "stitches/task1"


        self.pc_receive_thread = None
        self.stream_listener = None
        logging.info(
            "Detect matcher config: lead=%.2fs lag=%.2fs max_age=%.2fs fallback=%s",
            self.lead / 1e9,
            self.lag / 1e9,
            self.max_match_age_ns / 1e9,
            self.allow_recent_fallback,
        )

    def _prune_stale_detections(self, now_ns: int) -> None:

        keep_for_ns = int(max(self.lead, self.max_match_age_ns))
        cutoff = now_ns - keep_for_ns
        stale_ids = [
            img_id for img_id, (_, last_seen) in self.img_time_dict.items()
            if last_seen < cutoff
        ]
        for img_id in stale_ids:
            del self.img_time_dict[img_id]

    def on_result(self, result, frame):

        if result is not None:
            names = result.names


            largest_detection = None

            for box in result.boxes:
                class_index = int(box.cls[0].item())
                detected_image_id = names[class_index]
                if str(detected_image_id) in self.IMG_BLACKLIST:
                    continue

                w = float(box.xywh[0][2].item())
                h = float(box.xywh[0][3].item())
                area = w * h
                confidence = float(box.conf[0].item()) if getattr(box.conf, "ndim", 0) else float(box.conf.item())

                if (largest_detection is None or
                    area > largest_detection["area"] or
                    (area == largest_detection["area"] and confidence > largest_detection["conf"])):
                    largest_detection = {"detected_img_id": detected_image_id, "box": box, "area": area, "conf": confidence}


            if largest_detection is not None:
                detected_image_id = largest_detection["detected_img_id"]
                detection_confidence = largest_detection["conf"]

                add_to_stitching_dict(self.stitching_img_dict, detected_image_id, detection_confidence, frame)

                last_seen = time_ns()
                first_seen = self.img_time_dict.get(detected_image_id, (last_seen, None))[0]
                self.img_time_dict[detected_image_id] = (first_seen, last_seen)
                self._prune_stale_detections(last_seen)


    def get_overlap_interval(self, img_id, timestamp, first_seen, last_seen):

        if img_id not in self.img_time_dict:
            return 0

        search_interval = (timestamp - self.lead, timestamp + self.lag)
        img_interval = (first_seen, last_seen)
        overlap_start = max(search_interval[0], img_interval[0])
        overlap_end = min(search_interval[1], img_interval[1])
        overlap = max(0, overlap_end - overlap_start)
        return overlap

    def send_matched_pair(self, obstacle_id, img_id):

        logging.info(f"Matched image {img_id} to obstacle {obstacle_id}.")
        self.ids_to_stitch.append(img_id)
        logging.info(f"{len(self.ids_to_stitch)} images in current stitching array.")


        message_content = f"OBJECT,{obstacle_id},{self.stitching_img_dict[img_id][0]},{img_id}\n"
        self.IMG_BLACKLIST.append(str(img_id))
        logging.info(f"Sending: {message_content}")
        self.sock.send(message_content.encode("utf-8"))


        self.stitching_img_dict = {
            k: v for k, v in self.stitching_img_dict.items()
            if str(k) in self.IMG_BLACKLIST
        }

    def _handle_message(self, data_str: str) -> None:

        logging.info(f"Received from RPI: {data_str}")

        if data_str.startswith("OBSTACLES"):
            obstacles = self._parse_obstacles(data_str)
            logging.info(f"Parsed obstacles: {obstacles}")
            logging.info(f"Starting A* pathfinding for {len(obstacles)} obstacle(s)...")
            _t0 = time_ns()

            path = pathfinding(obstacles)

            _elapsed = (time_ns() - _t0) / 1e9
            logging.info(f"A* pathfinding done in {_elapsed:.3f}s.")
            commands = path['commands']
            segments = self._segment_commands(commands)
            segments['dirs'] = self.get_directions(path)
            logging.info(f"Segmented commands: {segments}")
            if obstacles and not segments.get('obstacle_ids'):
                logging.warning(
                    "No reachable obstacle viewpoints were found for the current map; "
                    "returning stop-only path."
                )
            self.sock.send(f"PATH,{json.dumps(segments)}\n".encode("utf-8"))
            logging.info("Sent path back to rpi.")

        elif data_str.startswith("DETECT"):
            obstacle_id = data_str.split(",")[1].strip()
            timestamp = time_ns()
            self._prune_stale_detections(timestamp)


            max_overlap = 0
            max_img_id = None
            recent_candidates = []
            for img_id, (first_seen, last_seen) in self.img_time_dict.items():
                age_ns = timestamp - last_seen
                if age_ns > self.max_match_age_ns:
                    continue
                recent_candidates.append((age_ns, img_id))
                overlap = self.get_overlap_interval(img_id, timestamp, first_seen, last_seen)
                logging.info(f"Overlap: {overlap}, Max overlap: {max_overlap}")
                if overlap > 0 and overlap >= max_overlap:
                    logging.info(f"Replacing max overlap with {overlap}")
                    max_overlap = overlap
                    max_img_id = img_id


            if max_img_id is None and self.allow_recent_fallback and recent_candidates:
                age_ns, max_img_id = min(recent_candidates, key=lambda x: x[0])
                logging.info(
                    "No overlap match; fallback matched recent image %s, seen %.2fs before DETECT",
                    max_img_id,
                    age_ns / 1e9,
                )

            if max_img_id is not None:
                self.send_matched_pair(obstacle_id, max_img_id)
                del self.img_time_dict[max_img_id]
            else:
                logging.info(
                    "No valid image match for obstacle %s (fresh detections=%d).",
                    obstacle_id,
                    len(recent_candidates),
                )

        elif data_str.startswith("STITCH"):
            self.stitch_len = int(data_str.split(",")[1].strip())

            if len(self.ids_to_stitch) < self.stitch_len:
                logging.info("Stitch request received, wait for completion...")
                self.should_stitch = True
                sleep(self.lag * 2e-9)
                stitch_images(self.ids_to_stitch, self.stitching_img_dict, filename=self.filename)
            else:
                logging.info("All images present, stitching now...")
                self.stream_listener.close()
                stitch_images(self.ids_to_stitch, self.stitching_img_dict, filename=self.filename)

    def pc_receive(self) -> None:
        self.connect()
        logging.info("PC Socket connection started successfully")
        pending_text = ""

        while not self.exit:
            try:
                chunk = self.sock.recv(4096).decode("utf-8")
                if not chunk:
                    logging.info("Connection closed by server.")
                    if self.ids_to_stitch:
                        stitch_images(self.ids_to_stitch, self.stitching_img_dict, filename=self.filename)
                    break
                pending_text += chunk

                while "\n" in pending_text:
                    message_text, pending_text = pending_text.split("\n", 1)
                    message_text = message_text.strip()
                    if message_text:
                        self._handle_message(message_text)
            except OSError as e:
                logging.error(f"Error in receiving data: {e}")
                break

    def start_stream(self):
        self.stream_listener = StreamListener(weights=self.model)
        self.stream_listener.start_stream_read(
            on_result=self.on_result,
            on_disconnect=self.on_stream_disconnect,
            conf_threshold=self.conf_threshold,
            show_video=False
        )
        logging.info("Stream Started")

    def on_stream_disconnect(self):
        logging.info("Stream disconnected.")

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
            self.sock.close()
            self.sock = None
            self.exit = True

    def start_task_1(self):
        self.pc_receive_thread = threading.Thread(target=self.pc_receive, daemon=True)
        self.stream_thread = threading.Thread(target=self.start_stream,daemon=True)
        print("Starting Task 1 server...")
        self.stream_thread.start()
        self.pc_receive_thread.start()
        self.pc_receive_thread.join()

    def _parse_obstacles(self, data_str):
        parts = data_str.split("OBSTACLES,")
        obstacle = json.loads(parts[1])
        return obstacle

    def _segment_commands(self, commands):

        segments = []
        current_segment = []
        obstacle_ids = []
        for command_text in commands:
            dist = command_text[2:]
            if command_text.startswith("BW"):
                command_text = "R"
                command_text += dist
            elif command_text.startswith("BL"):
                command_text = "RL"
                command_text += dist
            elif command_text.startswith("BR"):
                command_text = "RR"
                command_text += dist
            elif command_text.startswith("FW"):
                command_text = "F"
                command_text += dist
            elif command_text.startswith("FIN"):
                command_text = "S"
            if command_text.startswith("SNAP"):

                snap_payload = command_text[4:]
                snap_id = snap_payload.split("_", 1)[0]
                obstacle_ids.append(snap_id)
                segments.append(current_segment)
                current_segment = []
                continue
            current_segment.append(command_text)
        if current_segment:
            segments.append(current_segment)
        return {"obstacle_ids": obstacle_ids, "segments": segments}

    def direction_to_name(self, d):


        if hasattr(d, "name"):
            return d.name

        try:
            return Direction(int(d)).name
        except Exception:
            pass

        if isinstance(d, str):
            return d.upper()

        return str(d)

    def get_directions(self, full_path):
        path = full_path['path'][1:] if full_path else []


        dirs = [{
                'd': "NORTH",
                'x': 1,
                'y': 1
            }]

        return dirs

    def test_pathfinding(self):
        obstacles = [
            {"x": 10,  "y": 10,  "id": 1, "d": 2},
            {"x": 5, "y": 12,  "id": 2, "d": 4},
            {"x": 8, "y": 5,  "id": 3, "d": 0},
            {"x": 11, "y": 14, "id": 4, "d": 2},
            {"x": 15, "y": 12, "id": 5, "d": 6},
            {"x": 16, "y": 19, "id": 6, "d": 4},


        ]
        path = pathfinding(obstacles, robot_x=1, robot_y=1, big_turn=self.big_turn)
        print(path)
        snap = self.get_directions(path)
        print(snap)
        segments = self._segment_commands(path['commands'])
        print(segments)


server = Task1()

server.start_task_1()
