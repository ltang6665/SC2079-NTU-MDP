from pathlib import Path
import socket
import json
import threading
from time import time_ns, sleep
import logging

logging.basicConfig(level=logging.INFO)

from pathfinding.pathfinding import pathfinding
from image_stitching import add_to_stitching_dict, stitch_images
from stream_listener import StreamListener

PI_IP = "192.168.20.1"
PORT = 5000

class Checklist:
    def __init__(self, host=PI_IP, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.exit = False
        self.timeout = 2

        self.IMG_BLACKLIST = []
        self.conf_threshold = 0.7
        self.lead = 0.75e9
        self.lag = 1.5e9
        self.img_time_dict = {}
        self.pending_obstacles = []
        self.stitching_img_dict = {}
        self.ids_to_stitch = []
        self.should_stitch = False
        self.stitch_len = 0

        self.model = str(Path(__file__).resolve().parent / "bestv8n.pt")
        self.filename = "task1"


        self.focal_length = 500
        self.obstacle_real_width_cm = 10
        self.bullseye_last_distance = None
        self.approach_distance_cm = 85


        self.pc_receive_thread = None
        self.stream_listener = None

    def on_result(self, result, frame):

        if result is not None:
            names = result.names

            for box in result.boxes:
                detected_image_id = names[int(box.cls[0].item())]
                detection_confidence = box.conf.item()
                if detected_image_id in self.IMG_BLACKLIST:
                    continue


                bbox_pixel_width = box.xywh[0][2].item()
                if bbox_pixel_width > 0:
                    distance_cm = (self.focal_length * self.obstacle_real_width_cm) / bbox_pixel_width
                else:
                    distance_cm = float('inf')
                logging.info(f"Detected {detected_image_id} — bbox_w={bbox_pixel_width:.1f}px, dist≈{distance_cm:.1f}cm")

                if detected_image_id == "45":
                    self.bullseye_last_distance = distance_cm
                    continue

                add_to_stitching_dict(self.stitching_img_dict, detected_image_id, detection_confidence, frame)


                last_seen = time_ns()
                first_seen = last_seen
                if detected_image_id in self.img_time_dict:
                    first_seen = self.img_time_dict[detected_image_id][0]

                self.img_time_dict[detected_image_id] = (first_seen, last_seen)

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
        logging.info(f"Sending: {message_content}")
        self.sock.send(message_content.encode("utf-8"))

    def pc_receive(self) -> None:
        self.connect()
        logging.info("PC Socket connection started successfully")

        while not self.exit:
            try:
                data_str = self.sock.recv(1024).decode("utf-8")
                logging.info(f"Received from RPI: {data_str}")

                if data_str.startswith("OBSTACLES"):

                    obstacles = self._parse_obstacles(data_str)
                    logging.info(f"Parsed obstacles: {obstacles}")


                    path = pathfinding(obstacles, robot_x=10, robot_y=4, big_turn=1)

                    commands = path['commands']
                    segments = self._segment_commands(commands)
                    logging.info(f"Segmented commands: {segments}")


                    self.sock.send(f"PATH,{json.dumps(segments)}\n".encode("utf-8"))
                    logging.info(f"Sent path back to rpi.")

                elif "DETECT" in data_str:
                    obstacle_id = data_str.split(",")[1]
                    timestamp = time_ns()

                    max_overlap = 0
                    max_img_id = None
                    for img_id, (first_seen, last_seen) in self.img_time_dict.items():
                        overlap = self.get_overlap_interval(img_id, timestamp, first_seen, last_seen)
                        logging.info(f"Overlap: {overlap}, Max overlap: {max_overlap}")
                        if overlap > 0 and overlap >= max_overlap:
                            logging.info(f"Replacing max overlap with {overlap}")
                            max_overlap = overlap
                            max_img_id = img_id

                    if max_img_id is not None and max_img_id != "45":

                        self.send_matched_pair(obstacle_id, max_img_id)
                        del self.img_time_dict[max_img_id]
                    else:

                        dist = self.bullseye_last_distance
                        if dist is not None and dist <= self.approach_distance_cm:
                            logging.info(f"Bullseye at {dist:.1f} cm (≤{self.approach_distance_cm} cm) — sending TURN")
                            self.sock.send("TURN\n".encode("utf-8"))
                        else:
                            if dist is not None:
                                logging.info(f"Bullseye at {dist:.1f} cm (>{self.approach_distance_cm} cm) — sending FORWARD")
                            else:
                                logging.info("Nothing detected — sending FORWARD")
                            self.sock.send("FORWARD\n".encode("utf-8"))

                elif "STITCH" in data_str:
                    stitch_images(self.ids_to_stitch, self.stitching_img_dict, filename=self.filename)

                if not data_str:
                    logging.info("Connection closed by server.")
                    break
            except OSError as e:
                logging.error(f"Error in sending data: {e}")
                break

    def start_stream(self):
        self.stream_listener = StreamListener(weights=self.model)
        self.stream_listener.start_stream_read(
            on_result=self.on_result,
            on_disconnect=self.disconnect,
            conf_threshold=self.conf_threshold,
            show_video=False
        )

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

    def start_checklist(self):
        self.pc_receive_thread = threading.Thread(target=self.pc_receive, daemon=True)
        print("Starting Task 1 server...")
        self.stream_listener = threading.Thread(target=self.start_stream, daemon=True)
        self.stream_listener.start()
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
                command_text += "90"
            elif command_text.startswith("BR"):
                command_text = "RR"
                command_text += "90"
            elif command_text.startswith("FW"):
                command_text = "F"
                command_text += dist
            elif command_text.startswith("FR"):
                command_text = "FR"
                command_text += "90"
            elif command_text.startswith("FL"):
                command_text = "FL"
                command_text += "90"
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

server = Checklist()
server.start_checklist()
