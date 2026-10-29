import os
import json
import logging
from threading import Thread, Lock, Event, Timer
from time import time, sleep
logging.basicConfig(level=logging.INFO)

from dotenv import load_dotenv
load_dotenv()

from communications.android_link import Android
from communications.pc_link import PC
from communications.stm import STM

from streaming.camera_server import StreamServer

class Task1:


    def __init__(self):

        self.process_pc_stream = None
        self.pc_thread = None
        self.android_thread = None
        self.stm_thread = None

        self.android = Android()
        self.pc = PC()
        self.stm = STM()

        self.obstacles = []
        self.started = False
        self.path_requested = False

        self.segments = []
        self.segments_index = 0
        self.obstacle_order = []

        self.directions = []
        self.direction_index = 0

        self._idx_lock = Lock()
        self.image_done = Event()
        self.path_ready = Event()
        self._debounce_lock = Lock()
        self._calc_timer = None
        self._debounce_delay = 1.0

        self.timeout = 0.5
        try:
            self.segment_delay = max(0.0, float(os.getenv("SEGMENT_DELAY_S", "0.5")))
        except ValueError:
            self.segment_delay = 0.5
        try:
            self.detect_retries = max(0, int(os.getenv("DETECT_RETRY_COUNT", "1")))
        except ValueError:
            self.detect_retries = 1
        try:
            self.detect_retry_delay = max(0.0, float(os.getenv("DETECT_RETRY_DELAY_S", "0.2")))
        except ValueError:
            self.detect_retry_delay = 0.2
        logging.info(f"Configured inter-segment delay: {self.segment_delay:.2f}s")
        logging.info(
            f"Configured DETECT retry policy: retries={self.detect_retries}, "
            f"retry_delay={self.detect_retry_delay:.2f}s, timeout={self.timeout:.2f}s"
        )

    def _schedule_path_calculation(self):

        with self._debounce_lock:
            if self._calc_timer is not None:
                self._calc_timer.cancel()
            self._calc_timer = Timer(self._debounce_delay, self._request_path_from_pc)
            self._calc_timer.daemon = True
            self._calc_timer.start()
            logging.info(f"Path calculation scheduled in {self._debounce_delay}s...")

    def _request_path_from_pc(self) -> bool:

        if self.path_requested:
            logging.info("PATH request already in flight; skipping duplicate OBSTACLES send.")
            return False
        self.pc.send("OBSTACLES," + json.dumps(self.obstacles) + "\n")
        self.path_requested = True
        logging.info("Sent obstacles to PC for path computation.")
        return True

    def _send_next_segment(self) -> bool:

        with self._idx_lock:
            if self.segments_index < 0 or self.segments_index >= len(self.segments):
                return False
            seg = self.segments[self.segments_index]
            self.segments_index += 1

        command_text = ",".join(seg) + "\n"
        self.stm.send(command_text)
        logging.info(f"Sent path segment {self.segments_index}/{len(self.segments)} to STM: {seg}")
        return True

    def _request_detect_with_retry(self, obstacle_id: str) -> bool:

        total_attempts = self.detect_retries + 1
        for attempt in range(1, total_attempts + 1):
            self.image_done.clear()
            self.pc.send(f"DETECT,{obstacle_id}\n")
            logging.info(f"Sent DETECT for obstacle {obstacle_id} (attempt {attempt}/{total_attempts})")

            if self.image_done.wait(timeout=self.timeout):
                logging.info(f"Received OBJECT for obstacle {obstacle_id} on attempt {attempt}.")
                return True

            if attempt < total_attempts:
                logging.warning(
                    f"No OBJECT received for obstacle {obstacle_id} within {self.timeout:.2f}s; "
                    "retrying DETECT once more."
                )
                if self.detect_retry_delay > 0:
                    sleep(self.detect_retry_delay)

        logging.warning(
            f"No OBJECT received for obstacle {obstacle_id} after {total_attempts} DETECT attempt(s)."
        )
        return False

    def stream_start(self):
        StreamServer().connect()

    @staticmethod
    def _is_calculation_request(message):
        return message.split(",", 1)[0].strip() in {"PATH", "CALCULATE"} or message.startswith("ALG|")

    def _parse_tablet_obstacle(self, fields):
        if len(fields) < 4:
            raise ValueError("Obstacle messages require an ID and coordinates")
        obstacle_id, x, y = map(int, fields[1:4])
        previous = next((obstacle for obstacle in self.obstacles if obstacle["id"] == obstacle_id), None)
        direction = previous["d"] if previous is not None else 8
        if len(fields) >= 5:
            directions = {"NORTH": 0, "EAST": 2, "SOUTH": 4, "WEST": 6, "SKIP": 8}
            direction = directions[fields[4].strip().upper()]
        return {"id": obstacle_id, "x": x, "y": y, "d": direction}

    def android_receive(self) -> None:
        while True:
            try:
                tablet_payload = self.android.receive()
                if not tablet_payload:
                    continue
                tablet_payload = tablet_payload.strip()
                if not tablet_payload:
                    continue

                if tablet_payload == "BEGIN":

                    if not self.started:
                        logging.info("Received BEGIN from Android, ending obstacle input.")
                        self.started = True

                    with self._idx_lock:
                        self.segments_index = 0


                    if not self.path_ready.is_set():
                        logging.info("BEGIN received but PATH not ready yet; waiting for PC response.")
                        continue


                    if not self._send_next_segment():
                        logging.warning("BEGIN received but no segments available to send.")
                        continue
                else:
                    msg_parts = tablet_payload.split(',')
                    if msg_parts[0] == "OBSTACLE":
                        obstacle = self._parse_tablet_obstacle(msg_parts)


                        existing = next(
                            (o for o in self.obstacles if o["id"] == obstacle["id"]),
                            None
                        )

                        if existing is not None:
                            existing.update(obstacle)
                            logging.info(f"Updated obstacle: {obstacle}")
                        else:
                            self.obstacles.append(obstacle)
                            logging.info(f"Added obstacle: {obstacle}")

                        logging.info(
                            f"Current obstacles ({len(self.obstacles)}): {self.obstacles}"
                        )

                        self.path_ready.clear()
                        self.path_requested = False
                        self._schedule_path_calculation()

                    elif msg_parts[0] == "CLEAR":

                        obstacle_id = int(msg_parts[1])

                        original_count = len(self.obstacles)

                        self.obstacles = [
                            obstacle
                            for obstacle in self.obstacles
                            if obstacle["id"] != obstacle_id
                        ]

                        if len(self.obstacles) < original_count:
                            logging.info(f"Removed obstacle ID {obstacle_id}.")
                        else:
                            logging.warning(
                                f"CLEAR received for obstacle ID {obstacle_id}, "
                                "but no matching obstacle was found."
                            )

                        logging.info(
                            f"Current obstacles ({len(self.obstacles)}): {self.obstacles}"
                        )

                        self.path_ready.clear()
                        self.path_requested = False
                        self._schedule_path_calculation()

                    elif msg_parts[0] == "RESET":

                        with self._debounce_lock:
                            if self._calc_timer is not None:
                                self._calc_timer.cancel()
                                self._calc_timer = None

                        self.obstacles = []
                        self.obstacle_order = []
                        self.path_ready.clear()
                        self.path_requested = False

                        logging.info("Reset obstacle list.")
                    elif self._is_calculation_request(tablet_payload):
                        if self.started and self.path_ready.is_set():
                            logging.info("Ignoring PATH request from Android; PATH already ready and mission started.")
                        else:


                            with self._debounce_lock:
                                if self._calc_timer is not None:
                                    self._calc_timer.cancel()
                                    self._calc_timer = None
                            self.path_requested = False
                            self._request_path_from_pc()
            except (OSError, ValueError, KeyError, IndexError) as e:
                print(f"Error: {e}")
                continue

    def pc_receive(self) -> None:
        while True:
            try:
                pc_payload = self.pc.receive()
                if pc_payload.startswith("PATH"):
                    path = json.loads(pc_payload.split("PATH,")[1])
                    logging.info(f"Received path segments: {path}")

                    with self._idx_lock:
                        self.segments = path.get('segments', [])
                        self.obstacle_order = path.get('obstacle_ids', [])
                        self.directions = path.get('dirs', [])

                        self.direction_index = 0

                    self.path_requested = False
                    self.path_ready.set()
                    self.android.send("PATHREADY")


                    if self.started and self.segments_index == 0:
                        if not self._send_next_segment():
                            logging.warning("Received PATH but no segments were provided.")
                elif pc_payload.startswith("OBJECT"):
                    self.image_done.set()
                    message_fields = pc_payload.replace("\n", "").split(",")[1:]

                    obstacle_id, confidence_text, object_id = message_fields
                    confidence_level = None

                    try:
                        confidence_level = float(confidence_text)
                    except ValueError:
                        confidence_level = None

                    logging.info(f"OBJECT ID: {object_id}")

                    if confidence_level is not None:
                        self.android.send(f"TARGET,{obstacle_id},{object_id}")

            except OSError as e:
                print(f"Error: {e}")
                continue

    def stm_receive(self) -> None:
        while True:
            try:
                stm_msg = self.stm.receive()
                if not stm_msg:
                    continue
                logging.info(f"Received from STM: {stm_msg}")
                if "RESEND" in stm_msg:
                    with self._idx_lock:
                        if not self.segments:
                            logging.warning("RESEND received but no path segments are loaded yet.")
                            continue
                        last_idx = max(self.segments_index - 1, 0)
                        command_text = ",".join(self.segments[last_idx]) + "\n"
                    self.stm.send(command_text)
                    logging.info(f"STM Error. Resending path segment{self.segments_index - 1}/{len(self.segments)}: {command_text}")

                    self.image_done.wait(0.1)
                elif "OK" in stm_msg:
                    with self._idx_lock:
                        just_finished_idx = self.segments_index - 1
                        more_to_send = self.segments_index < len(self.segments)

                    if just_finished_idx >= 0 and just_finished_idx < len(self.obstacle_order):
                        self._request_detect_with_retry(str(self.obstacle_order[just_finished_idx]))

                    if more_to_send:
                        if self.segment_delay > 0:
                            sleep(self.segment_delay)
                        if not self._send_next_segment():
                            logging.warning("Expected more segments, but none available to send.")
                    else:
                        self.pc.send(f'STITCH,{len(self.segments) - 1}\n')


                elif "done" in stm_msg:
                    if self.direction_index < len(self.directions):


                        continue
            except OSError as e:
                print(f"Error: {e}")
                continue

    def reconnect(self):


        pass

    def start(self):

        logging.info("Starting Task 1.")
        self.android.start()
        self.pc.connect()
        self.stm.connect()

        self.process_pc_stream = Thread(target=self.stream_start)
        self.process_pc_stream.start()
        sleep(5)

        self.stm_thread = Thread(target=self.stm_receive)
        self.pc_thread = Thread(target=self.pc_receive)
        self.android_thread = Thread(target=self.android_receive)
        self.android_reconnect = Thread(target=self.reconnect)
        self.stm_thread.start()
        self.pc_thread.start()
        self.android_thread.start()

        self.stm_thread.join()
        self.pc_thread.join()
        self.android_thread.join()

        self.android_reconnect.start()
        self.android_reconnect.join()

        logging.info("Task 1 completed.")

if __name__ == "__main__":
    task1 = Task1()
    task1.start()
