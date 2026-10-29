import os
import json
import logging
from threading import Thread
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
        self.detect_image = False
        self.stm_stopped = False
        self.started = False

        self.segments = []
        self.segments_index = 0
        self.obstacle_order = []
        self.snap_no = []

        self.timeout = 2

    def stream_start(self):
        StreamServer().connect()

    def android_receive(self) -> None:
        while True:
            try:
                tablet_payload = self.android.receive()
                if not tablet_payload:
                    continue
                tablet_payload = tablet_payload.strip()
                if not tablet_payload:
                    continue

                if tablet_payload == "BEGIN" and not self.started:
                    logging.info("Received BEGIN from Android, ending obstacle input.")
                    self.started = True
                    logging.info("Sent obstacles to PC.")
                else:
                    msg_parts = tablet_payload.split(',')
                    if msg_parts[0] == "OBSTACLE":
                        obstacle = {
                            "id": int(msg_parts[1]),
                            "x": int(msg_parts[2]) / 10,
                            "y": int(msg_parts[3]) / 10,
                            "d": {"NORTH": 0, "EAST": 2, "SOUTH": 4, "WEST": 6, "SKIP": 8}.get(msg_parts[4].strip())
                        }
                        self.obstacles.append(obstacle)
                        logging.info(f"Added obstacle: {obstacle}")
                    elif msg_parts[0] == "CLEAR":
                        self.obstacles = []
                        self.obstacle_order = []
                        logging.info("Cleared obstacles list.")
                    elif msg_parts[0].strip() == "PATH":
                        self.pc.send("OBSTACLES," + json.dumps(self.obstacles))
            except OSError as e:
                print(f"Error: {e}")
                continue

    def pc_receive(self) -> None:
        while True:
            try:
                pc_payload = self.pc.receive()
                if pc_payload.startswith("PATH"):
                    path = json.loads(pc_payload.split("PATH,")[1])
                    logging.info(f"Received path segments: {self.segments}")
                    self.segments = path['segments']
                    self.obstacle_order = path['obstacle_ids']
                    self.snap_no = path["snap_no"]
                    command_text = self.segments[self.segments_index] + "\n"
                    self.stm.send(command_text)
                    logging.info(f"Sent path segment {self.segments_index + 1} to STM: {self.segments[self.segments_index]}")
                    self.segments_index += 1
                elif pc_payload.startswith("OBJECT"):
                    self.detect_image = False
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
                stm_msg = self.stm.wait_receive()
                logging.info(f"Received from STM: {stm_msg}")
                if "RESEND" in stm_msg:
                    command_text = ",".join(self.segments[self.segments_index - 1]) + "\n"
                    self.stm.send(command_text)
                    logging.info(f"STM Error. Resending path segment{self.segments_index - 1}/{len(self.segments)}: {command_text}")
                elif "OK" in stm_msg:
                    sleep(0.5)
                    if self.segments_index in self.snap_no:
                        message_content = f"DETECT,{self.obstacle_order[self.segments_index - 1]}"
                        self.pc.send(message_content)
                    end_time = time() + self.timeout
                    self.stm_stopped = True

                    while self.detect_image or time() < end_time:
                        pass
                    if self.segments_index < len(self.segments):
                        command_text = self.segments[self.segments_index] + "\n"
                        self.stm.send(command_text)
                        logging.info(f"Sent path segment {self.segments_index + 1}/{len(self.segments)} to STM: {self.segments[self.segments_index]}")
                        self.segments_index += 1
                    else:
                        self.pc.send(f'STITCH,{len(self.segments) - 1}')
                        self.android.disconnect()
                        self.pc.disconnect()
                        self.stm.disconnect()
            except OSError as e:
                print(f"Error: {e}")
                continue

    def start(self):

        logging.info("Starting Task 1.")
        self.android.connect()
        self.pc.connect()
        self.stm.connect()

        self.process_pc_stream = Thread(target=self.stream_start)
        self.process_pc_stream.start()
        sleep(5)

        self.android_thread = Thread(target=self.android_receive)
        self.pc_thread = Thread(target=self.pc_receive)
        self.stm_thread = Thread(target=self.stm_receive)
        self.android_thread.start()
        self.pc_thread.start()
        self.stm_thread.start()

        self.android_thread.join()
        self.pc_thread.join()
        self.stm_thread.join()

        logging.info("Task 1 completed.")

if __name__ == "__main__":
    task1 = Task1()
    task1.start()
