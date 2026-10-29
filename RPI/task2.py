import math
import logging
from threading import Thread, Lock, Event
from time import time, time_ns
logging.basicConfig(level=logging.INFO)

from dotenv import load_dotenv
load_dotenv()

from communications.android_link import Android
from communications.pc_link import PC
from communications.stm import STM

from streaming.camera_server import StreamServer
from firmware_protocol import TASK2_READY

class Task2:
    def __init__(self):
        self.process_pc_stream = None
        self.pc_thread = None
        self.android_thread = None
        self.stm_thread = None

        self.android = Android()
        self.pc = PC()
        self.stm = STM()

        self.lock = Lock()
        self.obstacle_lock = Lock()
        self.last_image = None
        self.last_image_confidence = None
        self.last_image_timestamp_ns = None
        self.prev_image = None
        self.image_update_event = Event()
        self.image_seq = 0
        self.obstacle2_min_image_seq = 0
        self.expected_image_obstacle = 1
        self.arrow_confidence_threshold = 0.7
        self.arrow_max_age_s = 2.0

        self.obstacle2_arrow_required_hits = 3
        self.obstacle2_arrow_candidate = None
        self.obstacle2_arrow_hits = 0
        self.obstacle2_arrow_last_hit_ns = None

        self.obstacle2_alignment_max_error_deg = 10.0
        self.obstacle2_alignment_max_yaw_dps = 20.0
        self.obstacle2_alignment_max_age_s = 1.0
        self.obstacle2_alignment_ready = False
        self.obstacle2_alignment_timestamp_ns = None
        self.obstacle2_approach_complete = False

        self.obstacle_order = []
        self.num_obstacle = 1
        self.begin_stage = 0
        self.route_start_requested = False
        self.timeout = 2

        self.on_arrow_callback = None
        self.obstacle1_direction = None
        self.obstacle1_arrow_id = None
        self.obstacle2_direction = None
        self.ultrasound_travelled_dist1 = None
        self.ultrasound_travelled_dist2 = None
        self.ultrasound_stop_dist1 = None
        self.ultrasound_stop_dist2 = None
        self.chassis_length = 28
        self.chassis_width = 19
        self.obstacle_length = 10
        self.slide_dist = 50
        self.slide_bridge_cm = 0
        self.slide_clearance_travel_cm = 0.0
        self.obstacle2_close_approach_cm = 32
        self.turn_bias = 15
        self.extra_dist = 15
        self.center_offset_diag = 10
        self.center_offset_ninty = 7
        self.ninty_return_bias = 40
        self.turn_x = 30
        self.turn_y = 15
        self.parking_pass_obstacle1_cm = -5

        self.parking_lateral_trim_by_direction_cm = {"L": 15.0, "R": 20.0}

        self.parking_reverse_correction_scale = 0.5
        self.parking_cross_move_deadband_cm = 7.0
        self.ninty_threshold_length = 110.0
        self.ninty_threshold_width = 45.0
        self.sanity_forward_cm = 5
        self.ir_wall_detect_min_dist = 0.1
        self.ir_wall_detect_max_age_s = 15.0
        self.short_obs_threshold = 40
        self.short_obs_reverse_cm = 10
        self.last_wall_hug_direction = None
        self.last_ir_direction = None
        self.last_ir_timestamp_ns = None
        self.obstacle2_ir_travel_cm = 0.0
        self.smooth_firmware_ready = False
        self.stm_handshake_last_send_ns = None
        self.stm_handshake_attempts = 0
        self.obstacle1_script_queued = False
        self.obstacle2_script_queued = False
        self.carpark_script_queued = False
        self.waiting_for_obstacle2 = False
        self.initial_reverse_min_seq = None
        self.obstacle2_ir_reports = 0
        self.route_faulted = False

        self.LEFT_ARROW_ID = "39"
        self.RIGHT_ARROW_ID = "38"
        self.directions = {"39": "L", "38": "R"}
        self.wall_hug_length = None

        self.start_time = None
        self.end_time = None

    def stream_start(self):
        StreamServer().connect()

    def android_receive(self) -> None:
        while True:
            try:
                tablet_payload = self.android.receive()

                if tablet_payload == "BEGIN":
                    if self.begin_stage == 0:
                        with self.lock:
                            self.route_start_requested = True
                        logging.info(
                            "BEGIN received; starting when a fresh, confident obstacle-1 arrow is available."
                        )
                        self._try_start_route()
                    else:
                        logging.info("Ignoring duplicate BEGIN after the route has started.")

            except OSError as e:
                print(f"Error: {e}")
                continue

    def pc_receive(self) -> None:
        receive_buffer = ""
        while True:
            try:
                pc_payload = self.pc.receive()
                if not pc_payload:
                    continue
                receive_buffer += pc_payload
                while "\n" in receive_buffer:
                    raw_message, receive_buffer = receive_buffer.split("\n", 1)
                    message_text = raw_message.strip()
                    if not message_text:
                        continue

                    if message_text == "NONE":
                        self.set_last_image("NONE")
                        continue

                    message_fields = [p.strip() for p in message_text.split(",")]
                    obstacle_id = None

                    if len(message_fields) == 2:
                        confidence_text, object_id = message_fields
                    elif len(message_fields) >= 3:
                        obstacle_text, confidence_text, object_id = message_fields[:3]
                        try:
                            obstacle_id = int(obstacle_text)
                        except ValueError:
                            logging.info(f"Malformed obstacle id from PC message: {message_text}")
                            continue
                    else:
                        continue

                    if obstacle_id is not None:
                        with self.lock:
                            expected_obstacle = self.expected_image_obstacle

                        if obstacle_id != expected_obstacle:
                            logging.info(
                                f"Ignoring stale/future image for obstacle {obstacle_id}; "
                                f"expecting obstacle {expected_obstacle}."
                            )
                            continue

                    confidence_level = None
                    try:
                        confidence_level = float(confidence_text)
                    except ValueError:
                        confidence_level = None

                    if (
                        confidence_level is not None
                        and confidence_level >= self.arrow_confidence_threshold
                        and object_id in self.directions
                    ):
                        self.prev_image = object_id
                        self.set_last_image(object_id, confidence_level)
                        self._try_start_route()
                    else:
                        self.set_last_image("NONE")

            except OSError as e:
                print(f"Error: {e}")
                continue

    def _try_stm_handshake(self) -> None:
        with self.lock:
            if self.smooth_firmware_ready or self.route_faulted or self.begin_stage != 0:
                return
            now = time_ns()
            if (self.stm_handshake_last_send_ns is not None
                    and now - self.stm_handshake_last_send_ns < 2_000_000_000):
                return
            if self.stm_handshake_attempts >= 10:
                self.route_faulted = True
                logging.error(
                    "No matching STM handshake after 10 attempts. Motion remains disabled. "
                    "Expected SMOOTH_REL_READY,50; check STM power/reset and the Pi serial port/link. "
                    "Restart this program after restoring the link."
                )
                return
            self.stm_handshake_attempts += 1
            self.stm_handshake_last_send_ns = now
            logging.info("Checking STM firmware (attempt %d/10).", self.stm_handshake_attempts)

            self.stm.send("\nSMOOTH_REL?\n")

    def stm_receive(self) -> None:
        while True:
            try:
                self._try_stm_handshake()
                controller_payload = (self.stm.wait_receive(timeout_s=0.5) or "").strip()
                if not controller_payload:
                    continue
                logging.info("Received from STM: %s", controller_payload)
                if controller_payload.startswith(("SMOOTH_BOOT,", "UART_RX_RECOVER,", "SMOOTH_REL_BUSY,")):
                    if controller_payload.startswith("SMOOTH_BOOT,"):
                        with self.lock:
                            reset_during_route = self.begin_stage != 0
                            if reset_during_route:
                                self.route_faulted = True
                                self.smooth_firmware_ready = False
                        if reset_during_route:
                            logging.error("STM rebooted during the route. Route progression is halted.")
                    continue
                if controller_payload.startswith("SLIDE,CLEARANCE,DONE,"):
                    parts = controller_payload.split(",")
                    try:
                        if len(parts) != 5 or parts[3] not in ("L", "R"):
                            raise ValueError("invalid slide clearance report")
                        clearance_cm = float(parts[4])
                        if not math.isfinite(clearance_cm) or clearance_cm < 0:
                            raise ValueError("invalid slide clearance distance")
                    except ValueError:
                        logging.warning("Ignoring malformed slide clearance report: %r", controller_payload)
                        continue
                    with self.lock:
                        if (self.begin_stage != 0 and not self.route_faulted
                                and self.num_obstacle == 2):
                            self.slide_clearance_travel_cm += clearance_cm
                            logging.info(
                                "STM slide clearance: +%.1f cm; accumulated %.1f cm for return-distance accounting.",
                                clearance_cm, self.slide_clearance_travel_cm,
                            )
                    continue
                if controller_payload.startswith(("SLIDE,", "SLIDE_SPEED,", "SLIDE_BRIDGE,", "SLIDE_US,")):
                    self._maybe_arm_obstacle2_from_slide(controller_payload)
                    self._update_obstacle2_alignment(controller_payload)
                    continue
                if controller_payload.startswith("IRH,"):
                    parts = controller_payload.split(",")
                    if len(parts) == 6:
                        logging.info(
                            "IR hold: mode=%s, heading error=%s deg, yaw rate=%s deg/s, correction=%s%%, servo=%s",
                            *parts[1:]
                        )
                    continue
                if controller_payload == TASK2_READY:
                    with self.lock:
                        self.smooth_firmware_ready = True
                    self._try_start_route()
                    continue
                if controller_payload.startswith("SMOOTH_REL_READY,"):
                    logging.error("STM firmware mismatch: expected SMOOTH_REL_READY,50, received %s. Rebuild/reflash the matching SmoothFlow main.c.", controller_payload)
                    continue
                if controller_payload.startswith(("FU_REF,", "FUH,", "FLOW_REF,", "FLOWH,")):
                    self._update_obstacle2_alignment(controller_payload)
                    continue
                if controller_payload.startswith(("TURN_DONE,", "TURN_REF,", "FLOW_REF,", "FLOWH,", "FLOW_ENTRY_READY", "TURN_CFG,", "EXIT_CAP,", "REV_SPEED,", "REV_CLEAR,", "REV_CENTER,", "REV_HEADING,", "PARK_SPEED,", "PARK_US,", "REAR_CARRY,", "ENTRY_CARRY,", "RETURN_CARRY,", "REAR_UNWIND,", "TURN_UNWIND,", "TURN_BEGIN,", "TDRV,", "TMOT,", "IR_EDGE,", "FU_REF,", "FU_APPROACH,", "FU_SELECT,", "FU_US,", "FU_HOLD,", "FUH,", "IRS,", "SVPWM,")):
                    continue
                if controller_payload.startswith("TURN_XY,"):
                    parts = controller_payload.split(",")
                    if len(parts) == 3:
                        logging.info(
                            "Turn displacement estimate: original-forward %s cm, original-left %s cm.",
                            *parts[1:]
                        )
                    continue
                if controller_payload.startswith("REV_DONE,"):
                    parts = controller_payload.split(",")
                    if len(parts) == 4:
                        logging.info("FU%s reverse completed: ultrasound range %s cm, backward encoder travel %s cm.", *parts[1:])
                    continue
                if controller_payload == "VISION,2":
                    self._arm_obstacle2_vision()
                    continue
                if controller_payload == "REVERSE,1":
                    with self.lock:
                        self.initial_reverse_min_seq = self.image_seq
                    logging.info("FU40 is reversing; a fresh confirmation of the locked arrow will release it.")
                    continue
                if controller_payload == "REVERSE,2":
                    self._try_queue_turn2()
                    continue
                if controller_payload == "OK":
                    if self.begin_stage == 0 or not self.smooth_firmware_ready or self.route_faulted:
                        logging.warning("Ignoring OK outside an active SmoothFlow route.")
                        continue
                    if self.num_obstacle == 1:
                        with self.lock:
                            self.initial_reverse_min_seq = None

                        if self.obstacle1_script_queued:
                            with self.obstacle_lock:
                                self.num_obstacle += 1
                            continue
                        direction = self.obstacle1_arrow_id
                        logging.info(f"Using BEGIN-locked obstacle 1 arrow: {direction}")
                        if direction in self.directions:
                            self.obstacle1_callback(direction)
                        else:
                            self.on_arrow_callback = self.obstacle1_callback

                    elif self.num_obstacle == 2:
                        self._arm_obstacle2_vision()
                        with self.lock:
                            self.obstacle2_approach_complete = True
                        self._try_queue_turn2()
                        if not self.obstacle2_script_queued:
                            logging.info("Approach complete; waiting for a fresh obstacle-2 arrow without a fixed delay.")

                    elif self.num_obstacle == 3:
                        with self.lock:
                            direction = self.obstacle2_direction

                        if direction not in ("L", "R"):
                            latest = self.get_last_image()
                            if latest in self.directions:
                                direction = self.directions[latest]
                                logging.info(
                                    f"Carpark direction fallback from latest image: {direction}"
                                )
                            else:
                                logging.info(
                                    f"Carpark direction unavailable (latest={latest}); defaulting to L."
                                )
                                direction = "L"

                        carpark_turn_direction = "L" if direction == "R" else "R"
                        self._queue_carpark(carpark_turn_direction, direction)
                        self.pc.send("STITCH\n")

                    with self.obstacle_lock:
                        if self.num_obstacle == 4:
                            logging.info(f"Timing: {(time_ns() - self.start_time) / 1e9:.3}s")
                        self.num_obstacle += 1
                elif controller_payload.startswith("ir"):
                    dist = controller_payload.split("ir")[1].replace("\n", "")
                    try:
                        dist = float(dist)
                    except:
                        logging.info(f"Couldn't cast WH dist {dist} to float")
                        continue
                    self.wall_hug_length = dist

                    if self.num_obstacle == 3:
                        self.obstacle2_ir_travel_cm = max(
                            self.obstacle2_ir_travel_cm, dist
                        )
                    self.last_ir_direction = self.last_wall_hug_direction
                    self.last_ir_timestamp_ns = time_ns()
                    if self.num_obstacle == 3:
                        self.obstacle2_ir_reports += 1
                        if self.obstacle2_ir_reports == 2:
                            direction = self.obstacle2_direction
                            if direction in ("L", "R"):
                                opposite = "L" if direction == "R" else "R"
                                self._queue_carpark(opposite, direction)
                elif controller_payload.startswith("us"):
                    controller_payload = controller_payload.replace("us", "").replace("\n", "")
                    dist = controller_payload.split(",")
                    try:
                        for i in range(2):
                            dist[i] = float(dist[i])
                    except:
                        logging.info(f"Couldn't cast US travelled dist {dist} to float")
                        continue
                    if self.num_obstacle == 1:
                        self.ultrasound_travelled_dist1 = dist[0]
                        self.ultrasound_stop_dist1 = dist[1]
                        logging.info(f"Set ultrasound_travelled_dist1 to {dist[0]} and ultrasound_stop_dist1 to {dist[1]}")
                    elif self.num_obstacle == 2:
                        self.ultrasound_travelled_dist2 = dist[0]
                        self.ultrasound_stop_dist2 = dist[1]
                        logging.info(f"Set ultrasound_travelled_dist2 to {dist[0]} and ultrasound_stop_dist2 to {dist[1]}")
                elif controller_payload.startswith("ERR") or controller_payload == "RST":
                    with self.lock:
                        self.route_faulted = True
                        self.waiting_for_obstacle2 = False
                        self.initial_reverse_min_seq = None
                        self.on_arrow_callback = None
                    logging.error("STM reported a fault/reset: %r. Route progression is halted.", controller_payload)
                else:
                    logging.warning("Unexpected STM reply %r; waiting for a complete OK acknowledgement.", controller_payload)

            except OSError as e:
                print(f"Error: {e}")
                continue

    def turn_and_go(self, full_length, half_width):
        angle_rad = math.atan2(abs(half_width), full_length)
        angle_deg = math.degrees(angle_rad)
        distance = math.hypot(full_length, half_width)
        return abs(angle_deg), int(distance)

    def ultrasound_forward(self, distance_to_stop, send = True):
        command_text = f"FU{str(distance_to_stop)}"
        if send:
            self.stm.send(command_text + "\n")
        return command_text

    def perform_turn1(self, direction, send = True):
        opp_dir = "L" if direction == "R" else "R"

        command_text = "S" + direction
        if self.slide_bridge_cm > 0:
            command_text += f",F{self.slide_bridge_cm}"
        command_text += ",S" + opp_dir

        command_text += "," + self.ultrasound_forward(self.obstacle2_close_approach_cm, False)
        if send:
            self.stm.send(command_text + "\n")
        return command_text

    def wall_hug(self, direction, send = True):
        command_text = "FI" + direction
        self.last_wall_hug_direction = direction
        if send:
            self.stm.send(command_text + "\n")
        return command_text

    def should_add_pre_turn_sanity_forward(self, side):
        if self.wall_hug_length is None:
            logging.info("Pre-turn IR sanity check skipped: no IR reading available.")
            return False

        if self.last_ir_direction != side:
            logging.info(
                f"Pre-turn IR sanity check skipped: last IR side={self.last_ir_direction}, required side={side}."
            )
            return False

        if self.last_ir_timestamp_ns is not None:
            age_s = (time_ns() - self.last_ir_timestamp_ns) / 1e9
            if age_s > self.ir_wall_detect_max_age_s:
                logging.info(
                    f"Pre-turn IR sanity check skipped: stale IR reading ({age_s:.2f}s old)."
                )
                return False

        wall_detected = self.wall_hug_length > self.ir_wall_detect_min_dist
        logging.info(
            f"Pre-turn IR sanity check side={side}: dist={self.wall_hug_length}, detected={wall_detected}"
        )
        return wall_detected

    def perform_turn2(self, direction, send = True):
        opp_dir = "L" if direction == "R" else "R"

        command_text = f"F{direction}90"

        command_text += "," + self.wall_hug(opp_dir, False)

        command_text += f",F{opp_dir}180,FI{opp_dir}O"
        command_text += "," + self.wall_hug(opp_dir, False)

        command_text += f",F{opp_dir}90"

        if send:
            self.stm.send(command_text + "\n")
        return command_text

    def perform_carpark(self, direction, sanity_side = None, send = True):
        ultrasound_travel_cm = self.ultrasound_travelled_dist2 or 0.0
        ultrasound_stop_cm = self.ultrasound_stop_dist2 or 0.0
        with self.lock:
            slide_clearance_cm = self.slide_clearance_travel_cm

        obs_1_to_2 = (ultrasound_travel_cm + ultrasound_stop_cm
                      + self.slide_dist + self.slide_bridge_cm + slide_clearance_cm)

        logging.info(
            "Return gap estimate: FU travel %.1f + stop range %.1f + slide %.1f + slide bridge %.1f + STM clearance %.1f cm.",
            ultrasound_travel_cm, ultrasound_stop_cm, self.slide_dist, self.slide_bridge_cm,
            slide_clearance_cm,
        )
        logging.info(f"Obs 1 to 2 dist: {obs_1_to_2}")

        rear_face_travel_cm = self.wall_hug_length or 0.0
        half_width = (rear_face_travel_cm / 2.0) + self.turn_bias + self.extra_dist + self.center_offset_diag
        return_to_turn_cm = int(round(obs_1_to_2 + self.parking_pass_obstacle1_cm))
        logging.info(
            "90-degree return: estimated obstacle gap %.1f + pass margin %.1f = %d cm.",
            obs_1_to_2, self.parking_pass_obstacle1_cm, return_to_turn_cm,
        )

        arc_cross_cm = self.turn_x + self.turn_y
        if arc_cross_cm > half_width - self.center_offset_ninty:
            cross_move_cm = -max(0, int(round(
                arc_cross_cm - (half_width - self.center_offset_ninty)
            )))
        else:
            cross_move_cm = max(0, int(round(
                half_width - self.center_offset_diag - arc_cross_cm
            )))
        senior_cross_move_cm = cross_move_cm
        lateral_trim_cm = self.parking_lateral_trim_by_direction_cm.get(
            direction, 0.0
        )
        corrected_cross_move_cm = senior_cross_move_cm - lateral_trim_cm
        cross_move_cm = int(round(corrected_cross_move_cm))
        logging.info(
            "Cross-bay estimate: rear IR travel %.1f cm, half-width %.1f cm, assumed arc shift %.1f cm, move %+d cm.",
            rear_face_travel_cm, half_width, arc_cross_cm, cross_move_cm,
        )
        logging.info(
            "Parking lateral correction (%s-first): senior move %+d - measured trim %.1f cm = %+d cm.",
            direction, senior_cross_move_cm, lateral_trim_cm, cross_move_cm,
        )
        if corrected_cross_move_cm < 0.0:
            reverse_scale = max(0.0, min(1.0, self.parking_reverse_correction_scale))
            scaled_cross_move_cm = corrected_cross_move_cm * reverse_scale
            logging.info(
                "Parking reverse correction: %+.1f cm x %.2f = %+.1f cm before deadband.",
                corrected_cross_move_cm, reverse_scale, scaled_cross_move_cm,
            )
            corrected_cross_move_cm = scaled_cross_move_cm
        cross_move_cm = int(round(corrected_cross_move_cm))

        if abs(corrected_cross_move_cm) <= self.parking_cross_move_deadband_cm:
            logging.info(
                "Parking cross correction %+.1f cm is within +/-%.1f cm; skipping the cross move.",
                corrected_cross_move_cm, self.parking_cross_move_deadband_cm,
            )
            cross_move_cm = 0
        opp_dir = "L" if direction == "R" else "R"
        commands = ["PARK"]
        if return_to_turn_cm > 0:
            commands.append(f"F{return_to_turn_cm}")
        commands.append(f"F{direction}90")
        if cross_move_cm > 0:
            commands.append(f"F{cross_move_cm}")
        elif cross_move_cm < 0:
            commands.append(f"R{-cross_move_cm}")
        commands.extend((f"F{opp_dir}90", "FX10"))
        command_text = ",".join(commands)
        logging.info(f"Carpark 90-degree command script: {command_text}")
        if send:
            self.stm.send(command_text + '\n')
        return command_text

    def obstacle1_callback(self, direction):
        direction = self.directions[direction]
        logging.info(f"Calling obstacle 1 turn with direction {direction}")

        with self.lock:
            self.obstacle1_direction = direction

        self.perform_turn1(direction)

        self.on_arrow_callback = None

    def obstacle2_callback(self, direction):
        direction = self.directions[direction]

        logging.info(f"Calling obstacle 2 turn with direction {direction}")
        with self.lock:
            self.obstacle2_direction = direction

        self.perform_turn2(direction)
        self.on_arrow_callback = None

    def _maybe_arm_obstacle2_from_slide(self, message):
        parts = message.split(",")
        if len(parts) < 5 or parts[0] != "SLIDE" or parts[2] != "IN":
            return
        with self.lock:
            if (self.route_faulted or not self.smooth_firmware_ready
                    or self.begin_stage == 0 or not self.obstacle1_script_queued
                    or self.num_obstacle != 2 or self.obstacle1_direction not in ("L", "R")):
                return

            final_side = "L" if self.obstacle1_direction == "R" else "R"
        if parts[1] == final_side:
            self._arm_obstacle2_vision(source="final slide inward phase")

    def _arm_obstacle2_vision(self, source="FU approach"):
        with self.lock:
            if self.route_faulted or self.waiting_for_obstacle2 or self.obstacle2_script_queued:
                return
            self.last_image = "NONE"
            self.prev_image = None
            self.image_seq += 1
            self.obstacle2_min_image_seq = self.image_seq
            self.expected_image_obstacle = 2
            self.obstacle2_arrow_candidate = None
            self.obstacle2_arrow_hits = 0
            self.obstacle2_arrow_last_hit_ns = None
            self.obstacle2_alignment_ready = False
            self.obstacle2_alignment_timestamp_ns = None
            self.obstacle2_approach_complete = False
            self.waiting_for_obstacle2 = True

        self.pc.send("SEEN\n")
        logging.info(
            "Obstacle-2 vision armed during %s (FU%d close / FU40 far); "
            "confirmation requires aligned heading and %d fresh matching reports.",
            source, self.obstacle2_close_approach_cm, self.obstacle2_arrow_required_hits,
        )

    def _obstacle2_alignment_allows_confirmation_locked(self, now_ns):
        if not self.obstacle2_alignment_ready or self.obstacle2_alignment_timestamp_ns is None:
            return False
        age_s = (now_ns - self.obstacle2_alignment_timestamp_ns) / 1e9
        return (age_s >= 0 and (self.obstacle2_approach_complete
                               or age_s <= self.obstacle2_alignment_max_age_s))

    def _update_obstacle2_alignment(self, message):
        parts = message.split(",")
        try:
            if parts[0] == "SLIDE":
                if len(parts) != 6 or parts[2] != "DONE":
                    return
                with self.lock:
                    final_side = "L" if self.obstacle1_direction == "R" else "R"
                if parts[1] != final_side:
                    return
                heading, reference, yaw_rate = map(float, parts[3:6])
            elif parts[0] == "FU_REF":
                if len(parts) != 5 or parts[1] != "2":
                    return
                reference, heading, yaw_rate = map(float, parts[2:5])
            elif parts[0] == "FUH":
                if len(parts) != 6 or parts[1] not in ("F", "R"):
                    return
                reference, heading, yaw_rate = map(float, parts[2:5])
            elif parts[0] in ("FLOW_REF", "FLOWH"):
                expected_length = 4 if parts[0] == "FLOW_REF" else 5
                if len(parts) != expected_length:
                    return
                reference, heading, yaw_rate = map(float, parts[1:4])
            else:
                return
        except ValueError:
            logging.warning("Ignoring malformed obstacle-2 heading telemetry: %r", message)
            return
        if not all(math.isfinite(value) for value in (reference, heading, yaw_rate)):
            return
        error_deg = (heading - reference + 180.0) % 360.0 - 180.0
        now_ns = time_ns()
        aligned = (abs(error_deg) <= self.obstacle2_alignment_max_error_deg
                   and abs(yaw_rate) <= self.obstacle2_alignment_max_yaw_dps)
        with self.lock:
            if (not self.waiting_for_obstacle2 or self.route_faulted
                    or self.obstacle2_script_queued or self.num_obstacle not in (2, 3)):
                return
            was_allowed = self._obstacle2_alignment_allows_confirmation_locked(now_ns)
            self.obstacle2_alignment_ready = aligned
            self.obstacle2_alignment_timestamp_ns = now_ns
            if not aligned or not was_allowed:
                self.obstacle2_arrow_candidate = None
                self.obstacle2_arrow_hits = 0
                self.obstacle2_arrow_last_hit_ns = None
                self.obstacle2_min_image_seq = self.image_seq
        if aligned and not was_allowed:
            logging.info(
                "Obstacle-2 alignment gate OPEN: heading error %.1f deg, yaw %.1f deg/s; requiring %d new matching reports.",
                error_deg, yaw_rate, self.obstacle2_arrow_required_hits,
            )
        elif not aligned and was_allowed:
            logging.info(
                "Obstacle-2 alignment gate CLOSED: heading error %.1f deg, yaw %.1f deg/s; confirmation reset.",
                error_deg, yaw_rate,
            )

    def _try_queue_turn2(self):
        with self.lock:
            if self.route_faulted or not self.waiting_for_obstacle2 or self.obstacle2_script_queued:
                return
            if not self._obstacle2_alignment_allows_confirmation_locked(time_ns()):
                return
            image = self.last_image
            confidence = self.last_image_confidence
            age_s = ((time_ns() - self.last_image_timestamp_ns) / 1e9
                     if self.last_image_timestamp_ns is not None else float("inf"))
            if (image not in self.directions or self.image_seq <= self.obstacle2_min_image_seq
                    or confidence is None or confidence < self.arrow_confidence_threshold
                    or age_s > self.arrow_max_age_s
                    or self.obstacle2_arrow_candidate != image
                    or self.obstacle2_arrow_hits < self.obstacle2_arrow_required_hits):
                return
            direction = self.directions[image]
            self.obstacle2_direction = direction
            self.obstacle2_script_queued = True
            self.waiting_for_obstacle2 = False
        command = self.perform_turn2(direction, send=False)

        self.stm.send(command + "\n")
        logging.info(
            "Confirmed obstacle-2 arrow %s after %d matching reports; route queued for handoff after adaptive ultrasound approach / reverse clearance.",
            direction, self.obstacle2_arrow_required_hits,
        )

    def _try_confirm_initial_reverse(self):
        with self.lock:
            if self.route_faulted or self.initial_reverse_min_seq is None:
                return
            if (self.image_seq <= self.initial_reverse_min_seq
                    or self.last_image != self.obstacle1_arrow_id
                    or self.last_image_confidence is None
                    or self.last_image_confidence < self.arrow_confidence_threshold):
                return
            self.initial_reverse_min_seq = None
        self.stm.send("CONFIRM\n")
        logging.info("Fresh obstacle-1 arrow confirmed; releasing FU40 reverse into the queued slides.")

    def _queue_carpark(self, direction, side):
        with self.lock:
            if self.route_faulted or self.carpark_script_queued:
                return
            self.carpark_script_queued = True
        self.perform_carpark(direction, sanity_side=side)

    def _prime_obstacle2_direction_window(self) -> None:
        with self.lock:
            self.last_image = "NONE"
            self.prev_image = None
            self.image_seq += 1
            self.obstacle2_min_image_seq = self.image_seq
            self.expected_image_obstacle = 2
            self.obstacle2_arrow_candidate = None
            self.obstacle2_arrow_hits = 0
            self.obstacle2_arrow_last_hit_ns = None
            self.obstacle2_alignment_ready = False
            self.obstacle2_alignment_timestamp_ns = None
            self.obstacle2_approach_complete = False

        self.image_update_event.clear()
        logging.info(
            f"Primed obstacle-2 direction window (min_seq={self.obstacle2_min_image_seq})."
        )

    def _wait_for_fresh_direction(self, min_seq: int, timeout_s: float):
        deadline = time() + timeout_s
        while True:
            with self.lock:
                current_image = self.last_image
                current_seq = self.image_seq

            if current_seq > min_seq and current_image in self.directions:
                logging.info(
                    f"Using fresh obstacle-2 image {current_image} "
                    f"(seq={current_seq}, min_seq={min_seq})."
                )
                return current_image

            remaining = deadline - time()
            if remaining <= 0:
                return None

            self.image_update_event.wait(timeout=min(remaining, 0.2))
            self.image_update_event.clear()

    def get_last_image(self) -> str:
        with self.lock:
            image = self.last_image
        logging.info(f"Returning last_image as {image}")
        return image

    def set_last_image(self, img, confidence=None) -> None:
        confirmation_progress = None
        with self.lock:
            now_ns = time_ns()
            self.last_image = img
            self.last_image_confidence = confidence
            self.last_image_timestamp_ns = now_ns
            self.image_seq += 1
            if (self.waiting_for_obstacle2 and self.expected_image_obstacle == 2
                    and not self.obstacle2_script_queued and not self.route_faulted):
                valid = (self._obstacle2_alignment_allows_confirmation_locked(now_ns)
                         and img in self.directions and confidence is not None
                         and math.isfinite(confidence)
                         and confidence >= self.arrow_confidence_threshold
                         and self.image_seq > self.obstacle2_min_image_seq)
                if not valid:
                    self.obstacle2_arrow_candidate = None
                    self.obstacle2_arrow_hits = 0
                    self.obstacle2_arrow_last_hit_ns = None
                else:
                    recent = (self.obstacle2_arrow_last_hit_ns is not None
                              and 0 <= now_ns - self.obstacle2_arrow_last_hit_ns
                              <= self.arrow_max_age_s * 1e9)
                    if self.obstacle2_arrow_candidate == img and recent:
                        self.obstacle2_arrow_hits += 1
                    else:
                        self.obstacle2_arrow_candidate = img
                        self.obstacle2_arrow_hits = 1
                    self.obstacle2_arrow_last_hit_ns = now_ns
                    confirmation_progress = (self.directions[img],
                                             self.obstacle2_arrow_hits,
                                             self.obstacle2_arrow_required_hits)

        if confirmation_progress is not None:
            logging.info("Obstacle-2 arrow confirmation %s: %d/%d matching reports.",
                         *confirmation_progress)
        self.image_update_event.set()
        self._try_confirm_initial_reverse()
        self._try_queue_turn2()

        if self.on_arrow_callback and img in self.directions:
            self.on_arrow_callback(img)

    def _try_start_route(self) -> None:
        with self.lock:
            if self.route_faulted or not self.route_start_requested or self.begin_stage != 0:
                return
            if not self.smooth_firmware_ready:
                return

            image = self.last_image
            confidence = self.last_image_confidence
            timestamp_ns = self.last_image_timestamp_ns
            age_s = (
                (time_ns() - timestamp_ns) / 1e9
                if timestamp_ns is not None
                else float("inf")
            )
            if (
                image not in self.directions
                or confidence is None
                or confidence < self.arrow_confidence_threshold
                or age_s > self.arrow_max_age_s
            ):
                return

            self.obstacle1_arrow_id = image
            self.obstacle1_direction = self.directions[image]
            self.route_start_requested = False
            self.begin_stage = 2
            self.obstacle1_script_queued = True
            self.slide_clearance_travel_cm = 0.0
            self.obstacle2_alignment_ready = False
            self.obstacle2_alignment_timestamp_ns = None
            self.obstacle2_approach_complete = False

        logging.info(
            "Starting route with obstacle 1 arrow %s (confidence %.3f, age %.2fs).",
            self.directions[image],
            confidence,
            age_s,
        )
        self.start_time = time_ns()
        first_pass = self.perform_turn1(self.directions[image], send=False)
        self.stm.send("FU40,END," + first_pass + "\n")

    def start(self):
        logging.info("Starting Task 2 SmoothFlow v50.4 calibrated (first-slide clearance, earlier parking corner, firmware50).")
        self.android.start()
        self.pc.connect()
        self.stm.connect()
        self._try_stm_handshake()
        self.stm_thread = Thread(target=self.stm_receive)
        self.stm_thread.start()

        self.process_pc_stream = Thread(target=self.stream_start)
        self.process_pc_stream.start()

        self.pc_thread = Thread(target=self.pc_receive)
        self.android_thread = Thread(target=self.android_receive)
        self.pc_thread.start()
        self.android_thread.start()

        self.stm_thread.join()
        self.pc_thread.join()
        self.android_thread.join()

        logging.info("Task 2 completed.")

if __name__ == "__main__":
    task1 = Task2()
    task1.start()
