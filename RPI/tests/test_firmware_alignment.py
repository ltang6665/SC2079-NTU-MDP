import ast
import contextlib
import io
import inspect
import json
import logging
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from threading import Event, Lock, RLock, Timer
from time import monotonic, sleep, time, time_ns
from typing import Optional
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "RPI"))
from firmware_protocol import TASK2_READY


class Endpoint:
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)


class ImmediateThread:
    def __init__(self, target, args=(), **kwargs):
        self.target = target
        self.args = args

    def start(self):
        self.target(*self.args)


def load_class(path, name, **extra):
    tree = ast.parse((ROOT / path).read_text())
    definition = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == name)
    namespace = dict(
        os=os, json=json, math=math, logging=logging, Event=Event, Lock=Lock,
        Timer=Timer, Thread=ImmediateThread, time=time, time_ns=time_ns,
        monotonic=monotonic, sleep=sleep, Optional=Optional,
        Android=Endpoint, PC=Endpoint, STM=Endpoint,
        TASK2_READY=TASK2_READY,
        PI_IP="127.0.0.1", PORT=5000, Path=Path, __file__=str(ROOT / path),
    )
    namespace.update(extra)
    exec(compile(ast.Module(body=[definition], type_ignores=[]), path, "exec"), namespace)
    return namespace[name]






class EndMessages(BaseException):
    pass


class Task1ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.task = load_class("RPI/task1.py", "Task1")()
        self.task.segments = [["F10"], ["FR90", "F20"], ["S"]]
        self.task.obstacle_order = [1, 2]
        self.task.segment_delay = 0
        self.scans = []
        self.task._request_detect_with_retry = lambda obstacle: self.scans.append(obstacle) or True

    def receive_one(self, message):
        self.task.stm.receive = mock.Mock(side_effect=[message, EndMessages()])
        with self.assertRaises(EndMessages):
            self.task.stm_receive()

    def test_segments_follow_the_archived_ok_scan_next_flow(self):
        self.assertTrue(self.task._send_next_segment())
        self.assertEqual(self.task.stm.messages, ["F10\n"])
        self.receive_one("done\n")
        self.assertEqual(self.scans, [])
        self.receive_one("OK\n")
        self.assertEqual(self.scans, ["1"])
        self.assertEqual(self.task.stm.messages[-1], "FR90,F20\n")
        self.receive_one("OK\n")
        self.assertEqual(self.scans, ["1", "2"])
        self.assertEqual(self.task.stm.messages[-1], "S\n")
        self.receive_one("OK\n")
        self.assertEqual(self.task.pc.messages, ["STITCH,2\n"])

    def test_out_of_range_segment_is_not_sent(self):
        self.task.segments_index = len(self.task.segments)
        self.assertFalse(self.task._send_next_segment())
        self.assertEqual(self.task.stm.messages, [])

    def test_android_calculate_and_legacy_requests_are_supported(self):
        for message in ("CALCULATE", "PATH", "ALG|1,2,3"):
            self.assertTrue(self.task._is_calculation_request(message))
        self.assertFalse(self.task._is_calculation_request("BEGIN"))

    def test_position_update_preserves_facing_and_unset_facing_is_skipped(self):
        obstacle = self.task._parse_tablet_obstacle(["OBSTACLE", "1", "7", "8"])
        self.assertEqual(obstacle, {"id": 1, "x": 7, "y": 8, "d": 8})
        self.task.obstacles = [{"id": 1, "x": 1, "y": 1, "d": 2}]
        self.assertEqual(self.task._parse_tablet_obstacle(["OBSTACLE", "1", "7", "8"])["d"], 2)
        self.assertEqual(self.task._parse_tablet_obstacle(["OBSTACLE", "1", "7", "8", "WEST"])["d"], 6)

    def test_android_is_notified_when_the_path_is_ready(self):
        payload = json.dumps({"segments": self.task.segments, "obstacle_ids": [1, 2], "dirs": []})
        self.task.pc.receive = mock.Mock(side_effect=["PATH," + payload + "\n", EndMessages()])
        with self.assertRaises(EndMessages):
            self.task.pc_receive()
        self.assertTrue(self.task.path_ready.is_set())
        self.assertEqual(self.task.android.messages, ["PATHREADY"])

    def test_both_pc_tasks_select_the_new_packaged_model(self):
        for filename, name in (("PC/Task1.py", "Task1"), ("PC/Task2.py", "Task2"), ("PC/Checklist.py", "Checklist")):
            task = load_class(filename, name, threading=__import__("threading"))()
            self.assertEqual(Path(task.model), ROOT / "PC/bestv8n.pt")
            self.assertTrue(Path(task.model).is_file())


class Task2Tests(unittest.TestCase):
    def setUp(self):
        self.task = load_class("RPI/task2.py", "Task2")()

    def test_firmware_handshake_matches_submission(self):
        firmware = (ROOT / "stm_submission/Grp28_Task2/Core/Src/main.c").read_text()
        self.assertIn('"' + TASK2_READY + '\\n"', firmware)
        self.task._try_stm_handshake()
        self.task._try_stm_handshake()
        self.assertEqual(self.task.stm.messages, ["\nSMOOTH_REL?\n"])

    def test_start_requires_handshake_and_fresh_confident_arrow(self):
        self.task.route_start_requested = True
        self.task.last_image = "39"
        self.task.last_image_confidence = 0.95
        self.task.last_image_timestamp_ns = time_ns()
        self.task._try_start_route()
        self.assertEqual(self.task.stm.messages, [])
        self.task.smooth_firmware_ready = True
        self.task.last_image_timestamp_ns = time_ns() - 5_000_000_000
        self.task._try_start_route()
        self.assertEqual(self.task.stm.messages, [])
        self.task.last_image_timestamp_ns = time_ns()
        self.task._try_start_route()
        self.task._try_start_route()
        self.assertEqual(self.task.stm.messages, ["FU40,END,SL,SR,FU32\n"])

    def test_initial_reverse_requires_new_matching_arrow(self):
        self.task.obstacle1_arrow_id = "39"
        self.task.initial_reverse_min_seq = self.task.image_seq
        self.task.set_last_image("38", 0.99)
        self.assertEqual(self.task.stm.messages, [])
        self.task.set_last_image("39", 0.99)
        self.task.set_last_image("39", 0.99)
        self.assertEqual(self.task.stm.messages, ["CONFIRM\n"])

    def test_obstacle2_requires_alignment_and_separate_reports(self):
        self.task.num_obstacle = 2
        self.task._arm_obstacle2_vision()
        self.assertEqual(self.task.pc.messages, ["SEEN\n"])
        for _ in range(3):
            self.task.set_last_image("38", 0.99)
        self.assertEqual(self.task.stm.messages, [])
        self.task._update_obstacle2_alignment("FU_REF,2,0,1,0")
        for _ in range(2):
            self.task.set_last_image("38", 0.99)
        self.assertEqual(self.task.stm.messages, [])
        self.task.set_last_image("38", 0.99)
        self.assertEqual(self.task.stm.messages, ["FR90,FIL,FL180,FILO,FIL,FL90\n"])
        self.task.set_last_image("38", 0.99)
        self.assertEqual(len(self.task.stm.messages), 1)


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read_all(self):
        return self.chunks.pop(0) if self.chunks else b""

    def read(self, length):
        return self.read_all()


class FramingTests(unittest.TestCase):
    def test_stream_callbacks_preserve_task1_and_task2_signatures(self):
        listener_class = load_class("PC/stream_listener.py", "StreamListener", inspect=inspect)
        listener = listener_class.__new__(listener_class)
        received = []
        listener._invoke_on_result(lambda a, b: received.append((a, b)), 1, 2, 3, 4)
        listener._invoke_on_result(lambda a, b, c: received.append((a, b, c)), 1, 2, 3, 4)
        listener._invoke_on_result(lambda a, b, c, d: received.append((a, b, c, d)), 1, 2, 3, 4)
        self.assertEqual(received, [(1, 2), (1, 2, 3), (1, 2, 3, 4)])

    def test_serial_retains_fragments_and_multiple_replies(self):
        link = load_class("RPI/communications/stm.py", "STM")()
        link.serial = FakeSerial([b"O", b"K\ndone\nir12.3\n"])
        self.assertIsNone(link.receive())
        self.assertEqual(link.receive(), "OK\n")
        self.assertEqual(link.receive(), "done\n")
        self.assertEqual(link.receive(), "ir12.3\n")
        self.assertIsNone(link.wait_receive(timeout_s=0))

    def test_serial_wait_returns_one_complete_message(self):
        link = load_class("RPI/communications/stm.py", "STM")()
        link.serial = FakeSerial([b"SMOOTH_REL_", b"READY,50\nOK\n"])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(link.wait_receive(timeout_s=0.5), TASK2_READY + "\n")
            self.assertEqual(link.wait_receive(timeout_s=0.5), "OK\n")

    def test_pc_control_messages_survive_tcp_fragmentation(self):
        task = load_class("PC/Task2.py", "Task2", threading=__import__("threading"))()
        task.sock = mock.Mock()
        task.sock.recv.side_effect = [b"SE", b"EN\nSTI", b"TCH\n", b""]
        task.connect = lambda: None
        received = []
        task._handle_control_message = received.append
        task.pc_receive()
        self.assertEqual(received, ["SEEN", "STITCH"])

    def test_pc_ignores_results_from_previous_obstacle_stage(self):
        task = load_class("PC/Task2.py", "Task2", threading=__import__("threading"))()
        task.obstacle_id = 2
        task.sock = mock.Mock()
        task.on_result(mock.Mock(), None, frame_obstacle_id=1)
        task.sock.sendall.assert_not_called()
        self.assertEqual(task.consecutive_hits, 0)


if __name__ == "__main__":
    unittest.main()
