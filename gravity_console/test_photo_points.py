"""Photo-point regression coverage using simulation only; never opens a socket."""
import copy
import json
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

import numpy as np

from gravity_console.server import GravityConsole
from gravity_console.camera_bridge import Operation
from gravity_console.shooting_control import MotionCancelled


def trajectory(slot=1, step=.02):
    return {"slot": slot, "label": "Test", "samples": [
        {"t": i*step, "q_rad": [0.]*6, "q_deg": [0.]*6,
         "effective_scale": [.8]*6} for i in range(8)]}


def point(tr, index=2):
    sample = tr["samples"][index]
    return {"id": str(uuid.uuid4()), "sample_index": index,
            "t": sample["t"], "q_rad": list(sample["q_rad"])}


class PhotoPointsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.app = GravityConsole(robot_root=root, log_dir=root, dry_run=True,
                                  max_percent=200, preset_file=None, trajectory_file=None)
        self.app._set_status(mode="running", enabled=True)
        self.app.photo_stable_window_s = .035
        self.app.photo_settle_timeout_s = .15
        self.app.photo_result_timeout_s = .2
        self.app._photo_run_post_delay_s = 0
        self.app._photo_run_nonce = "test-run"
        self.feedback = patch.object(self.app, "_read_positions", return_value=np.zeros(6))
        self.feedback.start()

    def tearDown(self):
        self.app.close()
        self.feedback.stop()
        if self.app._log_file:
            self.app._log_file.close()
        self.temp.cleanup()

    def recording(self):
        group = self.app._groups.groups[0]
        block = group["blocks"][0]
        if block["slot"] is None:
            self.app._groups.assign_slot(group["id"], block["id"], 1)
        self.app.recording = {"slot": 1, "label": "Recording", "samples": [], "photo_points": []}
        return {"group_id": group["id"], "block_id": block["id"]}

    def sample(self, t):
        z = np.zeros(6)
        self.app._record_sample(t=t, q=z, tau_model=z, tau_payload=z, tau_total=z,
                               tau_cmd=z, joint_percent=[80]*6,
                               effective_scale=np.array([.8]*6), camera_profile="heavy")

    def queue_mark(self, request):
        result = {}
        def mark():
            try:
                result["status"] = self.app.mark_photo_point(request)
            except Exception as exc:
                result["error"] = exc
        thread = threading.Thread(target=mark)
        thread.start()
        deadline = time.monotonic()+.5
        while not self.app._pending_photo_marks and thread.is_alive() and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertTrue(self.app._pending_photo_marks)
        return thread, result

    def connected(self):
        operation = self.app.camera.command("connect")
        self.assertTrue(operation.done.wait(1))
        self.assertTrue(operation.snapshot()["ok"])

    def test_mark_captures_next_sample_and_never_fires(self):
        request = self.recording()
        self.sample(0)
        with patch.object(self.app.camera, "start_shutter") as shutter:
            worker, result = self.queue_mark(request)
            self.assertEqual(self.app.recording["photo_points"], [])
            # Pending marks force a fresh sample even inside normal decimation interval.
            self.sample(.033)
            worker.join(.5)
            self.assertNotIn("error", result)
            captured = self.app.recording["photo_points"][0]
            self.assertEqual(captured["sample_index"], 1)
            self.assertEqual(captured["t"], .033)
            self.assertEqual(captured["q_rad"], self.app.recording["samples"][1]["q_rad"])
            shutter.assert_not_called()

    def test_pending_mark_cannot_move_to_new_recording(self):
        request = self.recording()
        worker, result = self.queue_mark(request)
        self.app._cancel_photo_marks("original recording stopped")
        self.recording()
        self.sample(1)
        worker.join(.5)
        self.assertIn("error", result)
        self.assertEqual(self.app.recording["photo_points"], [])

    def test_mark_rejects_other_block_and_inactive_recording(self):
        request = self.recording()
        request["block_id"] = self.app._groups.groups[0]["blocks"][1]["id"]
        with self.assertRaises(RuntimeError):
            self.app.mark_photo_point(request)
        self.app.recording = None
        with self.assertRaises(RuntimeError):
            self.app.mark_photo_point(request)

    def test_points_save_reload_delete_and_legacy_compatibility(self):
        request = self.recording()
        for i in range(8):
            self.sample(i*.1)
        self.app.recording["photo_points"] = [point(self.app.recording)]
        ident = self.app.recording["photo_points"][0]["id"]
        self.app.stop_recording({"slot": 1})
        saved = json.loads(self.app.trajectory_file.read_text())["trajectories"][0]
        self.assertEqual(saved["photo_points"][0]["id"], ident)
        self.app._trajectories = []
        self.app._load_trajectories()
        self.assertEqual(self.app._trajectories[0]["photo_points"], saved["photo_points"])
        self.app.delete_photo_point({**request, "point_id": ident})
        self.assertEqual(json.loads(self.app.trajectory_file.read_text())["trajectories"][0]["photo_points"], [])
        self.app._validate_trajectory(trajectory())  # old file without photo_points

    def test_invalid_points_rejected_before_worker_stop(self):
        mutations = [lambda p: p.update(sample_index=True), lambda p: p.update(sample_index=8),
                     lambda p: p.update(sample_index=-1), lambda p: p.update(id="bad"),
                     lambda p: p.update(t=p["t"]+.1), lambda p: p.update(q_rad=[float("nan")]*6),
                     lambda p: p.update(q_rad=[0]*5), lambda p: p.update(q_rad=[.1]*6)]
        for mutate in mutations:
            tr = trajectory()
            p = point(tr)
            mutate(p)
            tr["photo_points"] = [p]
            with self.subTest(point=p), patch.object(self.app, "_join_worker") as join:
                with self.assertRaises((ValueError, RuntimeError)):
                    self.app._play_request([tr], .25)
                join.assert_not_called()
        tr = trajectory()
        p = point(tr)
        tr["photo_points"] = [p, copy.deepcopy(p)]
        with self.assertRaises(ValueError):
            self.app._validate_trajectory(tr)

    def test_camera_offline_preflight_before_stopping_but_legacy_works(self):
        tr = trajectory()
        tr["photo_points"] = [point(tr)]
        with patch.object(self.app, "_join_worker") as join:
            with self.assertRaisesRegex(RuntimeError, "相机"):
                self.app._play_request([tr], 1)
            join.assert_not_called()
        with patch.object(self.app.camera, "start_shutter") as shutter:
            self.app._play_request([trajectory(step=.001)], 1)
            self.app.thread.join(1)
            self.assertFalse(self.app.thread.is_alive())
            shutter.assert_not_called()

    def test_pause_adjusts_clock_and_two_points_execute_once_in_order(self):
        tr = trajectory(step=.025)
        tr["photo_points"] = [point(tr, 2), point(tr, 5)]
        commands, captures, returned = [], [], []
        def capture(traj, p, index, scale):
            captures.append((p["id"], index))
            time.sleep(.07)
            returned.append(time.monotonic())
        with patch.object(self.app, "_transition"), \
             patch.object(self.app, "_position_command", side_effect=lambda *a: commands.append(time.monotonic())), \
             patch.object(self.app, "_perform_photo_point", side_effect=capture):
            self.app._play_trajectory_block(tr, 1)
        self.assertEqual(captures, [(tr["photo_points"][0]["id"], 1), (tr["photo_points"][1]["id"], 2)])
        self.assertGreaterEqual(commands[3]-returned[0], .017)
        self.assertGreaterEqual(commands[6]-returned[1], .017)
        self.assertEqual(len(commands), 8)

    def test_real_simulated_shutter_once_with_distinct_point_keys(self):
        self.connected()
        tr = trajectory()
        tr["photo_points"] = [point(tr, 1), point(tr, 4)]
        original = self.app.camera.start_shutter
        with patch.object(self.app.camera, "start_shutter", wraps=original) as shutter:
            for i, p in enumerate(tr["photo_points"], 1):
                self.app._perform_photo_point(tr, p, i, np.array([.8]*6))
            self.assertEqual(shutter.call_count, 2)
            keys = [call.args[0] for call in shutter.call_args_list]
            self.assertEqual(len(set(keys)), 2)
            self.assertTrue(all(k.startswith("test-run:1:") for k in keys))

    def test_jitter_and_offtarget_timeout_never_trigger(self):
        tr = trajectory()
        tr["photo_points"] = [point(tr)]
        for mode in ("jitter", "offtarget"):
            counter = [0]
            def feedback(target):
                counter[0] += 1
                q = target.copy()
                q[0] += np.deg2rad(3 if mode == "offtarget" else (.25 if counter[0] % 2 else -.25))
                return q
            with self.subTest(mode=mode), patch.object(self.app, "_tracking_check", side_effect=feedback), \
                 patch.object(self.app.camera, "start_shutter") as shutter:
                with self.assertRaisesRegex(RuntimeError, "超时"):
                    self.app._perform_photo_point(tr, tr["photo_points"][0], 1, np.array([.8]*6))
                shutter.assert_not_called()

    def test_cancel_before_handoff_never_calls_camera(self):
        tr = trajectory()
        tr["photo_points"] = [point(tr)]
        with patch.object(self.app, "_wait_photo_stable", side_effect=lambda *args: self.app.stop_requested.set()), \
             patch.object(self.app.camera, "start_shutter") as shutter:
            with self.assertRaises(MotionCancelled):
                self.app._perform_photo_point(tr, tr["photo_points"][0], 1, np.array([.8]*6))
            shutter.assert_not_called()

    def test_async_camera_wait_keeps_commands_and_cancel_stops_next_point(self):
        tr = trajectory(step=.001)
        tr["photo_points"] = [point(tr, 0), point(tr, 5)]
        operation = Operation("shutter", "fake")
        with patch.object(self.app, "_wait_photo_stable"), patch.object(self.app, "_transition"), \
             patch.object(self.app.camera, "start_shutter", return_value=operation) as shutter, \
             patch.object(self.app, "_position_command", wraps=self.app._position_command) as send:
            errors = []
            def replay():
                try:
                    self.app._play_trajectory_block(tr, 1)
                except Exception as exc:
                    errors.append(exc)
            worker = threading.Thread(target=replay)
            worker.start()
            deadline = time.monotonic()+.5
            while send.call_count < 4 and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertGreaterEqual(send.call_count, 4)
            self.app.stop_requested.set()
            self.app._photo_run_cancel.set()
            worker.join(.5)
            self.assertFalse(worker.is_alive())
            self.assertIsInstance(errors[0], MotionCancelled)
            self.assertEqual(shutter.call_count, 1)
            self.assertTrue(operation._cancel.is_set())


    def test_request_cancel_wakes_pending_mark_and_fresh_sample_cannot_append(self):
        request = self.recording()
        worker, result = self.queue_mark(request)
        generation = self.app._cancel_generation
        self.app.request_cancel()
        worker.join(.2)
        self.assertFalse(worker.is_alive())
        self.assertIn("error", result)
        self.assertEqual(self.app._cancel_generation, generation+1)
        self.assertTrue(self.app._photo_run_cancel.is_set())
        self.sample(.1)
        self.assertEqual(self.app.recording["photo_points"], [])
        self.assertEqual(self.app._pending_photo_marks, [])

    def test_stale_command_generation_cannot_prepare_worker(self):
        old_generation = self.app._command_generation
        self.app.request_cancel()
        self.app._command_generation = old_generation
        with self.assertRaisesRegex(RuntimeError, "取消"):
            self.app._prepare_worker()
        self.assertTrue(self.app.stop_requested.is_set())
        self.assertIsNone(self.app.thread)

    def test_direct_emergency_cancels_photo_before_delayed_motor_disable(self):
        entered, release = threading.Event(), threading.Event()
        observed = []
        class FakeArm:
            def disable(inner_self):
                observed.append(self.app._photo_run_cancel.is_set())
                entered.set()
                release.wait(1)
        class FakeRobot:
            arm = FakeArm()
        self.app.robot = FakeRobot()
        errors = []
        def emergency():
            try:
                self.app.emergency_disable()
            except Exception as exc:
                errors.append(exc)
        with patch.object(self.app.camera, "cancel_photo") as cancel, \
             patch.object(self.app, "_shutdown_robot"):
            worker = threading.Thread(target=emergency)
            worker.start()
            try:
                self.assertTrue(entered.wait(.5))
                self.assertEqual(observed, [True])
                cancel.assert_called()
                self.assertTrue(self.app.emergency_requested.is_set())
            finally:
                release.set()
                worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertFalse(self.app.status.enabled)

    def test_new_http_generation_recording_after_stop_accepts_new_mark(self):
        self.app.request_cancel()
        self.app.stop_hold()
        self.app._command_generation = self.app._cancel_generation
        group = self.app._groups.groups[0]
        request = {"group_id": group["id"], "block_id": group["blocks"][0]["id"]}
        self.app.record_block(request)
        deadline = time.monotonic()+.5
        while self.app.status.mode == "starting" and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertEqual(self.app.status.mode, "running")
        self.assertEqual(self.app._worker_generation, self.app._cancel_generation)
        self.assertFalse(self.app.stop_requested.is_set())
        result = self.app.mark_photo_point(request)
        self.assertEqual(len(result["recording_photo_points"]), 1)
        self.assertEqual(len(self.app.recording["photo_points"]), 1)
        self.app.request_cancel()
        self.app.stop_hold()

if __name__ == "__main__":
    unittest.main()
