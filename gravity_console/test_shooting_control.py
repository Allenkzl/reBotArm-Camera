"""Motion lifecycle regressions; all instances are dry-run and use temp files."""
import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np

from gravity_console.server import GravityConsole
from gravity_console.shooting_control import MotionCancelled


def trajectory(slot, duration=.021):
    return {"slot": slot, "label": f"Action {slot}", "samples": [
        {"t": duration*i/7, "q_rad": [0.] * 6, "q_deg": [0.] * 6,
         "effective_scale": [.8] * 6} for i in range(8)]}


class ShootingControlTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        self.app = GravityConsole(robot_root=root, log_dir=root, dry_run=True,
                                  max_percent=200, preset_file=None, trajectory_file=None)

    def tearDown(self):
        self.app.emergency_requested.set()
        self.app.stop_requested.set()
        if self.app.thread and self.app.thread.is_alive():
            self.app.thread.join(2)
        self.app.close()
        if self.app._log_file:
            self.app._log_file.close()
        self.directory.cleanup()

    def ready(self):
        self.app._set_status(mode="holding", enabled=True, connected=True)

    def group(self, count=3, interval=0):
        group = self.app._groups.groups[0]
        while len(group["blocks"]) > count:
            self.app._groups.delete_block(group["id"], group["blocks"][-1]["id"])
            group = self.app._groups.groups[0]
        for i, block in enumerate(group["blocks"], 1):
            self.app._groups.assign_slot(group["id"], block["id"], i)
        self.app._groups.update_group(group["id"], interval_s=interval)
        self.app._trajectories = [trajectory(i) for i in range(1, count+1)]
        return copy.deepcopy(self.app._groups.groups[0])

    def join(self):
        self.app.thread.join(2)
        self.assertFalse(self.app.thread.is_alive(), "dry-run worker did not finish")

    def test_record_and_replay_require_explicit_start(self):
        group = self.group()
        block = group["blocks"][0]
        actions = [lambda: self.app.start_recording({"slot": 1}),
                   lambda: self.app.record_block({"group_id": group["id"], "block_id": block["id"]}),
                   lambda: self.app.replay_trajectory({"slot": 1}),
                   lambda: self.app.play_group({"group_id": group["id"]})]
        with patch.object(self.app, "_ensure_robot") as hardware:
            for action in actions:
                with self.assertRaisesRegex(RuntimeError, "启动"):
                    action()
            hardware.assert_not_called()
        self.assertIsNone(self.app.thread)

    def test_entire_group_preflight_before_stopping_or_sending(self):
        self.ready()
        group = self.group()
        self.app._trajectories[-1]["samples"][-1]["q_rad"][1] = np.deg2rad(101)
        with patch.object(self.app, "_join_worker") as stop, patch.object(self.app, "_send_mit") as send:
            with self.assertRaisesRegex(RuntimeError, "soft limits"):
                self.app.play_group({"group_id": group["id"]})
            stop.assert_not_called()
            send.assert_not_called()
        self.assertIsNone(self.app.thread)

    def test_sparse_group_plays_saved_blocks_in_original_order(self):
        self.ready()
        group = self.group(count=4, interval=0)
        self.app._trajectories = [trajectory(1), trajectory(3)]
        plan = []
        with patch.object(self.app, "_read_positions", return_value=np.zeros(6)), \
             patch.object(self.app, "_play_trajectory_block", side_effect=lambda tr, speed: (plan.append((tr["slot"],self.app.status.group_playback["block_id"])) or np.zeros(6))):
            self.app.play_group({"group_id": group["id"]})
            self.join()
        self.assertEqual(plan, [(1,group["blocks"][0]["id"]),(3,group["blocks"][2]["id"])])
        self.assertEqual(self.app.status.mode, "holding")

    def test_all_empty_group_rejects_without_starting_worker(self):
        self.ready()
        group = self.app._groups.groups[0]
        with patch.object(self.app, "_join_worker") as join:
            with self.assertRaisesRegex(ValueError,"暂无已保存"):
                self.app.play_group({"group_id":group["id"]})
            join.assert_not_called()
        self.assertIsNone(self.app.thread)

    def test_failed_recording_reports_joint_range_and_clears_on_success(self):
        self.ready()
        self.app.recording = {"slot":1,"label":"测试动作","run_id":"test","samples":trajectory(1)["samples"]}
        self.app.recording["samples"][2]["q_rad"][5] = np.deg2rad(-107)
        with self.assertRaisesRegex(RuntimeError,"J6"):
            self.app.stop_recording({"slot":1})
        self.assertIn("允许 -70°",self.app.status.recording_errors["1"])
        self.assertIsNone(self.app.status.recording_slot)
        self.app.recording = {"slot":1,"label":"测试动作","run_id":"test","samples":trajectory(1)["samples"]}
        self.app.stop_recording({"slot":1})
        self.assertNotIn("1",self.app.status.recording_errors)

    def test_real_short_dryrun_order_and_intervals_only_between(self):
        self.ready()
        group = self.group(interval=.01)
        order, waits = [], []
        play, wait = self.app._play_trajectory_block, self.app._wait_between_blocks
        def observe_play(tr, speed):
            order.append(tr["slot"])
            return play(tr, speed)
        def observe_wait(target, seconds):
            waits.append((order[-1], seconds))
            return wait(target, seconds)
        with patch.object(self.app, "_read_positions", return_value=np.zeros(6)), \
             patch.object(self.app, "_play_trajectory_block", side_effect=observe_play), \
             patch.object(self.app, "_wait_between_blocks", side_effect=observe_wait):
            self.app.play_group({"group_id": group["id"], "speed": .25})
            self.join()
        self.assertEqual(order, [1, 2, 3])
        self.assertEqual(waits, [(1, .01), (2, .01)])
        self.assertEqual(self.app.status.mode, "holding")
        self.assertTrue(self.app.status.enabled)
        self.assertIsNone(self.app.status.group_playback)
        events = [json.loads(line) for line in Path(self.app.status.log_path).read_text().splitlines()]
        self.assertEqual([e["slot"] for e in events if e.get("kind") == "block_complete"], [1, 2, 3])

    def test_stop_during_real_wait_never_starts_next_block(self):
        self.ready()
        group = self.group(interval=5)
        played = []
        def play(tr, speed):
            played.append(tr["slot"])
            return np.zeros(6)
        with patch.object(self.app, "_play_trajectory_block", side_effect=play), \
             patch.object(self.app, "_read_positions", return_value=np.zeros(6)):
            self.app.play_group({"group_id": group["id"]})
            deadline = time.monotonic() + 1
            while self.app.status.mode != "group_waiting" and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(self.app.status.mode, "group_waiting")
            self.app.stop_hold()
            self.join()
        self.assertEqual(played, [1])
        self.assertEqual(self.app.status.mode, "holding")

    def test_second_block_failure_aborts_third(self):
        self.ready()
        group = self.group()
        played = []
        def play(tr, speed):
            played.append(tr["slot"])
            if tr["slot"] == 2:
                raise RuntimeError("injected tracking failure")
            return np.zeros(6)
        with patch.object(self.app, "_play_trajectory_block", side_effect=play):
            self.app.play_group({"group_id": group["id"]})
            self.join()
        self.assertEqual(played, [1, 2])
        self.assertEqual(self.app.status.mode, "error")
        self.assertIn("tracking failure", self.app.status.message)

    def test_invalid_angles_times_and_scales_rejected(self):
        mutations = [lambda s: s.update(q_rad=[float("nan")]*6),
                     lambda s: s.update(q_rad=[0]*5),
                     lambda s: s.update(q_rad=[[0]*6]),
                     lambda s: s.update(t=float("nan")),
                     lambda s: s.update(t=-1),
                     lambda s: s.update(t=0),
                     lambda s: s.update(effective_scale=[float("inf")]*6)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                tr = trajectory(1)
                mutate(tr["samples"][1])
                with self.assertRaises((ValueError, RuntimeError)):
                    self.app._validate_trajectory(tr)

    def test_shutdown_when_disabled_is_noop(self):
        with patch.object(self.app, "_join_worker") as join, \
             patch.object(self.app, "_home_and_disable") as home:
            self.app.home_disable()
            join.assert_not_called()
            home.assert_not_called()
        self.assertIsNone(self.app.thread)

    def test_emergency_cancels_home_and_forbids_future_commands(self):
        self.ready()
        started = threading.Event()
        original_send = self.app._send_mit
        def observed_send(*args, **kwargs):
            result = original_send(*args, **kwargs)
            started.set()
            return result
        with patch.object(self.app, "_read_positions", return_value=np.zeros(6)), \
             patch.object(self.app, "_send_mit", side_effect=observed_send) as send:
            self.app.home_disable()
            self.assertTrue(started.wait(1))
            self.app.emergency_disable()
            self.join()
            sent_after_disable = send.call_count
            self.app._home_and_disable()
            self.assertEqual(send.call_count, sent_after_disable)
            self.assertFalse(self.app.status.enabled)
            self.assertEqual(self.app.status.mode, "disabled")
            with self.assertRaises(MotionCancelled):
                original_send(np.zeros(6), np.zeros(6))

    def test_invalid_recording_preserves_previous_saved_action(self):
        old = trajectory(1)
        self.app._trajectories = [copy.deepcopy(old)]
        self.app._save_trajectories_file()
        previous = self.app.trajectory_file.read_bytes()
        invalid = trajectory(1)
        invalid["samples"][-1]["q_rad"][1] = np.deg2rad(101)
        self.app.recording = {"slot": 1, "label": "replacement", "samples": invalid["samples"]}
        with self.assertRaisesRegex(RuntimeError, "soft limits"):
            self.app.stop_recording({"slot": 1})
        self.assertEqual(self.app.trajectory_file.read_bytes(), previous)
        self.assertEqual(self.app._trajectories, [old])
        self.assertIsNone(self.app.recording)
        self.assertIsNone(self.app.status.recording_slot)

    def test_stop_during_delayed_connect_never_enables_or_sends(self):
        entered, release = threading.Event(), threading.Event()
        def connect():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test connect gate timed out")
        arm = SimpleNamespace(mode_mit=Mock(), enable=Mock())
        robot = SimpleNamespace(connect=connect, arm=arm)
        self.app.dry_run = False
        with patch.object(self.app, "_ensure_robot", return_value=robot), \
             patch.object(self.app, "_shutdown_robot") as shutdown, \
             patch.object(self.app, "_send_mit") as send:
            self.app.start({"camera_profile": "heavy"})
            try:
                self.assertTrue(entered.wait(1))
                self.app.stop_requested.set()
            finally:
                release.set()
            self.join()
            arm.mode_mit.assert_not_called()
            arm.enable.assert_not_called()
            send.assert_not_called()
            shutdown.assert_called_once_with(robot)
        self.assertIsNone(self.app.robot)
        self.assertFalse(self.app.status.enabled)
        self.assertEqual(self.app.status.mode, "disabled")

    def test_corrupt_legacy_file_never_creates_migration_or_overwrites(self):
        root = Path(self.directory.name)
        legacy, groups = root / "corrupt_trajectories.json", root / "new_groups.json"
        legacy.write_text('{"trajectories": [broken')
        original = legacy.read_bytes()
        with self.assertRaises(ValueError):
            GravityConsole(robot_root=root, log_dir=root, dry_run=True,
                           max_percent=200, preset_file=None,
                           trajectory_file=legacy, group_file=groups)
        self.assertEqual(legacy.read_bytes(), original)
        self.assertFalse(groups.exists())

    def test_recording_save_failure_clears_recording_and_preserves_old_data(self):
        old = trajectory(1)
        self.app._trajectories = [copy.deepcopy(old)]
        self.app._save_trajectories_file()
        previous = self.app.trajectory_file.read_bytes()
        replacement = trajectory(1)
        self.app.recording = {"slot": 1, "label": "replacement", "samples": replacement["samples"]}
        self.app._set_status(recording_slot=1, recording_samples=8)
        with patch("gravity_console.server.atomic_write_json", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.app.stop_recording({"slot": 1})
        self.assertIsNone(self.app.recording)
        self.assertIsNone(self.app.status.recording_slot)
        self.assertEqual(self.app.status.recording_samples, 0)
        self.assertEqual(self.app._trajectories, [old])
        self.assertEqual(self.app.trajectory_file.read_bytes(), previous)

    def test_persistent_nan_feedback_aborts_without_invalid_hold_commands(self):
        self.ready()
        group = self.group(count=1)
        with patch.object(self.app, "_read_positions", return_value=np.full(6, np.nan)), \
             patch.object(self.app, "_send_mit") as send:
            self.app.play_group({"group_id": group["id"]})
            self.join()
            send.assert_not_called()
        self.assertEqual(self.app.status.mode, "error")
        self.assertTrue(self.app.status.enabled)
        self.assertIsNone(self.app.status.group_playback)

    def test_group_recording_after_explicit_start_saves_samples(self):
        group = self.app._groups.groups[0]
        request = {"group_id": group["id"], "block_id": group["blocks"][0]["id"]}
        self.app.start({"camera_profile": "heavy"})
        deadline = time.monotonic() + 1
        while self.app.status.mode == "starting" and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(self.app.status.mode, "running")
        self.app.record_block(request)
        deadline = time.monotonic() + 2
        while len(self.app.recording["samples"]) < 8 and time.monotonic() < deadline:
            time.sleep(.01)
        self.app.stop_block_recording(request)
        self.app.stop_hold()
        block = self.app._groups.resolve_block(request["group_id"], request["block_id"])
        saved = self.app._trajectory_for_slot(block["slot"])
        self.assertGreaterEqual(len(saved["samples"]), 8)
        self.assertEqual(saved["camera_profile"], "heavy")
        persisted = json.loads(self.app.trajectory_file.read_text())
        self.assertEqual(persisted["trajectories"][0]["slot"], block["slot"])


if __name__ == "__main__":
    unittest.main()
