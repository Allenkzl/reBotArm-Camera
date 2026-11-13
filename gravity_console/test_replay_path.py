"""Bounded path planning and continuous replay; no robot or real camera access."""
import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from gravity_console.replay_path import ReplayPath, CONTROL_HZ
from gravity_console.server import GravityConsole
from gravity_console.shooting_control import MotionCancelled


def action(angles, step=.07):
    return {"slot": 1, "label": "continuous", "samples": [
        {"t": 10+i*step, "q_rad": np.deg2rad([0, 0, a, 0, 0, 0]).tolist(),
         "effective_scale": [.8]*6} for i, a in enumerate(angles)]}


class ReplayPathTests(unittest.TestCase):
    def test_shape_preserving_continuous_velocity_and_bounded_acceleration(self):
        tr = action([1, 1.1, 3, 16, 29, 37, 36, 36.1, 20, 4, 1])
        original = copy.deepcopy(tr)
        path = ReplayPath(tr["samples"], .25, [5])
        self.assertEqual(tr, original)
        self.assertGreater(path.duration, (tr['samples'][-1]['t']-tr['samples'][0]['t'])/.25)
        for i, h in enumerate(path.durations):
            qs = np.array([path.position(i, u) for u in np.linspace(0, 1, 1001)])
            np.testing.assert_allclose(qs[0], path.q[i], atol=1e-12)
            np.testing.assert_allclose(qs[-1], path.q[i+1], atol=1e-12)
            self.assertTrue(np.all(qs >= np.minimum(path.q[i], path.q[i+1])-1e-12))
            self.assertTrue(np.all(qs <= np.maximum(path.q[i], path.q[i+1])+1e-12))
            velocity = np.diff(qs, axis=0)/(h/1000)
            acceleration = np.diff(velocity, axis=0)/(h/1000)
            self.assertLessEqual(np.max(np.abs(velocity)), .150001)
            self.assertLessEqual(np.max(np.abs(acceleration)), .300001)
            if i < len(path.durations)-1:
                right = (path.position(i+1, .00001)-path.q[i+1])/(path.durations[i+1]*.00001)
                left = (path.q[i+1]-path.position(i, .99999))/(h*.00001)
                np.testing.assert_allclose(right, left, atol=1e-5)
        np.testing.assert_array_equal(path.velocity[[0, 5, len(path.q)-1]], 0.)

    def test_stationary_pauses_still_have_control_ticks_and_no_overshoot(self):
        tr = action([100]*8, step=.1)
        path = ReplayPath(tr['samples'], .25, [3])
        self.assertGreaterEqual(path.duration, 2.8-1e-10)
        for i in range(7):
            for u in (0, .1, .5, .9, 1):
                np.testing.assert_allclose(path.position(i, u), path.q[0], atol=1e-12)


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def wait(self, duration):
        self.now += max(0, duration)
        return False


class ContinuousReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.app = GravityConsole(robot_root=root, log_dir=root, dry_run=True,
                                  max_percent=200, preset_file=None, trajectory_file=None)
        self.app._set_status(mode='holding', enabled=True)
        self.clock = Clock()

    def tearDown(self):
        self.app.close()
        self.temp.cleanup()

    def replay(self, tr, send, capture=None, feedback=None):
        with patch('gravity_console.shooting_control.time', SimpleNamespace(monotonic=self.clock.monotonic)), \
             patch('gravity_console.photo_points.time', SimpleNamespace(monotonic=self.clock.monotonic)), \
             patch.object(self.app.stop_requested, 'wait', side_effect=self.clock.wait), \
             patch.object(self.app, '_transition'), \
             patch.object(self.app, '_position_command', side_effect=send), \
             patch.object(self.app, '_tracking_check', side_effect=feedback or (lambda q: q.copy())), \
             patch.object(self.app, '_perform_photo_point', side_effect=capture):
            return self.app._play_trajectory_block(tr, .25)

    def test_framing_check_holds_first_photo_target_without_shutter_or_tail(self):
        tr = action([1, 1, 3, 16, 29, 36, 20, 1])
        tr["photo_points"] = [
            {"sample_index": i, "q_rad": tr["samples"][i]["q_rad"], "id": str(i)}
            for i in (4, 7)
        ]
        tr["_framing_check"] = True
        sent = []
        with patch.object(self.app, "_write_log") as log, \
             patch.object(self.app.camera, "start_shutter") as shutter:
            target = self.replay(tr, lambda q, scale: sent.append(q.copy()),
                                 capture=AssertionError("photo callback reached"))
        shutter.assert_not_called()
        np.testing.assert_allclose(target, tr["samples"][4]["q_rad"])
        np.testing.assert_allclose(sent[-1], target)
        held = [call.args[0] for call in log.call_args_list if call.args[0]["kind"] == "photo_hold_sample"]
        self.assertGreaterEqual(len(held), 175)
        self.assertTrue(all(row["sample_index"] == 4 for row in held))
        self.assertTrue(all(row["kind"] != "replay_sample" or row["i"] <= 4
                            for row in (call.args[0] for call in log.call_args_list)))

    def test_regular_ticks_and_retimed_motion_land_on_each_photo_once(self):
        tr = action([1, 1, 3, 16, 29, 36, 20, 1])
        tr['photo_points'] = [{'sample_index': i, 'q_rad': tr['samples'][i]['q_rad'],
                               'id': str(i), 't': tr['samples'][i]['t']} for i in (0, 4, 7)]
        sent, photos, returns = [], [], []
        def send(q, scale):
            sent.append((self.clock.now, q.copy()))
        def capture(trajectory, point, number, scale):
            np.testing.assert_allclose(sent[-1][1], point['q_rad'], atol=1e-12)
            photos.append((point['sample_index'], number))
            self.clock.now += 5.
            returns.append(self.clock.now)
        last = self.replay(tr, send, capture)
        self.assertEqual(photos, [(0, 1), (4, 2), (7, 3)])
        np.testing.assert_array_equal(last, tr['samples'][-1]['q_rad'])
        self.assertGreater(len(sent), len(tr['samples'])*4)
        intervals = np.diff([t for t, q in sent])
        self.assertGreaterEqual(intervals.min(), 1/CONTROL_HZ-1e-10)
        for resumed in returns[:-1]:
            following = next(t for t, q in sent if t > resumed)
            self.assertGreaterEqual(following-resumed, 1/CONTROL_HZ-1e-10)
        for (t0,q0),(t1,q1) in zip(sent,sent[1:]):
            self.assertLessEqual(np.max(np.abs((q1-q0)/(t1-t0))), .150001)

    def test_slow_cycles_stretch_time_without_bursts_or_skipping_markers(self):
        tr = action([1, 1, 2, 3, 4, 3, 2, 1])
        sent = []
        def send(q, scale):
            sent.append((self.clock.now, q.copy()))
            self.clock.now += .08  # simulated slow feedback, no real sleep
        self.replay(tr, send)
        self.assertGreaterEqual(np.diff([t for t, q in sent]).min(), .08-1e-10)
        np.testing.assert_array_equal(sent[-1][1], tr['samples'][-1]['q_rad'])

    def test_framing_corrections_log_baseline_and_change_only_selected_joint(self):
        for joint, offset in ((1, -.5), (2, .5)):
            with self.subTest(joint=joint):
                self._check_framing_correction(joint, offset)

    def test_framing_sweep_skips_larger_step_after_motion_and_requires_return(self):
        for sticks in (False, True):
            with self.subTest(sticks=sticks):
                tr = action([1, 1, 3, 16, 29, 36, 20, 1])
                point = tr["samples"][4]
                target = np.array(point["q_rad"], dtype=float)
                tr.update(_framing_check=True, _framing_sweep=True,
                          photo_points=[{"sample_index": 4, "q_rad": point["q_rad"], "id": "first"}])
                moved = False

                def feedback(q):
                    nonlocal moved
                    moved |= q[2] - target[2] > np.deg2rad(.5)
                    if sticks and moved:
                        q = q.copy()
                        q[2] = max(q[2], target[2] + np.deg2rad(.4))
                    return q.copy()

                with patch.object(self.app, "_write_log") as log:
                    if sticks:
                        with self.assertRaisesRegex(RuntimeError, "未恢复基线"):
                            self.replay(tr, lambda q, scale: None, feedback=feedback)
                    else:
                        result = self.replay(tr, lambda q, scale: None, feedback=feedback)
                        np.testing.assert_allclose(result, target)
                phases = list(dict.fromkeys(c.args[0]["phase"] for c in log.call_args_list
                                            if c.args[0]["kind"] == "photo_hold_sample"))
                expected = ["framing_check", "framing_j3_step_0.75", "framing_j3_return"]
                if not sticks:
                    expected += ["framing_j2_step_0.75", "framing_j2_return"]
                self.assertEqual(phases, expected)

    def _check_framing_correction(self, joint, offset):
        tr = action([1, 1, 3, 16, 29, 36, 20, 1])
        point = tr["samples"][4]
        tr.update(_framing_check=True, _framing_correction=(joint, offset),
                  photo_points=[{"sample_index": 4, "q_rad": point["q_rad"], "id": "first"}])
        with patch.object(self.app, "_write_log") as log:
            result = self.replay(tr, lambda q, scale: None,
                                 capture=AssertionError("photo callback reached"))
        expected = np.array(point["q_rad"], dtype=float)
        expected[joint] += np.deg2rad(offset)
        np.testing.assert_allclose(result, expected)
        held = [c.args[0] for c in log.call_args_list if c.args[0]["kind"] == "photo_hold_sample"]
        self.assertEqual(list(dict.fromkeys(row["phase"] for row in held)),
                         ["framing_check", f"framing_j{joint+1}_ramp", f"framing_j{joint+1}_hold"])
        targets = np.array([row["q_target_deg"] for row in held])
        span = np.zeros(6)
        span[joint] = abs(offset)
        np.testing.assert_allclose(np.ptp(targets, axis=0), span)

    def test_cancel_inside_interpolation_prevents_photo_and_later_commands(self):
        tr = action([1, 1, 2, 3, 4, 3, 2, 1])
        tr['photo_points'] = [{'sample_index': 5, 'q_rad': tr['samples'][5]['q_rad']}]
        sent, photos = [], []
        def send(q, scale):
            sent.append(q.copy())
            if len(sent) == 4:
                self.app.stop_requested.set()
        with self.assertRaises(MotionCancelled):
            self.replay(tr, send, lambda *a: photos.append(a))
        self.assertEqual(len(sent), 4)
        self.assertEqual(photos, [])

    def test_tracking_fault_logs_joint_and_keeps_20_degree_threshold(self):
        self.app.dry_run = False
        q = np.zeros(6)
        target = q.copy(); target[2] = np.deg2rad(21)
        with patch.object(self.app, '_read_positions', return_value=q), \
             patch.object(self.app, '_write_log') as log:
            with self.assertRaisesRegex(RuntimeError, 'J3.*21.0.*20'):
                self.app._tracking_check(target)
        event = log.call_args.args[0]
        self.assertEqual(event['kind'], 'tracking_error')
        self.assertEqual(event['joint'], 'J3')
        self.assertAlmostEqual(event['error_deg'][2], 21)

    def test_long_control_stall_aborts_before_next_command(self):
        self.app.dry_run = False
        tr = action([1]*8)
        sent = []
        def send(q, scale):
            sent.append(q.copy())
            self.clock.now += .3
        with self.assertRaisesRegex(RuntimeError, '250毫秒'):
            self.replay(tr, send)
        self.assertEqual(len(sent), 1)

    def test_feedforward_uses_measured_pose_and_retains_position_target(self):
        measured, target = np.ones(6)*.1, np.ones(6)*.2
        self.app._replay_feedback_q = measured
        with patch.object(self.app, '_estimate_payload', return_value=(np.ones(6), np.ones(6))) as estimate, \
             patch.object(self.app, '_send_mit') as send:
            self.app._position_command(target, np.ones(6)*.8)
        np.testing.assert_array_equal(estimate.call_args.args[0], measured)
        np.testing.assert_array_equal(send.call_args.args[0], target)


if __name__ == '__main__':
    unittest.main()
