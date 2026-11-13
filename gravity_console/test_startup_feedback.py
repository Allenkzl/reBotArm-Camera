"""Startup feedback checks using fake motors only; no hardware or sockets."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from gravity_console.server import GravityConsole
from gravity_console.shooting_control import MotionCancelled


class StartupFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.app = GravityConsole(robot_root=Path(self.temp.name), log_dir=Path(self.temp.name),
                                  dry_run=True, max_percent=200, preset_file=None, trajectory_file=None)
        motors = {f'joint{i}': Mock() for i in range(1, 7)}
        for motor in motors.values():
            motor.robstride_get_param_f32.return_value = 0.0
        self.arm = SimpleNamespace(_mm=motors, mode_mit=Mock(), enable=Mock())
        self.app.robot = SimpleNamespace(arm=self.arm)
        self.records = []
        self.log = patch.object(self.app, '_write_log', side_effect=self.records.append)
        self.log.start()

    def tearDown(self):
        self.app.close()
        self.log.stop()
        self.temp.cleanup()

    def test_fresh_zero_is_valid_and_each_joint_is_queried_twice(self):
        np.testing.assert_array_equal(self.app._startup_feedback_preflight(), np.zeros(6))
        for motor in self.arm._mm.values():
            self.assertEqual(motor.robstride_get_param_f32.call_count, 2)
            motor.robstride_get_param_f32.assert_called_with(0x7019, timeout_ms=250)
        self.assertEqual(len(self.records), 2)
        self.arm.enable.assert_not_called()

    def test_no_reply_rejects_even_if_cached_pose_is_zero(self):
        self.arm._mm['joint3'].robstride_get_param_f32.side_effect = TimeoutError('no reply')
        self.app.dry_run = False
        with patch.object(self.app, '_shutdown_robot') as shutdown, \
             patch.object(self.app, '_send_mit') as send:
            self.app._run_loop(enable=True)
        self.arm.mode_mit.assert_not_called()
        self.arm.enable.assert_not_called()
        send.assert_not_called()
        shutdown.assert_called_once()
        self.assertFalse(self.app.status.enabled)
        self.assertFalse(self.app.status.connected)
        self.assertIsNone(self.app.robot)
        self.assertIn('J3', self.app.status.message)

    def test_nan_missing_joint_and_motion_are_rejected(self):
        for bad in ('nan', 'missing', 'moving'):
            with self.subTest(bad=bad):
                motor = self.arm._mm['joint6']
                motor.robstride_get_param_f32.side_effect = None
                motor.robstride_get_param_f32.return_value = 0.
                if bad == 'nan': motor.robstride_get_param_f32.return_value = float('nan')
                if bad == 'moving': motor.robstride_get_param_f32.side_effect = [0., .1]
                if bad == 'missing': del self.arm._mm['joint6']
                try:
                    with self.assertRaises(RuntimeError): self.app._startup_feedback_preflight()
                finally:
                    self.arm._mm['joint6'] = motor
        self.arm.enable.assert_not_called()

    def test_cancellation_during_read_does_not_query_next_joint(self):
        def reply(*args, **kwargs):
            self.app.stop_requested.set()
            return 0.
        self.arm._mm['joint1'].robstride_get_param_f32.side_effect = reply
        with self.assertRaises(MotionCancelled): self.app._startup_feedback_preflight()
        self.arm._mm['joint2'].robstride_get_param_f32.assert_not_called()

    def test_stop_during_model_initialization_never_changes_mode_or_enables(self):
        self.app.dry_run = False
        with patch.object(self.app, '_estimate_payload', side_effect=lambda q: self.app.stop_requested.set()), \
             patch.object(self.app, '_shutdown_robot'):
            self.app._run_loop(enable=True)
        self.arm.mode_mit.assert_not_called()
        self.arm.enable.assert_not_called()
        self.assertFalse(self.app.status.enabled)
        self.assertEqual(self.app.status.mode, 'disabled')

    def test_enable_failure_is_not_reported_as_disabled_or_closed(self):
        self.app.dry_run = False
        self.arm.enable.side_effect = RuntimeError('enable result uncertain')
        with patch.object(self.app, '_estimate_payload', return_value=(np.zeros(6), np.zeros(6))), \
             patch.object(self.app, '_shutdown_robot') as shutdown:
            self.app._run_loop(enable=True)
        shutdown.assert_not_called()
        self.assertTrue(self.app.status.enabled)
        self.assertEqual(self.app.status.mode, 'error')
