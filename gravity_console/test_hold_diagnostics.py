"""In-place diagnostic regression tests; fake feedback and no hardware."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from gravity_console.server import GravityConsole


class HoldDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.app = GravityConsole(robot_root=root, log_dir=root, dry_run=True,
                                  max_percent=200, preset_file=None, trajectory_file=None)
        self.app._set_status(mode='holding', enabled=True)
        self.records = []
        self.log = patch.object(self.app, '_write_log', side_effect=self.records.append)
        self.log.start()

    def tearDown(self):
        self.app.close()
        if self.app._log_file: self.app._log_file.close()
        self.log.stop()
        self.temp.cleanup()

    def test_explicit_enable_and_valid_duration_required_before_worker(self):
        with patch.object(self.app, '_join_worker') as join:
            for data in ({'duration_s': 0}, {'duration_s': 11}, {'duration_s': True},
                         {'duration_s': float('nan')}, {'target': [0]*6}, {'kp': [50]*6},
                         {'j3_probe_profile': .4}, {'j3_probe_profile': 'plus_040', 'j3_probe': True},
                         {'j3_probe_profile': 'plus_040', 'duration_s': 6}):
                with self.assertRaises(ValueError): self.app.diagnose_hold(data)
            join.assert_not_called()
            self.app._set_status(enabled=False)
            with self.assertRaisesRegex(RuntimeError, '启动'): self.app.diagnose_hold({})
            join.assert_not_called()

    def test_freezes_target_logs_offset_and_never_calls_camera_or_enable(self):
        target = np.deg2rad([0, 0, 22, 7, -1, 28])
        actual = target + np.deg2rad([0, 1, -2, 0, 0, 0])
        with patch.object(self.app, '_read_positions', return_value=target), \
             patch.object(self.app, '_tracking_check', return_value=actual), \
             patch.object(self.app, '_send_mit') as send, \
             patch.object(self.app.camera, 'start_shutter') as shutter, \
             patch.object(self.app, '_ensure_robot') as enable, \
             patch.object(self.app, '_transition') as transition:
            self.app._diagnose_hold_loop(.08, np.array([.8]*6))
        samples = [r for r in self.records if r['kind']=='hold_diagnostic_sample']
        self.assertGreaterEqual(len(samples), 2)
        for call in send.call_args_list: np.testing.assert_allclose(call.args[0], target)
        np.testing.assert_allclose(samples[-1]['error_actual_minus_target_deg'], [0,1,-2,0,0,0])
        self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_complete')
        self.assertEqual(self.app.status.mode, 'holding')
        self.assertTrue(self.app.status.enabled)
        shutter.assert_not_called(); enable.assert_not_called(); transition.assert_not_called()

    def test_cancellation_holds_and_never_completes(self):
        self.app.stop_requested.set()
        with patch.object(self.app, '_position_command') as send, \
             patch.object(self.app, '_hold_measured_position') as hold:
            self.app._diagnose_hold_loop(6, np.array([.8]*6))
        send.assert_not_called(); hold.assert_not_called()
        self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_cancelled')

    def test_j3_probe_changes_only_j3_feedforward_and_returns_to_baseline(self):
        self._check_j3_probe({'j3_probe': True}, 22., .20, 1.)

    def test_plus_040_probe_uses_slower_ramp_and_aborts_on_excursion(self):
        request = {'j3_probe_profile': 'plus_040'}
        self._check_j3_probe(request, 24., .40, 2.)
        self._check_j3_probe(request, 24., .40, 2., excursion=True)

    def _check_j3_probe(self, request, duration, peak, ramp, excursion=False):
        self.records.clear()
        self.app._set_status(mode='holding')
        target = np.deg2rad([0, 0, 22, 7, -1, 28])
        clock = [0.]

        def wait(seconds):
            clock[0] += max(seconds, .000001)

        def feedback(_):
            if excursion and clock[0] >= 13.:
                return target + np.deg2rad([0, 0, .8, 0, 0, 0])
            return target

        with patch('gravity_console.hold_diagnostics.threading.Thread') as worker:
            self.app.diagnose_hold(request)
            args = worker.call_args.kwargs['args']
            worker.return_value.start.assert_called_once()
        self.app.thread = None
        self.assertEqual((args[0], *args[2:]), (duration, True, peak, ramp))
        with patch.object(self.app, '_read_positions', return_value=target), \
             patch.object(self.app, '_tracking_check', side_effect=feedback), \
             patch.object(self.app, '_send_mit') as send, \
             patch.object(self.app, '_hold_measured_position') as hold, \
             patch.object(self.app.stop_requested, 'wait', side_effect=wait), \
             patch('gravity_console.hold_diagnostics.time.monotonic', side_effect=lambda: clock[0]):
            self.app._diagnose_hold_loop(*args)
        samples = [r for r in self.records if r['kind'] == 'hold_diagnostic_sample']
        self.assertEqual(max(r['j3_probe_nm'] for r in samples), peak)
        if excursion:
            self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_error')
            self.assertIn('位移', self.records[-1]['error'])
            hold.assert_called_once()
        else:
            self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_complete')
            self.assertEqual(samples[-1]['j3_probe_nm'], 0.)
            hold.assert_not_called()
        original = send.call_args_list[0].args[1]
        for call, sample in zip(send.call_args_list, samples):
            np.testing.assert_allclose(call.args[0], target)
            np.testing.assert_allclose(call.args[1] - original, [0, 0, sample['j3_probe_nm'], 0, 0, 0])
        for before, after in zip(samples, samples[1:]):
            self.assertLessEqual(abs(after['j3_probe_nm'] - before['j3_probe_nm']),
                                 .20 * (after['elapsed_s'] - before['elapsed_s']) + 1e-9)

    def test_excess_error_aborts_and_remains_enabled(self):
        with patch.object(self.app, '_read_positions', return_value=np.deg2rad([0,0,22,7,-1,28])), \
             patch.object(self.app, '_tracking_check', return_value=np.deg2rad([0,0,28,7,-1,28])), \
             patch.object(self.app, '_send_mit'), \
             patch.object(self.app, '_hold_measured_position') as hold:
            self.app._diagnose_hold_loop(6, np.array([.8]*6))
        hold.assert_called_once()
        self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_error')
        self.assertEqual(self.app.status.mode, 'error')
        self.assertTrue(self.app.status.enabled)

    def test_emergency_does_not_send_hold_or_overwrite_disabled_state(self):
        self.app.emergency_requested.set()
        self.app._set_status(mode='disabled', enabled=False)
        with patch.object(self.app, '_send_mit') as send:
            self.app._diagnose_hold_loop(6, np.array([.8]*6))
        send.assert_not_called()
        self.assertEqual(self.app.status.mode, 'disabled')

    def test_bad_start_position_rejected_without_motor_command(self):
        with patch.object(self.app, '_read_positions', return_value=np.deg2rad([101,0,0,0,0,0])), \
             patch.object(self.app, '_position_command') as send, \
             patch.object(self.app, '_hold_measured_position'):
            self.app._diagnose_hold_loop(6, np.array([.8]*6))
        send.assert_not_called()
        self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_error')

    def test_sustained_velocity_aborts_but_single_spike_does_not(self):
        for sustained in (True, False):
            with self.subTest(sustained=sustained):
                self.records.clear()
                calls = [0]
                def feedback(target):
                    i = calls[0]
                    calls[0] += 1
                    q = target.copy()
                    q[2] += .02*i if sustained else (.02 if i else 0.)
                    return q
                with patch.object(self.app, '_read_positions', return_value=np.deg2rad([0,0,22,7,-1,28])), \
                     patch.object(self.app, '_tracking_check', side_effect=feedback), \
                     patch.object(self.app, '_send_mit'), \
                     patch.object(self.app, '_hold_measured_position'):
                    self.app._diagnose_hold_loop(.16, np.array([.8]*6))
                self.assertEqual(self.records[-1]['kind'],
                                 'hold_diagnostic_error' if sustained else 'hold_diagnostic_complete')
                if sustained: self.assertIn('连续3轮', self.records[-1]['error'])

    def test_nonfinite_feedback_aborts_without_success(self):
        with patch.object(self.app, '_read_positions', return_value=np.deg2rad([0,0,22,7,-1,28])), \
             patch.object(self.app, '_tracking_check', return_value=np.full(6, float('nan'))), \
             patch.object(self.app, '_send_mit'), \
             patch.object(self.app, '_hold_measured_position'):
            self.app._diagnose_hold_loop(6, np.array([.8]*6))
        self.assertEqual(self.records[-1]['kind'], 'hold_diagnostic_error')

    def test_home_or_unscanned_pose_rejected_without_any_motor_command(self):
        for q in ([0,0,1.14,0,0,0], [2,0,22,7,-1,28]):
            with self.subTest(q=q), \
                 patch.object(self.app, '_read_positions', return_value=np.deg2rad(q)), \
                 patch.object(self.app, '_position_command') as send, \
                 patch.object(self.app, '_hold_measured_position') as hold:
                self.app._diagnose_hold_loop(6, np.array([.8]*6))
                send.assert_not_called()
                hold.assert_not_called()
                self.assertIn('首轮方案', self.records[-1]['error'])
