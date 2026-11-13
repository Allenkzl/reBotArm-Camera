"""Explicit in-place hold diagnostics; no camera, enable, or transition commands."""
from __future__ import annotations

import math
import threading
import time

import numpy as np

if __package__:
    from .shooting_control import MotionCancelled
else:
    from shooting_control import MotionCancelled


class HoldDiagnostics:
    def diagnose_hold(self, data):
        self._require_started()
        if self.recording is not None or self.status.mode not in ('running', 'holding'):
            raise RuntimeError('请先停止录制或其他动作')
        if set(data) - {'duration_s', 'j3_probe', 'j3_probe_profile'}:
            raise ValueError('原位诊断不接受自定义目标、增益或力矩')
        probe = data.get('j3_probe', False)
        if 'j3_probe_profile' in data:
            if data['j3_probe_profile'] != 'plus_040' or 'j3_probe' in data or 'duration_s' in data:
                raise ValueError('J3 探测档位无效或与其他设置冲突')
            probe = True
        probe_nm = .40 if 'j3_probe_profile' in data else .20
        if type(probe) is not bool or (probe and 'duration_s' in data):
            raise ValueError('J3 探测试验使用固定时序')
        duration = data.get('duration_s', 6.)
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 1 <= duration <= 10:
            raise ValueError('保持时长须为1至10秒')
        self._join_worker()
        self._prepare_worker()
        scale = self._effective_scale(self.settings).copy()
        self._open_run_log('hold_diagnostic')
        self._set_status(mode='replaying', message='原位保持诊断：正在读取实时位置', photo_capture=None)
        ramp_s = 2. if probe_nm == .40 else 1.
        self.thread = threading.Thread(target=self._diagnose_hold_loop,
                                       args=(20. + 2*ramp_s if probe else duration, scale, probe, probe_nm, ramp_s),
                                       name='hold-diagnostic', daemon=True)
        self.thread.start()
        return self.status.to_json()

    def _diagnose_hold_loop(self, duration, scale, probe=False, probe_nm=.20, ramp_s=1.):
        command_attempted = False
        try:
            self._check_cancel()
            # Freeze once. Never reset the target to actual feedback each cycle.
            with self.motor_lock:
                target = self._read_positions(0) if self.dry_run else self._startup_feedback_preflight()
            self._assert_soft_limits(target, label='原位诊断起点')
            # This first baseline is restricted to the candidate whose +/-6deg
            # neighborhood was sampled offline: 1deg placement + 5deg abort.
            nominal = np.array([0., 0., 22., 7., -1., 28.])
            if np.max(np.abs(np.rad2deg(target)-nominal)) > 1.:
                raise RuntimeError('原位诊断位置不符合首轮方案：J1–J6需接近[0°, 0°, 22°, 7°, −1°, 28°]，各关节允许±1°；未开始固定目标测试')
            self._replay_feedback_q = target.copy()
            self._write_log({'kind': 'hold_diagnostic_start', 'dry_run': self.dry_run,
                             'q_target_deg': np.rad2deg(target).tolist(), 'duration_s': duration,
                             'effective_scale': scale.tolist(), 'max_error_deg': 5.,
                             'max_update_gap_s': .25, 'j3_probe': probe,
                             'j3_probe_peak_nm': probe_nm if probe else 0.,
                             'j3_probe_ramp_s': ramp_s if probe else 0.})
            began = previous = time.monotonic()
            last_status = began - 1.
            count = 0
            previous_actual = None
            previous_feedback_time = None
            fast_samples = 0
            baseline_samples = []
            baseline = None
            while time.monotonic() - began < duration:
                self._check_cancel()
                now = time.monotonic()
                if now - previous > .25:
                    raise RuntimeError('原位诊断更新间隔超过250毫秒')
                previous = now
                elapsed = now-began
                bias = 0.
                if probe and elapsed >= 10.:
                    if baseline is None:
                        if len(baseline_samples) < 45 or np.max(np.ptp(baseline_samples, axis=0)) > math.radians(.15):
                            raise RuntimeError('探测前两秒未稳定，已中止')
                        baseline = np.mean(baseline_samples, axis=0)
                    ramp_end = 15. + 2*ramp_s
                    bias = probe_nm * max(0., min(1., (elapsed-10.)/ramp_s, (ramp_end-elapsed)/ramp_s))
                command_attempted = True
                if bias:
                    self._position_command(target, scale, j3_probe_nm=bias)
                else:
                    self._position_command(target, scale)
                actual = self._tracking_check(target)
                feedback_time = time.monotonic()
                if actual.shape != (6,) or not np.all(np.isfinite(actual)):
                    raise RuntimeError('原位诊断反馈无效')
                if feedback_time - previous > .25:
                    raise RuntimeError('原位诊断单轮控制耗时超过250毫秒')
                velocity = None
                if previous_actual is not None:
                    feedback_dt = feedback_time - previous_feedback_time
                    velocity = (actual-previous_actual)/max(feedback_dt, 1e-9)
                    fast_samples = fast_samples+1 if np.max(np.abs(velocity)) > .15 else 0
                previous_actual, previous_feedback_time = actual.copy(), feedback_time
                error = np.rad2deg(actual-target)
                if probe and 8. <= elapsed < 10.:
                    baseline_samples.append(actual.copy())
                self._write_log({'kind': 'hold_diagnostic_sample', 'dry_run': self.dry_run,
                                 'elapsed_s': time.monotonic()-began,
                                 'j3_probe_nm': bias,
                                 **self._position_diagnostics,
                                 'q_target_deg': np.rad2deg(target).tolist(),
                                 'q_actual_deg': np.rad2deg(actual).tolist(),
                                 'velocity_rad_s': None if velocity is None else velocity.tolist(),
                                 'error_actual_minus_target_deg': error.tolist()})
                if probe and any(self._position_diagnostics.get('ff_clipped', [])):
                    raise RuntimeError('探测遇到前馈限幅，已中止')
                if baseline is not None:
                    excursion = np.abs(np.rad2deg(actual-baseline))
                    if excursion[2] > .75 or np.max(excursion) > 1.:
                        raise RuntimeError('探测位移超出允许范围，已中止')
                if fast_samples >= 3:
                    raise RuntimeError('原位诊断实测速度连续3轮超过0.15rad/s，已中止')
                if np.max(np.abs(error)) > 5:
                    raise RuntimeError('原位诊断偏差超过5°，已中止')
                count += 1
                if now - last_status >= .2:
                    self._set_status(q_deg=np.rad2deg(actual).tolist(),
                                     message=f'原位保持诊断：{max(0., duration-(now-began)):.1f}秒')
                    last_status = now
                self.stop_requested.wait(max(0., min(previous+1/30, began+duration)-time.monotonic()))
            self._check_cancel()
            self._write_log({'kind': 'hold_diagnostic_complete', 'samples': count,
                             'elapsed_s': time.monotonic()-began})
            self._set_status(mode='holding', message='原位诊断完成，保持使能，等待安全收尾')
        except Exception as exc:
            self._write_log({'kind': 'hold_diagnostic_cancelled' if isinstance(exc, MotionCancelled)
                             else 'hold_diagnostic_error', 'error': str(exc)})
            if command_attempted:
                try:
                    self._hold_measured_position()
                except Exception as hold_error:
                    self._write_log({'kind': 'hold_diagnostic_stop_error', 'error': str(hold_error)})
            if not self.emergency_requested.is_set():
                self._set_status(mode='holding' if isinstance(exc, MotionCancelled) else 'error',
                                 message='原位诊断已停止，保持使能' if isinstance(exc, MotionCancelled) else str(exc))
