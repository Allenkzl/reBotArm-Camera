"""Shooting-group API and single-worker motion orchestration.

No hardware is opened at import, metadata load, or status inspection.
"""
from __future__ import annotations

import copy
import math
import threading
import time
import traceback
import uuid
from typing import Any

import numpy as np

if __package__:
    from .replay_path import ReplayPath, CONTROL_HZ, MAX_VELOCITY_RAD_S, MAX_ACCELERATION_RAD_S2
else:
    from replay_path import ReplayPath, CONTROL_HZ, MAX_VELOCITY_RAD_S, MAX_ACCELERATION_RAD_S2


class MotionCancelled(Exception):
    pass


class ShootingControl:
    def _require_started(self) -> None:
        if not self.status.enabled or self.emergency_requested.is_set():
            raise RuntimeError("请先点击「启动机械臂」")
        if self.status.mode in ("starting", "homing", "error"):
            raise RuntimeError("请等待当前操作结束或先关闭机械臂")

    def _join_worker(self) -> None:
        self._photo_run_cancel.set()
        self.camera.cancel_photo()
        self._cancel_photo_marks("当前操作已停止，拍照点未添加")
        self.stop_requested.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=8.0)
            if self.thread.is_alive():
                raise RuntimeError("当前控制线程尚未停止，未启动新的运动")

    def _prepare_worker(self) -> None:
        with self.motor_lock:
            if self.emergency_requested.is_set():
                raise RuntimeError("机械臂已紧急失能，请重新启动")
            if self._command_generation != self._cancel_generation:
                raise RuntimeError("当前请求已被停止操作取消")
            self._worker_generation = self._command_generation
            self.stop_requested.clear()

    def _editable_groups(self) -> None:
        if self.recording is not None or self.status.mode in (
            "starting", "homing", "replay_starting", "replaying", "group_waiting"
        ) or self.status.group_playback is not None:
            raise RuntimeError("录制或执行中不能修改拍摄组，请先停止")

    def groups_command(self, action: str, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self._editable_groups()
            group_id, block_id = data.get("group_id"), data.get("block_id")
            if action == "groups/add":
                self._groups.add_group()
            elif action == "groups/update":
                self._groups.update_group(group_id, name=data.get("name"), interval_s=data.get("interval_s"))
            elif action == "groups/delete":
                self._groups.delete_group(group_id)
            elif action == "blocks/add":
                self._groups.add_block(group_id)
            elif action == "blocks/update":
                self._groups.update_block(group_id, block_id, data.get("name"))
            elif action == "blocks/delete":
                self._groups.delete_block(group_id, block_id)
            else:
                raise ValueError("未知拍摄组操作")
            self._set_status(message="拍摄组设置已保存")
        return self.status.to_json()

    def record_block(self, data: dict[str, Any]) -> dict[str, Any]:
        self._require_started()
        with self.lock:
            self._editable_groups()
            block = self._groups.resolve_block(data.get("group_id"), data.get("block_id"))
            if block["slot"] is None:
                used = {t["slot"] for t in self._trajectories}
                used.update(b["slot"] for g in self._groups.groups for b in g["blocks"] if b["slot"] is not None)
                slot = next((s for s in range(1, 401) if s not in used), None)
                if slot is None:
                    raise ValueError("动作存储已满，请备份后整理历史轨迹")
                self._groups.assign_slot(data["group_id"], data["block_id"], slot)
                block = self._groups.resolve_block(data["group_id"], data["block_id"])
            return self.start_recording({"slot": block["slot"], "label": block["name"],
                                         "camera_profile": self.settings.camera_profile})

    def stop_block_recording(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            block = self._groups.resolve_block(data.get("group_id"), data.get("block_id"))
            if block["slot"] is None:
                raise ValueError("动作尚未录制")
            return self.stop_recording({"slot": block["slot"]})

    def play_block(self, data: dict[str, Any]) -> dict[str, Any]:
        self._require_started()
        with self.lock:
            block = self._groups.resolve_block(data.get("group_id"), data.get("block_id"))
            slot = block["slot"]
        if slot is None:
            raise ValueError("请先录制该动作")
        return self.replay_trajectory({"slot": slot, "speed": data.get("speed", .5),
                                       "point_mode": data.get("point_mode", False)})

    def _validate_trajectory(self, trajectory: dict[str, Any]) -> None:
        samples = trajectory.get("samples")
        name = trajectory.get("label", "动作")
        if not isinstance(samples, list) or len(samples) < 8:
            raise ValueError(f"{name} 尚未录制或有效样本不足")
        previous = -math.inf
        for sample in samples:
            t = float(sample["t"])
            if not math.isfinite(t) or t < 0 or t <= previous:
                raise ValueError(f"{name} 采样时间无效")
            previous = t
            self._assert_soft_limits(np.array(sample["q_rad"], dtype=float), label=name)
            if "effective_scale" in sample:
                scale = np.asarray(sample["effective_scale"], dtype=float)
                if scale.shape != (6,) or not np.all(np.isfinite(scale)) or np.any(scale < 0) or np.any(scale > 2.0):
                    raise ValueError(f"{name} 补偿倍率无效")
        self._validate_photo_points(trajectory)

    def _trajectory_for_slot(self, slot: int | None) -> dict[str, Any]:
        tr = next((t for t in self._trajectories if t.get("slot") == slot), None)
        if tr is None:
            raise ValueError("请先录制组内全部动作")
        self._validate_trajectory(tr)
        return copy.deepcopy(tr)

    def _play_request(self, trajectories: list[dict[str, Any]], speed: float,
                      group: dict[str, Any] | None = None) -> dict[str, Any]:
        self._require_started()
        if self.recording is not None:
            raise RuntimeError("请先结束录制")
        if self.status.mode not in ("running", "holding"):
            raise RuntimeError("当前动作正在执行，请先停止")
        if not math.isfinite(speed) or not .1 <= speed <= 1:
            raise ValueError("播放速度须在 10%–100% 之间")
        # Every block is validated before any worker is stopped or created.
        for trajectory in trajectories:
            self._validate_trajectory(trajectory)
            if not trajectory.get("_point_replay"):
                trajectory["_replay_path"] = ReplayPath(trajectory["samples"], speed,
                                                       [p["sample_index"] for p in trajectory.get("photo_points", [])])
        if (len(trajectories) == 1 and group is None and
                self.settings.camera_profile == "unloaded_50g" and
                not trajectories[0].get("_framing_check") and trajectories[0].get("photo_points")):
            if trajectories[0].get("camera_profile") != "unloaded_50g":
                raise ValueError("50 克精度测试仅接受在 50 克配置下录制的动作")
            trajectories[0]["_precision_check"] = True
        if not (len(trajectories) == 1 and
                (trajectories[0].get("_framing_check") or trajectories[0].get("_precision_check"))):
            self._photo_camera_preflight(trajectories)
        self._join_worker()
        self._prepare_worker()
        self._photo_run_nonce = str(uuid.uuid4())
        self._photo_run_cancel = threading.Event()
        self._photo_run_post_delay_s = self._photo_post_delay_s
        self._open_run_log("shooting_group" if group else "replay")
        if group:
            self._write_log({"kind": "group_plan", "group_id": group["id"],
                             "blocks": [{"id": b["id"], "name": b["name"], "slot": b["slot"]} for b in group["blocks"]],
                             "skipped_empty_blocks": group.get("skipped_empty_blocks", [])})
        self._playback_hold_scale = self._effective_scale(self.settings)
        self._replay_feedback_q = None
        self._set_status(mode="replay_starting", group_playback=None, photo_capture=None, message="正在准备播放")
        self.thread = threading.Thread(target=self._sequence_loop, args=(trajectories, speed, group),
                                       name="shooting-sequence", daemon=True)
        self.thread.start()
        return self.status.to_json()

    def replay_trajectory(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            trajectory = self._trajectory_for_slot(self._validated_slot(data))
        point_mode = data.get("point_mode", False)
        if type(point_mode) is not bool:
            raise ValueError("点位模式参数无效")
        if point_mode:
            if self.settings.camera_profile != "heavy" or trajectory.get("camera_profile") != "heavy":
                raise ValueError("点位拍照需要使用 950 克配置录制和播放")
            if not trajectory.get("photo_points"):
                raise ValueError("没有空格打点，请先录制拍照点")
            trajectory["_point_replay"] = True
        return self._play_request([trajectory], float(data.get("speed", .5)))

    def check_photo_framing(self, data: dict[str, Any]) -> dict[str, Any]:
        if set(data) not in ({"slot"}, {"slot", "profile"}) or type(data["slot"]) is not int:
            raise ValueError("无快门构图对照需要指定一个动作")
        correction = None
        sweep = False
        j3_probe = False
        if "profile" in data:
            profile = data["profile"]
            j3_probe = profile == "j3_hold_probe" and data["slot"] == 11
            if not j3_probe and (profile not in ("j2_minus_050", "j3_plus_050", "j3_j2_steps") or data["slot"] != 10):
                raise ValueError("单轴构图对照仅适用于已核对的动作")
            sweep = profile == "j3_j2_steps"
            correction = (1, -.5) if profile == "j2_minus_050" else (2, .5)
        with self.lock:
            trajectory = self._trajectory_for_slot(self._validated_slot(data))
        points = trajectory.get("photo_points")
        if not points:
            raise ValueError("动作没有拍照点")
        if j3_probe:
            point = points[0]
            expected = [37.552, 79.983, 60.207, 4.636, 1.055, -8.438]
            if point["sample_index"] != 199 or np.max(np.abs(np.rad2deg(point["q_rad"])-expected)) > .05:
                raise ValueError("拍照点与动作 2 已核对的示教姿态不一致")
            trajectory["_framing_j3_probe"] = True
        elif correction is not None:
            expected = [32.630, 66.228, 49.110, .176, -16.766, -18.348]
            point = points[0]
            if point["sample_index"] != 330 or np.max(np.abs(np.rad2deg(point["q_rad"])-expected)) > .05:
                raise ValueError("拍照点与已核对的示教姿态不一致")
            for joint, offset_deg in ((2, 1.25), (1, -1.25)) if sweep else (correction,):
                corrected = np.array(point["q_rad"], dtype=float)
                corrected[joint] += math.radians(offset_deg)
                self._assert_soft_limits(corrected, label=f"J{joint+1} 构图对照目标")
            trajectory["_framing_sweep"] = sweep
            trajectory["_framing_correction"] = correction
        trajectory["_framing_check"] = True
        return self._play_request([trajectory], .5)

    def play_group(self, data: dict[str, Any]) -> dict[str, Any]:
        self._require_started()
        with self.lock:
            group = copy.deepcopy(self._groups.resolve_group(data.get("group_id")))
            saved_slots = {t["slot"] for t in self._trajectories}
            skipped = [b["name"] for b in group["blocks"] if b["slot"] not in saved_slots]
            group["blocks"] = [b for b in group["blocks"] if b["slot"] in saved_slots]
            if not group["blocks"]:
                raise ValueError("本组暂无已保存动作，请先完成录制")
            trajectories = [self._trajectory_for_slot(block["slot"]) for block in group["blocks"]]
            for trajectory, block in zip(trajectories, group["blocks"]):
                trajectory["label"] = block["name"]
            group["skipped_empty_blocks"] = skipped
        return self._play_request(trajectories, float(data.get("speed", .5)), group)

    def _check_cancel(self) -> None:
        if self.stop_requested.is_set() or self.emergency_requested.is_set() or self._worker_generation != self._cancel_generation:
            raise MotionCancelled()

    def _position_command(self, q: np.ndarray, scale: np.ndarray, *, j3_probe_nm: float = 0.) -> None:
        self._check_cancel()
        # Gravity depends on the arm's measured pose, especially while lagging.
        measured = getattr(self, "_replay_feedback_q", None)
        model, payload = self._estimate_payload(q if measured is None else measured)
        unclipped = (model + payload) * scale
        if j3_probe_nm:
            if not 0. < j3_probe_nm <= .40:
                raise ValueError("J3 探测偏置超出范围")
            unclipped[2] += j3_probe_nm
            if np.any(np.abs(unclipped) > self.command_clamp_nm):
                raise RuntimeError("探测会触及前馈限幅，已中止")
        tau = np.clip(unclipped, -self.command_clamp_nm, self.command_clamp_nm)
        self._send_mit(q, tau, kp=self.replay_kp, kd=self.replay_kd)
        # Snapshot the exact feedforward calculation used above; diagnostics
        # must not add model evaluations, feedback reads, or motor commands.
        self._position_diagnostics = {
            "q_feedforward_deg": np.rad2deg(q if measured is None else measured).tolist(),
            "tau_model_nm": model.tolist(), "tau_payload_nm": payload.tolist(),
            "effective_scale": np.asarray(scale).tolist(),
            "tau_ff_unclipped_nm": unclipped.tolist(),
            "tau_ff_cmd_nm": tau.tolist(),
            "ff_clamp_nm": self.command_clamp_nm.tolist(),
            "ff_clipped": (np.abs(unclipped) > self.command_clamp_nm).tolist(),
            "kp_nominal": self.replay_kp.tolist(), "kd_nominal": self.replay_kd.tolist(),
        }

    def _transition(self, target: np.ndarray, scale: np.ndarray, *, speed: float = 1., point_id: str | None = None) -> None:
        start = self._read_positions(0)
        self._assert_soft_limits(start, label="当前姿态")
        self._replay_feedback_q = start.copy()
        delta = target - start
        if np.max(np.abs(delta)) < math.radians(.5):
            return
        # Minimum-jerk peak derivative is 1.875; bound target speed to 0.15 rad/s.
        distance = float(np.max(np.abs(delta)))
        duration = max(1., 1.875 * distance / MAX_VELOCITY_RAD_S,
                       math.sqrt(5.774 * distance / MAX_ACCELERATION_RAD_S2))
        duration /= speed
        steps = max(2, math.ceil(duration * 30))
        previous_tick = time.monotonic()
        for i in range(1, steps + 1):
            self.stop_requested.wait(max(0., previous_tick + duration/steps - time.monotonic()))
            self._check_cancel()
            now = time.monotonic()
            if point_id is not None and now - previous_tick > .25 and not self.dry_run:
                raise RuntimeError("点位过渡反馈更新超过 250 毫秒")
            previous_tick = now
            s = i / steps
            q = start + delta * (10*s**3 - 15*s**4 + 6*s**5)
            self._position_command(q, scale)
            actual = self._tracking_check(q)
            if point_id is not None:
                self._write_log({"kind": "point_move_sample", "point_id": point_id,
                                 "q_target_deg": np.rad2deg(q).tolist(),
                                 "q_actual_deg": np.rad2deg(actual).tolist()})
                if i % 6 == 0:
                    self._set_status(q_deg=np.rad2deg(actual).tolist())

    def _tracking_check(self, target: np.ndarray) -> np.ndarray:
        actual = self._read_positions(0)
        if actual.shape != (6,) or not np.all(np.isfinite(actual)):
            raise RuntimeError("关节反馈包含无效数值，已中止播放")
        errors = np.rad2deg(target - actual)
        if not self.dry_run and np.max(np.abs(errors)) > 20:
            joint = int(np.argmax(np.abs(errors)))
            self._write_log({"kind": "tracking_error", "joint": f"J{joint+1}",
                             "q_target_deg": np.rad2deg(target).tolist(),
                             "q_actual_deg": np.rad2deg(actual).tolist(), "error_deg": errors.tolist()})
            raise RuntimeError(f"J{joint+1}轨迹跟踪偏差 {abs(errors[joint]):.1f}° 超过 20°"
                               f"（目标 {math.degrees(target[joint]):.1f}° / 实际 {math.degrees(actual[joint]):.1f}°），已中止后续动作")
        # Simulated motor feedback follows replay targets; never evidence of field tracking.
        self._replay_feedback_q = target.copy() if self.dry_run else actual.copy()
        return self._replay_feedback_q.copy()

    def _play_trajectory_block(self, trajectory: dict[str, Any], speed: float) -> np.ndarray:
        samples = trajectory["samples"]
        first = samples[0]
        scale = np.array(first.get("effective_scale", self._effective_scale(self.settings)), dtype=float)
        if self.status.group_playback:
            progress = {**self.status.group_playback, "phase": "transition"}
            self._set_status(group_playback=progress)
        if trajectory.get("photo_points"):
            self._show_photo_target(trajectory["photo_points"][0])
        else:
            self._set_status(photo_capture=None)
        self._transition(np.array(first["q_rad"], dtype=float), scale)
        if self.status.group_playback:
            self._set_status(group_playback={**self.status.group_playback, "phase": "playing"})
        path = trajectory.get("_replay_path")
        if path is None:
            path = ReplayPath(samples, speed, [p["sample_index"] for p in trajectory.get("photo_points", [])])
        self._write_log({"kind": "replay_start", "slot": trajectory["slot"], "speed": speed,
                         "control_hz": CONTROL_HZ, "planned_duration_s": path.duration,
                         "max_velocity_rad_s": MAX_VELOCITY_RAD_S,
                         "max_acceleration_rad_s2": MAX_ACCELERATION_RAD_S2})
        points_by_sample: dict[int, list[tuple[int, dict[str, Any]]]] = {}
        for number, point in enumerate(trajectory.get("photo_points", []), 1):
            points_by_sample.setdefault(point["sample_index"], []).append((number, point))
        node_times = np.r_[0., np.cumsum(path.durations)]
        logical_time = 0.
        last_status = 0.
        def command(target, scale, index, fraction):
            nonlocal last_status
            self._assert_soft_limits(target, label="回放目标")
            self._playback_hold_scale = scale
            self._position_command(target, scale)
            actual = self._tracking_check(target)
            self._write_log({"kind": "replay_sample", "slot": trajectory["slot"], "i": index,
                             "fraction": fraction,
                             "q_target_deg": np.rad2deg(target).tolist(), "q_actual_deg": np.rad2deg(actual).tolist()})
            if time.monotonic() - last_status >= .2:
                upcoming = next(((number, p) for number, p in enumerate(trajectory.get("photo_points", []), 1)
                                 if node_times[p["sample_index"]] >= logical_time-1e-9), None)
                remaining = ""
                if upcoming:
                    number, point = upcoming
                    self._show_photo_target(point)
                    seconds = max(0., node_times[point["sample_index"]] - logical_time)
                    remaining = f" · 距拍照点{number}约{math.ceil(seconds)}秒"
                self._set_status(mode="replaying", q_deg=np.rad2deg(actual).tolist(),
                                 message=f"平稳回放 {trajectory['label']} · {index+1}/{len(samples)}{remaining}")
                last_status = time.monotonic()

        previous_tick = time.monotonic()
        command(path.q[0], scale, 0, 0.)
        for number, point in points_by_sample.get(0, []):
            if trajectory.get("_precision_check"):
                return self._hold_precision_point(trajectory, point, scale)
            if trajectory.get("_framing_check"):
                return self._hold_framing_point(trajectory, point, scale)
            self._perform_photo_point(trajectory, point, number, scale)
            previous_tick = time.monotonic()
        for segment, duration in enumerate(path.durations):
            steps = max(1, round(float(duration)*CONTROL_HZ))
            start_scale = np.array(samples[segment].get("effective_scale", self._effective_scale(self.settings)), dtype=float)
            end_scale = np.array(samples[segment+1].get("effective_scale", self._effective_scale(self.settings)), dtype=float)
            for tick in range(1, steps+1):
                # Advance one fixed logical tick only. Late cycles and photo
                # pauses stretch wall time; never catch up by jumping targets.
                self.stop_requested.wait(max(0., previous_tick + 1/CONTROL_HZ - time.monotonic()))
                self._check_cancel()
                now = time.monotonic()
                if now-previous_tick > .25 and not self.dry_run:
                    self._write_log({"kind": "control_timing_error", "gap_s": now-previous_tick})
                    raise RuntimeError("回放控制更新间隔超过250毫秒，已中止后续动作")
                previous_tick = now
                fraction = tick/steps
                scale = start_scale + (end_scale-start_scale)*fraction
                logical_time = float(node_times[segment] + duration*fraction)
                command(path.position(segment, fraction), scale,
                        segment+1 if tick == steps else segment, fraction)
            for number, point in points_by_sample.get(segment+1, []):
                if trajectory.get("_precision_check"):
                    return self._hold_precision_point(trajectory, point, scale)
                if trajectory.get("_framing_check"):
                    return self._hold_framing_point(trajectory, point, scale)
                self._perform_photo_point(trajectory, point, number, scale)
                previous_tick = time.monotonic()
        return np.array(samples[-1]["q_rad"], dtype=float)

    def _hold_framing_point(self, trajectory, point, scale):
        target = np.array(point["q_rad"], dtype=float)
        correction = trajectory.get("_framing_correction")
        command = target
        baseline_samples = []
        baseline = None
        self._photo_diagnostic_started = time.monotonic()
        self._photo_diagnostic_context = {"slot": trajectory["slot"], "point_id": point["id"],
                                          "sample_index": point["sample_index"], "phase": "framing_check"}
        try:
            if trajectory.get("_framing_j3_probe"):
                return self._framing_step_sequence(target, scale, j3_probe=True)
            if trajectory.get("_framing_sweep"):
                return self._framing_step_sequence(target, scale)
            end = self._photo_diagnostic_started + (18. if correction is not None else 6.)
            previous = self._photo_diagnostic_started
            last_status = -math.inf
            while (now := time.monotonic()) < end:
                if now - previous > .25 and not self.dry_run:
                    raise RuntimeError("构图对照控制更新间隔超过250毫秒")
                elapsed = now-self._photo_diagnostic_started
                message = "已到拍照点，正在保持，请查看相机取景"
                if correction is not None and elapsed >= 6.:
                    joint, offset_deg = correction
                    axis = f"J{joint+1}"
                    if baseline is None:
                        if len(baseline_samples) < 45 or np.max(np.ptp(baseline_samples, axis=0)) > math.radians(.15):
                            raise RuntimeError(f"构图对照基线未稳定，已中止 {axis} 校正")
                        baseline = np.mean(baseline_samples, axis=0)
                    fraction = min(1., (elapsed-6.)/2.)
                    command = target.copy()
                    command[joint] += math.radians(offset_deg)*fraction**2*(3.-2.*fraction)
                    self._photo_diagnostic_context["phase"] = f"framing_{axis.lower()}_" + ("hold" if fraction == 1. else "ramp")
                    message = (f"{axis} 校正后保持，请查看相机取景" if fraction == 1. else f"正在缓慢校正 {axis}")
                actual = self._photo_hold_sample(command, scale)
                if correction is not None:
                    if np.max(np.abs(actual-command)) > math.radians(5.):
                        raise RuntimeError("构图对照偏差超过5°，已中止")
                    if 4. <= elapsed < 6.:
                        baseline_samples.append(actual.copy())
                    if baseline is not None and np.max(np.abs(actual-baseline)) > math.radians(1.):
                        raise RuntimeError("构图对照位移超过1°，已中止")
                if now - last_status >= .2:
                    self._set_status(q_deg=np.rad2deg(actual).tolist(),
                                     message=message)
                    last_status = now
                previous = now
                self.stop_requested.wait(max(0., min(now + 1/CONTROL_HZ, end)-time.monotonic()))
            self._check_cancel()
            return command
        finally:
            self._photo_diagnostic_context = None

    def _framing_step_sequence(self, target, scale, *, j3_probe=False):
        baseline = None
        previous_tick = time.monotonic()

        def stage(start, goal, ramp_s, hold_s, phase, message, photo_label=None, *, bias_start=0., bias_end=0.):
            nonlocal previous_tick
            began = time.monotonic()
            samples = []
            last_status = -math.inf
            self._photo_diagnostic_context["phase"] = phase
            while (now := time.monotonic()) < began + ramp_s + hold_s:
                if now - previous_tick > .25:
                    raise RuntimeError("构图对照控制更新间隔超过250毫秒")
                previous_tick = now
                elapsed = now-began
                u = min(1., elapsed/ramp_s) if ramp_s else 1.
                command = start + (goal-start)*u*u*(3.-2.*u)
                bias = bias_start + (bias_end-bias_start)*u
                actual = (self._photo_hold_sample(command, scale, j3_probe_nm=bias)
                          if bias else self._photo_hold_sample(command, scale))
                if np.max(np.abs(actual-command)) > math.radians(5.):
                    raise RuntimeError("构图对照偏差超过5°，已中止")
                if baseline is not None and np.max(np.abs(actual-baseline)) > math.radians(1.):
                    raise RuntimeError("构图对照位移超过1°，已中止")
                if elapsed >= ramp_s + hold_s - 1.5:
                    samples.append(actual.copy())
                if now - last_status >= .2:
                    cue = message
                    if photo_label and elapsed >= ramp_s + 2.:
                        remaining = math.ceil(ramp_s+hold_s-elapsed)
                        cue = f"请拍照 · {photo_label} · 剩余 {remaining} 秒"
                    self._set_status(q_deg=np.rad2deg(actual).tolist(), message=cue)
                    last_status = now
                self.stop_requested.wait(max(0., min(now+1/CONTROL_HZ, began+ramp_s+hold_s)-time.monotonic()))
            self._photo_check_cancel()
            if len(samples) < 30 or np.max(np.ptp(samples, axis=0)) > math.radians(.15):
                raise RuntimeError("构图对照反馈未稳定，已中止后续调整")
            return np.median(samples, axis=0)

        if j3_probe:
            baseline = stage(target, target, 0., 6., "framing_check", "动作 2 · 正在记录基线")
            stage(target, target, 2., 6., "framing_j3_bias", "正在测量 J3 补偿响应", bias_end=.40)
            stage(target, target, 2., 6., "framing_j3_bias_return", "正在撤回 J3 探测补偿", bias_start=.40)
            return target

        baseline = stage(target, target, 0., 14., "framing_check",
                         "已到拍照点，正在稳定，准备拍基线", "基线")
        for joint, direction in ((2, 1.), (1, -1.)):
            axis = f"J{joint+1}"
            command = target
            for offset in (.75, 1.25):
                goal = target.copy()
                goal[joint] += math.radians(direction*offset)
                label = f"{axis} {direction*offset:+.2f}°"
                actual = stage(command, goal, 2., 14., f"framing_{axis.lower()}_step_{offset:.2f}",
                               f"正在调整下一步 · {label} · 请等待", label)
                command = goal
                if abs(actual[joint]-baseline[joint]) >= math.radians(.25):
                    break
            actual = stage(command, target, 2., 4., f"framing_{axis.lower()}_return",
                           f"正在调整下一步 · {axis} 回到基线 · 请等待")
            if np.max(np.abs(actual-baseline)) > math.radians(.20):
                raise RuntimeError(f"{axis} 未恢复基线，已中止后续调整")
        return target

    def _wait_between_blocks(self, target: np.ndarray, seconds: float) -> None:
        end, last_status = time.monotonic() + seconds, 0.
        scale = self._playback_hold_scale
        while time.monotonic() < end:
            self._position_command(target, scale)
            actual = self._tracking_check(target)
            if time.monotonic() - last_status >= .2:
                progress = {**self.status.group_playback, "phase": "waiting", "remaining_s": max(0., end-time.monotonic())}
                self._set_status(mode="group_waiting", group_playback=progress,
                                 q_deg=np.rad2deg(actual).tolist(), message="动作间隔等待中")
                last_status = time.monotonic()
            self.stop_requested.wait(min(1/30, max(0., end-time.monotonic())))
        self._check_cancel()

    def _hold_measured_position(self) -> None:
        # Intentional stop keeps internal motor PD active; emergency forbids sends.
        with self.motor_lock:
            if self.emergency_requested.is_set() or not self.status.enabled:
                return
            q = self._read_positions(0)
            if q.shape != (6,) or not np.all(np.isfinite(q)):
                raise RuntimeError("关节反馈无效，不能发送保持指令")
            model, payload = self._estimate_payload(q)
            tau = np.clip((model+payload)*self._effective_scale(self.settings),
                          -self.command_clamp_nm, self.command_clamp_nm)
            self._send_mit(q, tau, kp=self.replay_kp, kd=self.replay_kd)
            self._set_status(mode="holding", q_deg=np.rad2deg(q).tolist())

    def _show_photo_target(self, point):
        self._set_status(photo_capture={
            "precision_reference_deg": np.rad2deg(point["q_rad"]).tolist(),
            "tolerance_deg": .5,
        })

    def _play_taught_points(self, trajectory, speed):
        scale = self._playback_hold_scale
        tolerance = .5
        origin = self._read_positions(0).copy()
        self._assert_soft_limits(origin, label="按点回放起始姿态")
        self._write_log({"kind": "point_replay_origin", "slot": trajectory["slot"],
                         "q_deg": np.rad2deg(origin).tolist()})
        for index, point in enumerate(trajectory["photo_points"], 1):
            self._photo_check_cancel()
            target = np.asarray(point["q_rad"], dtype=float)
            self._show_photo_target(point)
            self._set_status(message=f"点位回放：正在前往第 {index} 个拍照点")
            self._write_log({"kind": "point_move", "slot": trajectory["slot"],
                             "point_id": point["id"], "target_deg": np.rad2deg(target).tolist()})
            self._transition(target, scale, speed=speed, point_id=point["id"])
            command = self._hold_precision_point(trajectory, point, scale, tolerance=tolerance)
            self._perform_photo_point(trajectory, point, index, scale, hold_command=command,
                                      tolerance=tolerance, post_delay=2.)
        self._photo_check_cancel()
        home = self._home_q.copy()
        self._set_status(message="拍照完成，正在返回 Home")
        self._transition(home, scale, speed=speed, point_id="return_home")
        actual = self._tracking_check(home)
        error = np.rad2deg(actual - home)
        self._write_log({"kind": "point_replay_return", "slot": trajectory["slot"],
                         "q_target_deg": np.rad2deg(home).tolist(),
                         "q_actual_deg": np.rad2deg(actual).tolist(), "error_deg": error.tolist()})
        self._set_status(q_deg=np.rad2deg(actual).tolist())
        if np.max(np.abs(error)) > 3.:
            raise RuntimeError("未可靠返回 Home，保持使能，请检查机械臂")
        return home

    def _sequence_loop(self, trajectories: list[dict[str, Any]], speed: float,
                       group: dict[str, Any] | None) -> None:
        try:
            for i, trajectory in enumerate(trajectories):
                self._check_cancel()
                progress = None if group is None else {"group_id": group["id"], "block_id": group["blocks"][i]["id"],
                            "index": i+1, "total": len(trajectories), "phase": "transition", "remaining_s": 0}
                self._set_status(mode="replaying", group_playback=progress)
                if trajectory.get("_point_replay"):
                    target = self._play_taught_points(trajectory, speed)
                else:
                    target = self._play_trajectory_block(trajectory, speed)
                self._write_log({"kind": "block_complete", "slot": trajectory["slot"]})
                if group and i < len(trajectories)-1:
                    self._wait_between_blocks(target, group["interval_s"])
            self._check_cancel()
            self._position_command(target, self._playback_hold_scale)
            message = ("构图对照完成，保持在拍照点；点击「关闭机械臂」回 Home 后失能"
                       if trajectories[0].get("_framing_check") else
                       "播放完成，保持使能；点击「关闭机械臂」回 Home 后失能")
            if trajectories[0].get("_point_replay"):
                message = "按点拍照完成，已返回 Home 并保持使能"
            if trajectories[0].get("_precision_check"):
                message = "精度测试通过：六轴持续 2 秒在 ±0.5° 内，正在保持"
            self._set_status(mode="holding", message=message)
            self._event("info", "拍摄组播放完成" if group else "动作播放完成")
            if trajectories[0].get("_point_replay"):
                self._hold_home_loop(self._read_positions(0))
                self._run_loop(enable=False)
        except MotionCancelled:
            self._hold_measured_position()
            if not self.emergency_requested.is_set():
                self._set_status(message="播放已停止，未继续后续动作")
        except Exception as exc:
            self._write_log({"kind": "replay_error", "error": repr(exc), "traceback": traceback.format_exc()})
            try:
                self._hold_measured_position()
            except Exception:
                pass
            if not self.emergency_requested.is_set():
                self._set_status(mode="error", message=str(exc))
                self._event("danger", f"播放中止：{exc}")
        finally:
            self._set_status(group_playback=None)

    def stop_hold(self) -> dict[str, Any]:
        self._join_worker()
        if self.recording is not None:
            self.recording = None
            self._set_status(recording_slot=None, recording_samples=0)
        self._hold_measured_position()
        if self.status.enabled:
            self._set_status(message="紧急停止：已中止动作并保持当前位置")
        self._event("warn", "当前动作已停止，仍保持使能" if self.status.enabled else "机械臂未使能，启动已取消")
        return self.status.to_json()

    def emergency_disable(self) -> dict[str, Any]:
        self._photo_run_cancel.set()
        self.camera.cancel_photo()
        self._cancel_photo_marks("已紧急停止，拍照点未添加")
        self.emergency_requested.set()
        self.stop_requested.set()
        with self.motor_lock:
            if self.robot is not None:
                self.robot.arm.disable()
        self._join_worker()
        with self.motor_lock:
            self._shutdown_robot(self.robot)
            self.robot = None
            self.recording = None
            self._set_status(mode="disabled", enabled=False, connected=False, recording_slot=None,
                             recording_samples=0, group_playback=None, message="已紧急失能")
        return self.status.to_json()

    def _hold_home_loop(self, start):
        target = self._home_q.copy()
        kp = np.array([50., 150., 150., 50., 50., 50.])
        kd = np.array([3., 10., 10., 5., 4., 4.])
        began = previous = time.monotonic()
        stable_since = None
        last_status = -math.inf
        self._set_status(mode="running", photo_capture=None, message="正在固定保持新 Home")
        while self.recording is None:
            self._check_cancel()
            now = time.monotonic()
            if now-previous > .25 and not self.dry_run:
                raise RuntimeError("Home 保持更新间隔超过 250 毫秒，请现场检查")
            previous = now
            actual = self._read_positions(0)
            self._assert_soft_limits(actual, label="Home 实测")
            error = np.rad2deg(actual-target)
            if np.max(np.abs(error)) > 5.:
                raise RuntimeError("Home 实测偏差超过 5°，保持目标已冻结，请人工支撑并检查")
            u = min(1., (now-began)/2.)
            command = start+(target-start)*u*u*(3.-2.*u)
            model, payload = self._estimate_payload(actual)
            tau = np.clip(model+payload, -self.command_clamp_nm, self.command_clamp_nm)
            self._send_mit(command, tau, kp=kp, kd=kd)
            within = u == 1. and np.max(np.abs(error)) <= .5
            stable_since = (now if stable_since is None else stable_since) if within else None
            if now-last_status >= .2:
                ready = stable_since is not None and now-stable_since >= 2.
                self._set_status(q_deg=np.rad2deg(actual).tolist(),
                                 message="新 Home 保持到位（±0.5°）" if ready else "正在固定保持新 Home，核对 ±0.5°")
                last_status = now
            self._write_log({"kind": "home_hold_sample", "q_target_deg": np.rad2deg(target).tolist(),
                             "q_command_deg": np.rad2deg(command).tolist(),
                             "q_actual_deg": np.rad2deg(actual).tolist(), "error_deg": error.tolist(),
                             "kp": kp.tolist(), "kd": kd.tolist(), "tau_ff_nm": tau.tolist()})
            self.stop_requested.wait(max(0., now+1/30-time.monotonic()))
        self._check_cancel()
        self._event("info", "开始录制，切换手动摆位模式，请托稳机械臂")

    def home_hold(self) -> dict[str, Any]:
        self._require_started()
        return self.home_disable(disable=False)

    def home_disable(self, *, disable=True) -> dict[str, Any]:
        if not self.status.enabled:
            return self.status.to_json()
        self._join_worker()
        self.recording = None
        self._prepare_worker()
        self._set_status(mode="homing", recording_slot=None, recording_samples=0,
                         group_playback=None, message="正在返回 Home，完成后失能" if disable else "正在返回 Home，完成后保持使能")
        self.thread = threading.Thread(target=self._home_and_disable, kwargs={"disable": disable}, name="shooting-home", daemon=True)
        self.thread.start()
        return self.status.to_json()

    def _home_and_disable(self, *, disable=True) -> None:
        try:
            start = self._read_positions(0)
            if start.shape != (6,) or not np.all(np.isfinite(start)):
                raise RuntimeError("关节反馈无效，未执行 Home")
            self._assert_soft_limits(start, label="Home 起始姿态")
            target = self._home_q.copy()
            self._assert_soft_limits(target, label="Home")
            distance = float(np.max(np.abs(target-start)))
            duration = max(4., 1.875*distance/.15, math.sqrt(5.774*distance/.30))
            kp = np.array([50., 150., 150., 50., 50., 50.])
            kd = np.array([3., 10., 10., 5., 4., 4.])
            steps = math.ceil(duration*30)
            began = previous = time.monotonic()
            actual = start.copy()
            self._write_log({"kind": "home_move_start", "q_start_deg": np.rad2deg(start).tolist(),
                             "q_target_deg": np.rad2deg(target).tolist(), "duration_s": duration})
            for i in range(1, steps+1):
                self._check_cancel()
                now = time.monotonic()
                if now-previous > .25 and not self.dry_run:
                    raise RuntimeError("Home 运动反馈更新超过 250 毫秒，已停止")
                previous = now
                s = i/steps
                q = start + (target-start)*(10*s**3-15*s**4+6*s**5)
                model, payload = self._estimate_payload(actual)
                tau = np.clip(model+payload, -self.command_clamp_nm, self.command_clamp_nm)
                self._send_mit(q, tau, kp=kp, kd=kd)
                actual = self._tracking_check(q)
                self._write_log({"kind": "home_move_sample", "q_target_deg": np.rad2deg(q).tolist(),
                                 "q_actual_deg": np.rad2deg(actual).tolist()})
                if i % 6 == 0:
                    self._set_status(q_deg=np.rad2deg(actual).tolist())
                self.stop_requested.wait(max(0., began+duration*i/steps-time.monotonic()))
            self._check_cancel()
            # Do not disable a suspended arm after a failed return.
            actual = self._read_positions(0)
            if not self.dry_run and (not np.all(np.isfinite(actual)) or np.max(np.abs(np.rad2deg(actual-target))) > 3):
                raise RuntimeError("未可靠返回 Home，保持使能，请人工支撑并处理")
            if not disable:
                self._write_log({"kind": "home_hold_complete", "q_deg": np.rad2deg(actual).tolist()})
                self._hold_home_loop(actual)
                self._run_loop(enable=False)
                return
            with self.motor_lock:
                self._check_cancel()
                if not self.dry_run and self.robot is not None:
                    self.robot.arm.disable()
                self._shutdown_robot(self.robot)
                self.robot = None
                self._set_status(mode="disabled", connected=False, enabled=False,
                                 q_deg=np.rad2deg(actual).tolist(), message="已返回 Home，机械臂已失能")
        except MotionCancelled:
            self._hold_measured_position()
        except Exception as exc:
            if not self.emergency_requested.is_set():
                try:
                    self._hold_measured_position()
                except Exception:
                    pass
                self._set_status(mode="error", message=str(exc))
                self._event("danger", f"关闭未完成：{exc}")
