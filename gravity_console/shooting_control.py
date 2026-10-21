"""Shooting-group API and single-worker motion orchestration.

No hardware is opened at import, metadata load, or status inspection.
"""
from __future__ import annotations

import copy
import math
import threading
import time
import traceback
from typing import Any

import numpy as np


class MotionCancelled(Exception):
    pass


class ShootingControl:
    def _require_started(self) -> None:
        if not self.status.enabled or self.emergency_requested.is_set():
            raise RuntimeError("请先点击「启动机械臂」")
        if self.status.mode in ("starting", "homing", "error"):
            raise RuntimeError("请等待当前操作结束或先关闭机械臂")

    def _join_worker(self) -> None:
        self.stop_requested.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=8.0)
            if self.thread.is_alive():
                raise RuntimeError("当前控制线程尚未停止，未启动新的运动")

    def _prepare_worker(self) -> None:
        with self.motor_lock:
            if self.emergency_requested.is_set():
                raise RuntimeError("机械臂已紧急失能，请重新启动")
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
                                         "camera_profile": "heavy"})

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
        return self.replay_trajectory({"slot": slot, "speed": data.get("speed", .25),
                                       "camera_profile": "heavy"})

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
        self._join_worker()
        self._prepare_worker()
        self._open_run_log("shooting_group" if group else "replay")
        if group:
            self._write_log({"kind": "group_plan", "group_id": group["id"],
                             "blocks": [{"id": b["id"], "name": b["name"], "slot": b["slot"]} for b in group["blocks"]],
                             "skipped_empty_blocks": group.get("skipped_empty_blocks", [])})
        self._playback_hold_scale = self._effective_scale(self.settings)
        self._set_status(mode="replay_starting", group_playback=None, message="正在准备播放")
        self.thread = threading.Thread(target=self._sequence_loop, args=(trajectories, speed, group),
                                       name="shooting-sequence", daemon=True)
        self.thread.start()
        return self.status.to_json()

    def replay_trajectory(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            trajectory = self._trajectory_for_slot(self._validated_slot(data))
        return self._play_request([trajectory], float(data.get("speed", .25)))

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
        return self._play_request(trajectories, float(data.get("speed", .25)), group)

    def _check_cancel(self) -> None:
        if self.stop_requested.is_set() or self.emergency_requested.is_set():
            raise MotionCancelled()

    def _position_command(self, q: np.ndarray, scale: np.ndarray) -> None:
        self._check_cancel()
        model, payload = self._estimate_payload(q)
        tau = np.clip((model + payload) * scale, -self.command_clamp_nm, self.command_clamp_nm)
        self._send_mit(q, tau, kp=self.replay_kp, kd=self.replay_kd)

    def _transition(self, target: np.ndarray, scale: np.ndarray) -> None:
        start = self._read_positions(0)
        self._assert_soft_limits(start, label="当前姿态")
        delta = target - start
        if np.max(np.abs(delta)) < math.radians(.5):
            return
        # Minimum-jerk peak derivative is 1.875; bound target speed to 0.15 rad/s.
        duration = max(1., 1.875 * float(np.max(np.abs(delta))) / .15)
        steps = max(2, math.ceil(duration * 30))
        start_time = time.monotonic()
        for i in range(1, steps + 1):
            self._check_cancel()
            s = i / steps
            q = start + delta * (10*s**3 - 15*s**4 + 6*s**5)
            self._position_command(q, scale)
            self._tracking_check(q)
            wait = start_time + duration*i/steps - time.monotonic()
            if wait > 0:
                self.stop_requested.wait(wait)

    def _tracking_check(self, target: np.ndarray) -> np.ndarray:
        actual = self._read_positions(0)
        if actual.shape != (6,) or not np.all(np.isfinite(actual)):
            raise RuntimeError("关节反馈包含无效数值，已中止播放")
        if not self.dry_run and np.max(np.abs(np.rad2deg(target - actual))) > 20:
            raise RuntimeError("轨迹跟踪偏差超过 20°，已中止后续动作")
        return actual

    def _play_trajectory_block(self, trajectory: dict[str, Any], speed: float) -> np.ndarray:
        samples = trajectory["samples"]
        first = samples[0]
        scale = np.array(first.get("effective_scale", self._effective_scale(self.settings)), dtype=float)
        if self.status.group_playback:
            progress = {**self.status.group_playback, "phase": "transition"}
            self._set_status(group_playback=progress)
        self._transition(np.array(first["q_rad"], dtype=float), scale)
        if self.status.group_playback:
            self._set_status(group_playback={**self.status.group_playback, "phase": "playing"})
        self._write_log({"kind": "replay_start", "slot": trajectory["slot"], "speed": speed})
        start_time, first_t = time.monotonic(), float(first["t"])
        last_status = 0.
        for i, sample in enumerate(samples):
            elapsed = (float(sample["t"]) - first_t) / speed
            wait = start_time + elapsed - time.monotonic()
            if wait > 0:
                self.stop_requested.wait(wait)
            self._check_cancel()
            target = np.array(sample["q_rad"], dtype=float)
            scale = np.array(sample.get("effective_scale", self._effective_scale(self.settings)), dtype=float)
            self._playback_hold_scale = scale
            self._position_command(target, scale)
            actual = self._tracking_check(target)
            self._write_log({"kind": "replay_sample", "slot": trajectory["slot"], "i": i,
                             "q_target_deg": np.rad2deg(target).tolist(), "q_actual_deg": np.rad2deg(actual).tolist()})
            if time.monotonic() - last_status >= .2:
                self._set_status(mode="replaying", q_deg=np.rad2deg(actual).tolist(),
                                 message=f"正在播放 {trajectory['label']} · {i+1}/{len(samples)}")
                last_status = time.monotonic()
        return np.array(samples[-1]["q_rad"], dtype=float)

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

    def _sequence_loop(self, trajectories: list[dict[str, Any]], speed: float,
                       group: dict[str, Any] | None) -> None:
        try:
            for i, trajectory in enumerate(trajectories):
                self._check_cancel()
                progress = None if group is None else {"group_id": group["id"], "block_id": group["blocks"][i]["id"],
                            "index": i+1, "total": len(trajectories), "phase": "transition", "remaining_s": 0}
                self._set_status(mode="replaying", group_playback=progress)
                target = self._play_trajectory_block(trajectory, speed)
                self._write_log({"kind": "block_complete", "slot": trajectory["slot"]})
                if group and i < len(trajectories)-1:
                    self._wait_between_blocks(target, group["interval_s"])
            self._check_cancel()
            self._position_command(target, self._playback_hold_scale)
            self._set_status(mode="holding", message="播放完成，保持使能；点击「关闭机械臂」回 Home 后失能")
            self._event("info", "拍摄组播放完成" if group else "动作播放完成")
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
        self._event("warn", "当前动作已停止，仍保持使能" if self.status.enabled else "机械臂未使能，启动已取消")
        return self.status.to_json()

    def emergency_disable(self) -> dict[str, Any]:
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

    def home_disable(self) -> dict[str, Any]:
        if not self.status.enabled:
            return self.status.to_json()
        self._join_worker()
        self.recording = None
        self._prepare_worker()
        self._set_status(mode="homing", recording_slot=None, recording_samples=0,
                         group_playback=None, message="正在返回 Home，完成后失能")
        self.thread = threading.Thread(target=self._home_and_disable, name="shooting-home", daemon=True)
        self.thread.start()
        return self.status.to_json()

    def _home_and_disable(self) -> None:
        try:
            start = self._read_positions(0)
            if start.shape != (6,) or not np.all(np.isfinite(start)):
                raise RuntimeError("关节反馈无效，未执行 Home")
            target = np.zeros(6)
            duration = max(4., 1.875*float(np.max(np.abs(start)))/.15)
            kp = np.array([50., 150., 150., 50., 50., 50.])
            kd = np.array([3., 10., 10., 5., 4., 4.])
            steps = math.ceil(duration*30)
            began = time.monotonic()
            for i in range(1, steps+1):
                self._check_cancel()
                s = i/steps
                q = start + (target-start)*(10*s**3-15*s**4+6*s**5)
                model, payload = self._estimate_payload(q)
                self._send_mit(q, model+payload, kp=kp, kd=kd)
                if i % 6 == 0:
                    actual = self._read_positions(0)
                    self._set_status(q_deg=np.rad2deg(actual).tolist())
                self.stop_requested.wait(max(0., began+duration*i/steps-time.monotonic()))
            self._check_cancel()
            # Do not disable a suspended arm after a failed return.
            actual = self._read_positions(0)
            if not self.dry_run and (not np.all(np.isfinite(actual)) or np.max(np.abs(np.rad2deg(actual))) > 3):
                raise RuntimeError("未可靠返回 Home，保持使能，请人工支撑并处理")
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
