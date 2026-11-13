"""Recording markers and timeline-triggered photography; never fire while recording."""
from __future__ import annotations

import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np

if __package__:
    from .shooting_groups import atomic_write_json
    from .shooting_control import MotionCancelled
else:
    from shooting_groups import atomic_write_json
    from shooting_control import MotionCancelled


class PhotoPointsControl:
    photo_result_timeout_s = 12.0

    @staticmethod
    def _point_summary(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [{k: p[k] for k in ("id", "sample_index", "t")} for p in points]

    def _load_photo_settings(self) -> None:
        self._photo_settings_path = self.log_dir / "camera_settings.json"
        self._photo_post_delay_s = 1.
        if self._photo_settings_path.exists():
            data = json.loads(self._photo_settings_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or data.get("version") != 1:
                raise ValueError("相机设置文件无效，原文件未改动")
            self._photo_post_delay_s = self._validate_post_delay(data.get("post_delay_s"))

    @staticmethod
    def _validate_post_delay(raw: Any) -> float:
        if type(raw) not in (int, float) or not math.isfinite(raw) or not 0 <= raw <= 10:
            raise ValueError("拍照后停留须为0–10秒的有限数值")
        return float(raw)

    def camera_command(self, action: str, data: dict[str, Any]) -> dict[str, Any]:
        self._editable_groups()
        if action == "settings":
            delay = self._validate_post_delay(data.get("post_delay_s"))
            atomic_write_json(self._photo_settings_path, {"version": 1, "post_delay_s": delay})
            self._photo_post_delay_s = delay
            self._set_status(camera={**self.camera.snapshot(), "post_delay_s": delay}, message="拍照停留已保存")
        else:
            self.camera.refresh()
            operation = self.camera.command(action)
            result = operation.snapshot()
            if operation.done.is_set() and not result["ok"]:
                raise RuntimeError(result["message"])
            self._set_status(camera={**self.camera.snapshot(), "post_delay_s": self._photo_post_delay_s},
                             message="相机操作已提交，请等待结果")
        return self.status.to_json()

    def mark_photo_point(self, data: dict[str, Any]) -> dict[str, Any]:
        self._require_started()
        point_mode = data.get("point_mode", False)
        if type(point_mode) is not bool:
            raise ValueError("点位模式参数无效")
        with self.lock:
            block = self._groups.resolve_block(data.get("group_id"), data.get("block_id"))
            if self.recording is None or self.recording["slot"] != block["slot"] or self.status.mode != "running":
                raise RuntimeError("仅能在当前动作录制中设置拍照点")
            if point_mode and (self.settings.camera_profile != "heavy" or self.recording["camera_profile"] != "heavy"):
                raise RuntimeError("点位拍照需要 950 克录制模式")
            if point_mode and not self.camera.snapshot()["ready"]:
                raise RuntimeError("请先连接相机并确认快门已就绪")
            if len(self.recording.get("photo_points", [])) + len(self._pending_photo_marks) >= 100:
                raise ValueError("每个动作最多100个拍照点")
            now = time.monotonic()
            if now - self._photo_last_mark_time < .25:
                raise RuntimeError("拍照点设置过于频繁，请松开空格后再按")
            self._photo_last_mark_time = now
            request = {"id": str(uuid.uuid4()), "recording": self.recording,
                       "event": threading.Event(), "point": None, "error": None, "cancelled": False,
                       "generation": self._cancel_generation}
            self._pending_photo_marks.append(request)
        request["event"].wait(2.)
        with self.lock:
            if request["point"] is None and not request["error"]:
                request["cancelled"] = True
                if request in self._pending_photo_marks:
                    self._pending_photo_marks.remove(request)
                raise RuntimeError("未收到新鲜关节采样，未添加拍照点")
            if request["error"]:
                raise RuntimeError(request["error"])
            if not point_mode:
                self._set_status(message="拍照点已记录，录制时不会拍照")
                return self.status.to_json()
            point = request["point"]
            key = f"teach:{block['slot']}:{point['id']}"
            if request["generation"] != self._cancel_generation or self.stop_requested.is_set():
                raise RuntimeError("拍照点已记录，快门因操作停止未发送")
            self._write_log({"kind": "teach_shutter_request", "key": key, "point_id": point["id"],
                             "slot": block["slot"], "sample_index": point["sample_index"],
                             "q_target_deg": np.rad2deg(point["q_rad"]).tolist()})
            operation = self.camera.start_shutter(key, self.stop_requested)
            self._set_status(message="六轴角度已记录，等待单次快门结果，请保持姿势")
        if not operation.done.wait(self.photo_result_timeout_s):
            operation.cancel()
            self._set_status(message="点位已记录，快门结果未知，请检查相机")
            raise RuntimeError("点位已记录，快门结果未知，请检查相机")
        result = operation.snapshot()
        self._write_log({"kind": "teach_shutter_result", "key": key,
                         "state": result["state"], "command_id": result.get("command_id")})
        if not result["ok"] or result["state"] != "completed":
            raise RuntimeError("点位已记录；" + (result.get("message") or "快门未完成"))
        self._set_status(message="六轴点位已记录，快门指令已完成")
        return self.status.to_json()

    def _capture_photo_marks(self, sample: dict[str, Any], index: int) -> None:
        # Called under self.lock by the sole control worker after appending a fresh sample.
        pending, self._pending_photo_marks = self._pending_photo_marks, []
        for request in pending:
            if request["cancelled"]:
                continue
            try:
                if self.recording is not request["recording"]:
                    raise RuntimeError("原动作已结束，拍照点未添加")
                if self.stop_requested.is_set() or self.emergency_requested.is_set() or request["generation"] != self._cancel_generation:
                    raise RuntimeError("操作已停止，拍照点未添加")
                self._assert_soft_limits(np.array(sample["q_rad"], dtype=float), label="拍照点")
                point = {"id": request["id"], "sample_index": index, "t": sample["t"],
                         "q_rad": list(sample["q_rad"])}
                self.recording.setdefault("photo_points", []).append(point)
                request["point"] = point
                self._set_status(message=f"已记录拍照点 {len(self.recording['photo_points'])}")
            except Exception as exc:
                request["error"] = str(exc)
            finally:
                request["event"].set()

    def _cancel_photo_marks(self, reason: str) -> None:
        with self.lock:
            for request in self._pending_photo_marks:
                request["cancelled"], request["error"] = True, reason
                request["event"].set()
            self._pending_photo_marks.clear()

    def delete_photo_point(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            block = self._groups.resolve_block(data.get("group_id"), data.get("block_id"))
            point_id = data.get("point_id")
            if self.recording is not None:
                if self.recording["slot"] != block["slot"]:
                    raise RuntimeError("只能修改正在录制的动作")
                points = self.recording.get("photo_points", [])
                if not any(p["id"] == point_id for p in points):
                    raise ValueError("拍照点不存在")
                self.recording["photo_points"] = [p for p in points if p["id"] != point_id]
            else:
                self._editable_groups()
                candidate = copy.deepcopy(self._trajectories)
                trajectory = next((t for t in candidate if t["slot"] == block["slot"]), None)
                if trajectory is None or not any(p["id"] == point_id for p in trajectory.get("photo_points", [])):
                    raise ValueError("拍照点不存在")
                trajectory["photo_points"] = [p for p in trajectory["photo_points"] if p["id"] != point_id]
                atomic_write_json(self.trajectory_file, {"updated_ms": int(time.time()*1000), "trajectories": candidate})
                self._trajectories = candidate
                self.status.trajectories = self._trajectory_summaries()
            self._set_status(message="拍照点已删除")
        return self.status.to_json()

    def _validate_photo_points(self, trajectory: dict[str, Any]) -> None:
        points = trajectory.get("photo_points", [])
        if not isinstance(points, list) or len(points) > 100:
            raise ValueError("拍照点数据无效")
        ids, previous = set(), -1
        for point in points:
            if not isinstance(point, dict):
                raise ValueError("拍照点格式无效")
            ident, index = point.get("id"), point.get("sample_index")
            try:
                valid_id = isinstance(ident, str) and str(uuid.UUID(ident)) == ident
            except ValueError:
                valid_id = False
            if not valid_id or ident in ids or type(index) is not int or not 0 <= index < len(trajectory["samples"]) or index < previous:
                raise ValueError("拍照点ID或采样索引无效")
            sample = trajectory["samples"][index]
            q = np.asarray(point.get("q_rad"), dtype=float)
            self._assert_soft_limits(q, label="拍照点")
            t = point.get("t")
            if type(t) not in (float, int) or not math.isfinite(t) or abs(t-float(sample["t"])) > .001 or not np.allclose(q, sample["q_rad"], atol=1e-8, rtol=0):
                raise ValueError("拍照点与录制采样不一致")
            ids.add(ident)
            previous = index

    def _photo_camera_preflight(self, trajectories: list[dict[str, Any]]) -> None:
        if any(t.get("photo_points") for t in trajectories):
            camera = self.camera.refresh()
            if not camera["ready"]:
                raise RuntimeError("动作包含拍照点，请先连接并确认相机就绪：" + (camera.get("error") or "相机未就绪/忙碌或结果待核对"))

    def _photo_hold_sample(self, target: np.ndarray, scale: np.ndarray, *, j3_probe_nm: float = 0.) -> np.ndarray:
        self._photo_check_cancel()
        if j3_probe_nm:
            self._position_command(target, scale, j3_probe_nm=j3_probe_nm)
        else:
            self._position_command(target, scale)
        # Keep the normal motion tracking/feedback checks; photography adds no
        # tighter position or stability gate in this flow-validation mode.
        actual = self._tracking_check(target)
        context = getattr(self, "_photo_diagnostic_context", None)
        if context is not None:
            self._write_log({"kind": "photo_hold_sample", **context,
                             "elapsed_s": time.monotonic() - self._photo_diagnostic_started,
                             "dry_run": self.dry_run,
                             "j3_probe_nm": j3_probe_nm,
                             **getattr(self, "_position_diagnostics", {}),
                             "q_target_deg": np.rad2deg(target).tolist(),
                             "q_actual_deg": np.rad2deg(actual).tolist(),
                             "error_actual_minus_target_deg": np.rad2deg(actual-target).tolist()})
        return actual

    def _hold_precision_point(self, trajectory, point, scale, *, tolerance=.5):
        """Hold the original taught point and validate measured joint error."""
        target = np.asarray(point["q_rad"], dtype=float)
        command = target.copy()
        offset_deg = np.zeros(6)
        started = previous = time.monotonic()
        last_status = -math.inf
        stable_since = None
        self._photo_diagnostic_started = started
        self._photo_diagnostic_context = {
            "slot": trajectory["slot"], "point_id": point["id"],
            "sample_index": point["sample_index"], "phase": "precision_hold",
            "precision_reference_deg": np.rad2deg(target).tolist(),
            "tolerance_deg": tolerance,
        }
        try:
            while True:
                now = time.monotonic()
                gap = now - previous
                if gap > .25 and not self.dry_run:
                    raise RuntimeError("精度测试反馈更新超过 250 毫秒")
                previous = now
                actual = self._photo_hold_sample(command, scale)
                error = np.rad2deg(target - actual)
                if np.max(np.abs(error)) > 5.:
                    details = "、".join(f"J{i+1} {e:+.2f}°" for i, e in enumerate(-error) if abs(e) > 5.)
                    raise RuntimeError("与原打点偏差超过 5°，已停止：" + details)
                within = np.max(np.abs(error)) <= tolerance
                if within:
                    if stable_since is None:
                        stable_since = now
                else:
                    stable_since = None
                if now - last_status >= .2:
                    self._set_status(q_deg=np.rad2deg(actual).tolist(),
                                     photo_capture={"precision_reference_deg": np.rad2deg(target).tolist(),
                                                    "tolerance_deg": tolerance},
                                     message=f"正在核对六轴 ±{tolerance:g}° 持续 2 秒")
                    last_status = now
                if stable_since is not None and now - stable_since >= 2.:
                    self._photo_check_cancel()
                    self._write_log({"kind": "precision_point_result", **self._photo_diagnostic_context,
                                     "passed": True, "error_deg": (-error).tolist()})
                    return command
                if now - started >= 20.:
                    self._write_log({"kind": "precision_point_result", **self._photo_diagnostic_context,
                                     "passed": False, "error_deg": (-error).tolist()})
                    details = "、".join(f"J{i+1} {e:+.2f}°" for i, e in enumerate(-error)
                                      if abs(e) > tolerance)
                    raise RuntimeError(f"原打点未达到 ±{tolerance:g}° 持续 2 秒：" + (details or "稳定时间不足"))
                if now - started >= 1. and not within:
                    # Limit both total offset and rate. Preserve the saved pose
                    # as the measurement reference throughout the diagnostic.
                    correction = np.where(np.abs(error) > tolerance, error, 0.)
                    offset_deg = np.clip(offset_deg + np.clip(correction*.25, -.3, .3)*min(gap, .05), -3., 3.)
                    command = target + np.deg2rad(offset_deg)
                    self._assert_soft_limits(command, label="精度微调目标")
                self.stop_requested.wait(max(0., now+1/30-time.monotonic()))
        finally:
            self._photo_diagnostic_context = None

    def _photo_check_cancel(self) -> None:
        self._check_cancel()
        if self._photo_run_cancel.is_set():
            raise MotionCancelled()

    def _perform_photo_point(self, trajectory: dict[str, Any], point: dict[str, Any],
                             index: int, scale: np.ndarray, *, hold_command=None, tolerance=.5, post_delay=None) -> None:
        target = np.array(point["q_rad"], dtype=float)
        command = target if hold_command is None else hold_command

        def sample_hold():
            actual = self._photo_hold_sample(command, scale)
            if hold_command is not None and np.max(np.abs(np.rad2deg(actual-target))) > tolerance:
                raise RuntimeError(f"六轴离开 ±{tolerance:g}° 到位范围，点位回放已停止")

        info = {"point_id": point["id"], "index": index,
                "total": len(trajectory["photo_points"]), "phase": "triggering", "message": "已执行到拍照点，发送单次快门"}
        if hold_command is not None:
            info.update(tolerance_deg=tolerance, precision_reference_deg=np.rad2deg(target).tolist())
        operation = None
        self._photo_diagnostic_started = time.monotonic()
        self._photo_diagnostic_context = {"slot": trajectory["slot"], "point_id": point["id"],
                                          "sample_index": point["sample_index"], "phase": "arrival"}
        try:
            self._set_status(photo_capture=info, message="已执行到拍照点")
            sample_hold()
            self._photo_check_cancel()
            # Each run has its own never-reused cancellation event and identity.
            key = f"{self._photo_run_nonce}:{trajectory['slot']}:{point['id']}"
            self._set_status(photo_capture={**info, "phase": "triggering", "message": "发送单次快门"})
            self._photo_check_cancel()
            self._write_log({"kind": "photo_shutter_request", **self._photo_diagnostic_context,
                             "key": key, "elapsed_s": time.monotonic()-self._photo_diagnostic_started})
            self._photo_check_cancel()
            operation = self.camera.start_shutter(key, self._photo_run_cancel)
            self._photo_diagnostic_context["phase"] = "waiting_result"
            end = time.monotonic() + self.photo_result_timeout_s
            self._set_status(photo_capture={**info, "phase": "waiting_result", "message": "保持位置，等待快门指令结果"})
            while not operation.done.is_set():
                sample_hold()
                if time.monotonic() >= end:
                    operation.cancel()
                    raise RuntimeError("快门结果未知，未自动重试，请在相机上核对")
                self.stop_requested.wait(1/30)
            self._photo_check_cancel()
            result = operation.snapshot()
            if not result["ok"] or result["state"] != "completed":
                raise RuntimeError(result["message"] or "快门结果未知，未重试")
            self._write_log({"kind": "photo_shutter_result", **self._photo_diagnostic_context,
                             "command_id": result.get("command_id"), "state": result["state"],
                             "elapsed_s": time.monotonic()-self._photo_diagnostic_started})
            self._photo_diagnostic_context["phase"] = "post_delay"
            self._set_status(photo_capture={**info, "phase": "post_delay", "message": "快门指令已完成，保持拍照停留"})
            delay_end = time.monotonic() + (self._photo_run_post_delay_s if post_delay is None else post_delay)
            while time.monotonic() < delay_end:
                sample_hold()
                self.stop_requested.wait(min(1/30, max(0., delay_end-time.monotonic())))
            self._photo_check_cancel()
            self._write_log({"kind": "photo_point_result", "point_id": point["id"], "key": key,
                             "command_id": result.get("command_id"), "state": "completed",
                             "elapsed_s": time.monotonic()-self._photo_diagnostic_started})
            self._set_status(photo_capture={**info, "phase": "complete", "message": "快门指令完成（照片请在相机上核对）"})
        except Exception as exc:
            if operation is not None and not operation.done.is_set():
                operation.cancel()
            self._set_status(photo_capture={**info, "phase": "error", "message": "操作已停止" if isinstance(exc, MotionCancelled) else str(exc)})
            self._write_log({"kind": "photo_point_error", **self._photo_diagnostic_context,
                             "error": str(exc), "elapsed_s": time.monotonic()-self._photo_diagnostic_started})
            raise
        finally:
            self._photo_diagnostic_context = None
