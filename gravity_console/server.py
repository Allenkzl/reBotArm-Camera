#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import queue
import shutil
import sys
import tempfile
import threading
import time
import traceback
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

if __package__:
    from .shooting_groups import ShootingGroupStore, atomic_write_json
    from .shooting_control import ShootingControl, MotionCancelled
else:
    from shooting_groups import ShootingGroupStore, atomic_write_json
    from shooting_control import ShootingControl, MotionCancelled


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
BASE_JOINT_SCALE = np.array([1.0, 0.98, 0.98, 1.0, 1.0, 1.0], dtype=float)
JOINT_NAMES = [f"J{i}" for i in range(1, 7)]
DEFAULT_JOINT_PERCENT = np.array([80.0, 80.0, 80.0, 80.0, 80.0, 80.0], dtype=float)
COMMAND_CLAMP_NM = np.array([1.0, 9.0, 12.0, 5.0, 0.8, 0.8], dtype=float)
LOOSE_KP = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1], dtype=float)
LOOSE_KD = np.array([0.35, 0.35, 0.35, 0.35, 0.25, 0.25], dtype=float)
REPLAY_KP = np.array([8.0, 30.0, 35.0, 22.0, 5.0, 5.0], dtype=float)
REPLAY_KD = np.array([0.8, 2.0, 2.4, 1.8, 0.4, 0.4], dtype=float)
# Supervised camera tests: J2 max +100 deg, J4 max +80 deg, J6 min -70 deg.
# Software replay bounds, not calibrated physical/collision limits.
SOFT_MIN_DEG = np.array([-55.0, -8.0, -1.5, -70.0, -35.0, -70.0], dtype=float)
SOFT_MAX_DEG = np.array([55.0, 100.0, 80.0, 80.0, 35.0, 90.0], dtype=float)
RECORD_EVERY_N = 2
DEFAULT_CAMERA_PROFILE = "heavy"
CAMERA_PROFILES = {
    "light": {
        "label": "Light Camera",
        "payload_kg": 0.0,
        "payload_offset_local_m": [0.06, 0.0, 0.0],
        "note": "Original URDF/gripper-equivalent compensation",
    },
    "heavy": {
        "label": "Heavy Camera (950 g)",
        "total_camera_kg": 0.95,
        # Increment above the existing approximate 800 g URDF-equivalent baseline.
        # This is a weight correction, not an identified camera/COM model.
        "payload_kg": 0.15,
        "payload_offset_local_m": [0.075, 0.0, -0.025],
        "note": "950g total camera: adds about 150g beyond the approximate 800g light/URDF baseline; COM estimate unchanged",
    },
}
MAX_PERCENT_DEFAULT = 200.0
MAX_TRAJECTORY_SLOTS = 400


def now_ms() -> int:
    return int(time.time() * 1000)


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(float(value), lo), hi)


def deg(rad: np.ndarray) -> list[float]:
    return [math.degrees(float(x)) for x in rad]


def minimum_jerk(s: float) -> float:
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def make_j1j6_config(path: Path) -> None:
    path.write_text(
        """
urdf_path: "urdf/RS/urdf/ReBot_Arm_RS.urdf"
end_effector_frame: "gripper_end"
arm_control_mode: "mit"
channel: can0
rate: 200
groups:
  arm:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
joints:
  - name: joint1
    motor_id: 0x01
    feedback_id: 0xFD
    model: "rs-06"
    vendor: "robstride"
    MIT: {kp: 50.0, kd: 3.0}
  - name: joint2
    motor_id: 0x02
    feedback_id: 0xFD
    model: "rs-06"
    vendor: "robstride"
    MIT: {kp: 150.0, kd: 10.0}
  - name: joint3
    motor_id: 0x03
    feedback_id: 0xFD
    model: "rs-06"
    vendor: "robstride"
    MIT: {kp: 150.0, kd: 10.0}
  - name: joint4
    motor_id: 0x04
    feedback_id: 0xFD
    model: "rs-00"
    vendor: "robstride"
    MIT: {kp: 50.0, kd: 5.0}
  - name: joint5
    motor_id: 0x05
    feedback_id: 0xFD
    model: "rs-00"
    vendor: "robstride"
    MIT: {kp: 50.0, kd: 4.0}
  - name: joint6
    motor_id: 0x06
    feedback_id: 0xFD
    model: "rs-00"
    vendor: "robstride"
    MIT: {kp: 50.0, kd: 4.0}
""".strip()
        + "\n",
        encoding="utf-8",
    )


@dataclass
class Settings:
    joint_percent: list[float] = field(default_factory=lambda: DEFAULT_JOINT_PERCENT.tolist())
    camera_profile: str = DEFAULT_CAMERA_PROFILE


@dataclass
class RobotStatus:
    mode: str = "idle"
    run_id: str | None = None
    log_path: str | None = None
    connected: bool = False
    enabled: bool = False
    dry_run: bool = False
    base_percent: float = 80.0
    target_percent: float = 80.0
    joint_percent: list[float] = field(default_factory=lambda: DEFAULT_JOINT_PERCENT.tolist())
    camera_profile: str = DEFAULT_CAMERA_PROFILE
    effective_scale: list[float] = field(default_factory=lambda: (DEFAULT_JOINT_PERCENT / 100.0).tolist())
    q_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_model_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_payload_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_total_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_cmd_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    presets: list[dict[str, Any]] = field(default_factory=list)
    trajectories: list[dict[str, Any]] = field(default_factory=list)
    recording_slot: int | None = None
    recording_samples: int = 0
    shooting_groups: list[dict[str, Any]] = field(default_factory=list)
    group_playback: dict[str, Any] | None = None
    message: str = "ready"
    updated_ms: int = field(default_factory=now_ms)
    revision: int = 0

    def to_json(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "run_id": self.run_id,
            "log_path": self.log_path,
            "connected": self.connected,
            "enabled": self.enabled,
            "dry_run": self.dry_run,
            "base_percent": self.base_percent,
            "target_percent": self.target_percent,
            "joint_percent": self.joint_percent,
            "camera_profile": self.camera_profile,
            "camera_profiles": CAMERA_PROFILES,
            "effective_scale": self.effective_scale,
            "q_deg": self.q_deg,
            "tau_model_nm": self.tau_model_nm,
            "tau_payload_nm": self.tau_payload_nm,
            "tau_total_nm": self.tau_total_nm,
            "tau_cmd_nm": self.tau_cmd_nm,
            "presets": self.presets,
            "trajectories": self.trajectories,
            "recording_slot": self.recording_slot,
            "recording_samples": self.recording_samples,
            "shooting_groups": self.shooting_groups,
            "group_playback": self.group_playback,
            "rated_camera_kg": 0.95,
            "message": self.message,
            "updated_ms": self.updated_ms,
            "revision": self.revision,
        }


class GravityConsole(ShootingControl):
    command_clamp_nm = COMMAND_CLAMP_NM
    replay_kp = REPLAY_KP
    replay_kd = REPLAY_KD

    def __init__(
        self,
        *,
        robot_root: Path,
        log_dir: Path,
        dry_run: bool,
        max_percent: float,
        preset_file: Path | None,
        trajectory_file: Path | None,
        group_file: Path | None = None,
    ) -> None:
        self.robot_root = robot_root
        self.log_dir = log_dir
        self.preset_file = preset_file or (log_dir / "gravity_presets.json")
        self.trajectory_file = trajectory_file or (log_dir / "teach_trajectories.json")
        self.dry_run = dry_run
        self.max_percent = max_percent
        self.status = RobotStatus(dry_run=dry_run)
        self.settings = Settings()
        self.recording: dict[str, Any] | None = None
        self._trajectories: list[dict[str, Any]] = []
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.motor_lock = threading.RLock()
        self.emergency_requested = threading.Event()
        self.events: list[dict[str, Any]] = []
        self.subscribers: list[queue.Queue[dict[str, Any]]] = []
        self.thread: threading.Thread | None = None
        self.stop_requested = threading.Event()
        self.robot = None
        self._robot_modules_loaded = False
        self._model = None
        self._frame_id = None
        self._temp_dir = Path(tempfile.mkdtemp(prefix="rebot-camera-console-"))
        self._temp_cfg = self._temp_dir / "rebotarm_camera_j1j6.yaml"
        make_j1j6_config(self._temp_cfg)
        self._log_file = None
        self._load_presets()
        self._load_trajectories()
        self._groups = ShootingGroupStore(group_file or log_dir / "shooting_groups.json", self._trajectories)
        self._set_status(message="准备就绪，请启动机械臂")

    def close(self) -> None:
        self.stop_requested.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def _load_robot_modules(self) -> None:
        if self._robot_modules_loaded or self.dry_run:
            return
        root = str(self.robot_root)
        if root not in sys.path:
            sys.path.insert(0, root)
        global pin
        import pinocchio as pin  # type: ignore
        from reBotArm_control_py.actuator import RebotArm  # noqa: F401
        from reBotArm_control_py.dynamics.inverse_dynamics import (  # noqa: F401
            compute_generalized_gravity,
        )
        from reBotArm_control_py.kinematics.robot_model import (  # noqa: F401
            get_end_effector_frame_id,
            load_robot_model,
            pad_q_for_model,
        )
        self._robot_modules_loaded = True

    def subscribe(self) -> queue.Queue[dict[str, Any]]:
        q: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=50)
        with self.lock:
            self.subscribers.append(q)
            q.put({"type": "status", "status": self.status.to_json()})
            for event in self.events[-20:]:
                q.put({"type": "event", "event": event})
        return q

    def unsubscribe(self, q: queue.Queue[dict[str, Any]]) -> None:
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def _publish(self, payload: dict[str, Any]) -> None:
        dead: list[queue.Queue[dict[str, Any]]] = []
        with self.lock:
            for q in self.subscribers:
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                if q in self.subscribers:
                    self.subscribers.remove(q)

    def _event(self, level: str, message: str, **extra: Any) -> None:
        event = {"ts": time.strftime("%H:%M:%S"), "level": level, "message": message, **extra}
        with self.lock:
            self.events.append(event)
            self.events = self.events[-200:]
        try:
            print(json.dumps({"event": event}, ensure_ascii=False), flush=True)
        except (BrokenPipeError, OSError):
            pass
        self._publish({"type": "event", "event": event})
        self._write_log({"kind": "event", **event})

    def _set_status(self, **updates: Any) -> None:
        with self.lock:
            for key, value in updates.items():
                setattr(self.status, key, value)
            if hasattr(self, "_groups"):
                self.status.shooting_groups = self._groups.snapshot(self._trajectory_summaries())
            self.status.updated_ms = now_ms()
            self.status.revision += 1
            snapshot = self.status.to_json()
        self._publish({"type": "status", "status": snapshot})

    def _write_log(self, record: dict[str, Any]) -> None:
        if self._log_file is None:
            return
        self._log_file.write(json.dumps({"t_wall": time.time(), **record}, ensure_ascii=False) + "\n")
        self._log_file.flush()

    def _open_run_log(self, prefix: str) -> None:
        if self._log_file is not None:
            self._log_file.close()
        run_id = f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns() % 1000000:06d}"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir / f"{run_id}.jsonl"
        self._log_file = log_path.open("w", buffering=1, encoding="utf-8")
        self._set_status(run_id=run_id, log_path=str(log_path))

    def _load_presets(self) -> None:
        try:
            if self.preset_file.exists():
                raw = json.loads(self.preset_file.read_text(encoding="utf-8"))
                presets = raw.get("presets", raw if isinstance(raw, list) else [])
                if isinstance(presets, list):
                    self.status.presets = presets
        except Exception as exc:
            self._event("warn", f"preset load failed: {exc!r}")

    def _save_presets_file(self) -> None:
        self.preset_file.parent.mkdir(parents=True, exist_ok=True)
        self.preset_file.write_text(
            json.dumps({"updated_ms": now_ms(), "presets": self.status.presets}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def _load_trajectories(self) -> None:
        if not self.trajectory_file.exists():
            return
        raw = json.loads(self.trajectory_file.read_text(encoding="utf-8"))
        trajectories = raw.get("trajectories") if isinstance(raw, dict) else raw
        if not isinstance(trajectories, list) or any(not isinstance(t, dict) for t in trajectories):
            raise ValueError("轨迹文件无效，原文件未被覆盖")
        self._trajectories = trajectories
        self.status.trajectories = self._trajectory_summaries()

    def _save_trajectories_file(self) -> None:
        atomic_write_json(self.trajectory_file, {"updated_ms": now_ms(), "trajectories": self._trajectories})

    def _trajectory_summaries(self) -> list[dict[str, Any]]:
        summaries: list[dict[str, Any]] = []
        for traj in self._trajectories:
            samples = traj.get("samples") if isinstance(traj, dict) else None
            summaries.append(
                {
                    "slot": traj.get("slot"),
                    "label": traj.get("label"),
                    "saved_ms": traj.get("saved_ms"),
                    "duration_s": traj.get("duration_s"),
                    "sample_count": len(samples) if isinstance(samples, list) else traj.get("sample_count", 0),
                    "camera_profile": traj.get("camera_profile", DEFAULT_CAMERA_PROFILE),
                    "start_q_deg": traj.get("start_q_deg"),
                    "end_q_deg": traj.get("end_q_deg"),
                }
            )
        summaries.sort(key=lambda item: int(item.get("slot") or 99))
        return summaries

    def _validated_settings(self, data: dict[str, Any]) -> Settings:
        raw = data.get("joint_percent", self.settings.joint_percent)
        if not isinstance(raw, list) or len(raw) != 6:
            raise ValueError("joint_percent must be a 6-element list")
        if not all(math.isfinite(float(x)) for x in raw):
            raise ValueError("补偿倍率必须是有限数值")
        profile = str(data.get("camera_profile", self.settings.camera_profile) or DEFAULT_CAMERA_PROFILE)
        if profile not in CAMERA_PROFILES:
            raise ValueError(f"camera_profile must be one of {', '.join(CAMERA_PROFILES)}")
        return Settings(joint_percent=[clamp(float(x), 0.0, self.max_percent) for x in raw], camera_profile=profile)

    def _effective_scale(self, settings: Settings) -> np.ndarray:
        return BASE_JOINT_SCALE * (np.array(settings.joint_percent, dtype=float) / 100.0)

    def _camera_profile(self) -> dict[str, Any]:
        with self.lock:
            name = self.settings.camera_profile
        profile = CAMERA_PROFILES.get(name, CAMERA_PROFILES[DEFAULT_CAMERA_PROFILE])
        return {"name": name, **profile}

    def _assert_soft_limits(self, q: np.ndarray, *, label: str) -> None:
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{label}: 需要六个有效关节角度")
        q_deg = np.array(deg(q), dtype=float)
        bad = np.where((q_deg < SOFT_MIN_DEG) | (q_deg > SOFT_MAX_DEG))[0]
        if len(bad) > 0:
            names = ", ".join(f"{JOINT_NAMES[i]}={q_deg[i]:.1f}deg" for i in bad)
            raise RuntimeError(f"{label} outside soft limits: {names}")

    def _assert_trajectory_soft_limits(self, trajectory: dict[str, Any]) -> None:
        label = str(trajectory.get("label") or f"Pose {trajectory.get('slot', '?')}")
        for sample in trajectory.get("samples") or []:
            self._assert_soft_limits(np.array(sample["q_rad"], dtype=float), label=label)

    def _validated_slot(self, data: dict[str, Any]) -> int:
        slot = int(data.get("slot", 0))
        if slot < 1 or slot > MAX_TRAJECTORY_SLOTS:
            raise ValueError(f"slot must be 1-{MAX_TRAJECTORY_SLOTS}")
        return slot

    def start(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if self.status.enabled or self.status.mode in ("starting", "homing"):
                raise RuntimeError("机械臂已经启动或正在关闭")
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError("gravity compensation is already running")
            self.settings = self._validated_settings(data)
            with self.motor_lock:
                self.emergency_requested.clear()
            self.stop_requested.clear()
            run_id = time.strftime("gc_console_%Y%m%d_%H%M%S")
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log_path = self.log_dir / f"{run_id}.jsonl"
            self._log_file = log_path.open("w", buffering=1, encoding="utf-8")
            self._set_status(
                mode="starting",
                run_id=run_id,
                log_path=str(log_path),
                base_percent=float(np.mean(self.settings.joint_percent)),
                target_percent=float(np.mean(self.settings.joint_percent)),
                joint_percent=list(self.settings.joint_percent),
                camera_profile=self.settings.camera_profile,
                effective_scale=self._effective_scale(self.settings).tolist(),
                presets=self.status.presets,
                trajectories=self.status.trajectories,
                message="starting J1-J6 joint gravity tuning",
            )
            self._write_log(
                {
                    "kind": "start",
                    "run_id": run_id,
                    "dry_run": self.dry_run,
                    "settings": {
                        "joint_percent": self.settings.joint_percent,
                        "camera_profile": self.settings.camera_profile,
                    },
                    "max_percent": self.max_percent,
                    "camera_profiles": CAMERA_PROFILES,
                    "command_clamp_nm": COMMAND_CLAMP_NM.tolist(),
                    "kp": LOOSE_KP.tolist(),
                    "kd": LOOSE_KD.tolist(),
                }
            )
            self.thread = threading.Thread(target=self._run_loop, kwargs={"enable": True}, name="gravity-console-loop", daemon=True)
            self.thread.start()
        self._event("info", f"started joint tuning {self.settings.joint_percent}")
        return self.status.to_json()

    def apply(self, data: dict[str, Any]) -> dict[str, Any]:
        self._editable_groups()
        settings = self._validated_settings(data)
        with self.lock:
            self.settings = settings
        self._set_status(
            base_percent=float(np.mean(settings.joint_percent)),
            target_percent=float(np.mean(settings.joint_percent)),
            joint_percent=list(settings.joint_percent),
            camera_profile=settings.camera_profile,
            message=f"applied {CAMERA_PROFILES[settings.camera_profile]['label']}",
        )
        self._event("info", "apply settings", joint_percent=settings.joint_percent, camera_profile=settings.camera_profile)
        return self.status.to_json()

    def save_preset(self, data: dict[str, Any]) -> dict[str, Any]:
        slot = int(data.get("slot", 0))
        if slot < 1 or slot > 3:
            raise ValueError("slot must be 1, 2, or 3")
        label = str(data.get("label") or f"Pose {slot}")
        note = str(data.get("note") or "")
        with self.lock:
            status = self.status.to_json()
            preset = {
                "slot": slot,
                "label": label,
                "note": note,
                "saved_ms": now_ms(),
                "camera_profile": status["camera_profile"],
                "q_deg": status["q_deg"],
                "joint_percent": status["joint_percent"],
                "effective_scale": status["effective_scale"],
                "tau_model_nm": status["tau_model_nm"],
                "tau_cmd_nm": status["tau_cmd_nm"],
                "run_id": status["run_id"],
            }
            presets = [p for p in self.status.presets if int(p.get("slot", -1)) != slot]
            presets.append(preset)
            presets.sort(key=lambda p: int(p.get("slot", 99)))
            self.status.presets = presets
            snapshot = self.status.to_json()
        self._save_presets_file()
        self._event("info", f"saved {label}", slot=slot, preset_file=str(self.preset_file))
        self._publish({"type": "status", "status": snapshot})
        return self.status.to_json()

    def apply_preset(self, data: dict[str, Any]) -> dict[str, Any]:
        slot = int(data.get("slot", 0))
        preset = next((p for p in self.status.presets if int(p.get("slot", -1)) == slot), None)
        if preset is None:
            raise ValueError(f"Pose {slot} is not saved")
        self.settings = self._validated_settings(
            {
                "joint_percent": preset["joint_percent"],
                "camera_profile": preset.get("camera_profile", self.settings.camera_profile),
            }
        )
        self._set_status(
            base_percent=float(np.mean(self.settings.joint_percent)),
            target_percent=float(np.mean(self.settings.joint_percent)),
            joint_percent=list(self.settings.joint_percent),
            camera_profile=self.settings.camera_profile,
            message=f"applied {preset.get('label', f'Pose {slot}')} compensation",
        )
        self._event("info", f"applied {preset.get('label', f'Pose {slot}')}", slot=slot)
        return self.status.to_json()

    def start_recording(self, data: dict[str, Any]) -> dict[str, Any]:
        self._require_started()
        slot = self._validated_slot(data)
        label = str(data.get("label") or f"Pose {slot}")
        with self.lock:
            if self.recording is not None:
                raise RuntimeError(f"already recording Pose {self.recording.get('slot')}")
            if self.thread is not None and self.thread.is_alive() and self.status.mode != "running":
                raise RuntimeError("wait for current motion to finish before recording")
            self.settings = self._validated_settings(data)
            if self.thread is None or not self.thread.is_alive():
                self.stop_requested.clear()
                run_id = time.strftime("gc_console_%Y%m%d_%H%M%S")
                self.log_dir.mkdir(parents=True, exist_ok=True)
                log_path = self.log_dir / f"{run_id}.jsonl"
                self._log_file = log_path.open("w", buffering=1, encoding="utf-8")
                self._set_status(
                    mode="starting",
                    run_id=run_id,
                    log_path=str(log_path),
                    base_percent=float(np.mean(self.settings.joint_percent)),
                    target_percent=float(np.mean(self.settings.joint_percent)),
                    joint_percent=list(self.settings.joint_percent),
                    camera_profile=self.settings.camera_profile,
                    effective_scale=self._effective_scale(self.settings).tolist(),
                    trajectories=self.status.trajectories,
                    message=f"starting live tuning for {label}",
                )
                self._write_log(
                    {
                        "kind": "start_for_recording",
                        "run_id": run_id,
                        "slot": slot,
                        "label": label,
                        "dry_run": self.dry_run,
                        "settings": {
                            "joint_percent": self.settings.joint_percent,
                            "camera_profile": self.settings.camera_profile,
                        },
                    }
                )
                self.thread = threading.Thread(target=self._run_loop, name="gravity-console-loop", daemon=True)
            self.recording = {
                "slot": slot,
                "label": label,
                "started_ms": now_ms(),
                "run_id": self.status.run_id,
                "camera_profile": self.settings.camera_profile,
                "samples": [],
            }
            if not self.thread.is_alive():
                self.thread.start()
        self._set_status(recording_slot=slot, recording_samples=0, message=f"recording {label}")
        self._event("info", f"recording started: {label}", slot=slot)
        return self.status.to_json()

    def stop_recording(self, data: dict[str, Any]) -> dict[str, Any]:
        slot = int(data.get("slot", 0) or 0)
        with self.lock:
            if self.recording is None:
                raise RuntimeError("not recording")
            rec = self.recording
            if slot and int(rec.get("slot", -1)) != slot:
                raise RuntimeError(f"currently recording Pose {rec.get('slot')}")
            self.recording = None
        samples = rec.get("samples") or []
        if len(samples) < 8:
            self._set_status(recording_slot=None, recording_samples=0, message="recording discarded; too few samples")
            raise RuntimeError("recording is too short; keep it running for at least one second")
        duration_s = float(samples[-1]["t"] - samples[0]["t"])
        if duration_s <= 0:
            duration_s = 0.0
        trajectory = {
            "slot": int(rec["slot"]),
            "label": rec["label"],
            "saved_ms": now_ms(),
            "run_id": rec.get("run_id"),
            "camera_profile": rec.get("camera_profile", DEFAULT_CAMERA_PROFILE),
            "duration_s": duration_s,
            "sample_count": len(samples),
            "start_q_deg": samples[0]["q_deg"],
            "end_q_deg": samples[-1]["q_deg"],
            "samples": samples,
        }
        try:
            self._validate_trajectory(trajectory)
        except Exception:
            self._set_status(recording_slot=None, recording_samples=0,
                             message="录制轨迹无效或超出软件范围，原动作已保留")
            raise
        with self.lock:
            candidate = [t for t in self._trajectories if int(t.get("slot", -1)) != int(rec["slot"])] + [trajectory]
            candidate.sort(key=lambda t: int(t.get("slot", 99)))
            try:
                atomic_write_json(self.trajectory_file, {"updated_ms": now_ms(), "trajectories": candidate})
            except Exception:
                self._set_status(recording_slot=None, recording_samples=0,
                                 message="录制保存失败，原动作保留，请检查磁盘后重试")
                raise
            self._trajectories = candidate
            self.status.trajectories = self._trajectory_summaries()
            snapshot = self.status.to_json()
        self._set_status(
            recording_slot=None,
            recording_samples=0,
            trajectories=self.status.trajectories,
            message=f"saved {rec['label']} trajectory",
        )
        self._event(
            "info",
            f"saved {rec['label']} trajectory",
            slot=rec["slot"],
            sample_count=len(samples),
            duration_s=round(duration_s, 2),
            trajectory_file=str(self.trajectory_file),
        )
        return self.status.to_json()

    def reset_trajectory(self, data: dict[str, Any]) -> dict[str, Any]:
        self._editable_groups()
        slot = self._validated_slot(data)
        with self.lock:
            if self.recording is not None and int(self.recording.get("slot", -1)) == slot:
                self.recording = None
            before = len(self._trajectories)
            self._trajectories = [t for t in self._trajectories if int(t.get("slot", -1)) != slot]
            self.status.trajectories = self._trajectory_summaries()
        self._save_trajectories_file()
        self._set_status(
            trajectories=self.status.trajectories,
            recording_slot=None if self.status.recording_slot == slot else self.status.recording_slot,
            recording_samples=0 if self.status.recording_slot == slot else self.status.recording_samples,
            message=f"reset Pose {slot} trajectory",
        )
        level = "info" if before != len(self._trajectories) else "warn"
        self._event(level, f"reset Pose {slot} trajectory", slot=slot, trajectory_file=str(self.trajectory_file))
        return self.status.to_json()

    def _ensure_robot(self) -> Any:
        if self.dry_run:
            return None
        self._load_robot_modules()
        from reBotArm_control_py.actuator import RebotArm
        return RebotArm(hw_yaml=str(self._temp_cfg))

    def _shutdown_robot(self, robot: Any) -> None:
        if robot is None or self.dry_run:
            return
        for ctrl in getattr(robot, "_ctrl_map", {}).values():
            try:
                ctrl.shutdown()
                time.sleep(0.1)
                ctrl.close()
            except Exception as exc:
                self._event("warn", f"controller close failed: {exc!r}")

    def _ensure_model(self) -> None:
        if self.dry_run:
            return
        if self._model is not None:
            return
        self._load_robot_modules()
        from reBotArm_control_py.kinematics.robot_model import get_end_effector_frame_id, load_robot_model
        self._model = load_robot_model()
        self._frame_id = get_end_effector_frame_id(self._model)

    def _estimate_payload(self, q6: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        profile = self._camera_profile()
        payload_kg = float(profile["payload_kg"])
        payload_offset = np.array(profile["payload_offset_local_m"], dtype=float)
        if self.dry_run:
            model_tau = np.array([0.0, 1.4, 6.7, 2.0, 0.0, 0.0], dtype=float)
            payload = np.array([0.0, payload_kg * 0.6, payload_kg * 2.5, payload_kg * 0.7, 0.0, 0.0], dtype=float)
            return model_tau, payload
        self._ensure_model()
        from reBotArm_control_py.dynamics.inverse_dynamics import compute_generalized_gravity
        from reBotArm_control_py.kinematics.robot_model import pad_q_for_model
        assert self._model is not None
        assert self._frame_id is not None
        q_model = pad_q_for_model(self._model, q6, controlled_joints=6)
        data = self._model.createData()
        tau_model = compute_generalized_gravity(model=self._model, q=q_model, data=data)[:6]
        data = self._model.createData()
        pin.forwardKinematics(self._model, data, q_model)
        pin.computeJointJacobians(self._model, data, q_model)
        pin.updateFramePlacements(self._model, data)
        jac = pin.getFrameJacobian(
            self._model,
            data,
            self._frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        r_world = data.oMf[self._frame_id].rotation @ payload_offset
        skew = np.array(
            [[0, -r_world[2], r_world[1]], [r_world[2], 0, -r_world[0]], [-r_world[1], r_world[0], 0]],
            dtype=float,
        )
        j_point = jac[:3, :] - skew @ jac[3:, :]
        payload = (payload_kg * 9.81 * j_point[2, :])[:6]
        return tau_model, payload

    def _read_positions(self, t: float) -> np.ndarray:
        if self.dry_run:
            return np.array(
                [
                    math.radians(-5.0),
                    math.radians(10.0 + 8.0 * math.sin(t / 7.0)),
                    math.radians(14.0 + 12.0 * math.sin(t / 5.0)),
                    math.radians(-5.0 + 10.0 * math.sin(t / 4.0)),
                    math.radians(1.0),
                    math.radians(33.0),
                ],
                dtype=float,
            )
        with self.motor_lock:
            if self.emergency_requested.is_set():
                raise MotionCancelled()
            assert self.robot is not None
            return self.robot.arm.get_positions().copy()

    def _send_mit(self, q: np.ndarray, tau: np.ndarray, *, kp: np.ndarray = LOOSE_KP, kd: np.ndarray = LOOSE_KD) -> None:
        with self.motor_lock:
            if self.emergency_requested.is_set():
                raise MotionCancelled()
            if any(np.asarray(v).shape != (6,) or not np.all(np.isfinite(v)) for v in (q, tau, kp, kd)):
                raise ValueError("电机指令必须包含六个有限数值")
            if self.dry_run:
                return
            assert self.robot is not None
            self.robot.arm.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

    def _record_sample(
        self,
        *,
        t: float,
        q: np.ndarray,
        tau_model: np.ndarray,
        tau_payload: np.ndarray,
        tau_total: np.ndarray,
        tau_cmd: np.ndarray,
        joint_percent: list[float],
        effective_scale: np.ndarray,
        camera_profile: str,
    ) -> None:
        with self.lock:
            if self.recording is None:
                return
            samples = self.recording["samples"]
            min_dt = RECORD_EVERY_N / 30.0
            if len(samples) > 0 and float(t) - float(samples[-1]["t"]) < min_dt:
                return
            sample = {
                "t": round(float(t), 3),
                "q_rad": [float(x) for x in q],
                "q_deg": deg(q),
                "tau_model_nm": [float(x) for x in tau_model],
                "tau_payload_nm": [float(x) for x in tau_payload],
                "tau_total_nm": [float(x) for x in tau_total],
                "tau_cmd_nm": [float(x) for x in tau_cmd],
                "joint_percent": [float(x) for x in joint_percent],
                "effective_scale": [float(x) for x in effective_scale],
                "camera_profile": camera_profile,
            }
            samples.append(sample)
            slot = int(self.recording["slot"])
            count = len(samples)
        if count % 8 == 0:
            self._set_status(recording_slot=slot, recording_samples=count, message=f"recording Pose {slot}: {count} samples")

    def _run_loop(self, *, enable: bool = False) -> None:
        current_scale = self._effective_scale(self.settings)
        rate_hz = 30.0
        dt = 1.0 / rate_hz
        last_status = 0.0
        t0 = time.time()
        try:
            self._check_cancel()
            if enable and not self.dry_run:
                if self.robot is None:
                    self.robot = self._ensure_robot()
                    self.robot.connect()
                with self.motor_lock:
                    if self.emergency_requested.is_set() or self.stop_requested.is_set():
                        raise MotionCancelled()
                    self.robot.arm.mode_mit()
                    self.robot.arm.enable()
            self._set_status(mode="running", connected=True, enabled=True, message="running")
            while not self.stop_requested.is_set():
                loop_t = time.time()
                q = self._read_positions(loop_t - t0)
                tau_model, tau_payload = self._estimate_payload(q)
                tau_total = tau_model + tau_payload
                with self.lock:
                    target_scale = self._effective_scale(self.settings)
                    joint_percent = list(self.settings.joint_percent)
                    camera_profile = self.settings.camera_profile
                    base_percent = float(np.mean(joint_percent))
                max_step = 0.15 * dt
                delta = np.clip(target_scale - current_scale, -max_step, max_step)
                current_scale = current_scale + delta
                tau_cmd = np.clip(tau_total * current_scale, -COMMAND_CLAMP_NM, COMMAND_CLAMP_NM)
                self._send_mit(q, tau_cmd)
                self._record_sample(
                    t=loop_t - t0,
                    q=q,
                    tau_model=tau_model,
                    tau_payload=tau_payload,
                    tau_total=tau_total,
                    tau_cmd=tau_cmd,
                    joint_percent=joint_percent,
                    effective_scale=current_scale,
                    camera_profile=camera_profile,
                )
                self._write_log(
                    {
                        "kind": "sample",
                        "t": round(loop_t - t0, 3),
                        "q_rad": [float(x) for x in q],
                        "q_deg": deg(q),
                        "tau_model_nm": [float(x) for x in tau_model],
                        "tau_payload_nm": [float(x) for x in tau_payload],
                        "tau_total_nm": [float(x) for x in tau_total],
                        "tau_cmd_nm": [float(x) for x in tau_cmd],
                        "joint_percent": joint_percent,
                        "camera_profile": camera_profile,
                        "effective_scale": [float(x) for x in current_scale],
                    }
                )
                if loop_t - last_status > 0.2:
                    self._set_status(
                        mode="running",
                        connected=True,
                        enabled=True,
                        base_percent=base_percent,
                        target_percent=base_percent,
                        joint_percent=joint_percent,
                        camera_profile=camera_profile,
                        effective_scale=[float(x) for x in current_scale],
                        q_deg=deg(q),
                        tau_model_nm=[float(x) for x in tau_model],
                        tau_payload_nm=[float(x) for x in tau_payload],
                        tau_total_nm=[float(x) for x in tau_total],
                        tau_cmd_nm=[float(x) for x in tau_cmd],
                        presets=self.status.presets,
                        trajectories=self.status.trajectories,
                        recording_slot=self.status.recording_slot,
                        recording_samples=self.status.recording_samples,
                        message="running joint tuning",
                    )
                    last_status = loop_t
                sleep_for = dt - (time.time() - loop_t)
                if sleep_for > 0:
                    time.sleep(sleep_for)
            q = self._read_positions(time.time() - t0)
            tau_model, tau_payload = self._estimate_payload(q)
            tau_total = tau_model + tau_payload
            self._send_mit(q, np.clip(tau_total * current_scale, -COMMAND_CLAMP_NM, COMMAND_CLAMP_NM))
            self._write_log({"kind": "stop_hold", "q_deg": deg(q)})
            self._set_status(mode="holding", enabled=True, q_deg=deg(q), message="stopped and holding current pose")
        except MotionCancelled:
            if not self.status.enabled:
                self._shutdown_robot(self.robot)
                self.robot = None
                self._set_status(mode="disabled", connected=False, enabled=False, message="启动已取消")
        except Exception as exc:
            tb = traceback.format_exc()
            self._write_log({"kind": "error", "error": repr(exc), "traceback": tb})
            self._event("danger", f"control error: {exc!r}")
            self._set_status(mode="error", message=repr(exc))
        finally:
            if self._log_file is not None:
                self._log_file.flush()



class Handler(BaseHTTPRequestHandler):
    server_version = "GravityConsole/0.1"

    @property
    def app(self) -> GravityConsole:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _send_json(self, data: Any, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            with self.app.lock:
                self._send_json(self.app.status.to_json())
            return
        if parsed.path == "/api/events":
            self._events()
            return
        path = parsed.path
        if path == "/":
            path = "/index.html"
        target = (STATIC_DIR / path.lstrip("/")).resolve()
        if not str(target).startswith(str(STATIC_DIR.resolve())) or not target.exists():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = target.read_bytes()
        ctype = "text/html; charset=utf-8"
        if target.suffix == ".css":
            ctype = "text/css; charset=utf-8"
        elif target.suffix == ".js":
            ctype = "application/javascript; charset=utf-8"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _events(self) -> None:
        q = self.app.subscribe()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                item = q.get(timeout=10.0)
                payload = f"data: {json.dumps(item, ensure_ascii=False)}\n\n".encode("utf-8")
                self.wfile.write(payload)
                self.wfile.flush()
        except Exception:
            self.app.unsubscribe(q)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            data = self._read_json()
            if not isinstance(data, dict):
                raise ValueError("请求必须为 JSON 对象")
            if path == "/api/emergency-disable":
                self.app.emergency_requested.set()
                self.app.stop_requested.set()
            with self.app.command_lock:
                result = self._dispatch_post(path, data)
            self._send_json(result)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=400)

    def _dispatch_post(self, path: str, data: dict[str, Any]) -> dict[str, Any]:
        action = path.removeprefix("/api/")
        if action in {"groups/add", "groups/update", "groups/delete", "blocks/add", "blocks/update", "blocks/delete"}:
            return self.app.groups_command(action, data)
        routes = {
            "/api/start": self.app.start,
            "/api/apply": self.app.apply,
            "/api/save-preset": self.app.save_preset,
            "/api/apply-preset": self.app.apply_preset,
            "/api/start-recording": self.app.start_recording,
            "/api/stop-recording": self.app.stop_recording,
            "/api/replay-trajectory": self.app.replay_trajectory,
            "/api/reset-trajectory": self.app.reset_trajectory,
            "/api/blocks/record": self.app.record_block,
            "/api/blocks/stop-recording": self.app.stop_block_recording,
            "/api/blocks/play": self.app.play_block,
            "/api/groups/play": self.app.play_group,
        }
        if path in routes:
            return routes[path](data)
        if path == "/api/stop":
            return self.app.stop_hold()
        if path == "/api/home-disable":
            return self.app.home_disable()
        if path == "/api/emergency-disable":
            return self.app.emergency_disable()
        raise ValueError("未知接口")


def main() -> int:
    parser = argparse.ArgumentParser(description="J1-J6 gravity compensation tuning console")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8012)
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/runs"))
    parser.add_argument("--preset-file", type=Path, default=None)
    parser.add_argument("--trajectory-file", type=Path, default=None)
    parser.add_argument("--group-file", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-percent", type=float, default=MAX_PERCENT_DEFAULT)
    args = parser.parse_args()

    app = GravityConsole(
        robot_root=args.robot_root,
        log_dir=args.log_dir,
        dry_run=args.dry_run,
        max_percent=args.max_percent,
        preset_file=args.preset_file,
        trajectory_file=args.trajectory_file,
        group_file=args.group_file,
    )
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.app = app  # type: ignore[attr-defined]
    print(f"gravity console listening on http://{args.host}:{args.port} dry_run={args.dry_run}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
