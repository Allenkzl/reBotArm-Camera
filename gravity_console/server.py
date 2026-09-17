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


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
BASE_JOINT_SCALE = np.array([1.0, 0.98, 0.98, 1.0, 1.0, 1.0], dtype=float)
PAYLOAD_KG = 0.8
PAYLOAD_OFFSET_LOCAL_M = np.array([0.06, 0.0, 0.0], dtype=float)
MAX_PERCENT_DEFAULT = 60.0


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
    base_percent: float = 20.0
    trims: dict[str, float] = field(
        default_factory=lambda: {"j2": 0.0, "j3": 0.0, "j4": 0.0}
    )


@dataclass
class RobotStatus:
    mode: str = "idle"
    run_id: str | None = None
    log_path: str | None = None
    connected: bool = False
    enabled: bool = False
    dry_run: bool = False
    base_percent: float = 20.0
    target_percent: float = 20.0
    trims: dict[str, float] = field(
        default_factory=lambda: {"j2": 0.0, "j3": 0.0, "j4": 0.0}
    )
    effective_scale: list[float] = field(default_factory=lambda: [0.2] * 6)
    q_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_model_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_payload_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_total_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    tau_cmd_nm: list[float] = field(default_factory=lambda: [0.0] * 6)
    message: str = "ready"
    updated_ms: int = field(default_factory=now_ms)

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
            "trims": self.trims,
            "effective_scale": self.effective_scale,
            "q_deg": self.q_deg,
            "tau_model_nm": self.tau_model_nm,
            "tau_payload_nm": self.tau_payload_nm,
            "tau_total_nm": self.tau_total_nm,
            "tau_cmd_nm": self.tau_cmd_nm,
            "message": self.message,
            "updated_ms": self.updated_ms,
        }


class GravityConsole:
    def __init__(
        self,
        *,
        robot_root: Path,
        log_dir: Path,
        dry_run: bool,
        max_percent: float,
    ) -> None:
        self.robot_root = robot_root
        self.log_dir = log_dir
        self.dry_run = dry_run
        self.max_percent = max_percent
        self.status = RobotStatus(dry_run=dry_run)
        self.settings = Settings()
        self.lock = threading.RLock()
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
        print(json.dumps({"event": event}, ensure_ascii=False), flush=True)
        self._publish({"type": "event", "event": event})
        self._write_log({"kind": "event", **event})

    def _set_status(self, **updates: Any) -> None:
        with self.lock:
            for key, value in updates.items():
                setattr(self.status, key, value)
            self.status.updated_ms = now_ms()
            snapshot = self.status.to_json()
        self._publish({"type": "status", "status": snapshot})

    def _write_log(self, record: dict[str, Any]) -> None:
        if self._log_file is None:
            return
        self._log_file.write(json.dumps({"t_wall": time.time(), **record}, ensure_ascii=False) + "\n")
        self._log_file.flush()

    def _validated_settings(self, data: dict[str, Any]) -> Settings:
        base = clamp(float(data.get("base_percent", self.settings.base_percent)), 0.0, self.max_percent)
        raw_trims = data.get("trims", {}) or {}
        trims = {
            "j2": clamp(float(raw_trims.get("j2", self.settings.trims["j2"])), -20.0, 20.0),
            "j3": clamp(float(raw_trims.get("j3", self.settings.trims["j3"])), -20.0, 20.0),
            "j4": clamp(float(raw_trims.get("j4", self.settings.trims["j4"])), -20.0, 20.0),
        }
        return Settings(base_percent=base, trims=trims)

    def _effective_scale(self, settings: Settings) -> np.ndarray:
        trims = np.array(
            [
                1.0,
                1.0 + settings.trims["j2"] / 100.0,
                1.0 + settings.trims["j3"] / 100.0,
                1.0 + settings.trims["j4"] / 100.0,
                1.0,
                1.0,
            ],
            dtype=float,
        )
        return BASE_JOINT_SCALE * trims * (settings.base_percent / 100.0)

    def start(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError("gravity compensation is already running")
            self.settings = self._validated_settings(data)
            self.stop_requested.clear()
            run_id = time.strftime("gc_console_%Y%m%d_%H%M%S")
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log_path = self.log_dir / f"{run_id}.jsonl"
            self._log_file = log_path.open("w", buffering=1, encoding="utf-8")
            self._set_status(
                mode="starting",
                run_id=run_id,
                log_path=str(log_path),
                base_percent=self.settings.base_percent,
                target_percent=self.settings.base_percent,
                trims=self.settings.trims,
                effective_scale=self._effective_scale(self.settings).tolist(),
                message="starting J1-J6 gravity compensation",
            )
            self._write_log(
                {
                    "kind": "start",
                    "run_id": run_id,
                    "dry_run": self.dry_run,
                    "settings": {
                        "base_percent": self.settings.base_percent,
                        "trims": self.settings.trims,
                    },
                    "max_percent": self.max_percent,
                    "payload_kg": PAYLOAD_KG,
                    "payload_offset_local_m": PAYLOAD_OFFSET_LOCAL_M.tolist(),
                }
            )
            self.thread = threading.Thread(target=self._run_loop, name="gravity-console-loop", daemon=True)
            self.thread.start()
        self._event("info", f"started {self.settings.base_percent:.0f}%")
        return self.status.to_json()

    def apply(self, data: dict[str, Any]) -> dict[str, Any]:
        settings = self._validated_settings(data)
        with self.lock:
            self.settings = settings
        self._set_status(
            target_percent=settings.base_percent,
            trims=settings.trims,
            message=f"ramping to {settings.base_percent:.0f}%",
        )
        self._event("info", f"apply {settings.base_percent:.0f}%", trims=settings.trims)
        return self.status.to_json()

    def stop_hold(self) -> dict[str, Any]:
        self.stop_requested.set()
        self._event("warn", "stop requested; holding current pose")
        if self.thread is not None:
            self.thread.join(timeout=8.0)
        return self.status.to_json()

    def emergency_disable(self) -> dict[str, Any]:
        self.stop_requested.set()
        self._event("danger", "emergency disable requested")
        try:
            if self.robot is not None:
                self.robot.arm.disable()
        finally:
            self._shutdown_robot(self.robot)
            self.robot = None
            self._set_status(mode="disabled", enabled=False, message="emergency disabled")
        return self.status.to_json()

    def home_disable(self) -> dict[str, Any]:
        self.stop_requested.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=8.0)
        self._event("warn", "home and disable requested")
        self._home_and_disable()
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
        if self.dry_run:
            model_tau = np.array([0.0, 1.4, 6.7, 2.0, 0.0, 0.0], dtype=float)
            payload = np.array([0.0, -2.6, 4.5, 2.7, 0.0, 0.0], dtype=float)
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
        r_world = data.oMf[self._frame_id].rotation @ PAYLOAD_OFFSET_LOCAL_M
        skew = np.array(
            [[0, -r_world[2], r_world[1]], [r_world[2], 0, -r_world[0]], [-r_world[1], r_world[0], 0]],
            dtype=float,
        )
        j_point = jac[:3, :] - skew @ jac[3:, :]
        payload = (PAYLOAD_KG * 9.81 * j_point[2, :])[:6]
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
        assert self.robot is not None
        return self.robot.arm.get_positions().copy()

    def _send_mit(self, q: np.ndarray, tau: np.ndarray) -> None:
        if self.dry_run:
            return
        assert self.robot is not None
        kp = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5], dtype=float)
        kd = np.array([0.8, 0.8, 0.8, 0.8, 0.8, 0.8], dtype=float)
        self.robot.arm.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)

    def _run_loop(self) -> None:
        current_scale = self._effective_scale(self.settings)
        rate_hz = 30.0
        dt = 1.0 / rate_hz
        last_status = 0.0
        t0 = time.time()
        try:
            if not self.dry_run:
                if self.robot is None:
                    self.robot = self._ensure_robot()
                self.robot.connect()
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
                    base_percent = self.settings.base_percent
                    trims = dict(self.settings.trims)
                max_step = 0.15 * dt
                delta = np.clip(target_scale - current_scale, -max_step, max_step)
                current_scale = current_scale + delta
                tau_cmd = tau_total * current_scale
                self._send_mit(q, tau_cmd)
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
                        "base_percent": base_percent,
                        "trims": trims,
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
                        trims=trims,
                        effective_scale=[float(x) for x in current_scale],
                        q_deg=deg(q),
                        tau_model_nm=[float(x) for x in tau_model],
                        tau_payload_nm=[float(x) for x in tau_payload],
                        tau_total_nm=[float(x) for x in tau_total],
                        tau_cmd_nm=[float(x) for x in tau_cmd],
                        message=f"running {base_percent:.0f}%",
                    )
                    last_status = loop_t
                sleep_for = dt - (time.time() - loop_t)
                if sleep_for > 0:
                    time.sleep(sleep_for)
            q = self._read_positions(time.time() - t0)
            tau_model, tau_payload = self._estimate_payload(q)
            tau_total = tau_model + tau_payload
            self._send_mit(q, tau_total * current_scale)
            self._write_log({"kind": "stop_hold", "q_deg": deg(q)})
            self._set_status(mode="holding", enabled=True, q_deg=deg(q), message="stopped and holding current pose")
        except Exception as exc:
            tb = traceback.format_exc()
            self._write_log({"kind": "error", "error": repr(exc), "traceback": tb})
            self._event("danger", f"control error: {exc!r}")
            self._set_status(mode="error", message=repr(exc))
        finally:
            if self._log_file is not None:
                self._log_file.flush()

    def _home_and_disable(self) -> None:
        self._set_status(mode="homing", message="homing to zero")
        run_robot = self.robot
        if run_robot is None and not self.dry_run:
            run_robot = self._ensure_robot()
            run_robot.connect()
            run_robot.arm.mode_mit()
            run_robot.arm.enable()
        try:
            if self.dry_run:
                time.sleep(1.0)
                self._set_status(mode="disabled", enabled=False, q_deg=[0.0] * 6, message="dry-run home disabled")
                return
            assert run_robot is not None
            q0 = run_robot.arm.get_positions().copy()
            home = np.zeros(6)
            kp = np.array([50.0, 150.0, 150.0, 50.0, 50.0, 50.0], dtype=float)
            kd = np.array([3.0, 10.0, 10.0, 5.0, 4.0, 4.0], dtype=float)
            max_err = float(np.max(np.abs(home - q0)))
            duration = max(4.0, 2.0 * max_err / 0.22)
            steps = max(2, int(duration * 50.0))
            self._write_log({"kind": "home_start", "q0_deg": deg(q0), "steps": steps, "duration_s": duration})
            for i in range(steps):
                s = (i + 1) / steps
                q = q0 + (home - q0) * minimum_jerk(s)
                tau_model, tau_payload = self._estimate_payload(q)
                tau = tau_model + tau_payload
                run_robot.arm.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=tau)
                if i % 20 == 0:
                    self._set_status(mode="homing", q_deg=deg(q), message="homing")
                time.sleep(duration / steps)
            run_robot.arm.send_mit(home, vel=np.zeros(6), kp=kp, kd=kd, tau=np.zeros(6))
            time.sleep(0.8)
            q_before = run_robot.arm.get_positions().copy()
            run_robot.arm.disable()
            self._write_log({"kind": "home_disable", "q_before_disable_deg": deg(q_before)})
            self._set_status(mode="disabled", enabled=False, q_deg=deg(q_before), message="home disabled")
            self._event("info", "home disabled")
        finally:
            self._shutdown_robot(run_robot)
            self.robot = None
            self._set_status(connected=False)


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
        parsed = urlparse(self.path)
        try:
            data = self._read_json()
            if parsed.path == "/api/start":
                self._send_json(self.app.start(data))
            elif parsed.path == "/api/apply":
                self._send_json(self.app.apply(data))
            elif parsed.path == "/api/stop":
                self._send_json(self.app.stop_hold())
            elif parsed.path == "/api/home-disable":
                self._send_json(self.app.home_disable())
            elif parsed.path == "/api/emergency-disable":
                self._send_json(self.app.emergency_disable())
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=400)


def main() -> int:
    parser = argparse.ArgumentParser(description="J1-J6 gravity compensation tuning console")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8012)
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/runs"))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-percent", type=float, default=MAX_PERCENT_DEFAULT)
    args = parser.parse_args()

    app = GravityConsole(
        robot_root=args.robot_root,
        log_dir=args.log_dir,
        dry_run=args.dry_run,
        max_percent=args.max_percent,
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
