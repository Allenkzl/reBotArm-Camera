#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
MECH_POS = 0x7019
MECH_VEL = 0x701A


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


def jsonable(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [jsonable(x) for x in value]
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    return repr(value)


def state_to_dict(state: Any) -> dict[str, Any] | None:
    if state is None:
        return None
    out: dict[str, Any] = {"type": type(state).__name__}
    if hasattr(state, "_asdict"):
        out.update(jsonable(state._asdict()))
        return out
    if hasattr(state, "__dataclass_fields__"):
        for name in state.__dataclass_fields__:
            out[name] = jsonable(getattr(state, name))
        return out
    for name in dir(state):
        if name.startswith("_"):
            continue
        try:
            value = getattr(state, name)
        except Exception as exc:
            out[name] = f"<error {exc!r}>"
            continue
        if callable(value):
            continue
        out[name] = jsonable(value)
    return out


def shutdown_robot(robot: Any) -> None:
    for ctrl in getattr(robot, "_ctrl_map", {}).values():
        try:
            ctrl.shutdown()
            time.sleep(0.1)
            ctrl.close()
        except Exception as exc:
            print(f"controller close warning: {exc!r}", file=sys.stderr)


def audit(args: argparse.Namespace) -> int:
    robot_root = str(args.robot_root)
    if robot_root not in sys.path:
        sys.path.insert(0, robot_root)

    from reBotArm_control_py.actuator import RebotArm

    args.log_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime("feedback_audit_%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"{run_id}.jsonl"

    with tempfile.TemporaryDirectory(prefix="rebot-payload-audit-") as temp_dir:
        cfg_path = Path(temp_dir) / "rebotarm_camera_j1j6.yaml"
        make_j1j6_config(cfg_path)
        robot = RebotArm(hw_yaml=str(cfg_path))
        started = time.time()
        with log_path.open("w", encoding="utf-8") as f:
            def write(record: dict[str, Any]) -> None:
                f.write(json.dumps({"t_wall": time.time(), **record}, ensure_ascii=False) + "\n")
                f.flush()

            write(
                {
                    "kind": "start",
                    "run_id": run_id,
                    "duration_s": args.duration,
                    "rate_hz": args.rate,
                    "robot_root": str(args.robot_root),
                    "config": str(cfg_path),
                    "note": "read-only feedback audit; no enable, no motion command, J1-J6 only",
                }
            )
            try:
                robot.connect()
                write({"kind": "connected", "groups": list(getattr(robot, "groups", {}).keys())})
                period = 1.0 / args.rate
                next_t = time.time()
                sample_idx = 0
                while time.time() - started < args.duration:
                    # Use the JointGroup path because this SDK falls back to
                    # RobStride mechPos (0x7019) when cached feedback frames are
                    # absent. The global robot.get_state() can return zeroes in
                    # this disabled read-only mode.
                    q = robot.arm.get_positions()
                    motors = {}
                    for name in JOINT_NAMES:
                        motor = getattr(robot.arm, "_mm", {}).get(name)
                        state = motor.get_state() if motor is not None else None
                        params: dict[str, Any] = {}
                        if motor is not None:
                            for param_name, param_id in (("mech_pos_0x7019", MECH_POS), ("mech_vel_0x701A", MECH_VEL)):
                                try:
                                    params[param_name] = float(motor.robstride_get_param_f32(param_id))
                                except Exception as exc:
                                    params[param_name] = f"<error {exc!r}>"
                        motors[name] = {
                            "cached_state": state_to_dict(state),
                            "params": params,
                        }
                    write(
                        {
                            "kind": "sample",
                            "idx": sample_idx,
                            "t": round(time.time() - started, 6),
                            "q_rad": [float(x) for x in q],
                            "q_deg": [float(np.degrees(x)) for x in q],
                            "motors": motors,
                        }
                    )
                    sample_idx += 1
                    next_t += period
                    sleep_for = next_t - time.time()
                    if sleep_for > 0:
                        time.sleep(sleep_for)
                write({"kind": "complete", "samples": sample_idx, "elapsed_s": round(time.time() - started, 3)})
            finally:
                shutdown_robot(robot)
                write({"kind": "closed"})

    print(str(log_path))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only J1-J6 feedback audit for payload identification")
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/payload-id"))
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()
    return audit(args)


if __name__ == "__main__":
    raise SystemExit(main())
