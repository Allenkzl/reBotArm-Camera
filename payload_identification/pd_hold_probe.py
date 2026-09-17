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

from feedback_audit import make_j1j6_config


DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
KP = np.array([8.0, 20.0, 18.0, 8.0, 5.0, 5.0], dtype=float)
KD = np.array([0.8, 1.5, 1.2, 0.6, 0.4, 0.4], dtype=float)
FF_CLAMP = np.array([1.0, 5.0, 8.0, 2.0, 0.3, 0.3], dtype=float)


def minimum_jerk(s: float) -> float:
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def deg(q: np.ndarray) -> list[float]:
    return [float(np.degrees(x)) for x in q]


def shutdown_robot(robot: Any) -> None:
    for ctrl in getattr(robot, "_ctrl_map", {}).values():
        try:
            ctrl.shutdown()
            time.sleep(0.1)
            ctrl.close()
        except Exception as exc:
            print(f"controller close warning: {exc!r}", file=sys.stderr)


def make_model_gravity(robot_root: Path) -> Any:
    root = str(robot_root)
    if root not in sys.path:
        sys.path.insert(0, root)
    from reBotArm_control_py.dynamics.inverse_dynamics import compute_generalized_gravity
    from reBotArm_control_py.kinematics.robot_model import load_robot_model, pad_q_for_model

    model = load_robot_model()

    def gravity(q6: np.ndarray) -> np.ndarray:
        q_model = pad_q_for_model(model, q6, controlled_joints=6)
        return compute_generalized_gravity(model=model, q=q_model, data=model.createData())[:6]
    return gravity


def run_probe(args: argparse.Namespace) -> int:
    args.log_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime("pd_hold_probe_%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"{run_id}.jsonl"

    if args.dry_run:
        print(f"dry-run would write {log_path}")
        print(f"duration_s={args.duration} rate_hz={args.rate}")
        print(f"kp={KP.tolist()} kd={KD.tolist()} ff_clamp_nm={FF_CLAMP.tolist()}")
        return 0

    robot_root = str(args.robot_root)
    if robot_root not in sys.path:
        sys.path.insert(0, robot_root)

    from reBotArm_control_py.actuator import RebotArm

    with tempfile.TemporaryDirectory(prefix="rebot-payload-pdhold-") as temp_dir:
        cfg_path = Path(temp_dir) / "rebotarm_camera_j1j6.yaml"
        make_j1j6_config(cfg_path)
        robot = RebotArm(hw_yaml=str(cfg_path))
        gravity = make_model_gravity(args.robot_root)
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
                    "kp": KP.tolist(),
                    "kd": KD.tolist(),
                    "ff_clamp_nm": FF_CLAMP.tolist(),
                    "max_error_rad": args.max_error,
                    "max_velocity_rad_s": args.max_velocity,
                    "return_s": args.return_s,
                    "note": "active conservative PD hold; J1-J6 only; no J7",
                }
            )
            try:
                robot.connect()
                q0 = robot.arm.get_positions().copy()
                q_prev = q0.copy()
                q_target = q0.copy()
                write({"kind": "connected", "q0_rad": q0.tolist(), "q0_deg": deg(q0)})

                robot.arm.mode_mit()
                robot.arm.enable()
                write({"kind": "enabled", "group": "arm"})

                dt = 1.0 / args.rate
                started = time.time()
                t_prev = started
                next_t = started
                i = 0
                abort_reason: str | None = None

                while time.time() - started < args.duration:
                    now = time.time()
                    q = robot.arm.get_positions().copy()
                    vel = (q - q_prev) / max(now - t_prev, 1e-4)
                    err = q_target - q
                    if np.max(np.abs(err)) > args.max_error:
                        abort_reason = f"position error exceeded {args.max_error:.3f} rad"
                        break
                    if np.max(np.abs(vel)) > args.max_velocity and i > int(args.rate):
                        abort_reason = f"velocity exceeded {args.max_velocity:.3f} rad/s"
                        break

                    tau_model = gravity(q)
                    tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                    tau_pd = KP * err - KD * vel
                    tau_cmd = tau_ff + tau_pd
                    robot.arm.send_mit(q_target, vel=np.zeros(6), kp=KP, kd=KD, tau=tau_ff)

                    write(
                        {
                            "kind": "sample",
                            "idx": i,
                            "t": round(now - started, 6),
                            "q_rad": q.tolist(),
                            "q_deg": deg(q),
                            "q_target_rad": q_target.tolist(),
                            "err_rad": err.tolist(),
                            "vel_fd_rad_s": vel.tolist(),
                            "tau_model_nm": tau_model.tolist(),
                            "tau_ff_nm": tau_ff.tolist(),
                            "tau_pd_est_nm": tau_pd.tolist(),
                            "tau_cmd_est_nm": tau_cmd.tolist(),
                        }
                    )
                    q_prev = q
                    t_prev = now
                    i += 1
                    next_t += dt
                    sleep_for = next_t - time.time()
                    if sleep_for > 0:
                        time.sleep(sleep_for)

                q_end = robot.arm.get_positions().copy()
                write(
                    {
                        "kind": "hold_complete" if abort_reason is None else "abort",
                        "reason": abort_reason,
                        "samples": i,
                        "q_end_rad": q_end.tolist(),
                        "q_end_deg": deg(q_end),
                    }
                )

                return_steps = max(1, int(args.return_s * args.rate))
                for step, s in enumerate(np.linspace(0.0, 1.0, return_steps)):
                    q = robot.arm.get_positions().copy()
                    q_return_target = q_end + (q0 - q_end) * minimum_jerk(float(s))
                    tau_model = gravity(q)
                    tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                    robot.arm.send_mit(q_return_target, vel=np.zeros(6), kp=KP, kd=KD, tau=tau_ff)
                    if step % max(1, int(args.rate)) == 0:
                        write(
                            {
                                "kind": "return_sample",
                                "idx": step,
                                "q_rad": q.tolist(),
                                "q_deg": deg(q),
                                "q_target_rad": q_return_target.tolist(),
                            }
                        )
                    time.sleep(1.0 / args.rate)
                q_return_done = robot.arm.get_positions().copy()
                write({"kind": "returned_to_start", "q_rad": q_return_done.tolist(), "q_deg": deg(q_return_done)})

                # Fade stiffness and feedforward at the current pose before disable.
                fade_steps = max(1, int(args.fade_s * args.rate))
                for k in np.linspace(1.0, 0.0, fade_steps):
                    q = robot.arm.get_positions().copy()
                    tau_model = gravity(q)
                    tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                    robot.arm.send_mit(q, vel=np.zeros(6), kp=KP * k, kd=KD, tau=tau_ff * k)
                    time.sleep(1.0 / args.rate)
                q_before_disable = robot.arm.get_positions().copy()
                robot.arm.disable()
                write({"kind": "disabled", "q_before_disable_rad": q_before_disable.tolist(), "q_before_disable_deg": deg(q_before_disable)})
            except Exception as exc:
                write({"kind": "error", "error": repr(exc)})
                try:
                    q = robot.arm.get_positions().copy()
                    robot.arm.send_mit(q, vel=np.zeros(6), kp=KP, kd=KD, tau=np.zeros(6))
                    time.sleep(1.0)
                    robot.arm.disable()
                    write({"kind": "disabled_after_error"})
                except Exception as disable_exc:
                    write({"kind": "disable_error", "error": repr(disable_exc)})
                raise
            finally:
                shutdown_robot(robot)
                write({"kind": "closed"})

    print(str(log_path))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Conservative active PD hold probe for camera payload identification")
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/payload-id"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument("--max-error", type=float, default=0.12)
    parser.add_argument("--max-velocity", type=float, default=0.4)
    parser.add_argument("--return-s", type=float, default=3.0)
    parser.add_argument("--fade-s", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    return run_probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
