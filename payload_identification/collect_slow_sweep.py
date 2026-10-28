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
from pd_hold_probe import KD, KP, deg, make_model_gravity, minimum_jerk, shutdown_robot


DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
FF_CLAMP = np.array([1.0, 5.0, 8.0, 1.5, 0.3, 0.3], dtype=float)


def make_segment(start: np.ndarray, end: np.ndarray, seconds: float, rate: float) -> list[np.ndarray]:
    steps = max(2, int(seconds * rate))
    return [start + (end - start) * minimum_jerk(float(s)) for s in np.linspace(0.0, 1.0, steps)]


def make_gains(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
    kp = KP.copy()
    kd = KD.copy()
    kp[1] = args.j2_kp
    kp[2] = args.j3_kp
    kp[3] = args.j4_kp
    kd[1] = args.j2_kd
    kd[2] = args.j3_kd
    kd[3] = args.j4_kd
    return kp, kd


def run_sweep(args: argparse.Namespace) -> int:
    args.log_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime("slow_sweep_%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"{run_id}.jsonl"
    joint_index = args.joint - 1
    amplitude_rad = np.radians(args.amplitude_deg)
    kp, kd = make_gains(args)

    if args.dry_run:
        print(f"dry-run would write {log_path}")
        print(
            "joint=J%d amplitude_deg=%.3f sweep_s=%.3f dwell_s=%.3f return_s=%.3f rate_hz=%.1f"
            % (args.joint, args.amplitude_deg, args.sweep_s, args.dwell_s, args.return_s, args.rate)
        )
        print(f"kp={kp.tolist()} kd={kd.tolist()} ff_clamp_nm={FF_CLAMP.tolist()}")
        return 0

    robot_root = str(args.robot_root)
    if robot_root not in sys.path:
        sys.path.insert(0, robot_root)

    from reBotArm_control_py.actuator import RebotArm

    with tempfile.TemporaryDirectory(prefix="rebot-payload-sweep-") as temp_dir:
        cfg_path = Path(temp_dir) / "rebotarm_camera_j1j6.yaml"
        make_j1j6_config(cfg_path)
        robot = RebotArm(hw_yaml=str(cfg_path))
        gravity = make_model_gravity(args.robot_root)
        with log_path.open("w", encoding="utf-8") as f:
            def write(record: dict[str, Any], show: bool = False) -> None:
                event = {"t_wall": time.time(), **record}
                f.write(json.dumps(event, ensure_ascii=False) + "\n")
                f.flush()
                if show:
                    msg = json.dumps(event, ensure_ascii=False)
                    print(msg, flush=True)

            write(
                {
                    "kind": "start",
                    "run_id": run_id,
                    "joint": args.joint,
                    "amplitude_deg": args.amplitude_deg,
                    "sweep_s": args.sweep_s,
                    "dwell_s": args.dwell_s,
                    "return_s": args.return_s,
                    "rate_hz": args.rate,
                    "kp": kp.tolist(),
                    "kd": kd.tolist(),
                    "ff_clamp_nm": FF_CLAMP.tolist(),
                    "max_error_rad": args.max_error,
                    "max_velocity_rad_s": args.max_velocity,
                    "velocity_alpha": args.velocity_alpha,
                    "velocity_trip_count": args.velocity_trip_count,
                    "note": "active conservative single-joint slow sweep; J1-J6 only; no J7",
                },
                show=True,
            )
            try:
                robot.connect()
                q0 = robot.arm.get_positions().copy()
                q_neg = q0.copy()
                q_pos = q0.copy()
                q_neg[joint_index] -= amplitude_rad
                q_pos[joint_index] += amplitude_rad
                write(
                    {
                        "kind": "connected",
                        "q0_rad": q0.tolist(),
                        "q0_deg": deg(q0),
                        "q_neg_deg": deg(q_neg),
                        "q_pos_deg": deg(q_pos),
                    },
                    show=True,
                )

                robot.arm.mode_mit()
                robot.arm.enable()
                write({"kind": "enabled", "group": "arm"}, show=True)

                dt = 1.0 / args.rate
                segments: list[tuple[str, list[np.ndarray]]] = [
                    ("move_to_negative", make_segment(q0, q_neg, args.return_s, args.rate)),
                    ("dwell_negative", [q_neg.copy()] * max(1, int(args.dwell_s * args.rate))),
                    ("sweep_negative_to_positive", make_segment(q_neg, q_pos, args.sweep_s, args.rate)),
                    ("dwell_positive", [q_pos.copy()] * max(1, int(args.dwell_s * args.rate))),
                    ("sweep_positive_to_negative", make_segment(q_pos, q_neg, args.sweep_s, args.rate)),
                    ("dwell_negative_2", [q_neg.copy()] * max(1, int(args.dwell_s * args.rate))),
                    ("return_to_start", make_segment(q_neg, q0, args.return_s, args.rate)),
                ]

                q_prev = robot.arm.get_positions().copy()
                t_prev = time.time()
                vel_filt = np.zeros(6)
                high_velocity_count = 0
                sample_idx = 0
                abort_reason: str | None = None
                started = time.time()

                for segment_name, targets in segments:
                    write({"kind": "segment_start", "segment": segment_name, "samples": len(targets)}, show=True)
                    next_t = time.time()
                    for target_idx, q_target in enumerate(targets):
                        now = time.time()
                        q = robot.arm.get_positions().copy()
                        raw_vel = (q - q_prev) / max(now - t_prev, 1e-4)
                        vel_filt = args.velocity_alpha * raw_vel + (1.0 - args.velocity_alpha) * vel_filt
                        err = q_target - q

                        if np.max(np.abs(err)) > args.max_error:
                            abort_reason = f"position error exceeded {args.max_error:.3f} rad"
                            break
                        if np.max(np.abs(vel_filt)) > args.max_velocity and sample_idx > int(args.rate):
                            high_velocity_count += 1
                        else:
                            high_velocity_count = 0
                        if high_velocity_count >= args.velocity_trip_count:
                            abort_reason = (
                                f"filtered velocity exceeded {args.max_velocity:.3f} rad/s "
                                f"for {args.velocity_trip_count} samples"
                            )
                            break

                        tau_model = gravity(q)
                        tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                        tau_pd = kp * err - kd * vel_filt
                        tau_cmd = tau_ff + tau_pd
                        robot.arm.send_mit(q_target, vel=np.zeros(6), kp=kp, kd=kd, tau=tau_ff)

                        if sample_idx % max(1, int(args.print_every_s * args.rate)) == 0:
                            write(
                                {
                                    "kind": "progress",
                                    "idx": sample_idx,
                                    "segment": segment_name,
                                    "target_idx": target_idx,
                                    "t": round(now - started, 6),
                                    "q_deg": deg(q),
                                    "target_deg": deg(q_target),
                                    "err_deg": deg(err),
                                    "vel_filt_rad_s": vel_filt.tolist(),
                                    "tau_pd_est_nm": tau_pd.tolist(),
                                    "tau_ff_nm": tau_ff.tolist(),
                                },
                                show=True,
                            )

                        write(
                            {
                                "kind": "sample",
                                "idx": sample_idx,
                                "segment": segment_name,
                                "target_idx": target_idx,
                                "t": round(now - started, 6),
                                "q_rad": q.tolist(),
                                "q_deg": deg(q),
                                "q_target_rad": q_target.tolist(),
                                "q_target_deg": deg(q_target),
                                "err_rad": err.tolist(),
                                "vel_raw_rad_s": raw_vel.tolist(),
                                "vel_filt_rad_s": vel_filt.tolist(),
                                "tau_model_nm": tau_model.tolist(),
                                "tau_ff_nm": tau_ff.tolist(),
                                "tau_pd_est_nm": tau_pd.tolist(),
                                "tau_cmd_est_nm": tau_cmd.tolist(),
                            }
                        )
                        q_prev = q
                        t_prev = now
                        sample_idx += 1
                        next_t += dt
                        sleep_for = next_t - time.time()
                        if sleep_for > 0:
                            time.sleep(sleep_for)
                    write({"kind": "segment_end", "segment": segment_name, "abort_reason": abort_reason}, show=True)
                    if abort_reason is not None:
                        break

                q_end = robot.arm.get_positions().copy()
                write(
                    {
                        "kind": "sweep_complete" if abort_reason is None else "abort",
                        "reason": abort_reason,
                        "samples": sample_idx,
                        "q_end_rad": q_end.tolist(),
                        "q_end_deg": deg(q_end),
                    },
                    show=True,
                )

                return_steps = max(1, int(args.return_s * args.rate))
                for step, s in enumerate(np.linspace(0.0, 1.0, return_steps)):
                    q = robot.arm.get_positions().copy()
                    q_return_target = q_end + (q0 - q_end) * minimum_jerk(float(s))
                    tau_model = gravity(q)
                    tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                    robot.arm.send_mit(q_return_target, vel=np.zeros(6), kp=kp, kd=kd, tau=tau_ff)
                    time.sleep(dt)
                q_return_done = robot.arm.get_positions().copy()
                write({"kind": "returned_to_start", "q_rad": q_return_done.tolist(), "q_deg": deg(q_return_done)}, show=True)

                fade_steps = max(1, int(args.fade_s * args.rate))
                for k in np.linspace(1.0, 0.0, fade_steps):
                    q = robot.arm.get_positions().copy()
                    tau_model = gravity(q)
                    tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                    robot.arm.send_mit(q, vel=np.zeros(6), kp=kp * k, kd=kd, tau=tau_ff * k)
                    time.sleep(dt)
                q_before_disable = robot.arm.get_positions().copy()
                robot.arm.disable()
                write(
                    {
                        "kind": "disabled",
                        "q_before_disable_rad": q_before_disable.tolist(),
                        "q_before_disable_deg": deg(q_before_disable),
                    },
                    show=True,
                )
                time.sleep(args.post_disable_s)
                try:
                    q_after_disable = robot.arm.get_positions().copy()
                    write(
                        {
                            "kind": "post_disable_pose",
                            "settle_s": args.post_disable_s,
                            "q_rad": q_after_disable.tolist(),
                            "q_deg": deg(q_after_disable),
                        },
                        show=True,
                    )
                except Exception as exc:
                    write({"kind": "post_disable_pose_error", "error": repr(exc)}, show=True)
            except Exception as exc:
                write({"kind": "error", "error": repr(exc)}, show=True)
                try:
                    q = robot.arm.get_positions().copy()
                    robot.arm.send_mit(q, vel=np.zeros(6), kp=kp, kd=kd, tau=np.zeros(6))
                    time.sleep(1.0)
                    robot.arm.disable()
                    write({"kind": "disabled_after_error"}, show=True)
                except Exception as disable_exc:
                    write({"kind": "disable_error", "error": repr(disable_exc)}, show=True)
                raise
            finally:
                shutdown_robot(robot)
                write({"kind": "closed"}, show=True)

    print(str(log_path), flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Conservative single-joint slow sweep for camera payload identification")
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/payload-id"))
    parser.add_argument("--joint", type=int, default=3, choices=range(1, 7))
    parser.add_argument("--amplitude-deg", type=float, default=3.0)
    parser.add_argument("--sweep-s", type=float, default=4.0)
    parser.add_argument("--dwell-s", type=float, default=2.0)
    parser.add_argument("--return-s", type=float, default=3.0)
    parser.add_argument("--fade-s", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument("--max-error", type=float, default=0.14)
    parser.add_argument("--max-velocity", type=float, default=0.25)
    parser.add_argument("--velocity-alpha", type=float, default=0.2)
    parser.add_argument("--velocity-trip-count", type=int, default=5)
    parser.add_argument("--print-every-s", type=float, default=1.0)
    parser.add_argument("--j2-kp", type=float, default=float(KP[1]))
    parser.add_argument("--j2-kd", type=float, default=float(KD[1]))
    parser.add_argument("--j3-kp", type=float, default=float(KP[2]))
    parser.add_argument("--j3-kd", type=float, default=float(KD[2]))
    parser.add_argument("--j4-kp", type=float, default=float(KP[3]))
    parser.add_argument("--j4-kd", type=float, default=float(KD[3]))
    parser.add_argument("--post-disable-s", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    return run_sweep(args)


if __name__ == "__main__":
    raise SystemExit(main())
