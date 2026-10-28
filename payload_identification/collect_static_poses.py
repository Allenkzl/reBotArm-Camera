#!/usr/bin/env python3
"""Target 3.4 - multi-pose static equilibrium sampling for payload identification.

Replaces the continuous sweep of Target 3.3.  Instead of dragging the joints
through a trajectory (where the measured PD residual is contaminated by tracking
lag), this script *commands a fixed target and waits*, so that at the end of each
dwell the servos are in static equilibrium and

    kp * (q_target - q_actual) = tau_gravity_true - tau_feedforward

holds exactly.  That residual is the measurement.

Experiment design (see `design_poses.py` for the derivation)
-----------------------------------------------------------
* Only J2/J3/J4 move.  J1 is the vertical axis (a payload produces no gravity
  torque about it) and J5/J6 only twist the camera cable without adding much
  information, so all three are left alone.
* The 10 targets below are the greedy scaled-D-optimal subset of 86 safe
  candidates.  Expected parameter uncertainty at a 0.05 N*m noise floor:
  sigma_m ~ 2.7 %, sigma_rx ~ 10 mm.
* The first target is repeated at the end.  Comparing the two visits measures
  repeatability and hysteresis of the whole pipeline directly.

Safety
------
* Every target is clamped into the joint limits with a margin.
* A pre-flight scan evaluates forward kinematics on a +/- 4 deg box around each
  target and refuses to move if the camera or any wrist frame would come near
  the table.  This runs before anything is enabled.
* Abort conditions: tracking error, and filtered (not single-sample) velocity.
* The arm returns to its starting pose, then is disabled.  J7 is never
  registered or commanded.
"""

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
from pd_hold_probe import deg, minimum_jerk, shutdown_robot


DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

# [J1, J2, J3, J4, J5, J6] in degrees, absolute
POSES_DEG: list[list[float]] = [
    [-6.11, 30.08, 60.97, 13.85, 0.21, 32.86],
    [-6.11, 50.08, 45.97, -16.15, 0.21, 32.86],
    [-6.11, 10.08, 60.97, 28.85, 0.21, 32.86],
    [-6.11, 10.08, 30.97, -31.15, 0.21, 32.86],
    [-6.11, 10.08, 60.97, 13.85, 0.21, 32.86],
    [-6.11, 40.08, 30.97, -1.15, 0.21, 32.86],
    [-6.11, 10.08, 45.97, 28.85, 0.21, 32.86],
    [-6.11, 10.08, 60.97, -31.15, 0.21, 32.86],
    [-6.11, 50.08, 60.97, -31.15, 0.21, 32.86],
    [-6.11, 20.08, 60.97, 28.85, 0.21, 32.86],
]

KP = np.array([8.0, 35.0, 40.0, 25.0, 5.0, 5.0])
KD = np.array([0.8, 2.2, 2.8, 2.0, 0.4, 0.4])
FF_CLAMP = np.array([1.0, 9.0, 12.0, 5.0, 0.8, 0.8])

LOWER_DEG = np.array([-160.0, 0.0, 0.0, -89.9, -89.9, -179.9])
UPPER_DEG = np.array([160.0, 179.9, 179.9, 89.9, 89.9, 179.9])


# --------------------------------------------------------------------------
# planning helpers
# --------------------------------------------------------------------------
def order_poses(poses: np.ndarray, start: np.ndarray) -> list[int]:
    """Greedy nearest-neighbour ordering so consecutive moves are short."""
    remaining = list(range(len(poses)))
    order: list[int] = []
    current = start
    while remaining:
        nxt = min(remaining, key=lambda i: float(np.max(np.abs(poses[i] - current))))
        order.append(nxt)
        remaining.remove(nxt)
        current = poses[nxt]
    return order


def clamp_to_limits(q_deg: np.ndarray, margin_deg: float) -> tuple[np.ndarray, list[str]]:
    lo = LOWER_DEG + margin_deg
    hi = UPPER_DEG - margin_deg
    clamped = np.clip(q_deg, lo, hi)
    notes = [
        f"J{i+1} {q_deg[i]:+.2f} -> {clamped[i]:+.2f}"
        for i in range(6)
        if abs(clamped[i] - q_deg[i]) > 1e-9
    ]
    return clamped, notes


def make_segment(start: np.ndarray, end: np.ndarray, seconds: float, rate: float) -> list[np.ndarray]:
    steps = max(2, int(seconds * rate))
    return [start + (end - start) * minimum_jerk(float(s)) for s in np.linspace(0.0, 1.0, steps)]


def preflight(robot_root: Path, poses_deg: np.ndarray, box_deg: float, min_camera_z: float, min_lowest_z: float) -> bool:
    """Forward-kinematics safety scan. Loads the URDF only; never touches CAN."""
    root = str(robot_root)
    if root not in sys.path:
        sys.path.insert(0, root)
    import pinocchio as pin

    from reBotArm_control_py.kinematics.robot_model import (
        get_end_effector_frame_id,
        load_robot_model,
        pad_q_for_model,
    )

    model = load_robot_model()
    frame_id = get_end_effector_frame_id(model)
    # camera offset along the end-effector +X
    cam_off = np.array([0.06, 0.0, 0.0])
    watch = ["link3", "link4", "link5", "link6", "gripper_end"]

    print()
    print("=== pre-flight FK safety scan (+/- %.1f deg box around each target) ===" % box_deg)
    print(f"{'pose':>5} {'J2':>7} {'J3':>7} {'J4':>7} | {'cam_z':>7} {'lowest':>8} {'frame':>12}   verdict")
    ok = True
    for i, q_deg in enumerate(poses_deg):
        worst_cam, worst_low, worst_frame = np.inf, np.inf, ""
        deltas = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for a in (-box_deg, 0.0, box_deg):
            for b in (-box_deg, 0.0, box_deg):
                for c in (-box_deg, 0.0, box_deg):
                    d = deltas.copy()
                    d[1], d[2], d[3] = a, b, c
                    q6 = np.radians(q_deg + d)
                    qm = pad_q_for_model(model, q6, controlled_joints=6)
                    data = model.createData()
                    pin.forwardKinematics(model, data, qm)
                    pin.updateFramePlacements(model, data)
                    T = data.oMf[frame_id]
                    cam = T.rotation @ cam_off + T.translation
                    worst_cam = min(worst_cam, float(cam[2]))
                    for name in watch:
                        fid = model.getFrameId(name)
                        if fid < len(model.frames):
                            z = float(data.oMf[fid].translation[2])
                            if z < worst_low:
                                worst_low, worst_frame = z, name
        good = worst_cam >= min_camera_z and worst_low >= min_lowest_z
        ok = ok and good
        print(
            f"{i:>5} {q_deg[1]:>7.2f} {q_deg[2]:>7.2f} {q_deg[3]:>7.2f} | "
            f"{worst_cam:>7.3f} {worst_low:>8.3f} {worst_frame:>12}   {'OK' if good else 'UNSAFE'}"
        )
    print(f"pre-flight verdict: {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def run(args: argparse.Namespace) -> int:
    poses_deg = np.array(POSES_DEG, dtype=float) if args.pose_json is None else np.array(
        json.loads(Path(args.pose_json).read_text(encoding="utf-8"))["selected_q_deg"], dtype=float
    )
    margin = args.limit_margin_deg
    clamped = []
    for i, p in enumerate(poses_deg):
        c, notes = clamp_to_limits(p, margin)
        if notes:
            print(f"pose {i} clamped to limits: {'; '.join(notes)}")
        clamped.append(c)
    poses_deg = np.array(clamped)

    if args.pose_indices:
        idx = [int(x) for x in args.pose_indices.split(",")]
        poses_deg = poses_deg[idx]
        print(f"verification subset: using poses {idx} -> {poses_deg.tolist()}")

    if not args.no_preflight:
        if not preflight(args.robot_root, poses_deg, args.preflight_box_deg, args.min_camera_z, args.min_lowest_z):
            if not args.force:
                print("pre-flight FAILED -> refusing to move. Use --force only after a human check.", file=sys.stderr)
                return 4

    if args.dry_run:
        order = order_poses(poses_deg, poses_deg[0])
        print(f"dry-run: {len(poses_deg)} poses, order={order}")
        print(f"kp={KP.tolist()} kd={KD.tolist()} ff_clamp={FF_CLAMP.tolist()}")
        return 0

    robot_root = str(args.robot_root)
    if robot_root not in sys.path:
        sys.path.insert(0, robot_root)
    from reBotArm_control_py.actuator import RebotArm

    args.log_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime("static_poses_%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"{run_id}.jsonl"

    # sequence: ordered poses, then a repeat of pose 0, then back to start
    order = order_poses(poses_deg, poses_deg[0])
    sequence = [(f"pose_{i}", poses_deg[i]) for i in order]
    sequence.append(("pose_0_repeat", poses_deg[0]))

    with tempfile.TemporaryDirectory(prefix="rebot-payload-static-") as temp_dir:
        cfg_path = Path(temp_dir) / "rebotarm_camera_j1j6.yaml"
        make_j1j6_config(cfg_path)
        robot = RebotArm(hw_yaml=str(cfg_path))
        with log_path.open("w", encoding="utf-8") as f:
            def write(record: dict[str, Any], show: bool = False) -> None:
                event = {"t_wall": time.time(), **record}
                f.write(json.dumps(event, ensure_ascii=False) + "\n")
                f.flush()
                if show:
                    print(json.dumps(event, ensure_ascii=False), flush=True)

            write(
                {
                    "kind": "start",
                    "run_id": run_id,
                    "poses_deg": poses_deg.tolist(),
                    "order": order,
                    "dwell_s": args.dwell_s,
                    "settle_s": args.settle_s,
                    "rate_hz": args.rate,
                    "kp": KP.tolist(),
                    "kd": KD.tolist(),
                    "ff_clamp_nm": FF_CLAMP.tolist(),
                    "max_error_rad": args.max_error,
                    "max_velocity_rad_s": args.max_velocity,
                    "velocity_trip_count": args.velocity_trip_count,
                    "note": "Target 3.4 static equilibrium sampling; J1-J6 only; J7 never registered or commanded",
                },
                show=True,
            )
            try:
                robot.connect()
                q0 = robot.arm.get_positions().copy()
                write({"kind": "connected", "q0_rad": q0.tolist(), "q0_deg": deg(q0)}, show=True)

                robot.arm.mode_mit()
                robot.arm.enable()
                write({"kind": "enabled", "group": "arm"}, show=True)

                dt = 1.0 / args.rate
                q_prev = robot.arm.get_positions().copy()
                t_prev = time.time()
                vel_filt = np.zeros(6)
                high_velocity_count = 0
                sample_idx = 0
                abort_reason: str | None = None
                started = time.time()
                q_hold = q0.copy()

                for label, q_target_deg in sequence:
                    q_target = np.radians(q_target_deg)
                    max_delta = float(np.max(np.abs(q_target - q_hold)))
                    ramp_s = max(args.min_ramp_s, np.degrees(max_delta) / args.max_deg_per_s)
                    write(
                        {
                            "kind": "move_start",
                            "label": label,
                            "q_target_deg": deg(q_target),
                            "max_delta_deg": round(float(np.degrees(max_delta)), 3),
                            "ramp_s": round(ramp_s, 2),
                        },
                        show=True,
                    )

                    blocks: list[tuple[str, list[np.ndarray]]] = [
                        ("ramp", make_segment(q_hold, q_target, ramp_s, args.rate)),
                        ("dwell", [q_target.copy()] * max(2, int(args.dwell_s * args.rate))),
                    ]
                    for phase, targets in blocks:
                        if abort_reason:
                            break
                        if phase == "dwell":
                            write({"kind": "dwell_start", "label": label}, show=True)
                        for q_cmd in targets:
                            now = time.time()
                            q = robot.arm.get_positions().copy()
                            raw_vel = (q - q_prev) / max(now - t_prev, 1e-4)
                            vel_filt = args.velocity_alpha * raw_vel + (1.0 - args.velocity_alpha) * vel_filt
                            q_prev, t_prev = q, now
                            err = q_cmd - q

                            if np.max(np.abs(err)) > args.max_error:
                                abort_reason = f"tracking error {np.max(np.abs(err)):.3f} rad > {args.max_error}"
                                break
                            if np.max(np.abs(vel_filt)) > args.max_velocity and sample_idx > int(args.rate):
                                high_velocity_count += 1
                            else:
                                high_velocity_count = 0
                            if high_velocity_count >= args.velocity_trip_count:
                                abort_reason = (
                                    f"filtered velocity > {args.max_velocity} rad/s for "
                                    f"{args.velocity_trip_count} consecutive samples"
                                )
                                break

                            tau_model = args.gravity(q)
                            tau_ff = np.clip(tau_model, -FF_CLAMP, FF_CLAMP)
                            tau_pd = KP * err - KD * vel_filt
                            robot.arm.send_mit(q_cmd, vel=np.zeros(6), kp=KP, kd=KD, tau=tau_ff)

                            if sample_idx % max(1, int(args.print_every_s * args.rate)) == 0:
                                write(
                                    {
                                        "kind": "sample",
                                        "label": label,
                                        "phase": phase,
                                        "i": sample_idx,
                                        "q_deg": deg(q),
                                        "q_target_deg": deg(q_cmd),
                                        "err_rad": err.tolist(),
                                        "vel_filt_rad_s": vel_filt.tolist(),
                                        "tau_model_nm": tau_model.tolist(),
                                        "tau_ff_nm": tau_ff.tolist(),
                                        "tau_pd_est_nm": tau_pd.tolist(),
                                    }
                                )
                            sample_idx += 1
                            next_t = now + dt
                            sleep_for = next_t - time.time()
                            if sleep_for > 0:
                                time.sleep(sleep_for)
                        if phase == "dwell" and not abort_reason:
                            q_end = robot.arm.get_positions().copy()
                            write(
                                {
                                    "kind": "dwell_end",
                                    "label": label,
                                    "q_target_deg": deg(q_target),
                                    "q_deg": deg(q_end),
                                    "err_deg": deg(q_target - q_end),
                                    "tau_pd_est_nm": (KP * (q_target - q_end)).tolist(),
                                },
                                show=True,
                            )
                    if abort_reason:
                        write({"kind": "abort", "reason": abort_reason, "label": label}, show=True)
                        break
                    q_hold = q_target

                if not abort_reason:
                    write({"kind": "all_poses_done", "elapsed_s": round(time.time() - started, 2)}, show=True)
                    # return to the starting pose before relaxing
                    ret_s = max(args.min_ramp_s, np.degrees(np.max(np.abs(q0 - q_hold))) / args.max_deg_per_s)
                    write({"kind": "return_start", "return_s": round(ret_s, 2)}, show=True)
                    for q_cmd in make_segment(q_hold, q0, ret_s, args.rate):
                        tau_ff = np.clip(args.gravity(robot.arm.get_positions()), -FF_CLAMP, FF_CLAMP)
                        robot.arm.send_mit(q_cmd, vel=np.zeros(6), kp=KP, kd=KD, tau=tau_ff)
                        time.sleep(dt)
                    write({"kind": "returned_to_start"}, show=True)

                    # fade gains out, keeping the target at the start pose
                    write({"kind": "fade_start", "fade_s": args.fade_s}, show=True)
                    steps = max(2, int(args.fade_s * args.rate))
                    for s in np.linspace(0.0, 1.0, steps):
                        tau_ff = np.clip(args.gravity(robot.arm.get_positions()), -FF_CLAMP, FF_CLAMP) * (1.0 - s)
                        robot.arm.send_mit(q0, vel=np.zeros(6), kp=KP * (1.0 - s), kd=KD * (1.0 - s), tau=tau_ff)
                        time.sleep(dt)
                    write({"kind": "fade_complete"}, show=True)

                q_before = robot.arm.get_positions().copy()
                write({"kind": "pre_disable_pose", "q_rad": q_before.tolist(), "q_deg": deg(q_before)}, show=True)
                robot.arm.disable()
                write(
                    {
                        "kind": "disabled",
                        "abort_reason": abort_reason,
                        "q_deg": deg(q_before),
                        "samples": sample_idx,
                        "elapsed_s": round(time.time() - started, 2),
                    },
                    show=True,
                )
                time.sleep(args.post_disable_s)
                try:
                    q_after = robot.arm.get_positions().copy()
                    write(
                        {
                            "kind": "post_disable_pose",
                            "settle_s": args.post_disable_s,
                            "q_rad": q_after.tolist(),
                            "q_deg": deg(q_after),
                        },
                        show=True,
                    )
                except Exception as exc:
                    write({"kind": "post_disable_pose_error", "error": repr(exc)}, show=True)
            except Exception as exc:
                write({"kind": "error", "error": repr(exc)}, show=True)
                try:
                    robot.arm.disable()
                    write({"kind": "disabled_after_error"}, show=True)
                except Exception as exc2:
                    write({"kind": "disable_after_error_failed", "error": repr(exc2)}, show=True)
            finally:
                shutdown_robot(robot)
                write({"kind": "closed"})

    print(str(log_path))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Target 3.4 multi-pose static equilibrium sampling")
    parser.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    parser.add_argument("--log-dir", type=Path, default=Path("logs/payload-id"))
    parser.add_argument("--pose-json", type=Path, default=None, help="use selected poses from a design_poses.py run")
    parser.add_argument("--pose-indices", default=None, help="comma list, run only these poses (verification subset)")
    parser.add_argument("--dwell-s", type=float, default=6.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument("--min-ramp-s", type=float, default=4.0)
    parser.add_argument("--max-deg-per-s", type=float, default=4.0)
    parser.add_argument("--max-error", type=float, default=0.30)
    parser.add_argument("--max-velocity", type=float, default=0.5)
    parser.add_argument("--velocity-alpha", type=float, default=0.2)
    parser.add_argument("--velocity-trip-count", type=int, default=8)
    parser.add_argument("--print-every-s", type=float, default=1.0)
    parser.add_argument("--fade-s", type=float, default=2.0)
    parser.add_argument("--post-disable-s", type=float, default=1.5)
    parser.add_argument("--limit-margin-deg", type=float, default=2.0)
    parser.add_argument("--preflight-box-deg", type=float, default=4.0)
    parser.add_argument("--min-camera-z", type=float, default=0.09)
    parser.add_argument("--min-lowest-z", type=float, default=0.06)
    parser.add_argument("--no-preflight", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if not args.dry_run:
        root = str(args.robot_root)
        if root not in sys.path:
            sys.path.insert(0, root)
        from pd_hold_probe import make_model_gravity

        args.gravity = make_model_gravity(args.robot_root)
    else:
        args.gravity = lambda q: np.zeros(6)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())