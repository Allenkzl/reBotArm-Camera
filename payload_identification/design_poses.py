#!/usr/bin/env python3
"""Offline pose design, safety preview and identifiability analysis.

Never touches CAN.  Pure Pinocchio arithmetic.

Physics / why the residual is small
-----------------------------------
The shipped URDF still contains the J7 gripper assembly:

    gripper_end   0.65    kg
    gripper_left  0.0752  kg
    gripper_right 0.0752  kg
    --------------------------
    total         0.8004  kg

so `compute_generalized_gravity` already carries the gripper's weight.  After
the gripper is swapped for a camera, what the PD loop must make up is the
*change* in payload:

    tau_pd = tau_gravity_true - tau_model
           = (tau_arm_true - tau_arm) + tau_camera - tau_gripper

Assuming the arm-body masses are right:

    tau_pd = tau_camera - tau_gripper          (measurable)

and therefore

    tau_pd + tau_gripper(q) = tau_camera(q) = A(q) @ theta,   theta = [m, m*rx, m*ry, m*rz]

A(q) is linear in theta (derivation in `payload_regressor`).  The problem is
that tau_camera and tau_gripper co-vary strongly with pose, so the *measurable*
signal (tau_pd) has a much smaller dynamic range than either term.  This script
quantifies whether the remaining variation is enough to identify theta given a
realistic torque noise floor, and picks poses that make it as well conditioned
as possible.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
DEFAULT_URDF = "urdf/RS/urdf/ReBot_Arm_RS.urdf"
G = 9.81
GRIPPER_LINKS = ("gripper_end", "gripper_left", "gripper_right")
SAFETY_FRAMES = ("link3", "link4", "link5", "link6", "gripper_end")


def load_robot_modules(robot_root: Path):
    root = str(robot_root)
    if root not in sys.path:
        sys.path.insert(0, root)
    import pinocchio as pin

    from reBotArm_control_py.kinematics.robot_model import (
        get_end_effector_frame_id,
        load_robot_model,
        pad_q_for_model,
    )

    return pin, load_robot_model, get_end_effector_frame_id, pad_q_for_model


def build_model_without_gripper(pin, urdf_path: Path, tmp_dir: Path):
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()
    touched = []
    for link in root.findall("link"):
        if link.get("name") not in GRIPPER_LINKS:
            continue
        touched.append(link.get("name"))
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        if mass is not None:
            mass.set("value", "1e-9")
        inertia = inertial.find("inertia")
        if inertia is not None:
            for axis in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia.set(axis, "1e-12")
    out = tmp_dir / "no_gripper.urdf"
    tree.write(str(out), encoding="utf-8", xml_declaration=True)
    return pin.buildModelFromUrdf(str(out)), touched


def gravity_tau(pin, model, q_model: np.ndarray) -> np.ndarray:
    from reBotArm_control_py.dynamics.inverse_dynamics import compute_generalized_gravity

    return compute_generalized_gravity(model=model, q=q_model, data=model.createData())[: model.nv]


def payload_regressor(pin, model, q_model: np.ndarray, frame_id: int) -> np.ndarray:
    """A (nv x 4) such that tau_payload = A @ [m, m*rx, m*ry, m*rz]."""
    data = model.createData()
    pin.forwardKinematics(model, data, q_model)
    pin.computeJointJacobians(model, data, q_model)
    pin.updateFramePlacements(model, data)
    jac = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    R = data.oMf[frame_id].rotation
    A = np.zeros((model.nv, 4))
    A[:, 0] = G * jac[2, :]
    omega = jac[3:, :]
    for k in range(3):
        A[:, 1 + k] = -G * R[0, k] * omega[1, :] + G * R[1, k] * omega[0, :]
    return A


def sdk_payload_tau(pin, model, q_model, frame_id, kg, offset) -> np.ndarray:
    data = model.createData()
    pin.forwardKinematics(model, data, q_model)
    pin.computeJointJacobians(model, data, q_model)
    pin.updateFramePlacements(model, data)
    jac = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    r = data.oMf[frame_id].rotation @ offset
    skew = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    j_point = jac[:3, :] - skew @ jac[3:, :]
    return (kg * G * j_point[2, :])[: model.nv]


def safety_metrics(pin, model, q_model, frame_id, camera_offset) -> dict:
    data = model.createData()
    pin.forwardKinematics(model, data, q_model)
    pin.updateFramePlacements(model, data)
    lowest_name, lowest_z = None, np.inf
    for name in SAFETY_FRAMES:
        fid = model.getFrameId(name)
        if fid >= len(model.frames):
            continue
        z = float(data.oMf[fid].translation[2])
        if z < lowest_z:
            lowest_name, lowest_z = name, z
    T = data.oMf[frame_id]
    cam = T.rotation @ camera_offset + T.translation
    return {
        "lowest_frame": lowest_name,
        "lowest_z": round(float(lowest_z), 4),
        "camera_z": round(float(cam[2]), 4),
        "camera_xy": [round(float(cam[0]), 4), round(float(cam[1]), 4)],
        "cam_r_xy": round(float(np.hypot(cam[0], cam[1])), 4),
    }


def uncertainty(design: np.ndarray, sigma_tau: float) -> np.ndarray:
    """Parameter covariance for tau = design @ params, white noise sigma_tau."""
    design = np.atleast_2d(design)
    gram = design.T @ design
    return sigma_tau**2 * np.linalg.pinv(gram)


def report(sigma_theta: np.ndarray, label: str, nominal_kg: float = 0.85) -> dict:
    """sigma_theta is the covariance of theta = [m, m*rx, m*ry, m*rz]."""
    sd = np.sqrt(np.diag(sigma_theta))
    # sigma of the COM components: r_k = (m*r_k)/m, so relative sigma propagates.
    sigma_r = [float(sd[1 + k] / nominal_kg) for k in range(3)]
    out = {
        "label": label,
        "sigma_m_kg": float(sd[0]),
        "sigma_m_rel_pct": float(100.0 * sd[0] / nominal_kg),
        "sigma_rx_m": sigma_r[0],
        "sigma_ry_m": sigma_r[1],
        "sigma_rz_m": sigma_r[2],
    }
    print(
        f"  {label:<28} sigma_m={sd[0]:.4f} kg ({out['sigma_m_rel_pct']:4.1f}%)   "
        f"sigma_v=[{sd[1]:.5f}, {sd[2]:.5f}, {sd[3]:.5f}] kg*m   "
        f"-> sigma_r(COM) ~ [{sigma_r[0]*1000:.1f}, {sigma_r[1]*1000:.1f}, {sigma_r[2]*1000:.1f}] mm"
    )
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline pose design and identifiability analysis")
    ap.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--q0-deg", default="-6.11,0.08,0.97,-1.15,0.21,32.86")
    ap.add_argument("--camera-kg", type=float, default=0.85)
    ap.add_argument("--camera-offset", default="0.06,0.0,0.0")
    ap.add_argument("--d2", default="0,10,20,30,40,50")
    ap.add_argument("--d3", default="0,15,30,45,60")
    ap.add_argument("--d4", default="-30,-15,0,15,30")
    ap.add_argument("--d5", default="-20,0,20")
    ap.add_argument("--d6", default="-40,0,40")
    ap.add_argument("--select", type=int, default=8)
    ap.add_argument("--min-lowest-z", type=float, default=0.07)
    ap.add_argument("--min-camera-z", type=float, default=0.09)
    ap.add_argument("--sigma-tau", type=float, default=0.05)
    ap.add_argument("--kp", default="8,35,40,25,5,5", help="PD kp actually used in the experiment")
    ap.add_argument("--ff-clamp", default="1.5,8.0,12.0,5.0,0.8,0.8")
    ap.add_argument("--limit-margin-deg", type=float, default=3.0, help="required clearance of the SETTLED pose from any limit")
    ap.add_argument("--out", type=Path, default=Path("logs/payload-id/pose_design.json"))
    args = ap.parse_args()

    pin, load_robot_model, get_ee_id, pad_q = load_robot_modules(args.robot_root)
    urdf_path = Path(args.robot_root) / args.urdf
    model = load_robot_model()
    frame_id = get_ee_id(model)
    q0 = np.radians(np.array([float(x) for x in args.q0_deg.split(",")]))
    cam_off = np.array([float(x) for x in args.camera_offset.split(",")])

    print(f"model nq={model.nq} nv={model.nv} ee_frame={model.frames[frame_id].name}")

    q_model0 = pad_q(model, q0, controlled_joints=6)
    A0 = payload_regressor(pin, model, q_model0, frame_id)
    theta_true = np.array([args.camera_kg, *(args.camera_kg * cam_off)])
    direct = sdk_payload_tau(pin, model, q_model0, frame_id, args.camera_kg, cam_off)
    err = float(np.max(np.abs(direct[:6] - (A0 @ theta_true)[:6])))
    print(f"self-test regressor vs SDK payload: max abs diff = {err:.3e} N*m")
    if err > 1e-8:
        print("REGRESSOR MISMATCH", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="pose-design-") as td:
        model_nog, touched = build_model_without_gripper(pin, urdf_path, Path(td))
    print(f"gripper links zeroed: {touched}")

    grids = {
        1: [float(x) for x in args.d2.split(",")],
        2: [float(x) for x in args.d3.split(",")],
        3: [float(x) for x in args.d4.split(",")],
        4: [float(x) for x in args.d5.split(",")],
        5: [float(x) for x in args.d6.split(",")],
    }
    lower = np.radians(np.array([-160.0, 0.0, 0.0, -89.9, -89.9, -179.9]))
    upper = np.radians(np.array([160.0, 179.9, 179.9, 89.9, 89.9, 179.9]))
    kp = np.array([float(x) for x in args.kp.split(",")])
    ff_clamp = np.array([float(x) for x in args.ff_clamp.split(",")])
    margin = np.radians(args.limit_margin_deg)

    def evaluate(q_target: np.ndarray):
        """Predict the settled pose under PD control and evaluate everything there.

        At equilibrium kp*(q_target - q_settled) = tau_pd, so a joint settles at
        q_settled = q_target - tau_pd/kp.  The regressor and the safety checks must
        use q_settled, because that is the pose the robot will actually report.
        """
        q_a = q_target.copy()
        for _ in range(3):
            qm = pad_q(model, q_a, controlled_joints=6)
            A = payload_regressor(pin, model, qm, frame_id)[:6]
            tau_cam = A @ theta_true
            tau_grip = (gravity_tau(pin, model, qm) - gravity_tau(pin, model_nog, qm))[:6]
            tau_model = gravity_tau(pin, model, qm)[:6]
            tau_pred = tau_cam - tau_grip
            q_a = q_target - tau_pred / kp
        return q_a, A, tau_cam, tau_grip, tau_pred, tau_model

    max_model = np.zeros(6)
    cands = []
    n_limit_reject = 0
    for combo in itertools.product(*[grids[k] for k in sorted(grids)]):
        q_t = q0.copy()
        for idx, k in enumerate(sorted(grids)):
            q_t[k] += np.radians(combo[idx])
        if np.any(q_t < lower) or np.any(q_t > upper):
            continue
        q_a, A, tau_cam, tau_grip, tau_pred, tau_model = evaluate(q_t)
        max_model = np.maximum(max_model, np.abs(tau_model))
        # the SETTLED pose, not the commanded one, is what must respect limits
        if np.any(q_a < lower + margin) or np.any(q_a > upper - margin):
            n_limit_reject += 1
            continue
        qm_a = pad_q(model, q_a, controlled_joints=6)
        safe = safety_metrics(pin, model, qm_a, frame_id, cam_off)
        if safe["lowest_z"] < args.min_lowest_z or safe["camera_z"] < args.min_camera_z:
            continue
        cands.append(
            {
                "dq_deg": list(combo),
                "q_target_deg": [round(float(np.degrees(v)), 3) for v in q_t],
                "q_settled_deg": [round(float(np.degrees(v)), 3) for v in q_a],
                "sag_deg": [round(float(np.degrees(v)), 3) for v in (q_t - q_a)],
                "A": A,
                "tau_camera": tau_cam[:6],
                "tau_gripper": tau_grip[:6],
                "tau_pred_pd": tau_pred[:6],
                **safe,
            }
        )

    print(f"max |tau_model| per joint over candidates: {np.round(max_model, 2).tolist()}")
    covered = np.all(max_model <= ff_clamp)
    print(f"  -> ff_clamp {ff_clamp.tolist()} {'COVERS it (no clamping)' if covered else 'DOES NOT COVER (clamping occurs)'}")
    print(f"candidates rejected because the SETTLED pose hit a limit: {n_limit_reject}")
    print(f"candidates surviving joint limits AND safety: {len(cands)}")
    if len(cands) < args.select:
        print("not enough safe candidates", file=sys.stderr)
        return 3

    # ---- the real difficulty: how much does the measurable residual vary? ----
    pred = np.array([c["tau_pred_pd"] for c in cands])
    print()
    print("=== dynamic range of the MEASURABLE residual tau_pd (N*m) ===")
    names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    for i in range(6):
        col = pred[:, i]
        print(f"  {names[i]}: min={col.min():+.3f} max={col.max():+.3f} span={col.max()-col.min():.3f} |absmax|={np.abs(col).max():.3f}")

    # ---- scaled D-optimal selection ----
    scale = np.array([0.85, 0.85 * 0.06, 0.85 * 0.06, 0.85 * 0.06])
    def design_of(idx):
        A = np.vstack([cands[i]["A"] for i in idx])          # 6*len(idx) x 4
        return A @ np.diag(scale)

    remaining = list(range(len(cands)))
    selected = []
    sv = {i: np.linalg.svd(design_of([i]), compute_uv=False)[-1] for i in remaining}
    best = max(sv, key=sv.get)
    selected.append(best)
    remaining.remove(best)
    while len(selected) < args.select and remaining:
        best_i, best_score = None, -np.inf
        for i in remaining:
            sig = np.linalg.svd(design_of(selected + [i]), compute_uv=False)
            score = float(np.sum(np.log(np.maximum(sig, 1e-12))))
            if score > best_score:
                best_i, best_score = i, score
        selected.append(best_i)
        remaining.remove(best_i)

    sel = [cands[i] for i in selected]
    print()
    print(f"=== selected {len(sel)} poses (greedy scaled D-optimal) ===")
    for c in sel:
        print(
            f"  dq={[round(x) for x in c['dq_deg']]}  target={c['q_target_deg']}  settled={c['q_settled_deg']}  "
            f"sag={c['sag_deg']}"
        )
        print(
            f"      tau_pd_pred(J2,J3,J4)={np.round(c['tau_pred_pd'][1:4], 3).tolist()}  "
            f"cam_z={c['camera_z']}  lowest_z={c['lowest_z']} ({c['lowest_frame']})"
        )

    D = design_of(selected)
    sig = np.linalg.svd(D, compute_uv=False)
    print(f"  scaled singular values: {np.round(sig, 4).tolist()}   cond={float(sig[0]/sig[-1]):.2f}")

    # ---- identifiability with a realistic torque noise floor ----
    print()
    print(f"=== expected parameter uncertainty (sigma_tau = {args.sigma_tau} N*m per joint-sample) ===")
    results = [report(uncertainty(D, args.sigma_tau), "4 params, no nuisance", args.camera_kg)]
    # per-joint constant offsets absorb friction / constant model bias
    n = len(selected)
    B = np.zeros((6 * n, 6))
    for p in range(n):
        B[6 * p : 6 * p + 6, :] = np.eye(6)
    Dn = np.hstack([np.vstack([cands[i]["A"] for i in selected]), B])
    cov = uncertainty(Dn, args.sigma_tau)[:4, :4]
    results.append(report(cov, "4 params + 6 joint offsets", args.camera_kg))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        json.dumps(
            {
                "q0_deg": args.q0_deg,
                "camera_kg": args.camera_kg,
                "camera_offset": args.camera_offset,
                "sigma_tau": args.sigma_tau,
                "selected": [
                    {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in c.items() if k != "A"}
                    for c in sel
                ],
                "selected_q_deg": [c["q_target_deg"] for c in sel],
                "selected_q_settled_deg": [c["q_settled_deg"] for c in sel],
                "kp": [float(x) for x in args.kp.split(",")],
                "ff_clamp": [float(x) for x in args.ff_clamp.split(",")],
                "uncertainty": results,
                "n_candidates": len(cands),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())