#!/usr/bin/env python3
"""Fit the URDF's arm-body model error from the 2026-07-17 gravity sweeps.

The URDF masses were calibrated on 2026-07-17 with the J7 gripper still fitted.
Those contact-free PD sweeps are archived in

    tools/gravity_calibration/data/pdsweep_*.json

and are the *baseline* the camera-identification problem was missing.  Each
sample carries the full 6-joint pose, the active joint, the measured holding
torque and the sweep direction.

Model error
-----------
Write the gravity torque split into rigidly-attached bodies:

    tau_model(q) = sum_i Y_i(q) phi_i^URDF            (what the URDF predicts)
    g_true_old(q) = sum_i Y_i(q) phi_i^true           (reality, gripper fitted)

with Y_i the unit-parameter gravity regressor of body i (only the first four
parameters [m, m*cx, m*cy, m*cz] matter for gravity).  Then

    tau_pd_old(q) = g_true_old(q) - tau_model(q) = sum_i Y_i(q) dphi_i

and this same error function carries over to the camera build, because the arm
bodies did not change:

    tau_pd_new(q) = sum_i Y_i(q) dphi_i  +  g_camera(q)  -  tau_gripper(q)

so the camera is isolated by

    tau_pd_new(q) + tau_gripper_URDF(q) - E_old(q) = A(q) theta_camera

Note link6 and gripper_end are rigidly connected, so their regressors coincide;
the fit therefore estimates their *lumped* error, which is exactly what the
final correction needs.
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

DEFAULT_ROBOT_ROOT = Path("/home/com/reBotArm_control_py")
DEFAULT_DATA = "/home/com/reBotArm_control_py/tools/gravity_calibration/data/pdsweep_*.json"
DEFAULT_URDF = "urdf/RS/urdf/ReBot_Arm_RS.urdf"

# bodies the fit is allowed to correct: the six arm bodies (joint6 already
# carries the lumped gripper_end inertia) plus the two gripper fingers.
FIT_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
              "gripper_joint1", "gripper_joint2")


def load_robot_modules(robot_root: Path):
    root = str(robot_root)
    if root not in sys.path:
        sys.path.insert(0, root)
    import pinocchio as pin

    from reBotArm_control_py.kinematics.robot_model import load_robot_model, pad_q_for_model

    return pin, load_robot_model, pad_q_for_model


def gravity_tau(pin, model, q_model: np.ndarray) -> np.ndarray:
    from reBotArm_control_py.dynamics.inverse_dynamics import compute_generalized_gravity

    return compute_generalized_gravity(model=model, q=q_model, data=model.createData())[: model.nv]


def joint_regressor(pin, model, q_model: np.ndarray, joints) -> np.ndarray:
    data = model.createData()
    Y = pin.computeJointTorqueRegressor(model, data, q_model, np.zeros(model.nv), np.zeros(model.nv))
    cols = []
    for name in joints:
        jid = model.getJointId(name)
        if jid == 0:
            raise ValueError(f"joint {name!r} not found")
        base = 10 * (jid - 1)
        cols.extend([base, base + 1, base + 2, base + 3])
    return Y[:, cols]


def main() -> int:
    ap = argparse.ArgumentParser(description="Fit the URDF arm-body model error from the old sweeps")
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--model", choices=("scale", "dphi"), default="scale",
                    help="'scale': per-joint model scale k (well conditioned, matches the published calibration); "
                         "'dphi': free body parameters (rank deficient with this data)")
    ap.add_argument("--qmin", type=float, default=0.20, help="drop samples whose active joint is below this (contact region)")
    ap.add_argument("--vel-max", type=float, default=1.0, help="drop samples above this |vel_fd| [rad/s]")
    ap.add_argument("--scale-joints", default="joint2,joint3,joint4",
                    help="joints given a free model scale k (J1/J6 carry no gravity torque, J5 is below friction)")
    ap.add_argument("--no-friction", action="store_true", help="drop the direction-dependent friction column")
    ap.add_argument("--out", type=Path, default=Path("logs/payload-id/arm_error_fit.json"))
    args = ap.parse_args()

    pin, load_robot_model, pad_q = load_robot_modules(args.robot_root)
    model = load_robot_model()
    urdf_path = Path(args.robot_root) / args.urdf

    files = sorted(glob.glob(args.data))
    scale_joints = [x.strip() for x in args.scale_joints.split(",") if x.strip()]
    blobs = [(Path(p), json.loads(Path(p).read_text(encoding="utf-8"))) for p in files]
    active_joints = sorted({b["joint"] for _, b in blobs}, key=lambda n: int(n[5:]))
    print(f"sweeps: {len(files)}   model: {args.model}   scale_joints: {scale_joints}")
    print(f"joints present in the data: {active_joints}")
    rows_y, rows_X = [], []
    n_fit = 4 * len(FIT_JOINTS) if args.model == "dphi" else len(scale_joints)
    per_file = {}
    for path, blob in blobs:
        active = blob["joint"]
        kp = float(blob["kp"])
        kept = 0
        for s in blob["samples"]:
            q6 = np.array(s["q6"], dtype=float)
            j = int(active[5:]) - 1
            if abs(q6[j]) < args.qmin:
                continue
            if abs(float(s.get("vel_fd", 0.0))) > args.vel_max:
                continue
            qm = pad_q(model, q6, controlled_joints=6)
            tau_model = gravity_tau(pin, model, qm)[:6]
            if args.model == "dphi":
                Y = joint_regressor(pin, model, qm, FIT_JOINTS)[:6]
                row = Y[j, :].copy()
                y = float(s["tau_est"]) - tau_model[j]
            else:
                row = np.zeros(n_fit)
                if active in scale_joints:
                    row[scale_joints.index(active)] = tau_model[j]
                y = float(s["tau_est"])
            bias = np.zeros(len(active_joints))
            bias[active_joints.index(active)] = 1.0
            fric = np.zeros(len(active_joints))
            fric[active_joints.index(active)] = float(s.get("dir", 1.0))
            rows_X.append(np.concatenate([row, bias, fric]))
            rows_y.append(y)
            kept += 1
        per_file[path.name] = {"active": active, "kp": kp, "kept": kept, "total": len(blob["samples"])}
        print(f"  {path.name:<40} active={active:<8} kp={kp:<5} kept {kept}/{len(blob['samples'])}")

    Y = np.array(rows_y)
    X = np.array(rows_X)
    if args.no_friction:
        X = np.delete(X, np.s_[n_fit + len(active_joints) : n_fit + 2 * len(active_joints)], axis=1)
    print(f"\nequations: {X.shape[0]}   parameters: {X.shape[1]}")

    theta, *_ = np.linalg.lstsq(X, Y, rcond=None)
    resid = Y - X @ theta
    sig = np.linalg.svd(X, compute_uv=False)
    cond = sig[0] / sig[-1] if sig[-1] > 1e-300 else float("inf")
    dof = max(1, X.shape[0] - X.shape[1])
    sigma2 = float(resid @ resid) / dof
    cov = sigma2 * np.linalg.pinv(X.T @ X)
    se = np.sqrt(np.diag(cov))
    print(f"fit rms = {np.sqrt(np.mean(resid**2)):.4f} N*m   max = {np.max(np.abs(resid)):.4f} N*m   cond = {cond:.1f}")
    nb = len(active_joints)
    print(f"bias     ({active_joints}) = {np.round(theta[n_fit:n_fit+nb], 3).tolist()}")
    print(f"friction ({active_joints}) = {np.round(theta[n_fit+nb:n_fit+2*nb], 3).tolist()}")
    scales = None
    if args.model == "scale":
        scales = {name: float(v) for name, v in zip(scale_joints, theta[:n_fit])}
        print("model scale k     = {" + ", ".join(f"{n}: {v:.4f}" for n, v in scales.items()) + "}")
        print("  +- 1 sigma      = {" + ", ".join(
            f"{n}: {s:.4f}" for n, s in zip(scale_joints, se[:n_fit])) + "}")
        print("  (published 2026-07-17 calibration: J2 0.948, J3 0.952, J4 0.885)")
    else:
        q_ref = pad_q(model, np.radians(np.array([0.0, 40.0, 60.0, 15.0, 0.0, 30.0])), controlled_joints=6)
        E_ref = joint_regressor(pin, model, q_ref, FIT_JOINTS)[:6] @ theta[:n_fit]
        tau_ref = gravity_tau(pin, model, q_ref)[:6]
        print("\nimplied model scale k = 1 + E/tau_model at a reference pose:")
        for i in range(1, 4):
            k = 1.0 + E_ref[i] / tau_ref[i] if abs(tau_ref[i]) > 1e-6 else float("nan")
            print(f"  J{i+1}: E={E_ref[i]:+.3f}  tau_model={tau_ref[i]:+.3f}  k={k:.3f}")

    # per-joint residual breakdown
    print("\nresidual rms per active joint:")
    offset = 0
    for name, meta in per_file.items():
        n = meta["kept"]
        r = resid[offset : offset + n]
        if n:
            print(f"  {meta['active']:<8} rms={np.sqrt(np.mean(r**2)):7.4f}  max={np.max(np.abs(r)):7.4f}  n={n}")
        offset += n

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        json.dumps(
            {
                "fit_joints": list(FIT_JOINTS),
                "model": args.model,
                "scales": scales,
                "dphi": theta[:n_fit].tolist() if args.model == "dphi" else None,
                "bias_joints": active_joints,
                "bias": theta[n_fit : n_fit + len(active_joints)].tolist(),
                "friction": theta[n_fit + len(active_joints) : n_fit + 2 * len(active_joints)].tolist(),
                "scale_std": se[:n_fit].tolist(),
                "resid_rms_nm": float(np.sqrt(np.mean(resid**2))),
                "resid_max_nm": float(np.max(np.abs(resid))),
                "n_equations": int(X.shape[0]),
                "cond": float(sig[0] / sig[-1]),
                "per_file": per_file,
                "qmin": args.qmin,
                "vel_max": args.vel_max,
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