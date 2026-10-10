#!/usr/bin/env python3
"""Offline least-squares identification of camera mass and centre of mass.

Reads a `collect_static_poses.py` JSONL log, reduces each dwell to a settled
static equilibrium point, and fits

    theta = [m, m*rx, m*ry, m*rz]        (camera mass and first moment)

against several competing models of what the measured PD residual actually is.
The models differ in how the URDF's own J7 gripper assembly is accounted for,
and the data decides which is right.

Static balance
--------------
At a settled dwell the servos hold

    tau_feedforward + kp*(q_target - q_settled) = g_true(q_settled)

so the measured residual is

    tau_pd := kp*(q_target - q_settled) = g_true - tau_model

with

    g_true  = g_arm_true + g_camera
    tau_model = g_arm_URDF + g_gripper_URDF        (the URDF still ships the gripper)

Hence, if the arm-body masses are right,

    tau_pd = g_camera - g_gripper                  (model "grip_corrected")

The experiment showed the measured sign is the *opposite* of that prediction on
J2, J3 and J4 simultaneously, so the alternatives are fitted as well:

    "cam_only"       tau_pd = g_camera
    "grip_corrected" tau_pd + g_gripper = g_camera
    "sign_flipped"   tau_pd = g_gripper - g_camera
    "full_arm"       tau_pd + g_gripper = sum_l Delta_phi_l Y_l + g_camera
                     (fits the arm-body mass/COM error at the same time)

Every model is also fitted with a free constant offset per joint, because the
measured residual carries a large pose-independent component (friction, cable
forces, constant model bias) that would otherwise be aliased into theta.
"""

from __future__ import annotations

import argparse
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
# NOTE: in the pinocchio model the joint names are joint1..joint6 (the URDF joint
# names), not the link names.  Body j of joint id J occupies parameter columns
# 10*(J-1) .. 10*(J-1)+9 of computeJointTorqueRegressor.
DEFAULT_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


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


def arm_regressor(pin, model, q_model: np.ndarray, joints=DEFAULT_ARM_JOINTS) -> np.ndarray:
    """nv x (4*len(joints)) regressor for each body's [m, m*cx, m*cy, m*cz].

    `computeJointTorqueRegressor` gives tau = Y @ phi for the full dynamic
    parameter vector of every body; with zero velocity and acceleration only the
    gravity part survives, and gravity only depends on the first four parameters
    of each body, so those columns are selected.
    """
    data = model.createData()
    Y = pin.computeJointTorqueRegressor(model, data, q_model, np.zeros(model.nv), np.zeros(model.nv))
    cols = []
    for name in joints:
        jid = model.getJointId(name)
        if jid == 0:
            raise ValueError(f"joint {name!r} not found in the model (names: {list(model.names)})")
        base = 10 * (jid - 1)
        cols.extend([base + 0, base + 1, base + 2, base + 3])
    return Y[: model.nv, cols]


# --------------------------------------------------------------------------
# dwell reduction
# --------------------------------------------------------------------------
def load_dwells(path: Path, settle_s: float) -> tuple[list[dict], dict]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    start = next((r for r in records if r.get("kind") == "start"), {})
    samples: dict[str, list[dict]] = {}
    for r in records:
        if r.get("kind") == "sample" and r.get("phase") == "dwell":
            samples.setdefault(r["label"], []).append(r)

    rate = float(start.get("rate_hz", 40.0))
    kp = np.array(start["kp"])
    keep = max(2, int(settle_s * rate))
    dwells = []
    for label, rows in samples.items():
        rows = sorted(rows, key=lambda r: r["i"])
        window = rows[-keep:]
        q = np.array([r["q_deg"] for r in window])
        q_target = np.array([r["q_target_deg"] for r in window])
        tau_pd = np.array([r["tau_pd_est_nm"] for r in window])
        tau_model = np.array([r["tau_model_nm"] for r in window])
        vel = np.array([r["vel_filt_rad_s"] for r in window])
        dt = 1.0 / rate
        drift = float(np.max(np.abs(q[-1] - q[0])))
        dwells.append(
            {
                "label": label,
                "n": len(window),
                "q_deg": q.mean(axis=0),
                "q_target_deg": q_target.mean(axis=0),
                "err_deg": (q_target.mean(axis=0) - q.mean(axis=0)),
                "tau_pd_nm": tau_pd.mean(axis=0),
                "tau_pd_std": tau_pd.std(axis=0),
                "tau_model_nm": tau_model.mean(axis=0),
                "q_std_deg": q.std(axis=0),
                "vel_max": float(np.max(np.abs(vel))),
                "drift_deg": drift,
                "kp": kp,
                "duration_s": len(window) * dt,
            }
        )
    return dwells, start


# --------------------------------------------------------------------------
# fitting
# --------------------------------------------------------------------------
def arm_error(qm, pin, model, scales: dict) -> np.ndarray:
    """E_old(q) = sum_j (k_j - 1) * tau_model_j(q) for the calibrated joints."""
    if not scales:
        return np.zeros(6)
    tau_model = gravity_tau(pin, model, qm)[:6]
    E = np.zeros(6)
    for name, k in scales.items():
        j = int(name[5:]) - 1
        E[j] = (float(k) - 1.0) * tau_model[j]
    return E


def build_design(dwells, pin, model, model_nog, frame_id, pad_q, model_name: str, with_offsets: bool,
                 arm_joints, arm_scales=None):
    ys, As, meta = [], [], []
    for d in dwells:
        q6 = np.radians(d["q_deg"])
        qm = pad_q(model, q6, controlled_joints=6)
        A = payload_regressor(pin, model, qm, frame_id)[:6]
        tau_grip = (gravity_tau(pin, model, qm) - gravity_tau(pin, model_nog, qm))[:6]
        tau_pd = d["tau_pd_nm"]
        E = arm_error(qm, pin, model, arm_scales)
        if model_name == "cam_only":
            y, X = tau_pd, A
        elif model_name == "grip_corrected":
            y, X = tau_pd + tau_grip, A
        elif model_name == "sign_flipped":
            y, X = tau_pd - tau_grip, -A
        elif model_name == "calibrated":
            # tau_pd = E_old + g_camera - tau_gripper   ->   tau_pd + tau_gripper - E_old = A theta
            y, X = tau_pd + tau_grip - E, A
        elif model_name == "calibrated_nogrip":
            y, X = tau_pd - E, A
        elif model_name == "calibrated_flip":
            y, X = tau_pd - tau_grip - E, -A
        elif model_name == "full_arm":
            y, X = tau_pd + tau_grip, np.hstack([A, arm_regressor(pin, model, qm, arm_joints)[:6]])
        else:
            raise ValueError(model_name)
        ys.append(y)
        As.append(X)
        meta.append({"label": d["label"], "tau_grip": tau_grip, "tau_pd": tau_pd, "E_old": E})
    Y = np.concatenate(ys)
    X = np.vstack(As)
    if with_offsets:
        n = len(dwells)
        B = np.zeros((6 * n, 6))
        for p in range(n):
            B[6 * p : 6 * p + 6, :] = np.eye(6)
        X = np.hstack([X, B])
    return Y, X, meta


def fit_and_report(Y, X, n_cam=4):
    theta, *_ = np.linalg.lstsq(X, Y, rcond=None)
    resid = Y - X @ theta
    cam = theta[:n_cam]
    m = float(cam[0])
    r = cam[1:4] / m if abs(m) > 1e-9 else np.full(3, np.nan)
    sig = np.linalg.svd(X, compute_uv=False)
    return {
        "m_kg": m,
        "r_local_m": r.tolist(),
        "theta": cam.tolist(),
        "offsets": theta[n_cam + 4 * 6 : n_cam + 4 * 6 + 6].tolist() if len(theta) > n_cam else None,
        "n_params": len(theta),
        "resid_rms_nm": float(np.sqrt(np.mean(resid**2))),
        "resid_max_nm": float(np.max(np.abs(resid))),
        "cond": float(sig[0] / max(sig[-1], 1e-12)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Fit camera mass and COM from static dwell data")
    ap.add_argument("log", type=Path, help="JSONL produced by collect_static_poses.py")
    ap.add_argument("--robot-root", type=Path, default=DEFAULT_ROBOT_ROOT)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--settle-s", type=float, default=2.5, help="use this much of the tail of each dwell")
    ap.add_argument("--models", default="cam_only,grip_corrected,calibrated")
    ap.add_argument("--arm-joints", default=",".join(DEFAULT_ARM_JOINTS), help="joints whose mass/COM error is fitted by 'full_arm'")
    ap.add_argument("--arm-error", type=Path, default=None, help="arm_error_fit.json from fit_arm_error.py")
    ap.add_argument("--arm-scales", default=None, help='override, e.g. "joint2=0.948,joint3=0.952,joint4=0.885"')
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--detail", default=None, help="model tag (e.g. calibrated) whose per-pose residuals to print")
    ap.add_argument("--mass-scan", default=None, help="comma list of masses; fix m and solve for the first moment")
    args = ap.parse_args()
    arm_joints = tuple(x.strip() for x in args.arm_joints.split(",") if x.strip())

    arm_scales = None
    if args.arm_scales:
        arm_scales = {k: float(v) for k, v in (p.split("=") for p in args.arm_scales.split(","))}
    elif args.arm_error:
        arm_scales = json.loads(args.arm_error.read_text(encoding="utf-8")).get("scales")
    print(f"arm model scales (k) : {arm_scales}")

    pin, load_robot_model, get_ee_id, pad_q = load_robot_modules(args.robot_root)
    model = load_robot_model()
    frame_id = get_ee_id(model)
    urdf_path = Path(args.robot_root) / args.urdf

    dwells, start = load_dwells(args.log, args.settle_s)
    print(f"log      : {args.log}")
    print(f"run_id   : {start.get('run_id')}")
    print(f"kp       : {start.get('kp')}")
    print(f"dwells   : {len(dwells)}")

    # ---- self-test: the torque regressor must reproduce the model's own gravity ----
    q_test = pad_q(model, np.radians(np.array([0.0, 30.0, 60.0, 15.0, 0.0, 30.0])), controlled_joints=6)
    all_joints = [n for n in model.names if n != "universe"]
    Y_full = pin.computeJointTorqueRegressor(
        model, model.createData(), q_test, np.zeros(model.nv), np.zeros(model.nv)
    )
    phi_full = np.concatenate(
        [model.inertias[model.getJointId(n)].toDynamicParameters() for n in all_joints]
    )
    g_model = gravity_tau(pin, model, q_test)[:6]
    err_full = float(np.max(np.abs(g_model - (Y_full @ phi_full)[:6])))
    # and the truncated [m, m*cx, m*cy, m*cz] columns used by full_arm must agree too
    Y_sel = arm_regressor(pin, model, q_test, all_joints)
    phi_sel = np.concatenate(
        [model.inertias[model.getJointId(n)].toDynamicParameters()[:4] for n in all_joints]
    )
    err_sel = float(np.max(np.abs(g_model - (Y_sel @ phi_sel)[:6])))
    print(f"regressor self-test : full 10-param err = {err_full:.3e}, truncated 4-param err = {err_sel:.3e} "
          f"({'OK' if max(err_full, err_sel) < 1e-8 else 'FAIL'})")
    print(f"arm joints fitted by full_arm: {list(arm_joints)}")
    print()
    print("=== settled dwell summary ===")
    print(f"{'label':<18}{'n':>4} {'drift_deg':>9} {'q_std_max':>10} {'vel_max':>9} | {'tau_pd (N*m) J1..J6'}")
    for d in sorted(dwells, key=lambda d: d["label"]):
        print(
            f"{d['label']:<18}{d['n']:>4} {d['drift_deg']:>9.4f} {np.max(d['q_std_deg']):>10.4f} {d['vel_max']:>9.5f} | "
            + " ".join(f"{v:+7.3f}" for v in d["tau_pd_nm"])
        )

    # repeatability: first pose vs its repeat visit
    rep = [d for d in dwells if d["label"] == "pose_0_repeat"]
    first = [d for d in dwells if d["label"] == "pose_0"]
    if rep and first:
        dq = rep[0]["q_deg"] - first[0]["q_deg"]
        dtau = rep[0]["tau_pd_nm"] - first[0]["tau_pd_nm"]
        print()
        print("=== repeatability (pose_0 vs pose_0_repeat) ===")
        print(f"  dq_deg   = {np.round(dq, 4).tolist()}")
        print(f"  dtau_pd  = {np.round(dtau, 4).tolist()}  (max |dtau| = {np.max(np.abs(dtau)):.4f} N*m)")

    with tempfile.TemporaryDirectory(prefix="payload-fit-") as td:
        model_nog, touched = build_model_without_gripper(pin, urdf_path, Path(td))
    print(f"  gripper links zeroed: {touched}")

    results = {}
    print()
    print("=" * 100)
    if args.mass_scan:
        grid = [float(x) for x in args.mass_scan.split(",")]
        for model_name in args.models.split(","):
            for with_offsets in (False, True):
                Y, X, meta = build_design(
                    dwells, pin, model, model_nog, frame_id, pad_q, model_name, with_offsets, arm_joints, arm_scales
                )
                print(f"\n=== mass scan: {model_name}{'+offsets' if with_offsets else ''} ===")
                print(f"{'m [kg]':>8} {'vx':>9} {'vy':>9} {'vz':>9} {'r=[x,y,z] m':>28} {'rms N*m':>9} {'max':>8}")
                best = None
                for m in grid:
                    Y2 = Y - m * X[:, 0]
                    X2 = np.hstack([X[:, 1:4], X[:, 4:]]) if X.shape[1] > 4 else X[:, 1:4]
                    th, *_ = np.linalg.lstsq(X2, Y2, rcond=None)
                    r = Y2 - X2 @ th
                    rms = float(np.sqrt(np.mean(r**2)))
                    v = th[:3]
                    rr = v / m if abs(m) > 1e-9 else np.zeros(3)
                    print(
                        f"{m:>8.3f} {v[0]:>9.5f} {v[1]:>9.5f} {v[2]:>9.5f} "
                        f"[{rr[0]:>+8.4f},{rr[1]:>+8.4f},{rr[2]:>+8.4f}] {rms:>9.4f} {np.max(np.abs(r)):>8.4f}"
                    )
                    if best is None or rms < best[0]:
                        best = (rms, m, rr.copy(), th.copy())
                print(f"  best on this grid: m={best[1]:.3f} kg  r=[{best[2][0]:+.4f}, {best[2][1]:+.4f}, "
                      f"{best[2][2]:+.4f}] m  rms={best[0]:.4f} N*m")
        print("=" * 100)
        return 0

    for model_name in args.models.split(","):
        for with_offsets in (False, True):
            tag = f"{model_name}{'+offsets' if with_offsets else ''}"
            Y, X, meta = build_design(
                dwells, pin, model, model_nog, frame_id, pad_q, model_name, with_offsets, arm_joints, arm_scales
            )
            res = fit_and_report(Y, X)
            results[tag] = res
            print(
                f"{tag:<26} m={res['m_kg']:+8.4f} kg   r_local=[{res['r_local_m'][0]:+.4f}, "
                f"{res['r_local_m'][1]:+.4f}, {res['r_local_m'][2]:+.4f}] m   "
                f"rms={res['resid_rms_nm']:7.4f} N*m  max={res['resid_max_nm']:7.4f}  "
                f"npar={res['n_params']:>2}  cond={res['cond']:.1f}"
            )
            if res["offsets"] is not None:
                print(f"{'':<26} offsets (J1..J6) = {np.round(res['offsets'], 3).tolist()}")
            if args.detail and tag == args.detail:
                theta_full, *_ = np.linalg.lstsq(X, Y, rcond=None)
                pred = X @ theta_full
                resid_all = Y - pred
                print(f"\n  --- per-pose detail for {tag} (N*m) ---")
                print(f"  {'label':<14} {'joint':>5} {'y(meas)':>9} {'pred':>9} {'resid':>9}")
                for p, d in enumerate(dwells):
                    for j in (1, 2, 3):
                        row = 6 * p + j
                        print(
                            f"  {d['label']:<14} {'J'+str(j+1):>5} {Y[row]:>9.3f} {pred[row]:>9.3f} "
                            f"{resid_all[row]:>9.3f}"
                        )
                print()
            if model_name == "full_arm":
                arm = np.array(res["theta"])[4:]
                names = [f"{j}:{k}" for j in arm_joints for k in ("m", "m*cx", "m*cy", "m*cz")]
                print(f"{'':<26} arm corrections: " + ", ".join(f"{n}={v:+.4f}" for n, v in zip(names, arm)))
    print("=" * 100)
    print()
    print("A physically credible answer needs m in roughly [0.7, 1.0] kg (the camera was weighed at ~0.8-0.9 kg)")
    print("and the smallest residual.  A negative or absurd mass means that model's sign convention is wrong.")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(
            json.dumps(
                {
                    "log": str(args.log),
                    "settle_s": args.settle_s,
                    "dwells": [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in d.items()} for d in dwells],
                    "results": results,
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