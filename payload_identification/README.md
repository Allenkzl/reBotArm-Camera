# Payload Identification

Utilities for estimating the camera payload mass and center of mass from robot
feedback and safe quasi-static motions.

For the full background, the physics, the pitfalls and the current status, read
`agent.md` at the repository root. This file is only the quick reference.

## Scripts

| script | touches CAN? | what it does |
|---|---|---|
| `feedback_audit.py` | read-only | J1-J6 position audit; never enables or moves. Also holds `make_j1j6_config()`, the temporary `/tmp` J1-J6-only config used by every active script. |
| `pd_hold_probe.py` | active | Minimal PD hold probe. Also holds shared helpers: `KP`, `KD`, `minimum_jerk`, `deg`, `make_model_gravity`, `shutdown_robot`. |
| `collect_slow_sweep.py` | active | Single-joint minimum-jerk sweep (Target 3.3). Superseded for identification purposes; see the correction note below. |
| `design_poses.py` | **no** | Offline pose design and identifiability analysis. Payload regressor, gripper-removal model, DO-optimal selection, settled-pose limit check, FK safety preview, expected `sigma_m` / `sigma_r`. |
| `collect_static_poses.py` | active | Multi-pose static equilibrium sampling (Target 3.4). The main data collection script. |
| `fit_arm_error.py` | **no** | Fits the URDF arm-body model error from the archived 2026-07-17 pre-swap gravity sweeps. |
| `fit_payload.py` | **no** | Offline camera mass/COM fit, with model comparison, per-pose residual detail and a mass scan diagnostic. |

## Read-only audit

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/feedback_audit.py \
  --duration 5 --rate 5
```

Registers J1-J6 only, does not enable motors, does not command motion.

## Offline pose design (safe, no motion)

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/design_poses.py \
  --d5=0 --d6=0 --select=10 --min-camera-z=0.20 --min-lowest-z=0.10
```

Watch for the two self-tests at the top of the output; both must be at the
1e-16 level before any result is trusted.

## Static sampling

Always dry-run first. The dry run performs the FK safety pre-flight scan and
prints the pose table without enabling anything:

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py --dry-run
```

Small verification run (2 poses, short dwell):

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py \
  --pose-indices 0,2 --dwell-s 2.5 --min-ramp-s 3
```

Full run (about 3 minutes; requires the operator beside the arm and explicit
confirmation):

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py --dwell-s 6.0
```

The script clamps targets into the joint limits, refuses to move if the
pre-flight scan fails, aborts on excessive tracking error or sustained filtered
velocity, repeats the first pose at the end to measure repeatability, then
returns home, fades the gains out and disables. J7 is never registered or
commanded.

## Offline fitting

```bash
# 1) arm model error from the archived pre-swap sweeps
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_arm_error.py --model scale

# 2) camera parameters (model comparison)
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_payload.py \
  logs/payload-id/static_poses_20260917_180506.jsonl \
  --models calibrated,grip_corrected,cam_only \
  --arm-error logs/payload-id/arm_error_fit.json --settle-s 2.5

# 3) mass scan diagnostic
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_payload.py \
  logs/payload-id/static_poses_20260917_180506.jsonl \
  --models calibrated --arm-error logs/payload-id/arm_error_fit.json \
  --mass-scan 0,0.2,0.4,0.6,0.8,0.85,0.9,1.0,1.2
```

## Corrections worth knowing before you reuse old advice

- **Do not set the target equal to the measured current pose.** `err = q_target
  - q_actual`, so that zeroes the signal being measured. Hold the target fixed
  and wait for the settled equilibrium instead.
- **Do not filter on small tracking error.** The Target 3.3 "best" `negative
  dwell` was limit saturation (the joint resting on its hard stop), not a
  gravity sample. Accept a window when `std(tau_pd)` is flat, the target is
  inside the reachable range, and no limit is touched.
- **Continuous sweep data is not fit-ready.** It is only useful for checking
  that the controls can move the loaded arm and that the residual varies with
  posture.

## Current status

The camera/gripper swap means the measurable residual is
`tau_camera - tau_gripper`, because the URDF still contains the J7 gripper
(0.8004 kg). Those two are mathematically degenerate, so a pre-swap baseline is
required. With the archived baseline applied the fit still returns about 0.21 kg
instead of the expected 0.85 kg, and the residual rises monotonically with
assumed mass. The identification is **not** converged; see `agent.md` section 9
and 10.
