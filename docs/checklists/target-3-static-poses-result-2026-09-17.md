# Target 3.4 Static Balance Point Sampling - 2026-09-17

## Scope

This target replaced the continuous sweep of Target 3.3 with multi-pose static
equilibrium sampling: command a fixed target, wait for the servos to settle, and
use the settled PD residual as the measurement.

Everything ran with:

- J1-J6 only.
- No J7 registration or command.
- Temporary `/tmp` J1-J6-only runtime config (no persistent config touched).
- MIT mode active control.
- JSONL logging plus console event output.
- Pre-flight FK safety scan before enabling.
- Return-to-home, gain fade-out, disable, controller close.

## The measurement

At a settled dwell the servos hold

```text
tau_feedforward + kp*(q_target - q_settled) = g_true(q_settled)
```

so the settled PD residual is

```text
tau_pd := kp*(q_target - q_settled) = g_true - tau_model
```

which stays clean as long as the target is held fixed and the joint is not
against a limit. `tau_ff = clip(tau_model, +/-FF_CLAMP)`, and `FF_CLAMP` was
checked offline to cover every commanded pose, so no clamping term applies here.

## Scripts

```text
payload_identification/design_poses.py          offline pose design (no CAN)
payload_identification/collect_static_poses.py  sampling run
payload_identification/fit_arm_error.py         arm model error from old sweeps
payload_identification/fit_payload.py           camera mass/COM fit
```

`design_poses.py` contains two self-tests that must pass before any result is
trusted:

```text
payload regressor vs SDK payload torque      max abs diff = 4.441e-16 N*m
torque regressor vs model gravity (fit_*)    max abs diff = 8.882e-16 N*m
```

## Pose design

Candidate poses were scored with the payload regressor

```text
tau_camera(q) = A(q) @ [m, m*rx, m*ry, m*rz]
```

and selected with a greedy scaled-D-optimal criterion. Expected parameter
uncertainty for the chosen design (11 dwells, assumed sigma_tau = 0.05 N*m):

```text
4 camera parameters only                  sigma_m = 1.3 %   sigma_rx = 90.4 mm
4 camera parameters + 6 joint offsets     sigma_m = 2.7 %   sigma_rx = 10.7 mm
```

The per-joint constant offsets are not optional: the measured residual carries a
large pose-independent component (friction, cable forces, constant model bias)
that would otherwise be aliased into the camera parameters.

Excitation strategy trade-off:

```text
(a) J2/J3/J4 only            sigma_m 2.2 %   sigma_rx  9.1 mm
(b) + J5 +/-15, J6 +/-30     sigma_m 1.8 %   sigma_rx  8.5 mm
(c) + J5 +/-20, J6 +/-40     sigma_m 1.5 %   sigma_rx  6.8 mm
```

Strategy (a) was chosen. J5/J6 rotate about the tool axis and twist the camera
cable, for roughly 2 mm of benefit that is not worth the cable risk. J1 and J6
carry no payload gravity information at all (measured residual spans: J1 0.000,
J6 0.001 N*m).

## Safety pre-flight

The pre-flight scan evaluates forward kinematics on a +/- 4 degree box around
each target and refuses to move if the camera or any wrist frame comes too close
to the table.

Two decisions came out of it:

1. **The limit check must use the settled pose, not the commanded pose.**
   At equilibrium `q_settled = q_target - tau_pd/kp`, and the sag can exceed
   3 degrees. With the settled-pose check active, 50 of 80 candidates were
   rejected. This is the root cause of the Target 3.3 failure mode: commanding
   a target below the reachable range parks the joint on a hard stop, and the
   "very repeatable residual" measured there is the stop reaction, not gravity.
2. **The first design was unsafe.** Two of its ten poses passed a commanded-pose
   check but failed the +/- 4 degree box check:

```text
pose 1 (J2=30.08, J3=30.97, J4=-31.15)   camera low point 0.086 m
pose 8 (J2=20.08, J3=15.97, J4=-16.15)   camera low point 0.078 m
```

The design was redone with `min_camera_z = 0.20 m`; all ten final poses keep the
camera at or above 0.173 m even inside the box, and the pre-flight verdict is
PASS.

Also measured: **the J3 physical hard stop is at about +1.03 deg, while the URDF
declares 0 deg.**

## Pose list

J1 = -6.11, J5 = 0.21, J6 = 32.86 deg for every pose.

```text
 #    J2      J3      J4      camera z [m]
 0   30.08   60.97   13.85   0.657
 1   50.08   45.97  -16.15   0.210
 2   10.08   60.97   28.85   0.752
 3   10.08   30.97  -31.15   0.223
 4   10.08   60.97   13.85   0.716
 5   40.08   30.97   -1.15   0.214
 6   10.08   45.97   28.85   0.683
 7   10.08   60.97  -31.15   0.485
 8   50.08   60.97  -31.15   0.270
 9   20.08   60.97   28.85   0.751
```

Gains and limits used:

```text
kp        = [8, 35, 40, 25, 5, 5]
kd        = [0.8, 2.2, 2.8, 2.0, 0.4, 0.4]
ff_clamp  = [1.0, 9.0, 12.0, 5.0, 0.8, 0.8]
move cap  = 4 deg/s, minimum ramp 4 s
dwell     = 6 s, sample rate 40 Hz
```

`ff_clamp` covers the largest model torque seen across all candidates
`[0.0, 7.9, 7.14, 2.01, 0.0, 0.0]`, so no feedforward clamping occurred.

## Runs

### Verification run - 2 poses, short dwell

```text
log: logs/payload-id/static_poses_20260917_180322.jsonl
poses: 0 and 2, dwell 2.5 s
result: no abort, returned home, disabled, post-disable pose stable
```

This run is where the sign problem below was first seen.

### Full run - 10 poses plus a repeat of pose 0

```text
log: logs/payload-id/static_poses_20260917_180506.jsonl
samples: 5654
pose phase duration: 141.91 s
total duration: 160.28 s
abort: none
events: all_poses_done -> returned_to_start -> fade_complete -> disabled -> closed
post-disable pose: [-8.31, 0.09, 0.13, 1.08, 0.48, 32.85] deg (stable)
```

Settled residual `tau_pd` (N*m, mean over the last 2.5 s of each dwell):

```text
label            J1      J2      J3      J4      J5      J6
pose_0        +0.277  +0.466  -1.386  -0.577  -0.024  +0.003
pose_9        +0.273  -1.213  -1.157  -0.371  -0.024  +0.001
pose_2        +0.275  -0.969  -0.648  -0.355  -0.024  +0.002
pose_6        +0.272  -0.825  -1.487  -0.454  -0.023  +0.002
pose_4        +0.273  -0.836  -0.725  -0.546  -0.024  +0.002
pose_5        +0.274  +1.438  -2.558  -1.190  -0.023  +0.001
pose_1        +0.272  +0.928  -2.015  -1.131  -0.024  +0.003
pose_8        +0.275  +0.889  -1.710  -1.107  -0.024  +0.001
pose_3        +0.273  -0.366  -2.347  -1.089  -0.023  +0.001
pose_7        +0.274  -0.648  -1.337  -1.088  -0.022  +0.001
pose_0_repeat +0.273  +0.494  -1.454  -0.580  -0.023  +0.001
```

### Data quality

Repeatability between the two visits of pose 0:

```text
dq_deg  = [0.0146, -0.0403, 0.0952, 0.011, -0.0, 0.022]
dtau_pd = [-0.0033, 0.0283, -0.0681, -0.0035, 0.0001, -0.0021]
max |dtau_pd| = 0.0681 N*m
```

Sanity checks that behaved as expected:

- J1 residual is constant at about +0.273 N*m. J1 is the vertical axis, so a
  payload produces no gravity torque about it; this is pure friction/bias.
- J5 and J6 residuals are approximately zero, as expected.

**The sampling chain is proven. The measurement noise floor is 0.068 N*m.**

## Finding 1 - the URDF still contains the gripper

`tau_model` includes the J7 gripper assembly:

```text
gripper_end    0.65    kg
gripper_left   0.0752  kg
gripper_right  0.0752  kg
--------------------------
total          0.8004  kg    COM at about x = -0.10 m in the gripper_end frame
```

So the measurable residual after the gripper-to-camera swap is the *change* in
distal load, not the camera alone:

```text
tau_pd = (g_arm_true - g_arm_URDF) + tau_camera - tau_gripper
```

The camera (0.85 kg) and the gripper (0.80 kg) have nearly the same mass with
opposite moment arms, which is why early residuals were only 0.1-0.3 N*m while
the camera alone would be about 4.5 N*m. The earlier explanation that "the
residual is too weak" was therefore a misdiagnosis.

## Finding 2 - the measured sign is opposite to the prediction

At pose 0:

```text
                         predicted (tau_cam - tau_grip)     measured
tau_pd (J2, J3, J4)      [-1.12, +1.22, +1.13]             [+0.46, -1.37, -0.58]
sag    (J2, J3, J4)      [-1.83, +1.74, +2.60]             [+0.75, -1.96, -1.31]
```

The sign is opposite on all three joints simultaneously. Physically the rig
behaves as if about 0.73 kg at -0.18 m had been *removed*, whereas the URDF
predicts the camera should be heavier in effect than the gripper.

## Finding 3 - camera and gripper are exactly degenerate

The URDF gripper's gravity torque is exactly

```text
tau_gripper(q) = A(q) @ [0.8004, -0.0824, 0, 0]
```

so `cam_only`, `grip_corrected` and `sign_flipped` are the same fit up to an
offset of exactly the gripper mass:

```text
cam_only                 m = -0.7294 kg   rms = 0.2583 N*m
grip_corrected           m = +0.0710 kg   rms = 0.2583 N*m
sign_flipped             m = +1.5298 kg   rms = 0.2583 N*m
```

All three are physically implausible, and their residuals are identical by
construction. **The camera cannot be separated from the gripper using
post-swap data alone; a pre-swap baseline is required.**

## Finding 4 - the pre-swap baseline exists

The 2026-07-17 gravity calibration was done with the gripper fitted, and its raw
sweeps are archived:

```text
/home/com/reBotArm_control_py/tools/gravity_calibration/data/pdsweep_*.json
/home/com/reBotArm_control_py/docs/gravity_calibration_rs_2026-07-17.md
```

The published calibration states the URDF model is correct to about 5-11 % on
every load-bearing joint. Fitting a per-joint model scale from those sweeps
(`fit_arm_error.py`):

```text
equations 2818, parameters 11, cond 23.2, fit rms 0.2422 N*m

joint   fitted k   +/-1 sigma   published k
J2      0.9630     0.0020       0.948
J3      0.9484     0.0019       0.952
J4      0.9566     0.0285       0.885
```

Coulomb friction agrees closely with the published values, which validates the
data path:

```text
joint   friction   published   bias      published
J2      0.532      0.53        -0.155    -0.70
J3      0.505      0.49        +0.017    +0.12
J4      0.289      0.30        +0.007    +0.49
J5      0.209      0.21        +0.025    -
```

Note: the free 32-parameter body model (`--model dphi`) is rank deficient
(condition number 1e14, parameters diverge to +/-71 N*m) because the archived
sweeps move one joint at a time. Do not repeat that attempt.

This gives the final identification equation:

```text
y := tau_pd_new + tau_gripper_URDF(q) - E_old(q) = A(q) @ theta_camera

E_old(q) = sum_j (k_j - 1) * tau_model_j(q)      j in {J2, J3, J4}
```

## Finding 5 - the fit still fails

```text
calibrated (fitted k)     m = +0.2138 kg   r = [+0.0441, +0.0068, -0.0005]   rms = 0.2471
calibrated (published k)  m = +0.2118 kg   r = [+0.0652, -0.0016, +0.0125]   rms = 0.2291
calibrated + offsets      m = +0.0941 kg   r = [-0.1047, -0.2179, +0.3507]   rms = 0.1867
full_arm (28 params)      m = +0.0216 kg   converged poorly                    cond = 1.0e14
```

Expected values are m about 0.85 kg and rx about +0.06 m. The residual stays at
0.19-0.25 N*m, three to four times the 0.068 N*m noise floor, so a systematic
term is still missing.

### Mass scan (decisive diagnostic)

Fixing the mass and solving only for the first moment:

```text
m [kg]   rms (no offsets)   rms (+offsets)
0.000    0.2862             0.1885    <- minimum on the grid
0.200    0.2473             0.1890
0.400    0.2772             0.2052
0.800    0.4665             0.2712
0.850    0.4954             0.2815
1.200    0.7100             0.3603
```

The residual rises monotonically as the mass approaches the physically expected
value. **The current model cannot explain a 0.85 kg camera.**

### Residual structure

```text
label    joint   y(meas)   pred     resid
pose_0   J2       0.059   -0.350   0.408
pose_0   J3       0.964    0.781   0.182
pose_0   J4       0.439    0.468  -0.028
pose_9   J2      -0.800   -0.050  -0.750
pose_9   J3       0.474    0.509  -0.035
pose_9   J4       0.145    0.245  -0.099
pose_3   J2      -1.322   -0.587  -0.735
pose_3   J3       0.814    1.073  -0.260
pose_3   J4       0.459    0.697  -0.238
pose_7   J4       0.338    0.647  -0.310
```

The J4 residuals are systematically negative (all between -0.03 and -0.34 N*m),
and the J2 residuals scatter widely even at the same J2 angle. The two visits of
pose 0 give nearly identical residuals (+0.408 and +0.442), confirming this is
repeatable systematic error, not noise.

## Conclusion

Target 3.4 achieved its engineering goal: the static sampling chain is built,
safety-checked, and produces repeatable measurements with a 0.068 N*m noise
floor. Several long-standing misdiagnoses were corrected along the way (the
gripper is still in the URDF; the Target 3.3 "best data" was limit saturation;
"set target equal to current pose" is self-defeating).

The identification itself has **not** converged. The data does not support a
0.85 kg camera under any model tried, and the reason is a missing term rather
than measurement noise.

## Recommended next steps

1. **Verify the effective MIT `kp` (highest priority).** Every torque number in
   this project, including the 2026-07-17 calibration, rests on the unverified
   assumption that the motor's real `kp` equals the commanded value; there is no
   independent torque sensor. Commanded gains here (35/40/25) also differ from
   the runtime config (150/150/50). A known torque offset versus the resulting
   position shift would measure `kp_eff / kp_nominal` directly, in minutes.
2. **Repeat the auto-float test in the camera build.** Fade PD and float the
   joint on gravity feedforward alone; the drift direction immediately shows
   whether the model over- or under-estimates the load.
3. **Measure the geometry by hand.** Camera plus bracket mass, camera COM
   relative to the `gripper_end` origin, and the mass of the original gripper if
   it still exists. The gripper mass anchors the degenerate direction.
4. **Consider a decoupling experiment**, for example a temporary known mass at
   the end effector, which would calibrate the residual-to-first-moment mapping
   independently of the gripper model.

## Related files

```text
docs/checklists/target-3-feedback-audit-2026-09-17.md
docs/checklists/target-3-pd-hold-probe-2026-09-17.md
docs/checklists/target-3-slow-sweep-result-2026-09-17.md
agent.md
logs/payload-id/static_poses_20260917_180322.jsonl
logs/payload-id/static_poses_20260917_180506.jsonl
logs/payload-id/pose_design_a.json
logs/payload-id/arm_error_fit.json
logs/payload-id/fit_20260917.json
```
