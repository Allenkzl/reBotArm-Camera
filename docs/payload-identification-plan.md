# Payload Identification Plan

## Goal

Estimate the camera payload mass and center of mass from safe robot data instead
of relying only on manual weighing and guessed COM.

The target payload parameters are:

```text
m      payload mass, kg
rx     payload COM X offset in flange/end-effector frame, m
ry     payload COM Y offset in flange/end-effector frame, m
rz     payload COM Z offset in flange/end-effector frame, m
```

The desired output is a validated payload estimate that can replace manual
global percentage tuning in gravity compensation.

## Current Lessons To Carry Forward

- J7 is removed and must remain unregistered in active tests.
- Global gravity-compensation scaling is not sufficient. J4 can overcompensate
  while J2/J3 still feel weak.
- The active web console must log both file-side JSONL data and visible
  page-side events.
- The latest useful global-tuning run is:

```text
logs/runs/gc_console_20260917_153944.jsonl
```

- The detailed tuning retrospective is:

```text
docs/checklists/target-2-global-gravity-tuning-result-2026-09-17.md
```

## Core Idea

For each sampled pose, the extra camera gravity torque can be modeled from the
payload parameters. If the robot can provide a usable estimate of motor effort,
we can compare measured effort against the original robot model and solve a
least-squares problem:

```text
tau_external(q) ~= tau_measured(q) - tau_robot_model(q)
tau_external(q) ~= payload_gravity(q, m, rx, ry, rz)
```

Payload gravity is linear in these grouped parameters:

```text
[m, m*rx, m*ry, m*rz]
```

After solving those grouped values, COM is recovered by dividing by `m`.

## Unknowns To Verify First

Before commanding any new motion, perform a read-only feedback audit:

1. Confirm whether J1-J6 feedback includes current, torque, effort, or q-axis
   current, not only position.
2. Confirm units and sign conventions for any effort-like feedback.
3. Confirm whether commanded MIT torque is applied consistently enough to be
   used as a proxy if direct effort feedback is unavailable.
4. Confirm the existing model gravity signs against a few static poses.

If effort/current feedback is not trustworthy, the project can still continue,
but the method becomes rougher: use small commanded torque ramps and observe
drift direction or near-balance thresholds.

## Proposed Stages

### Stage 0 - Read-Only Feedback Audit

No enable, no motion.

Tasks:

- Run J1-J6-only scan.
- Read several seconds of raw feedback while disabled or idle.
- Inspect available fields for each motor.
- Save one raw JSONL file showing every available feedback field.

Acceptance:

- We know whether torque/current feedback exists and which fields are usable.
- No motors were enabled.

### Stage 1 - Static Hold Data Collection

Low risk, quasi-static.

Tasks:

- Move only through a small set of known safe poses, or start from poses the
  operator manually places while the arm is supported.
- Enable J1-J6 with low stiffness and conservative torque caps.
- At each pose, dwell for several seconds.
- Log q, optional dq, model gravity, commanded torque, effort/current feedback,
  temperature/error state, and operator notes.

Candidate poses:

- Home / near Home.
- Elbow slightly up.
- Elbow slightly down.
- Forearm raised.
- Forearm lowered.
- Wrist/flange X rotated through two safe orientations if cable clearance is
  still good.

Acceptance:

- At least 8-12 static samples spanning different J2/J3/J4 configurations.
- No uncontrolled motion, oscillation, overheating, or cable tension.
- Arm returns Home and disables safely.

### Stage 2 - Slow Excitation Motions

Only after Stage 1 looks stable.

Tasks:

- Run slow sinusoidal or minimum-jerk motions with small amplitudes.
- Prefer one joint family at a time:
  - J2 lift/lower sweep.
  - J3 elbow sweep.
  - J4 wrist pitch/roll influence sweep.
- Keep speed low enough that inertia is small compared with gravity.
- Move in both directions to separate friction from gravity.

Suggested first bounds:

```text
max_speed: 0.04-0.08 rad/s
amplitude: 5-10 deg around the current safe pose
dwell: 2-4 s at each endpoint
torque cap: start below previous 60% observed command levels
```

Acceptance:

- Logs show smooth q traces with low velocity.
- No joint leaves the agreed soft test window.
- Residual effort changes with pose in a repeatable way.

### Stage 3 - Offline Parameter Fit

No robot motion.

Tasks:

- Build a script that loads the JSONL logs.
- Compute model gravity at each sample.
- Convert any reliable motor effort/current into joint torque.
- Remove or reduce samples with high velocity, acceleration, saturation, or
  obvious operator contact.
- Fit `[m, m*rx, m*ry, m*rz]` with robust least squares.
- Report confidence, residuals, and which joints/poses contributed most.

Acceptance:

- Fitted mass is physically plausible.
- COM estimate is stable when fitting different subsets of data.
- Residuals improve versus the current manual `0.8 kg, X=0.06 m` estimate.

### Stage 4 - Validation Run

Active but conservative.

Tasks:

- Use the fitted payload parameters in the gravity console.
- Start with low per-joint caps and reduced global scale.
- Test the same poses used in global tuning.
- Verify J4 no longer over-assists while J2/J3 support improves.

Acceptance:

- Less need for manual global scaling.
- J2/J3 support improves without J4 flying.
- Logs and user observations agree.

## Safety Rules

- Do not register or command J7.
- Do not run unbounded global scaling above the previous unsafe range.
- Every active run must have:
  - operator next to the arm,
  - clear stop path,
  - JSONL log,
  - return-Home/disable plan,
  - torque and speed caps.
- Prefer quasi-static tests before dynamic excitation.
- If the arm rises by itself, dives, oscillates, heats, or sounds abnormal,
  stop and hold first; disable only when the arm is supported or near Home.

## Implementation Sketch

New branch:

```text
feature/payload-identification
```

Likely files:

```text
payload_identification/
  feedback_audit.py
  collect_static_samples.py
  collect_slow_sweeps.py
  fit_payload.py
docs/checklists/
  target-3-feedback-audit-YYYY-MM-DD.md
  target-3-static-identification-YYYY-MM-DD.md
logs/payload-id/
```

The first executable should be `feedback_audit.py` because it is read-only and
decides whether the rest of the plan can use real effort/current feedback.

## First Small Goal

Target 3.1: read-only feedback audit.

Scope:

- SSH to `com-desktop.local`.
- Confirm `can0` is PCAN.
- Create a temporary J1-J6-only config.
- Read J1-J6 feedback without enable or motion.
- Save raw feedback samples to `logs/payload-id/`.
- Document available feedback fields and whether they can support payload
  identification.

No active motion is allowed in this first small goal.

Status on 2026-09-17:

```text
completed: docs/checklists/target-3-feedback-audit-2026-09-17.md
```

The audit found usable read-only position feedback through RobStride `0x7019`,
but no independent torque/current/effort feedback in disabled mode. The next
identification method should estimate holding torque from controlled PD tracking
error during quasi-static active tests.

Target 3.2 status on 2026-09-17:

```text
completed: docs/checklists/target-3-pd-hold-probe-2026-09-17.md
```

The first active hold probe completed without abort and returned close to the
starting pose. It produced stable data but only at one near-Home pose, so it is
not enough to identify mass/COM. The next active step should be a tiny J3 sweep,
not a large lift or flip.
