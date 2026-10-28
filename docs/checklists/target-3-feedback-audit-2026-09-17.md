# Target 3.1 Feedback Audit - 2026-09-17

## Scope

Read-only audit for camera payload identification.

Allowed:

- SSH to `com-desktop.local`.
- Confirm `can0` state.
- Register J1-J6 only with a temporary config.
- Request/read feedback fields.
- Save raw JSONL logs.

Not allowed and not performed:

- No J7 registration or command.
- No enable.
- No MIT/POS/VEL motion command.
- No calibration write.

## Environment

```text
host: com-desktop.local
user: com
channel: can0
can0: PCAN, ERROR-ACTIVE, 1 Mbps
branch: feature/payload-identification
script: payload_identification/feedback_audit.py
```

The idle gravity-console service was stopped after confirming it was in
`disabled` state, to avoid any CAN ownership ambiguity during the audit.

## Logs

First attempt:

```text
logs/payload-id/feedback_audit_20260917_160728.jsonl
```

This used `robot.get_state()` and returned zero arrays with no cached motor
state. It showed that the global `RebotArm.get_state()` path is not suitable
for disabled read-only feedback on this setup.

Second attempt:

```text
logs/payload-id/feedback_audit_20260917_161220.jsonl
```

This used `robot.arm.get_positions()` and per-motor RobStride parameter reads.
It successfully read J1-J6 position through `mechPos` parameter `0x7019`.

## Findings

### Available In Read-Only Disabled Mode

- J1-J6 position is readable.
- `robot.arm.get_positions()` works because the SDK falls back to RobStride
  `mechPos` parameter `0x7019`.
- Direct per-motor `robstride_get_param_f32(0x7019)` also works.

Example first sample from the successful audit:

```text
q_deg = [-15.232, 0.115, 0.903, -1.145, -0.088, 33.416]
```

Position span over 5 seconds:

```text
[0.01465, 0.01953, 0.01465, 0.01758, 0.02197, 0.01538] deg
```

This confirms the arm was essentially still during the audit.

### Not Available In Read-Only Disabled Mode

- `cached_state` was `None` for every joint.
- No torque/current/effort field was available from cached motor state.
- `robot.get_state()` returned zero position/velocity/torque arrays in this
  mode and should not be used for this audit.
- `mechVel` parameter `0x701A` read as `0.0` while still. Existing
  `tools/gravity_calibration/README.md` also says `mechVel` is not reliable as
  rad/s on this firmware.

## Existing Calibration Note

The existing control repo already contains an important warning:

```text
/home/com/reBotArm_control_py/tools/gravity_calibration/README.md
```

It states there is no independent torque sensor. Prior gravity identification
used a quasi-static PD sweep and estimated holding torque from tracking error:

```text
tau ~= kp * (target - q) - kd * velocity
```

This means payload identification cannot be solved from disabled read-only
feedback alone. It will require a carefully bounded active test if we want
measured effort information.

## Conclusion

Target 3.1 succeeded as a read-only audit:

- J1-J6-only config works.
- Position feedback is available via `0x7019`.
- Independent torque/current/effort feedback is not available in disabled
  read-only mode.
- The next stage should be quasi-static active identification using conservative
  PD holds/sweeps, not open-loop torque or global gravity scaling.

## Recommended Next Small Goal

Target 3.2 should be planning-only or dry-run first:

1. Reuse the existing `tools/gravity_calibration/pd_sweep_id.py` approach.
2. Adapt it to the camera branch and J1-J6-only config.
3. Start with one static hold pose or one tiny J3 sweep.
4. Keep J4 capped because prior tests showed J4 can overcompensate.
5. Estimate holding torque from PD tracking error, not from unavailable torque
   feedback.
