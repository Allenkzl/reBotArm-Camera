# Target 2 Active Gravity Compensation Run: 20 Percent

Date: 2026-09-17

## Scope

This was the first active gravity-compensation test after the camera was
mounted.

User/operator conditions:

- Operator was next to the arm.
- Surrounding environment was reported safe.
- Camera cable was previously reported as not being pulled.
- Operator wanted logs for debugging hold quality and future COM/payload tuning.

Safety boundaries used:

- J7 was not registered in the temporary runtime config.
- Only J1-J6 were registered and commanded.
- No gripper command was sent.
- No firmware, calibration, ID-write, or parameter-write command was run.
- A temporary `/tmp` YAML config was used instead of editing the persistent
  `rebotarm_rs.yaml`.

## Run Parameters

Remote log:

```text
/tmp/rebot_camera_gc/camera_gc_20pct_20260917-142840.jsonl
```

Local archived log:

```text
logs/target-2/camera_gc_20pct_20260917-142840.jsonl
```

Parameters:

```text
duration_s: 90
payload_kg: 0.8
payload_offset_local_m: [0.06, 0.0, 0.0]
tau_scale: [0.20, 0.196, 0.196, 0.20, 0.20, 0.20]
kp: 1.5
kd: 0.8
transition_s: 2.0
registered_joints: J1-J6 only
J7: not registered
```

The script printed `All motors enabled`, but in this temporary config that means
all registered motors, i.e. J1-J6 only.

## Pose Range Captured

90 samples were recorded.

| Joint | Min deg | Max deg | Span deg |
| --- | ---: | ---: | ---: |
| J1 | -6.31 | -4.55 | 1.76 |
| J2 | 0.02 | 28.10 | 28.08 |
| J3 | 0.07 | 35.79 | 35.73 |
| J4 | -45.20 | 6.92 | 52.12 |
| J5 | -7.80 | 1.10 | 8.90 |
| J6 | 31.99 | 33.05 | 1.05 |

## Peak Estimates

Peak estimated camera-payload torque magnitude:

```text
max_abs_payload_Nm = [0.00, 2.89, 4.53, 2.74, 0.02, 0.00]
```

Peak total estimated hold torque magnitude:

```text
max_abs_total_est_Nm = [0.00, 2.30, 11.32, 4.75, 0.03, 0.00]
```

Peak actually commanded gravity-compensation torque magnitude at 20 percent:

```text
max_abs_cmd_Nm = [0.00, 0.71, 1.33, 0.40, 0.00, 0.00]
```

The largest total estimated J3 hold torque occurred around sample 74:

```text
t=74.197s
deg=[-5.7, +6.8, +6.4, -4.7, +1.1, +33.0]
J3 total_est=11.317 N.m
J3 payload=4.527 N.m
J3 commanded=1.331 N.m
```

The largest total estimated J4 hold torque occurred around sample 79:

```text
t=79.210s
deg=[-5.2, +0.1, +2.8, -5.4, +1.0, +33.0]
J4 total_est=4.750 N.m
J4 payload=2.741 N.m
J4 commanded=0.402 N.m
```

The largest J2 total magnitude occurred around sample 87:

```text
t=87.230s
deg=[-6.3, +28.1, +28.9, +2.7, +1.1, +33.0]
J2 total_est=-2.303 N.m
J2 payload=-2.887 N.m
J2 commanded=0.114 N.m
```

## End State

The script stopped its control loop and sent a current-position hold command.

```text
final_deg=[-5.69, +7.89, +6.44, +1.60, -1.56, +32.98]
done status=motors_enabled_hold_position
```

This means the motors may remain enabled in a hold state after the script exits.
Do not leave the loaded arm unattended in this state. Disable only when the arm
is physically supported or in a safe pose.

## Interpretation

- The arm remained readable and controllable through a range of hand-guided
  poses.
- 20 percent compensation is conservative. It reduces part of the gravity load
  but is not expected to fully hold the 0.8 kg camera payload.
- J3 remains the dominant load joint in this test.
- J4 is the secondary load joint.
- J2 becomes more important in some raised poses.
- J5 and J6 are minor for this payload model and sampled pose range.

## Next Tuning Options

Do not jump directly to full compensation.

Suggested next active test:

1. Confirm the arm is still safe and supported.
2. Run a second active test at 40 percent.
3. Keep the same J1-J6-only temporary config.
4. Use the same logging format.
5. Stop immediately if the arm rises by itself, dives, oscillates, heats, or
   sounds abnormal.

If 40 percent still feels too weak but behaves correctly, then consider 60
percent. If 40 percent behaves badly, use the log to revisit joint direction,
model signs, and payload COM before increasing torque.
