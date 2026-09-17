# Target 2 Read-Only Gravity Result: 2026-09-17

Time: 2026-09-17

## Scope

The operator measured the camera payload at approximately `0.8 kg` and
estimated its center of mass as approximately `0.06 m` along the flange/end
effector local X axis.

Note: an earlier calculation briefly used a local Y-axis offset after the
operator's first description. The operator corrected the direction to local X
before this result was committed; the values below use the corrected X-axis
offset.

This run was read-only:

- Read J1-J6 angle feedback.
- Loaded the existing Pinocchio/URDF model.
- Estimated the original model gravity torque.
- Estimated the additional payload gravity torque for `0.8 kg` at local
  offset `[0.06, 0.0, 0.0] m` from `gripper_end`.

No enable, disable, motion, gripper/J7 command, gravity-compensation output,
calibration, firmware, ID-write, or parameter-write command was run.

## Model Context

Pinocchio model:

```text
nq=8
nv=8
arm joints: joint1 ... joint6
gripper joints present in URDF: gripper_joint1, gripper_joint2
end_effector_frame: gripper_end
```

For this calculation, J1-J6 were read from motor feedback and the two gripper
joints were kept at zero in the model.

## Static Pose Sample

Current pose:

```text
Q_ARM_RAD [-0.108018, 0.000767, 0.000852, 0.014036, -0.023930, 0.515609]
Q_ARM_DEG [-6.189, 0.044, 0.049, 0.804, -1.371, 29.542]
```

Estimated end-effector and payload COM positions in the model base frame:

```text
EE_POS_BASE_M          [0.3003, 0.0277, 0.2220]
PAYLOAD_COM_POS_BASE_M [0.3601, 0.0328, 0.2229]
```

Estimated torques:

| Joint | Angle deg | Model gravity Nm | Payload hold Nm | Total estimated hold Nm |
| --- | ---: | ---: | ---: | ---: |
| joint1 | -6.189 | +0.0000 | +0.0000 | +0.0000 |
| joint2 | +0.044 | +1.5483 | -2.6808 | -1.1325 |
| joint3 | +0.049 | +6.7601 | +4.5330 | +11.2931 |
| joint4 | +0.804 | +1.9971 | +2.7437 | +4.7408 |
| joint5 | -1.371 | +0.0003 | +0.0007 | +0.0010 |
| joint6 | +29.542 | -0.0007 | +0.0000 | -0.0007 |

Signs are model-coordinate signs. For safety planning, compare magnitudes and
how they change across poses.

## 20-Second Monitor

A 20-second read-only monitor was run after the X-axis correction. The arm pose
did not materially change during the capture, so this validates the current
static pose only.

Peak magnitudes during the monitor:

```text
max_abs_payload_Nm = [0.00, 2.68, 4.53, 2.74, 0.00, 0.00]
max_abs_total_Nm   = [0.00, 1.13, 11.29, 4.74, 0.00, 0.00]
```

## Interpretation

- The estimated payload contribution is concentrated on J2-J4.
- J3 is the dominant joint in the current pose: about `4.53 N·m` from the
  added camera payload, and about `11.29 N·m` total estimated hold torque with
  the current URDF model.
- J4 is second: about `2.74 N·m` from the added payload.
- J2 also changes by about `2.68 N·m` in this pose, though its sign is opposite
  the current model gravity term.
- J6 is essentially unaffected by the corrected local X offset in the current
  pose.
- This result is only one static pose. It does not yet validate the intended
  four photo poses.

## Target 2 Status

- [x] Payload mass estimate recorded: `0.8 kg`.
- [x] Payload COM estimate recorded: local X offset about `0.06 m`.
- [x] Current-pose read-only gravity estimate completed.
- [x] No active control or gravity-compensation output was run.
- [ ] Multi-pose read-only sampling still needed for the intended photo poses.

## Next Step

Run the same read-only monitor while the operator hand-guides the arm through
the intended photo poses. Recommended procedure:

1. Operator supports the forearm or camera payload.
2. Start the read-only monitor.
3. Slowly move to pose 1 and hold for 5 seconds.
4. Slowly move to pose 2 and hold for 5 seconds.
5. Repeat for pose 3 and pose 4.
6. Stop if the camera cable tightens, the arm feels unexpectedly heavy, or any
   joint approaches an uncomfortable angle.

Only after multi-pose read-only sampling looks reasonable should low-ratio
gravity-compensation output be considered.
