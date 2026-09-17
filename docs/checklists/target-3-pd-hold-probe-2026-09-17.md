# Target 3.2 PD Hold Probe - 2026-09-17

## Scope

First active payload-identification probe after the read-only feedback audit.

This was intentionally not a sweep and not a lift/flip motion. It was a minimal
hold test:

- Enable J1-J6 only.
- Hold the current pose for 6 seconds.
- Use original robot model gravity feedforward, not camera payload feedforward.
- Estimate camera/residual effort from PD tracking error.
- Return to the starting pose.
- Fade stiffness/feedforward.
- Disable.

J7 was not registered or commanded.

## Command

Remote command:

```bash
cd /home/com/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/pd_hold_probe.py \
  --duration 6 \
  --rate 40 \
  --return-s 3 \
  --fade-s 2 \
  --max-error 0.08 \
  --max-velocity 0.3
```

Log:

```text
logs/payload-id/pd_hold_probe_20260917_163417.jsonl
```

Post-run read-only pose check:

```text
logs/payload-id/feedback_audit_20260917_163553.jsonl
```

## Result

The probe completed without abort:

```text
hold_complete
returned_to_start
disabled
closed
```

Hold samples:

```text
samples = 240
```

Start pose:

```text
q0_deg = [-15.234, 0.117, 0.898, -1.147, -0.073, 33.416]
```

End of hold before return:

```text
q_end_deg = [-14.612, 1.099, 1.121, -0.637, -0.022, 32.938]
```

Observed span during the 6 second hold:

```text
span_deg = [0.6153, 1.0107, 0.2637, 0.5054, 0.0659, 0.5054]
max_err_deg = [0.6221, 1.0034, 0.2441, 0.5098, 0.0725, 0.5219]
```

Average estimated torques during hold:

```text
avg_tau_ff_nm  = [0.0000, 1.5414, 6.7646, 2.0000, -0.0000, -0.0007]
avg_tau_pd_nm  = [-0.0865, -0.3313, -0.0722, -0.0701, -0.0064, 0.0469]
avg_tau_cmd_nm = [-0.0865, 1.2101, 6.6924, 1.9299, -0.0064, 0.0462]
```

Post-run disabled read-only pose:

```text
q_deg ~= [-14.66, 0.08, 0.81, -0.91, 0.01, 32.85]
```

## Interpretation

This proves the PD-error method can collect stable active data without a large
motion. It does not yet identify camera mass/COM because the test used only one
pose, and the residual torque from PD error was small.

The average PD residual around this near-Home pose was most visible on J2:

```text
J2 avg_tau_pd ~= -0.33 N*m
```

The result is much smaller than the payload-model estimates from the global
gravity console. That can mean one or more of:

- the current near-Home pose is poorly conditioned for identifying payload,
- model feedforward already covers much of the load at this pose,
- the low-gain PD hold allows small pose drift rather than fully expressing the
  payload as tracking error,
- friction and static stiction mask a small residual,
- a multi-pose sweep is required.

## Issues Found

- Single-sample velocity spikes appear in the finite-difference estimate because
  RobStride position is quantized. Future abort logic should use filtered
  velocity or require consecutive high-velocity samples.
- During the return-to-start phase, J2 briefly moved farther from the target
  before settling near the start pose. Future scripts should log final return
  pose directly and may need stronger return gains or a slower return segment.

## Next Recommendation

Do not jump directly to large flip/lift motions. Next run should be one tiny
single-joint sweep:

- J3 first, because global tests showed J3 is load-bearing and under-supported.
- Amplitude: 2-3 degrees.
- Speed: 0.03-0.05 rad/s.
- Keep J4 feedforward capped.
- Fit only whether the up/down PD residual is repeatable, not full COM yet.
