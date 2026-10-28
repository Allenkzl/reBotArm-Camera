# Target 3.3 Slow Sweep Result - 2026-09-17

> ## Correction added 2026-09-17, after Target 3.4
>
> Two conclusions in this record were later shown to be wrong. The measurements
> below are unchanged and still valid; only their interpretation is corrected.
>
> 1. **The `negative dwell` residual is not a gravity sample.** It is limit
>    saturation. J3's lower limit is `0 deg` and Home sits at `+0.83 deg`; this
>    run commanded J3 to `-5.02 deg`, which is below the reachable range. 546 of
>    the 960 samples had a J3 target below zero, and the joint stopped dead on
>    its hard stop (`q3` constant at `+1.033 deg`, window `std = 0.0000 deg`).
>    The "excellent repeatability" of `tau_pd_J3 = -4.22 / -4.23 N*m` is the
>    stop reaction force, not gravity. The valid gravity sample in this run is
>    the `positive dwell` (`q3 = +5.46 deg`, tracking error `1.51 deg`).
> 2. **The recommendation "at each pose, set the target to the measured current
>    pose" is self-defeating.** Since `err = q_target - q_actual`, doing that
>    zeroes the very PD signal being measured. The target must be held fixed
>    while the joint settles. For the same reason, a filter of "tracking error
>    <= 1 deg" is wrong: it would have discarded the valid `positive dwell`.
>    The correct acceptance criterion is a flat `std(tau_pd)` window, a target
>    inside the reachable range, and no limit saturation.
>
> See `agent.md` sections 6 and 12, and
> `docs/checklists/target-3-static-poses-result-2026-09-17.md`.

## Scope

This target tested whether small active motions can produce enough signal to
estimate the camera payload mass and center of mass.

All runs used:

- J1-J6 only.
- No J7 registration or command.
- Temporary runtime config only.
- MIT mode active control.
- JSONL logging plus console event output.
- Return-to-start, fade-out, disable, and controller close.

## Script

```text
payload_identification/collect_slow_sweep.py
```

The script runs a single-joint minimum-jerk sweep around the current pose and
logs:

- actual `q`,
- target `q`,
- filtered velocity,
- model gravity feedforward,
- estimated PD residual torque,
- estimated total command torque,
- segment events,
- disabled and post-disable pose.

## Runs

### Run 1 - J3 +/-3 deg

Log:

```text
logs/payload-id/slow_sweep_20260917_165524.jsonl
```

Command shape:

```text
J3 target amplitude: +/-3 deg
sweep_s: 4
J2/J3/J4 gains: base hold-probe gains
```

Result:

```text
samples: 800
target J3 range: -2.19 to +3.81 deg
actual J3 range: +0.81 to +1.16 deg
actual J3 span: 0.35 deg
actual J4 span: 0.18 deg
actual J2 span: 1.01 deg
```

Interpretation:

This was safe but too weak. J3 did not materially follow the commanded sweep,
so the data mainly measures tracking error, not a useful gravity/payload
response.

### Run 2 - J3 +/-6 deg, stronger J2/J3

Log:

```text
logs/payload-id/slow_sweep_20260917_165908.jsonl
```

Command shape:

```text
J3 target amplitude: +/-6 deg
sweep_s: 6
J2 kp/kd: 30 / 2.0
J3 kp/kd: 30 / 2.2
J4 kp/kd: base hold-probe gains
```

Result:

```text
samples: 960
target J3 range: -5.31 to +6.69 deg
actual J3 range: +0.68 to +4.04 deg
actual J3 span: 3.36 deg
actual J4 range: -5.91 to -0.33 deg
actual J4 span: 5.58 deg
actual J2 span: 0.75 deg
post-disable pose: [-14.52, 0.20, 0.92, -1.12, 0.15, 32.85] deg
```

Interpretation:

This made J3 move, but J4 drifted too much. The data is not a clean J3
excitation because wrist posture changed substantially during the positive
portion of the sweep.

### Run 3 - J3 +/-6 deg, stronger J2/J3 and J4 hold

Log:

```text
logs/payload-id/slow_sweep_20260917_170219.jsonl
```

Command shape:

```text
J3 target amplitude: +/-6 deg
sweep_s: 6
J2 kp/kd: 35 / 2.2
J3 kp/kd: 40 / 2.8
J4 kp/kd: 25 / 2.0
```

Result:

```text
samples: 960
target J3 range: -5.02 to +6.98 deg
actual J3 range: +0.83 to +5.47 deg
actual J3 span: 4.64 deg
actual J4 range: -2.97 to -0.70 deg
actual J4 span: 2.26 deg
actual J2 span: 0.66 deg
max filtered velocity: [0.009, 0.057, 0.120, 0.106, 0.006, 0.003] rad/s
post-disable pose: [-14.52, 0.20, 0.92, -1.10, 0.20, 32.87] deg
```

Dwell residual comparison:

```text
negative dwell:
  actual q3 ~= +1.03 deg
  actual q4 ~= -0.74 deg
  tau_pd_j3 ~= -4.22 N*m
  tau_pd_j4 ~= -0.18 N*m

positive dwell:
  actual q3 ~= +5.46 deg
  actual q4 ~= -2.94 deg
  tau_pd_j3 ~= +1.06 N*m
  tau_pd_j4 ~= +0.78 N*m

negative dwell repeated:
  actual q3 ~= +1.04 deg
  actual q4 ~= -0.73 deg
  tau_pd_j3 ~= -4.23 N*m
  tau_pd_j4 ~= -0.19 N*m
```

Interpretation:

This is the best sweep so far. It proves that:

- stronger J3 gains are needed to move the loaded arm,
- stronger J4 holding reduces wrist drift,
- the negative dwell residual is repeatable,
- residual torque changes with posture.

It is still not enough for a reliable mass/COM solve because:

- J3 target error is still large, up to about 6 deg,
- residual torque still includes intentional trajectory tracking effort,
- the actual motion mostly spans one joint family,
- J4 still changes by about 2.26 deg and must be included in any fit,
- there is no independent torque/current feedback to separate gravity,
  friction, and tracking error.

## Conclusion

The "motion-based payload identification" path is feasible, but the current
sweep data should not yet be treated as a valid camera weight/COM estimate.

The important progress is that Run 3 produced repeatable posture-dependent
residuals without aborting or ending in an unsafe pose. The next step should
collect cleaner quasi-static equilibrium data, not keep increasing sweep
aggressiveness.

## Next Recommendation

Target 3.4 should switch from continuous sweep to short static balance points:

1. Move slowly to a small set of actual poses.
2. At each pose, hold the target fixed and wait for the settled equilibrium.
   (Superseded: the original text here said "set the target to the measured
   current pose", which zeroes the PD signal. See the correction at the top.)
3. Hold for several seconds with stronger J2/J3/J4 posture control.
4. Log only the settled dwell window, accepted when `std(tau_pd)` is flat,
   the target is inside the reachable range, and no limit is touched.
5. Use several actual J2/J3/J4 pose combinations.
6. Fit payload parameters only from those settled windows.

This should make the PD residual closer to holding torque and less polluted by
trajectory tracking error.
