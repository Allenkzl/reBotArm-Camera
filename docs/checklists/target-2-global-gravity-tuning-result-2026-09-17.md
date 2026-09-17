# Target 2 Global Gravity Tuning Result - 2026-09-17

## Context

The camera payload was mounted in place of the J7 gripper. The active web
console used a temporary J1-J6-only runtime config, so J7 was not registered or
commanded.

The operator tested global gravity-compensation percentages from 20 percent to
60 percent. The page was later fixed so the command torque used the estimated
model gravity plus the estimated camera-payload gravity:

```text
tau_cmd = (tau_model + tau_payload) * scale
```

Latest live run:

```text
logs/runs/gc_console_20260917_153944.jsonl
```

Final service status after the test was `disabled`, with J1-J6 returned near
Home.

## Operator Observation

- Around 40-50 percent, J1/J4 could visibly lift or support parts of the arm.
- At 60 percent, J1/J4 could overcompensate; the arm could move opposite to
  gravity even when not intentionally moved.
- J2 and J3 did not feel able to lift or support the arm across 20-60 percent.
- A global percentage therefore does not produce balanced compensation across
  the loaded arm.

## Log Evidence

Grouped by global percentage from the latest run:

| Global percent | J2 avg cmd Nm | J3 avg cmd Nm | J4 avg cmd Nm | Notable pose span |
| --- | ---: | ---: | ---: | --- |
| 20 | -0.216 | 2.199 | 0.946 | J4 span 42.0 deg |
| 25 | -0.260 | 2.741 | 1.184 | J2 span 21.1 deg, J3 span 31.3 deg |
| 30 | -0.351 | 3.290 | 1.418 | J2 span 32.7 deg, J3 span 37.2 deg |
| 35 | -0.377 | 3.825 | 1.656 | J3 span 41.2 deg |
| 40 | -0.432 | 4.353 | 1.848 | J4 span 57.1 deg |
| 45 | -0.450 | 4.868 | 2.111 | J2 span 29.7 deg, J3 span 39.7 deg |
| 50 | -0.350 | 5.106 | 2.141 | J1/J4/J6 moved substantially |
| 60 | -0.250 | 6.175 | 2.418 | J4 span 47.6 deg |

At the final near-Home status, the estimated total gravity terms were roughly:

```text
tau_total_nm = [0.00, -0.43, 10.59, 4.05, 0.05, -0.00]
tau_cmd_nm at 60% = [0.00, -0.25, 6.22, 2.43, 0.03, -0.00]
```

This explains the observed behavior:

- J4 can be over-assisted because its commanded torque is already large enough
  for some sampled poses.
- J3 is still below the estimated full hold torque at 60 percent.
- J2 receives very little torque because the estimated arm-model gravity and
  camera-payload contribution partly cancel around these poses.

## Pitfalls Found

1. The first web-console version applied the percentage only to the model
   gravity term. It displayed payload torque but did not include payload torque
   in the command. This made 50 percent feel much weaker than expected.
2. A single global percentage is not a reliable tuning parameter. Increasing it
   to help J3 can overcompensate J4.
3. Small trim ranges, such as +/-20 percent, are not enough once different
   joints need different compensation behavior.
4. J2 is especially ambiguous in the current pose range because the estimated
   total torque is small and changes by cancellation. It should not be tuned
   blindly with a global multiplier.
5. Page-side logging must show client actions and stream status, not only saved
   JSONL files, otherwise a real run can look like it produced no logs.

## Practical Conclusion

Do not continue testing by raising the global percentage above 60 percent.
The next step should be one of these:

- add independent per-joint compensation scaling and a safer J4 cap, or
- estimate the payload mass and center of mass from measured robot responses,
  then use a better pose-dependent payload model.

The second option is the preferred next investigation because it can replace
manual global tuning with an identified payload model.
