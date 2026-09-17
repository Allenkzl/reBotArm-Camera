# Target 2 Home And Disable: 2026-09-17

## Scope

After the 20 percent gravity-compensation test, the operator requested that the
arm return to Home and then be disabled.

Safety boundaries:

- Used a temporary J1-J6-only runtime config.
- J7 was not registered.
- No gripper command was sent.
- No persistent config was changed.
- No calibration, firmware, ID-write, or parameter-write command was run.

## Log

Remote log:

```text
/tmp/rebot_camera_gc/home_disable_20260917-143657.jsonl
```

Local archived log:

```text
logs/target-2/home_disable_20260917-143657.jsonl
```

## Motion

Initial pose before Home:

```text
[-5.67, +7.91, +3.89, -0.55, -1.63, +32.94] deg
```

Home trajectory:

```text
minimum-jerk
261 steps
5.2 seconds
J1-J6 target = 0 rad
```

Pose before disable:

```text
[-0.02, +0.31, +0.83, +0.09, -0.18, -0.11] deg
```

Result:

```text
home_then_disabled_j1j6
```

## Status

J1-J6 were returned close to zero and disabled. J7 was not touched.
