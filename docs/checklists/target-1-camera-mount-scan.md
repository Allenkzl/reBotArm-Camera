# Target 1: Camera Mount And Read-Only Scan

Date: 2026-09-17

## Purpose

Confirm that the gripper has been removed, the camera payload is mounted, J1-J6
remain readable, and the camera cable routing is safe enough to proceed to
read-only gravity analysis later.

This checklist does not authorize motor motion, gravity compensation output, J7
commands, Home changes, URDF edits, or calibration writes.

## Operator Setup

- [ ] J7 gripper has been mechanically removed.
- [ ] Camera is mechanically attached to the end of the arm.
- [ ] Camera plus mount approximate mass recorded: `_____ kg`.
- [ ] Approximate payload center of mass relative to mount recorded or sketched.
- [ ] Camera cable has slack through the expected manual range.
- [ ] Cable cannot wrap around J4/J5/J6 during the planned four photo poses.
- [ ] A person is physically near the arm and can support the forearm/payload.
- [ ] Emergency stop or power cutoff is reachable.

## Read-Only Remote Checks

Use `com-desktop.local` first. Fall back to the last known IP only if mDNS fails.

```bash
ssh com@com-desktop.local
hostname
ip -details link show can0
for i in /sys/class/net/can*; do
  n=$(basename "$i")
  printf "%s " "$n"
  readlink -f "$i/device/driver" | xargs basename
done
```

Expected:

- Host is `com-desktop`.
- `can0` is driven by `pcan`.
- `can1`, if present, is Jetson internal `mttcan` and is not used here.

## Motor Scan

Because J7 is removed, the first project-specific acceptance target is J1-J6.
Do not treat missing ID `0x07` as a failure for this stage.

```bash
/home/com/rebot-venv/bin/motorbridge-cli scan \
  --vendor robstride \
  --transport socketcan \
  --channel can0 \
  --model rs-00 \
  --start-id 0x01 \
  --end-id 0x06 \
  --feedback-ids 0xFD,0xFF,0xFE \
  --timeout-ms 800 \
  --param-timeout-ms 800
```

Expected:

- IDs `0x01` through `0x06` are found.
- No command is sent to J7.
- No ID write, parameter write, calibration, enable, or motion command is run.

Optional diagnostic only, if we need to prove J7 is absent:

```bash
/home/com/rebot-venv/bin/motorbridge-cli scan \
  --vendor robstride \
  --transport socketcan \
  --channel can0 \
  --model rs-00 \
  --start-id 0x07 \
  --end-id 0x07 \
  --feedback-ids 0xFD,0xFF,0xFE \
  --timeout-ms 800 \
  --param-timeout-ms 800
```

Expected: no J7 motor found after removal.

## Angle Feedback

Read J1-J6 feedback only. If a local helper already exists on the remote host,
prefer the established read-only monitor:

```bash
python3 /home/com/rebot_control/tools/angle_watch_7col.py
```

If that monitor hard-fails because J7 is missing, use a J1-J6-only read path in
the next stage rather than reconnecting J7 or changing motor parameters.

Manual observations:

| Check | Result |
| --- | --- |
| J1-J6 values appear stable at rest | |
| Manual support/micro movement changes the expected joints | |
| No large unexpected jumps in angle feedback | |
| Camera cable remains slack while gently checking planned directions | |

## Acceptance For Target 1

- [ ] J1-J6 scan succeeds on `can0`.
- [ ] J7 is not commanded.
- [ ] Angle feedback for J1-J6 is available or a J1-J6-only monitor requirement is recorded.
- [ ] Camera mount is rigid enough for careful gravity-compensation investigation.
- [ ] Camera cable has enough slack for the four intended photo poses.
- [ ] No active motion or gravity compensation has been run in this target.

## Notes

Record scan output, cable observations, photos, or screenshots here before
moving to target 2.
