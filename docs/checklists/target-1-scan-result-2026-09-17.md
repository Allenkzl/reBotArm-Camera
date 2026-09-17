# Target 1 Scan Result: 2026-09-17

Time: 2026-09-17 11:47 CST

## Scope

The operator confirmed that the camera was mounted and the cable was not being
pulled. This run performed read-only remote checks only.

No enable, disable, gripper, J7, motion, gravity compensation, calibration,
firmware, ID-write, or parameter-write command was run.

## Host And CAN Check

Command target:

```text
com@com-desktop.local
```

Result:

```text
HOSTNAME=com-desktop
USER=com
DATE=2026-09-17T11:47:01+08:00
```

`can0` status:

```text
can0: <NOARP,UP,LOWER_UP>
can state ERROR-ACTIVE (berr-counter tx 0 rx 0)
bitrate 1000000
parentbus usb parentdev 1-2.2:1.0
```

CAN driver mapping:

```text
can0 pcan
can1 mttcan
```

Conclusion: `can0` is the PCAN-USB bus used by the arm. `can1` is Jetson
internal CAN and was not used.

## J1-J6 Motor Scan

Command:

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

Result:

```text
[hit] probe=0x01 vendor=robstride via=ping feedback_id=0xFD device_id=1 responder_id=254
[hit] probe=0x02 vendor=robstride via=ping feedback_id=0xFD device_id=2 responder_id=254
[hit] probe=0x03 vendor=robstride via=ping feedback_id=0xFD device_id=3 responder_id=254
[hit] probe=0x04 vendor=robstride via=ping feedback_id=0xFD device_id=4 responder_id=254
[hit] probe=0x05 vendor=robstride via=ping feedback_id=0xFD device_id=5 responder_id=254
[hit] probe=0x06 vendor=robstride via=ping feedback_id=0xFD device_id=6 responder_id=254
scan done: 6 motor(s) found
```

Conclusion: J1-J6 are online on `can0`.

## J1-J6 Angle Feedback

Angle feedback was read with RobStride parameter `0x7019`, IDs 1 through 6 only.

| Joint | Angle rad | Approx deg |
| --- | ---: | ---: |
| J1 | -0.07371632009744644 | -4.22 |
| J2 | 0.0008522007265128195 | 0.05 |
| J3 | 0.0008948213653638959 | 0.05 |
| J4 | 0.014112639240920544 | 0.81 |
| J5 | -0.02021019533276558 | -1.16 |
| J6 | 0.5472476482391357 | 31.35 |

Conclusion: J1-J6 feedback is readable.

## Target 1 Acceptance Status

- [x] J1-J6 scan succeeds on `can0`.
- [x] J7 was not commanded.
- [x] Angle feedback for J1-J6 is available.
- [x] No active motion or gravity compensation was run.
- [ ] Camera mount rigidity remains an operator/mechanical observation.
- [x] Camera cable was reported by the operator as not being pulled.

## Result

Target 1 electrical/communication checks passed for J1-J6 after the camera was
mounted. The next stage should remain read-only: payload mass/center-of-mass
recording and gravity torque monitoring before any gravity-compensation output
is authorized.
