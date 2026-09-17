# Payload Identification

Utilities for estimating the camera payload mass and center of mass from robot
feedback and safe quasi-static motions.

Start with the read-only audit:

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/feedback_audit.py \
  --duration 5 \
  --rate 5
```

The audit registers J1-J6 only, does not enable motors, and does not command
motion.

After the audit, the first active probe is a short conservative hold:

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/pd_hold_probe.py \
  --duration 8 \
  --rate 40
```

The hold probe enables J1-J6, holds the current pose with bounded PD gains and
original model gravity feedforward, estimates payload residual from tracking
error, fades out, and disables.

For controlled single-joint excitation, use the slow sweep collector:

```bash
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_slow_sweep.py \
  --joint 3 \
  --amplitude-deg 6 \
  --sweep-s 6 \
  --dwell-s 2 \
  --j2-kp 35 \
  --j2-kd 2.2 \
  --j3-kp 40 \
  --j3-kd 2.8 \
  --j4-kp 25 \
  --j4-kd 2.0
```

The sweep data is useful for checking whether residual torque changes with
posture, but it should not be used directly as a final mass/COM estimate while
tracking error is still large.
