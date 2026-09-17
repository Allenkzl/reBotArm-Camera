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
