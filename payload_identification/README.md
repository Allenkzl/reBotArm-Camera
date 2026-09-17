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
