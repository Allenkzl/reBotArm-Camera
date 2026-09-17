# Gravity Console

The gravity console is a local web tool for tuning low-ratio gravity
compensation with the camera payload installed.

The `Total` percentage is a global multiplier on the estimated model plus
camera-payload gravity torque. The gravity torque itself is still recalculated
continuously from the current pose, so 35 percent does not mean a fixed torque;
it means "send 35 percent of the current pose's total gravity estimate." The
payload estimate is logged and displayed separately so we can later tune the
payload mass/center-of-mass correction.

## Safety Model

- Runtime config registers J1-J6 only.
- J7 is never registered by the console.
- Default maximum total compensation is 60 percent.
- Total percentage changes ramp gradually in the control loop.
- Every active run writes a JSONL log.
- `Emergency disable` can drop a loaded arm; prefer `Stop hold` or `Home disable`
  when the arm is suspended.

## Run On reComputer

From this repository after copying it to the reComputer:

```bash
cd /path/to/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python gravity_console/server.py \
  --host 127.0.0.1 \
  --port 8012 \
  --robot-root /home/com/reBotArm_control_py \
  --log-dir logs/runs
```

From the Mac:

```bash
ssh -L 8012:127.0.0.1:8012 com@com-desktop.local
```

Open:

```text
http://127.0.0.1:8012
```

## Dry Run

For UI testing without robot hardware:

```bash
python3 gravity_console/server.py --dry-run --port 8012
```

## First Tuning Procedure

1. Keep one hand supporting the forearm or payload.
2. Start at 20 percent.
3. Move slowly through the target photo poses.
4. Increase in small steps: 25, 30, 35, 40.
5. Stop immediately if the arm rises by itself, dives, oscillates, heats, or
   sounds abnormal.
6. Use `Home disable` before leaving the arm.
