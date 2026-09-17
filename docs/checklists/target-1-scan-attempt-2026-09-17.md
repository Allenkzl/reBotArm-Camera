# Target 1 Scan Attempt: 2026-09-17

Time: 2026-09-17 11:09:46 CST

## Operator Confirmation

- Camera has been mounted.
- Cable routing has been checked by the operator and is not being pulled.
- The requested action was read-only scanning.

## Commands Attempted

All commands were read-only. No enable, disable, gripper, J7, motion, gravity
compensation, calibration, firmware, ID-write, or parameter-write command was
run.

### SSH via mDNS hostname

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 com@com-desktop.local '...read-only host and CAN checks...'
```

Result:

```text
ssh: Could not resolve hostname com-desktop.local: nodename nor servname provided, or not known
```

### SSH via last known IP

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 com@192.168.50.194 '...read-only host and CAN checks...'
```

Result:

```text
ssh: connect to host 192.168.50.194 port 22: Operation timed out
```

### Local network observations

Local Mac interface:

```text
en0 inet 192.168.50.109/24
```

ARP entries relevant to this check:

```text
192.168.50.1
192.168.50.109
192.168.50.128
192.168.50.161
192.168.50.194 incomplete
```

Ping sweep found:

```text
192.168.50.109
192.168.50.128
192.168.50.161
192.168.50.191
```

SSH port checks:

```text
192.168.50.128:22 connection refused
192.168.50.161:22 timed out
192.168.50.191:22 connection refused
```

mDNS service browsing:

```text
_ssh._tcp.local: no matching service returned during the observation window
_workstation._tcp.local: no matching service returned during the observation window
```

Bare hostname check:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=6 com@com-desktop 'hostname'
```

Result:

```text
ssh: Could not resolve hostname com-desktop: nodename nor servname provided, or not known
```

## Result

Target 1 motor scanning was not reached. The blocker is network discovery or
SSH reachability of the reComputer, not a detected motor/CAN failure.

No motor scan was executed because the remote host could not be reached.

## Next Checks

- Confirm the reComputer is powered on.
- Confirm it is connected to the same `192.168.50.0/24` network as the Mac, or
  provide its current IP.
- Confirm SSH service is running on the reComputer.
- If connected by display/keyboard, run `hostname -I` and `systemctl status ssh`
  on the reComputer.
- After SSH is reachable, continue with the J1-J6-only scan in
  `target-1-camera-mount-scan.md`.
