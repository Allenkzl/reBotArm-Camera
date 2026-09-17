# reBotArm-Camera

This repository tracks the staged conversion of a Seeed reBot Arm B601/RS00
from a gripper end effector to a camera payload.

## Goal

Replace the J7 gripper with an approximately 0.5 kg camera payload, validate
that J1-J6 can safely support and move the new end load, use gravity
compensation to hand-guide the arm to four fixed photo poses, and later
automate photo capture at those poses.

## Current Stage

Target 1: remove the gripper, mount the camera, and perform read-only checks.

This stage does not authorize active motion, motor parameter writes, firmware
changes, calibration writes, or gravity-compensation force output. The first
acceptance check is that J1-J6 are online and readable after J7 is removed, and
that the camera mount and cable routing look mechanically safe.

## Hardware Baseline

- Arm: Seeed reBot Arm B601/RS00
- Remote host: `com@com-desktop.local`
- Fallback IP from earlier sessions: `192.168.50.194`
- CAN: PCAN-USB on SocketCAN `can0`
- Previous gripper: J7 / CAN ID `0x07`
- New payload: camera plus mount and cable, expected around 0.5 kg

## Safety Rules

- Treat J7 as removed for this project unless explicitly reinstalled.
- Do not run gripper open, close, set, or J7 recovery commands.
- Read-only scans and angle reads are allowed after the user confirms the
  camera is mounted and the arm is physically supported.
- Active motion, enable, gravity compensation, trajectory execution, Home
  changes, URDF payload edits, and calibration writes each require separate
  confirmation.
- Do not disable a loaded or suspended arm unless it is already in a supported
  safe pose.

## Documents

- `docs/checklists/target-1-camera-mount-scan.md`: target 1 checklist and
  read-only scan plan.
- `docs/sessions/`: session notes written on archive/exit or explicit request.
