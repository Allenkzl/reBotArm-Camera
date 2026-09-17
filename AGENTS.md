# reBotArm-Camera

## 项目简介

本项目用于记录和实现 reBot Arm B601/RS00 从 J7 夹爪末端改为相机负载末端的全过程。近期目标是拆除 J7 夹爪、安装约一斤重的相机，先通过只读扫描和人工检查确认 J1-J6、相机固定和线缆没有明显问题，再分阶段进入重力补偿、四个定点记录和拍照自动化。

## 技术栈

- Python 3.10
- motorbridge / SocketCAN / PCAN-USB
- ROS2 Humble / rebotarmcontroller
- Pinocchio / reBotArm_control_py
- Markdown runbook and checklists

## 目录结构

- `README.md`: 项目目标、阶段和安全边界。
- `docs/checklists/`: 每个目标的操作检查单和验收记录。
- `docs/sessions/`: 仅在用户要求归档或退出时写入会话日志。

## 当前状态

- 活跃分支：`main`，用于项目初始化和目标一只读检查准备。
- 已完成功能：
  - 建立本地仓库并绑定远程 `https://github.com/Allenkzl/reBotArm-Camera.git`。
  - 建立目标一：相机安装后的只读扫描检查单。
- 进行中：
  - 2026-09-17 用户已安装相机并确认线缆不会被拉扯；首次只读扫描尝试未能连接到 `com-desktop.local` 或历史 IP `192.168.50.194`，尚未执行电机扫描。

## 重要约定

- 本项目中 J7 默认视为已拆除，不再要求 J7 在线，不执行夹爪 open/close/set，也不执行 J7 回零。
- 目标一只允许只读检查：SSH、USB/CAN 状态、J1-J6 扫描、J1-J6 角度读取、相机固定和线缆人工检查。
- 主动运动、使能、轨迹执行、重力补偿输出、电机参数写入、固件/校准/Home/URDF 变更，都必须单独确认。
- 机械臂装载相机后，不允许在悬空或未支撑姿态下直接 disable；若回安全姿态失败，应保持使能并人工处理。
- `com-desktop.local` 优先于历史 IP；使用 `can0` 前必须再次确认它仍是 PCAN-USB 而非 Jetson 内置 CAN。
- 旧抓取项目中的 gripper Home、J7 零点、手眼标定和 `gripper_end` TCP 都不能直接当作相机负载阶段的最终事实。

## 变更日志

| 日期 | 分支 | 说明 |
| --- | --- | --- |
| 2026-09-17 | main | 初始化本地仓库文档，记录相机负载改造目标、安全边界和目标一只读检查计划。 |
| 2026-09-17 | main | 记录目标一首次只读扫描尝试：相机已安装，但 reComputer 当前无法通过 mDNS 或历史 IP SSH 访问，电机扫描尚未执行。 |
