## 任务目标

使用已连接电脑的 Seeed Studio XIAO ESP32S3 Sense，模拟佳能 BR-E1 蓝牙快门，在网页中提供相机配对、连接状态、断开、清除配对以更换相机，以及单次拍照。

2026-10-15 用户确认：“现在整个流程跑通了，能够通过网页连接这个相机，并且能够拍照了。”因此首版网页连接与实际拍照的实机验收通过，按用户要求存档。

## 数据来源（如果有）

- 用户本次实机反馈：网页连接 Canon EOS R6 并触发实际拍照成功。
- 开发阶段硬件检查：ESP32-S3、8MB Flash、USB `303a:1001`，USB 序列号 `28:84:85:B4:0A:34`，当时串口 `/dev/cu.usbmodem11301`。
- 开发阶段设备状态：相机名称 `seeedr6`；完成蓝牙认证与遥控器注册，日志曾记录 3 次快门按下/释放写入成功。照片成功由用户另行确认，不以串口写入成功替代实拍验收。
- 开发阶段编译与烧录：Arduino ESP32 3.3.8（S3 自带 NimBLE）、ArduinoJson 7.0.4，目标 `esp32:esp32:XIAO_ESP32S3`；0.1.1 烧录数据校验及固件标识读回通过。
- 开发阶段自动检查：12 项 Python 测试、JavaScript 语法检查通过。本次存档只更新文档，未重新烧录、触发快门或操作机械臂。

## 文件变更

首版功能源码提交：`584520b`（`feat: 新增 XIAO 佳能蓝牙快门固件与网页控制`）。

- `canon_shutter/firmware/canon_shutter/canon_shutter.ino`：0.1.1 固件，串口 JSON 控制、佳能 BLE 扫描/加密/身份握手、持久配对、断开/清除配对、快门按下与释放、超时处理。
- `canon_shutter/server.py`：Python / pyserial 本机网页桥，根据 USB 序列号寻找设备；离线、状态过期和忙碌时限制操作，不重发快门。
- `canon_shutter/static/`：中文控制页，显示真实状态、连接记录和已发送快门次数。
- `canon_shutter/test_server.py`：12 项命令处理与 HTTP 接口测试。
- `canon_shutter/README.md`、`THIRD_PARTY_NOTICES.md`、`requirements.txt`：操作说明、协议来源与许可、运行依赖。
- `docs/superpowers/specs/2026-10-15-esp32-canon-shutter-design.md`：首版设计和实施范围。
- 本次存档更新 `AGENTS.md`、`canon_shutter/README.md`，并新增本文件，将“待确认实际照片”改为“用户确认实拍成功”。

本机保留、未提交 Git 的文件：

- 原完整 8MB Flash：`canon_shutter/backups/xiao-esp32s3-before-canon-20261015.bin`，同目录附 SHA-256。
- 当前固件二进制及哈希清单：`canon_shutter/build/`；版本 `canon-shutter-0.1.1`。
- 应用二进制 `canon_shutter.ino.bin` 的 SHA-256：`8d9640cd98c02f253440245b0979804e8701764a5771651ff8611a1f2f75b5c8`。
- 合并固件 `canon_shutter.ino.merged.bin` 的 SHA-256：`1ffe88618783ee413e6d1f634fd3e0cd4a426acac4f17165e1bfbe12ca340ae7`。

原有 4 个 `gravity_console` 文件的未提交改动保留，不属于本次快门功能存档，不纳入本次提交。

## 关键决策

- 使用分支 `feature/esp32-canon-shutter`；开发与烧录已获用户授权。本次沿用该分支记录已验证效果，不合并、不推送。
- 第一版链路：浏览器 → 本机 Python HTTP 服务 → USB CDC → XIAO → BLE → EOS R6。电脑与 XIAO 仍需 USB；Sense 板载摄像头不参与工作。
- 运行入口：在项目根目录执行 `python3 canon_shutter/server.py`，浏览器打开 `http://127.0.0.1:8023/`。服务仅监听本机；重启电脑后可用同一命令恢复，未配置开机自启。
- 配对时相机先进入“Wi-Fi/蓝牙连接 → 连接至无线遥控器 → 添加设备”，再按网页“配对新相机”。遥控器名称 `XIAO-Shutter`。
- 照片模式使用“自拍：10 秒/遥控”或“自拍：2 秒/遥控”驱动模式；网页“拍照”发送单次即时快门按下与释放。照片由佳能相机保存。
- 普通断开保留配对；清除配对用于换相机。重配同一台相机时还需删除相机端旧遥控器连接信息。
- 启动、网页刷新、重连和超时均不自动拍照或重发快门。
- 0.1.0 曾出现配对操作已完成、网页却未收到结束消息的情况。0.1.1 增加 USB TX 缓冲，并在心跳内持续携带最近的 `last_result`，网页按命令 ID 恢复完成状态。

## 遗留问题

- 已确认网页连接和实际拍照成功。休眠恢复、长时间稳定性及清除配对后更换另一台相机，尚未逐项专项验收。
- 当前未提供板端 Wi-Fi 网页；如要脱离电脑使用，需要新增 Wi-Fi 网络和网页服务。
- 当前不提供照片回传、实时取景或照片存卡确认；网页的“已发送”仅表示快门指令写入结果。
- 分支已存档，等待用户确认后才合并到 `main`；既有机械臂网页未提交改动仍保留在工作区。
