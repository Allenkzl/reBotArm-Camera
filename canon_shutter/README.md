# XIAO ESP32S3 Sense → Canon EOS 蓝牙快门

第一版使用电脑本地网页，通过 USB CDC 控制 XIAO 的 BLE 遥控功能。电脑与 XIAO 需要保持 USB 连接；当前固件不提供 Wi-Fi 网页。相机与 XIAO 之间不需要线缆。Sense 板载摄像头不参与工作。

## 本地网页

已验证的运行环境：Python 3.11.3、pyserial 3.5。

```bash
python3 canon_shutter/server.py
```

打开 http://127.0.0.1:8023 。服务仅绑定本机，不依赖互联网或路由器。

默认绑定本次设备 USB 序列号 `28:84:85:B4:0A:34`，重插后自动寻找同一块板。
使用其他板时，通过 `--serial-number` 指定对应序列号，使用 `--port` 指定初始串口。
空序列号 `--serial-number ''` 则只使用给定串口。

## 配对 EOS R6

1. 相机启用 Wi-Fi 设置，在「Wi-Fi/蓝牙连接 → 连接至无线遥控器 → 添加要连接的设备」停留在配对画面。
2. 网页点击「配对新相机」。ESP32 扫描约 8 秒；只找到一台时自动连接，多台时在结果列表选择。
3. 相机如出现确认提示，按相机指示完成确认。遥控器标识为 `XIAO-Shutter`。
4. 照片模式下，驱动模式使用「自拍：10 秒/遥控」或「自拍：2 秒/遥控」。首次可手动对焦测试。
5. 网页显示「已连接」后点「拍照」。页面仅报告快门指令是否成功写入，请在相机上检查照片。

官方流程：https://cam.start.canon/en/C004/manual/html/UG-06_Network_0090.html

「断开连接」保留配对，之后可点「连接相机」。设备启动后不会自动重连或拍照。
「清除配对 / 更换相机」删除本机保存的相机和对应 BLE bond；重新配对同一台相机时，也要在相机上删除旧遥控器连接信息。清除后再给新相机配对。当前只保存一台相机。

## 编译与烧录

本次使用已安装的 Arduino CLI、`esp32:esp32@3.3.8`（S3 自带 NimBLE 后端）和 ArduinoJson 7.0.4。固件使用 ESP32 自带 BLE / Preferences，无需额外安装旧版 Canon BLE 库。

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 \
  --build-path /private/tmp/canon-shutter-build canon_shutter/firmware/canon_shutter

# 先停止本项目 server.py，释放串口，再执行：
arduino-cli upload --fqbn esp32:esp32:XIAO_ESP32S3:UploadSpeed=115200 \
  --port /dev/cu.usbmodem11301 --input-dir /private/tmp/canon-shutter-build \
  canon_shutter/firmware/canon_shutter
```

不执行全片擦除；构建产物与原 flash 备份不提交 Git。原 flash 含原应用及可能的配置，应仅留本机。
编译参数采用板型默认 USB CDC、8MB Flash。固件当前标识：`canon-shutter-0.1.1`。

## USB 协议

每行一个 JSON。请求包含唯一 `id`、`action` 和可选 `address`：

```json
{"id":"example","action":"status"}
```

`action` 支持 `status / pair / scan / connect / disconnect / forget / shutter`。
`connect` 无地址时连接已保存相机，有地址时只能选择当前扫描候选。
设备每 750 ms 输出 `type=status`；命令完成输出 `type=result`，包含原 `id`、`ok`、`message`。
状态心跳也重复携带最近一次 `last_result`，避免一次完成消息丢失导致网页持续忙碌。
USB 中断、超时或设备重启时，网页不会重发快门；照片结果未知时需人工核对。

## 验证

```bash
cd canon_shutter
python3 -m unittest -v test_server.py
```

12 项测试覆盖状态过期和未连接时拒绝快门、重复点击、重启不重放、串口失败不重试、完成消息遗漏的心跳恢复，以及 HTTP 请求来源边界。

2026-10-15 实机结果：已确认 XIAO ESP32-S3 / 8MB Flash，完整原固件备份在 `backups/xiao-esp32s3-before-canon-20261015.bin`（附 SHA-256）；0.1.1 烧录校验与固件状态读回成功。此前 0.1.0 已与用户的 `seeedr6` 完成认证和遥控器注册，并收到 3 次快门按下/释放写入成功；尚未取得用户对实际照片的确认。配对记录在升级后保留。新版二进制及构建哈希保存在忽略目录 `build/`。

0.1.0 实测一次配对结束消息未到达网页，导致按钮忙碌直到超时；0.1.1 增加 USB TX 缓冲，并由心跳持续携带完成结果。实际照片、休眠恢复及换相机能力仍需要逐项实机验收。

协议参考和许可见 `THIRD_PARTY_NOTICES.md`。
