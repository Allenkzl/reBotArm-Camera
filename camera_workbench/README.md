# reBot Studio 工作台

网页由 reComputer 上的 `camera_workbench.server` 提供，默认监听 `127.0.0.1:8012`。入口、安装和当前运行参数见 [项目 README](../README.md)。

`app.py` 负责工作台状态、分组动作、拍摄点、回放和过温保护。 `ros_backend.py` 通过 `/rebot_arm_rs` 与六轴控制器通信。 `motor_temperature.py` 被动读取 `can0` 的 RobStride 温度和复位反馈。 `static/` 提供中英文界面。

拍摄点保存六轴原始角度。界面显示相对于软件 Home 的数值。回放使用持有兼容 Home 参考的记录。旧坐标留存查看，需在有效基准下重新录制后才能播放。

工作台数据保存在设备的 `deployment/recomputer/workbench-data`：

| 文件 | 用途 |
|---|---|
| `shooting_groups.json` | 分组、动作顺序、动作间隔和拍照前等待 |
| `teach_trajectories.json` | 已录制动作和拍摄点 |
| `replay_shutter_angles.jsonl` | 快门命令前六轴样本、容差、目标及实际误差 |
| `photo_journal.jsonl` | 快门请求和结果 |
| `camera_settings.json` | 拍照后停留 |
| `hardware_permission.json` | 实机和手动示教入口授权 |
| `thermal_trip.json` | 过温保护锁定。确认冷却后可在界面解除 |

离线验证：`python3 -m unittest camera_workbench.test_app -v`。
