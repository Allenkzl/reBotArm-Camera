# reBot Studio · 机械臂拍摄工作台

本仓库归档 2026-11-19 的阶段交付版本。工作台用于在 reComputer 上控制六轴 reBot Arm，记录拍摄点，并联动佳能相机拍摄。日常入口为 [reBot Studio](http://127.0.0.1:8012/)。

## 当前版本

| 项目 | 配置与行为 |
|---|---|
| 机械臂与相机 | J1–J6，CAN ID 1–6，PCAN SocketCAN `can0`。末端标称负载 950 g，动力学模型采用估算质心 |
| 软件零点 | `user-home-20261119_135636`。Home 各轴绝对偏差必须小于 0.5° |
| 动作录制 | 手动示教时按空格或点「设置拍照点」，记录当时的六轴原始角度并触发快门 |
| 回放与快门 | 单动作和整组均按拍摄点回放。六轴拍摄容差为 ±0.7°，各组可独立设置拍照前等待和动作间隔 |
| 速度与回程 | 工作台移动参数为 1.0 rad/s。整组每个动作完成后回 Home，到位后等待 5 秒 |
| 温度保护 | 六轴卡片显示实时反馈，70°C 起显示黄色并预警，80°C 起显示红色。工作台取消当前任务，反馈正常时回 Home 后失能，并锁定新的运动指令 |
| 工作台 | 中文和英文切换，软件停止、原位失能、角度与回放误差日志 |

温度保护依赖新鲜的电机反馈和可用的控制器。80°C 保护流程已用模拟温度验证，未通过真机过温触发验收。若无法确认回 Home 或失能，工作台会提示现场处理。

## 日常使用

1. 确认机械臂周围净空、相机固定和线缆走向，打开工作台。按现场流程启用机械臂并连接相机。
2. 选择拍摄组。录制时使用手动示教，到目标姿态后按空格保存拍摄点，完成后停止录制。
3. 设置本组的「拍照前等待」和「动作间隔」，再单独回放或整组播放。快门结果以相机中实际保存的照片为准。
4. 拍摄结束后使用「关闭机械臂」，等待回 Home 和失能状态。需要原位失能时，先在现场可靠支撑机械臂。

## 从归档还原

测试过的部署环境是 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。归档以 [上游固定版本](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS/tree/455d008e82530b1b6f645e2427f89881c2aa071e) 为底层，附上线上的控制器补丁、六轴配置、相机模型和启动脚本。

在 reComputer 上准备一份干净的上游 checkout。以下步骤安装源码、构建工作区并打开网页服务。网页服务启动时保持硬件未启用。

```bash
git clone https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS.git
git -C ReBot_Arm_DigitalTwin_RS checkout --detach 455d008e82530b1b6f645e2427f89881c2aa071e
./deployment/recomputer/apply-stage.sh ReBot_Arm_DigitalTwin_RS
cd ReBot_Arm_DigitalTwin_RS
./scripts/setup_rs_workspace.sh
ROS_LOCALHOST_ONLY=0 ./scripts/start_legacy_camera_workbench.sh
```

新部署的工作台默认关闭运动和手动示教权限。工程人员确认安装、六轴配置、Home 与实物一致后，可按现场授权将 `deployment/recomputer/workbench-data/hardware_permission.json` 中相应的 `allow_motion` 和 `allow_manual_teach` 设为 `true`。实机启动由现场操作员在工作台发起。控制器的 `startup_enabled` 为 `false`。

佳能快门桥使用 [USB 与 BLE 部署说明](canon_shutter/README.md)。若快门桥运行在操作电脑上，可用同一个 SSH 会话进行正向和反向转发：

```bash
ssh -NT -L 127.0.0.1:8012:127.0.0.1:8012 \
  -R 127.0.0.1:8023:127.0.0.1:8023 USER@RECOMPUTER
```

## 归档内容与验证

- [工作台说明](camera_workbench/README.md)记录实现结构和离线测试入口。
- [六轴部署目录](deployment/recomputer/)包含固定上游补丁、启动脚本、机型配置和 950 g 模型。
- [软件零点](config/recomputer_home.json)保存本机阶段校准值。其他机械臂需使用现场确认的参考，保留电机内部编码器零点。
- [佳能快门项目](canon_shutter/)包含 ESP32 固件、USB 桥和配套测试。
- 在线拍摄组、客户拍摄点、快门记录和日志保存在设备的 `deployment/recomputer/workbench-data`。阶段归档保留源码与配置，业务数据和访问凭据单独保管。

在本仓库执行以下命令可复核工作台和快门桥逻辑。快门桥测试请在已激活的 Python 虚拟环境中运行。

```bash
python3 -m unittest camera_workbench.test_app -v
python3 -m pip install -r canon_shutter/requirements.txt
python3 -m unittest discover -s canon_shutter -p test_server.py -v
```

工作台的 23 项离线回归通过。快门桥的 12 项回归中有 11 项通过。在单调时钟起点不足 3 秒的测试环境里，USB 写入失败后可能短暂显示在线，因为当前实现以 `0` 表示反馈已失效。这一行为随现用版本归档。真实运动、过温保护和新设备安装需按现场条件验收。
