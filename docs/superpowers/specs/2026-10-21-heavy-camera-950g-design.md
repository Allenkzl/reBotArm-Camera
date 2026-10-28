# Heavy Camera 950 g 参数修正

用户确认新开分支，将整套相机总重更新为 950 g，供后续轨迹调试。

沿用既有近似补偿方案：约 800 g 的 light/URDF 等效基线加 150 g 增量；不将 950 g 再次整体叠加到 URDF 上。另一方案是替换 URDF 中旧夹爪的完整惯性模型，但需要真实质量分布和坐标几何，本次不采用。

Heavy profile 增加总重元数据 0.95 kg，增量从 0.4 改为 0.15 kg；网页按钮及状态说明显示 950 g。质心估计继续使用 `[0.075, 0, -0.025] m`，控制增益、倍率、速度及力矩限幅不变。共用 `_estimate_payload()` 的录制、回放、保持和 Home 自动采用新增量；历史轨迹的角度和时序保持原样。

验证：Python/JavaScript 语法、差异检查；离线 dry-run 检查状态元数据、Light 不变及 Heavy 新增力矩为旧增量的 37.5%；HTTP 页面与状态接口验证。检查远端连接和运行状态，只有确认不在控制电机时才能更新并重启本项目网页。真实机械臂不自动使能或执行轨迹。

这属于重量参数修正，约 800 g 等效基线和质心仍是近似；模型辨识、有效 kp 和真实控制效果需要后续独立实测，不能以本次离线检查宣称完成校准。

实施结果：Python 编译、JavaScript 语法及 `git diff --check` 通过。临时 localhost dry-run 服务验证页面显示 950 g，`/api/status` 返回总重 0.95 kg、增量 0.15 kg、`enabled=false`；Light 和质心不变，Heavy 额外力矩为旧值的 37.5%。未连接硬件进行主动测试。

远端部署暂未完成：2026-10-21 通过 `com@com-desktop.local` 只读连接时 SSH 返回 `Permission denied (publickey,password)`，因此未上传文件、重启远端服务或执行运动；远端网页可能仍为旧参数。恢复 SSH 访问后应先读 `/api/status` 确认非使能/控制中，再部署本次修正。

后续部署已完成：用户重新登录后复用其 SSH 会话，确认远端 disabled/connected=false，按精确旧文本仅替换 Heavy profile 和按钮，保留远端其他代码。备份位于 `/home/com/reBotArm-Camera/logs/deployment-backups/heavy-camera-950g-20261021_105456/`，新服务 PID 887907，启动日志 `/home/com/reBotArm-Camera/logs/runs/web_950g_20261021_105456.log`。恢复原 profile=heavy 和各关节 80% 设置；未调用 Start/录制/回放/Home。

远端及本机转发接口确认 total_camera_kg=0.95、payload_kg=0.15、enabled=false；刷新 Chrome 后页面显示 `Heavy Camera (950 g)` 及 150 g 增量说明，三条轨迹仍在。轨迹文件 SHA-256 更新前后均为 `a2848723520ceb60de3b24dfbd47083a55424a1b6c2b38d821a0e2b54eef6b12`。远端文件目录未初始化 Git，本次直接部署经过验证的最小补丁；本地分支未合并/推送。真实轨迹效果仍待用户实测。
