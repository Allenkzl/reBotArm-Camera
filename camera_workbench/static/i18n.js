"use strict";

const workbenchLocale = (() => {
  const labels = new Map(`
reBot 拍摄工作台|reBot Camera Workbench
机械臂拍摄工作台|Robot Camera Workbench
连接中…|Connecting…
六轴实时角度|Live Joint Angles
六轴实时角度 · 等待本次打点|Live Joint Angles · Awaiting Capture Point
等待读数|Waiting for feedback
无快门保持测试|Hold Test (No Shutter)
等待测试，继续托稳|Waiting for test. Keep supporting the arm.
出现松手提示前，请持续托住相机和支架。|Support the camera and mount until prompted to release.
剩余秒数|Seconds remaining
预览 10 秒倒计时|Preview 10 s Countdown
机械臂控制|Arm Controls
未使能|Disabled
启动机械臂|Enable Arm
自动回 Home · 到位后保持|Return Home and hold
回零点（Home）|Return Home
回到零位 · 保持使能|Return Home and stay enabled
关闭机械臂|Disable Arm
回到 Home · 然后失能|Return Home, then disable
紧急停止并保持|Emergency Stop & Hold
蓝牙相机|Bluetooth Camera
连接蓝牙相机|Connect Camera
断开连接|Disconnect
正在读取相机状态…|Checking camera status…
拍照设置|Capture Settings
拍照后停留（秒）|Post-shot hold (s)
保存|Save
快门完成后按设置停留。首次配对请使用原有快门网页完成。|Hold for this duration after each shot. Use the shutter page for initial pairing.
相机状态|Camera Status
配重设置|Payload Settings
额定配重|Rated Payload
查看配置|View Setup
等待负载配置|Waiting for payload setup
末端负载（克）|End-effector payload (g)
手动拖动并录制动作|Guide the arm and record an action
选择拍摄组|Select a shooting group
按顺序播放组内动作|Play the actions in order
完成后关闭|Disable when finished
Home 零位后失能|Return Home before disabling
拍摄组|Shooting Groups
新增拍摄组|Add Group
拍摄组选择|Select shooting group
当前拍摄组|Current Group
拍摄组名称|Group name
删除组|Delete Group
动作间隔|Action Interval
拍照前等待|Pre-shot Wait
到达拍摄点后，等待再触发快门|Wait at the capture point before firing the shutter
拍照前等待请填写 0–60 秒，精度为 0.1 秒|Enter a pre-shot wait from 0 to 60 seconds in 0.1-second increments
拍照前等待已保存|Pre-shot wait saved
秒|s
前一个动作完成后，等待再执行下一个|Wait between actions
播放整组|Play Group
执行顺序 / 动作|Order / Action
录制与预览|Record & Playback
新增动作块|Add Action
整组播放后停留，点击「关闭机械臂」完成收尾|The arm stays enabled after playback. Select Disable Arm to finish.
正在读取机械臂状态…|Checking arm status…
运行详情|Run Details
紧急失能|Emergency Disable
紧急停止会停止当前动作并保持使能；紧急失能会立即卸去支撑力，必须先人工支撑机械臂。|Emergency Stop stops motion and holds position. Emergency Disable removes motor support immediately. Support the arm first.
运行事件|Run events
温度预警|Temperature warning
温度|Temp.
温度 —|Temp. —
温度保护|Thermal protection
正在停止当前任务|Stopping the current task
正在回 Home|Returning Home
已回 Home 并失能|At Home and disabled
等待确认冷却|Waiting for cooldown confirmation
无法确认回 Home 并失能，请现场处理|Return Home and disable unconfirmed. Check the arm on site.
确认冷却并解除保护|Confirm Cooldown and Reset
请确认机械臂已失能，且六个舵机都已冷却到 70°C 以下。解除保护？|Confirm the arm is disabled and all six motors have cooled below 70°C. Reset thermal protection?
温度保护已锁定，请先确认舵机冷却|Thermal protection is locked. Confirm motor cooldown first.
请先确认所有舵机已冷却到 70°C 以下|Confirm all motors have cooled below 70°C first
温度保护仍在处理，请先确认机械臂已失能|Thermal protection is still active. Confirm the arm is disabled first.
温度保护已解除 · 等待启动机械臂|Thermal protection reset · Ready to enable
温度保护状态未能保存|Could not save the thermal protection state
舵机达到 80°C · 已取消当前任务|Motor reached 80°C · Current task cancelled
温度保护 · 已返回 Home 并失能|Thermal protection · At Home and disabled
温度保护 · 机械臂已失能|Thermal protection · Arm disabled
温度保护 · 无法确认回 Home 并失能，请现场处理|Thermal protection · Return Home and disable unconfirmed. Check the arm on site.
语言|Language
网页服务未连接|Workbench disconnected
请先启动机械臂|Enable the arm first
请等待当前请求完成|Wait for the current request to finish
请先结束录制或停止当前动作|Finish recording or stop the current action first
请先处理机械臂异常状态|Resolve the arm fault first
拍照结果待核实，请检查相机后重新点击连接确认|Check the camera for the shot, then reconnect to confirm
相机桥未连接|Camera bridge disconnected
ESP32 未连接|ESP32 disconnected
相机正在处理操作|Camera busy
请先连接蓝牙相机|Connect the Bluetooth camera first
相机尚未就绪|Camera not ready
运行异常|Operation error
已失能 · 暂无实时反馈|Disabled · No live feedback
等候实时反馈|Waiting for joint feedback
回 Home 时读数 · 已失能|Home readings · Disabled
正在获取机械臂状态|Checking arm status
等待工作台连接|Waiting for workbench connection
连接中断 · 机械臂状态未知|Disconnected · Arm state unknown
请检查网络及现场机械臂状态|Check the network and the arm on site
操作已停止 · 请检查|Operation stopped · Check status
机械臂正在启动|Enabling arm
正在检查反馈并准备控制|Checking joint feedback
机械臂正在返回 Home|Returning Home
机械臂已失能|Arm disabled
J1–J6 均处于复位|J1–J6 are in reset mode
舵机已复位，控制器仍报告使能，启动暂不可用|Drives are in reset. The controller still reports enabled, so startup is locked.
已返回 Home，等待下一次操作|At Home. Ready for the next operation.
等待启动机械臂|Waiting for the arm to be enabled
拍照完成 · 正在返回 Home|Capture complete · Returning Home
正在校正拍摄点 · 等待到位|Settling at capture point
已到达拍摄点 · 正在触发快门|At capture point · Triggering shutter
已到达拍摄点 · 等待相机结果|At capture point · Waiting for camera
拍照完成 · 正在保持位置|Capture complete · Holding position
机械臂正在运动 · 前往拍摄点|Moving to capture point
正在准备回放|Preparing playback
正在检查动作与相机状态|Checking action and camera status
等待下一个动作|Waiting for next action
已返回 Home · 保持使能|At Home · Arm enabled
回 Home 并等待|Return Home and wait
每个动作完成后回 Home，到位后等待 5 秒。|After each action, the arm returns Home and waits 5 seconds.
已到达拍摄点 · 保持位置|At capture point · Holding position
机械臂已停止 · 保持使能|Arm stopped · Holding position
新 Home 保持到位|Holding at Home
正在固定保持新 Home|Holding at the new Home
正在录制 · 相机尚未就绪|Recording · Camera not ready
空格和鼠标打点需要相机就绪。请先停止录制，再连接蓝牙相机。|Stop recording and connect the Bluetooth camera before marking a capture point.
点位已记录 · 等待拍照结果|Point saved · Waiting for camera
点位已记录 · 拍照指令完成|Point saved · Shutter command complete
正在录制 · 请手动摆位|Recording · Guide the arm by hand
摆到目标后稳住，按空格记录点位|Hold the desired pose and press Space to save the point
已到达拍摄点|At capture point
机械臂正在执行动作|Executing action
执行中，请保持活动范围畅通|Keep the arm's workspace clear
机械臂已启动 · 等待操作|Arm enabled · Ready
可开始录制或回放|Ready to record or play
正在处理相机连接操作|Connecting camera
请继续托稳，等松手提示|Keep supporting the arm until prompted
正在确认起始位置。|Checking the starting position.
现在缓慢松手|Slowly release support now
即将结束，保持当前托举状态|Test ending. Keep your current level of support.
测试结束，请托稳相机|Test complete. Support the camera.
双手留在旁边。若相机明显下沉或抖动，立即托住并点「紧急停止」。|Keep your hands nearby. If the camera drops or shakes, support it and select Emergency Stop.
实时状态中断，请继续托稳|Live feedback lost. Keep supporting the arm.
等待工作台恢复实时状态。|Waiting for live feedback to resume.
测试已结束，请托稳相机|Test ended. Support the camera.
请等待收尾指令。|Wait for the next instruction.
演示模式 · 请勿实际松手|Demo · Keep supporting the arm
这是界面演示。实际操作以正式测试的提示为准。|Preview only. Follow the prompts during the actual test.
退出演示|Exit Preview
六轴实时角度 · 新 Home（相对零位）|Live Joint Angles · Relative to Home
六轴实时角度 · 当前打点（相对新 Home）|Live Joint Angles · Capture Point
六轴实时角度 · 空格记录点|Live Joint Angles · Saved Point
旧工作台已恢复 · 真机操作尚未开放|Workbench ready · Hardware control is locked
已连接机械臂服务，请确认当前运行状态|Arm controller connected. Check its current state.
已连接，点击「启动机械臂」开始操作|Connected. Select Enable Arm to begin.
操作已提交|Request submitted
未启动|Not started
正在启动|Enabling
拖动模式|Hand-Guided Mode
保持位置|Holding
播放中|Playing
准备播放|Preparing playback
动作间隔中|Between actions
正在返回 Home|Returning Home
异常，请查看详情|Fault · See details
已失能|Disabled
已使能|Enabled
录制中|Recording
连接中断 / 状态未知|Disconnected / State unknown
状态未知|Unknown state
未连接网页服务|Workbench disconnected
模拟服务 · 不控制电机|Simulation · No motor control
实机服务 · J1–J6|Hardware · J1–J6
等待拍摄组数据|Loading shooting groups
停止播放|Stop Playback
停止后保持使能，可继续录制或关闭机械臂。|The arm holds after stopping. You can record again or disable it.
历史坐标记录，请在新 Home 基准下重新录制|Legacy coordinates. Record again using the current Home.
请在 950 克配置下录制并按空格打点|Use the 950 g setup and press Space to mark a point
正在录制|Recording
暂无已保存录制|No saved recording
J1–J6 轨迹|J1–J6 Trajectory
历史记录已保留；请在新基准下重新录制，旧坐标不会直接回放。|Legacy recording retained. Record again using the current Home before playback.
按|Press
空格|Space
保存六轴位置并拍照，请保持姿势至结果出现。|to save the six joint angles and take a photo. Hold until the result appears.
保存六轴位置。|to save the six joint angles.
停止录制|Stop Recording
重新录制|Re-record
开始录制|Start Recording
设置拍照点|Set Capture Point
到首个打点测试六轴 ±0.5°，不触发快门|Move to the first point and check ±0.5° on all six axes without firing the shutter
无快门精度回放|Precision Playback (No Shutter)
按点回放|Play Capture Points
单独播放|Play Action
正在读取拍摄组…|Loading shooting groups…
暂无已保存动作|No saved actions
移动到动作起点|Moving to start
播放动作|Playing action
正在播放单个动作…|Playing action…
停稳中|Settling
正在发出快门指令|Sending shutter command
等待相机结果|Waiting for camera
拍照后停留|Post-shot hold
拍照点已完成|Capture point complete
拍照异常|Capture error
未知|Unknown
已连接|Connected
未连接|Disconnected
暂无|None
等待配置|Awaiting setup
单动作按点拍照|Capture at saved points
相机状态未知 · 服务未连接|Camera status unknown · Workbench disconnected
拍照结果未知，请先人工核实，重新连接确认后才能继续。不会自动补拍。|Capture result unknown. Check the camera and reconnect to continue. No automatic retry.
相机已就绪|Camera ready
首次配对请使用原有快门网页完成。|Use the shutter page for initial pairing.
固件状态未知|Firmware version unknown
正在记录点位并发送快门，请保持姿势|Saving point and triggering shutter. Hold the pose.
六轴点位已记录，快门指令已完成，请查看相机|Six joint angles saved. Shutter command complete. Check the camera.
上次拍照结果未知。请检查相机照片，确认无需补拍后重新连接。继续确认并连接？|The last capture result is unknown. Check the camera and confirm no retake is needed. Reconnect?
相机连接请求已提交，请查看相机状态|Connect request submitted. Check camera status.
相机断开请求已提交|Disconnect request submitted
拍照后停留请填写 0–10 秒|Enter a post-shot hold of 0–10 seconds
拍照后停留设置已保存|Post-shot hold saved
已记录六轴参考角度|Six reference angles saved
启动后机械臂会从当前位置自动返回新 Home，然后固定保持。请确认整段回程畅通、人员离开运动范围。|The arm will return Home from its current position and hold. Confirm the full path is clear and everyone is outside the workspace.
启动请求已提交，请等待设备确认|Enable request submitted. Waiting for controller confirmation.
返回 Home 零位并保持使能？请确认回程畅通。|Return Home and stay enabled? Confirm the return path is clear.
已请求回零点，完成后保持使能|Return Home requested. The arm will stay enabled.
中止当前动作，回 Home 后失能？请确认回程畅通。|Stop the current action, return Home, then disable? Confirm the return path is clear.
已请求返回 Home，完成后将失能|Return Home requested. The arm will then disable.
已请求停止当前动作并保持使能；请核实设备状态|Stop and hold requested. Check the arm status.
立即失能会失去支撑力，机械臂可能下落。确认已人工支撑机械臂，并执行紧急失能？|Disabling removes motor support immediately and the arm may drop. Confirm the arm is physically supported, then disable?
已请求紧急失能，请现场确认机械臂状态|Emergency disable requested. Check the arm on site.
已新增拍摄组（默认四个动作，间隔五秒）|Group added with four actions and a five-second interval
已删除拍摄组|Group deleted
拍摄组名称已保存|Group name saved
动作间隔请填写 0–60 秒|Enter an action interval of 0–60 seconds
动作间隔已保存|Action interval saved
已新增动作块|Action added
已请求停止播放并保持使能|Stop playback and hold requested
整组播放已提交|Group playback requested
动作名称已保存|Action name saved
拍照点已删除|Capture point deleted
删除这个动作块？此操作不可撤销。|Delete this action? This cannot be undone.
动作块已删除|Action deleted
动作录制已保存|Recording saved
请托稳机械臂。重新录制会切换手动摆位，并替换此动作已有轨迹，继续？|Support the arm. Re-recording enters hand-guided mode and replaces the saved recording. Continue?
请先托稳机械臂。开始录制将切换手动摆位模式，继续？|Support the arm. Recording enters hand-guided mode. Continue?
正在录制，请手动拖动机械臂，完成后点击停止录制|Recording. Guide the arm by hand, then select Stop Recording.
开始按点回放？六轴同时在 ±0.5° 内持续 2 秒才拍照；最后一张拍完保持 2 秒，再回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。|Play capture points? All six joints must hold within ±0.5° for 2 s before each shot. After the final shot the arm holds for 2 s, returns Home, and stays enabled. Confirm both paths are clear and everyone is outside the workspace.
开始按点回放？J2 在 ±0.6°、其余五轴在 ±0.5° 内连续停稳 2 秒后拍照。最后一张拍完保持 2 秒，再回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。|Play capture points? Hold J2 within ±0.6° and all other joints within ±0.5° for 2 s before firing. After the final shot the arm holds for 2 s, returns Home, and stays enabled. Confirm both paths are clear and everyone is outside the workspace.
开始按点回放？J2 进入 ±0.6°、其余五轴进入 ±0.5° 后拍照。最后一张完成后按设置停留，再回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。|Play capture points? The shutter fires when J2 is within ±0.6° and all other joints are within ±0.5°. After the final shot and any configured hold, the arm returns Home and stays enabled. Confirm both paths are clear and everyone is outside the workspace.
开始按点回放？J2 进入 ±0.6°、其余五轴进入 ±0.5° 后，按本组设置等待并拍照。最后一张完成后回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。|Play capture points? Once J2 is within ±0.6° and all other joints are within ±0.5°, the arm waits as configured for this group, then takes the photo. After the final shot, it returns Home and stays enabled. Confirm both paths are clear and everyone is outside the workspace.
按点回放已提交|Capture point playback requested
开始无快门精度回放？机械臂会走到第一个打点，尝试微调并保持。请确认路径畅通、人员离开运动范围。|Start precision playback without the shutter? The arm will move to the first capture point, fine-tune its position, and hold. Confirm the path is clear and everyone is outside the workspace.
无快门精度回放已提交|Precision playback requested
动作播放已提交|Action playback requested
网页服务连接中断，设备状态未知。请检查网络和现场机械臂状态。|Workbench disconnected. Check the network and the arm on site.
历史坐标记录|Legacy coordinates
拍摄序列|Shooting Sequences
超出软件范围|Outside software limits
旧工作台界面已恢复 · 新底层未启动|Workbench ready · Controller not started
点位必须包含六轴角度|A capture point requires six joint angles
点位包含无效角度|The capture point contains an invalid angle
动作记录文件格式无效|Invalid action recording format
界面已恢复；真机操作尚未开放，机械臂不会启动|Workbench ready. Hardware controls are locked.
手动示教入口暂未开放：此前切换曾前倾，未改变控制模式|Hand-guided mode is locked after a previous forward tilt. Control mode was unchanged.
相机负载 · 950 g|Camera payload · 950 g
六轴相机配置 · 新 RS 底层|Six-axis camera setup · RS controller
上一操作尚未结束|The previous operation is still running
操作已取消|Operation cancelled
拍照已取消|Capture cancelled
快门结果未确认，请检查相机，不会自动重拍|Shutter result unconfirmed. Check the camera. No automatic retry.
操作已停止 · 请核实机械臂保持状态|Operation stopped · Verify the arm is holding
未取得有效采样，原动作未覆盖|No valid sample received. The saved action was retained.
该动作属于旧坐标或尚未录制，请在新基准下重新录制|Record this action using the current Home reference
动作没有拍照点，请录制并按空格设置点位|This action has no capture points. Record it and press Space to mark a point.
机械臂正在前往拍摄点|Moving to capture point
正在核对六轴 · ±0.5° 连续停稳2秒|Checking all six joints · Hold within ±0.5° for 2 s
正在核对六轴 · J2 ±0.6°，其余 ±0.5° · 连续停稳 2 秒|Checking joint angles · J2 ±0.6°, others ±0.5° · Hold for 2 s
六轴未在容差内连续停稳 2 秒（J2 ±0.6°，其余 ±0.5°），未触发拍照|The joints did not hold within tolerance for 2 s (J2 ±0.6°, others ±0.5°). Shutter not triggered.
正在核对六轴 · J2 ±0.6°，其余 ±0.5°|Checking joint angles · J2 ±0.6°, others ±0.5°
六轴未全部进入容差（J2 ±0.6°，其余 ±0.5°），未触发拍照|Joint error outside tolerance (J2 ±0.6°, others ±0.5°). Shutter not triggered.
请求已被停止操作取消|Request cancelled by stop
录制或执行中不能编辑分组|Groups cannot be edited during recording or playback
拍摄组设置已保存|Group settings saved
拍照停留须为0–10秒|Post-shot hold must be 0–10 seconds
相机操作已提交|Camera request submitted
动作未录制|Action not recorded
紧急失能已提交，请现场确认|Emergency disable requested. Verify the arm on site.
正在启动新六轴底层|Starting the six-axis controller
新底层启动入口未配置|The controller launcher is not configured
底层启动结果未确认，请核实状态，不会自动重试|Controller startup unconfirmed. Check its status. No automatic retry.
启动已取消，请核实设备状态|Startup cancelled. Check the arm status.
已返回 Home · 已失能|At Home · Arm disabled
请先停止当前录制或动作|Stop the current recording or action first
请先启动机械臂并连接相机|Enable the arm and connect the camera first
录制启动已取消，已请求恢复保持|Recording start cancelled. Position hold requested.
正在录制 · 请手动摆位，空格记录六轴点位|Recording · Guide the arm and press Space to save all six angles
录制已保存 · 保持当前位置|Recording saved · Holding position
请先开始录制|Start recording first
请松开空格后再按|Release Space before pressing it again
只能对正在录制的动作打点|You can only mark points for the action being recorded
打点已取消|Point marking cancelled
点位已记录 · 等待单次快门|Point saved · Waiting for shutter
点位已记录 · 快门指令已完成|Point saved · Shutter command complete
请先停止录制|Stop recording first
没有已录制的新点位|No new capture points recorded
此旧诊断接口未接入；工作台不会启动旧控制程序|This legacy diagnostic is unavailable
没有收到新鲜六轴反馈，未记录点位|No fresh six-axis feedback. Point was not recorded.
新底层响应结果未知，请核对状态；不要重复提交|Controller result unknown. Check its status before resubmitting.
新六轴控制器未启动；工作台不会自动使能设备|Six-axis controller is offline. The arm will not enable automatically.
目标必须包含六个有限角度|The target requires six valid joint angles
回放速度超出范围|Playback speed is out of range
新底层轨迹接口未连接|Controller trajectory interface is disconnected
动作已取消|Action cancelled
新底层拒绝轨迹请求|Controller rejected the trajectory
动作已取消或超时，已请求停止并保持|Action cancelled or timed out. Stop and hold requested.
六轴反馈中断，已请求停止|Joint feedback lost. Stop requested.
轨迹未完成|Trajectory did not complete
请求过大|Request too large
请求必须是对象|Request must be a JSON object
尚未读取相机状态|Waiting for camera status
模拟相机|Simulated camera
相机桥状态已过期|Camera bridge status expired
相机桥已关闭|Camera bridge is closed
相机操作正在进行|Camera operation already in progress
此拍摄尝试已使用，禁止重复发送|This capture request was already used
上次拍摄结果未知，请人工检查并重新连接|Last capture result unknown. Check the camera and reconnect.
相机或 USB 状态离线/过期|Camera or USB connection is offline
相机正在忙碌|Camera busy
请先手动连接相机|Connect the camera first
相机操作失败|Camera operation failed
操作在发送前取消|Operation cancelled before sending
模拟相机操作完成|Simulated camera operation complete
相机桥未返回命令 ID，结果未知|Camera bridge returned no command ID. Result unknown.
指令发送后取消，拍摄结果未知；未重试|Cancelled after sending. Capture result unknown. No retry.
相机操作完成|Camera operation complete
相机结果等待超时，未重试|Camera response timed out. No retry.
连接确认日志写入失败|Could not save the connection confirmation
相机结果日志写入失败，请人工检查|Could not save the camera result. Check the camera.
快门指令已发送，请在相机上确认照片|Shutter command sent. Check the camera for the photo.
快门指令已发送|Shutter command sent
等待 USB 设备|Waiting for USB device
USB 写入失败；操作结果未知，未自动重试|USB write failed. Result unknown. No automatic retry.
设备已重启，操作结果未知；未自动重试|Device restarted. Result unknown. No automatic retry.
设备状态已过期，请检查 USB 连接|Device status expired. Check the USB connection.
设备初始化失败|Device initialization failed
操作等待超时，结果未知；未自动重试|Operation timed out. Result unknown. No automatic retry.
USB 已断开，操作结果未知；未自动重试|USB disconnected. Result unknown. No automatic retry.
等待连接相机|Waiting for camera connection
蓝牙断开尚未完成，请稍后重试或重新插拔设备|Bluetooth is still disconnecting. Try again or reconnect the device.
已断开连接，配对信息已保留|Disconnected. Pairing retained.
等待相机响应超时，正在结束连接；未自动重试|Camera timed out. Disconnecting without retry.
相机已断开，请唤醒相机并点击连接|Camera disconnected. Wake the camera and select Connect Camera.
搜索佳能相机中，请保持相机处于无线遥控器配对画面…|Searching for Canon cameras. Keep the camera on the wireless remote pairing screen.
未找到佳能遥控服务。请在相机上打开“连接至无线遥控器 → 添加设备”，再重试。|Canon remote service not found. On the camera, select Connect to Wireless Remote → Add device, then retry.
找到相机，请选择要连接的相机|Camera found. Select a camera to connect.
更换相机前，请先点击“清除配对 / 更换相机”|Clear pairing before switching cameras
有未完成的配对记录，请先清除配对再换相机|Pairing is incomplete. Clear it before switching cameras.
无法保存配对准备信息，请重启设备后重试|Could not save pairing setup. Restart the device and retry.
蓝牙连接失败，请唤醒相机后重试；换相机前请先清除配对。|Bluetooth connection failed. Wake the camera and retry. Clear pairing before switching cameras.
蓝牙已接通，正在验证配对…|Bluetooth connected. Verifying pairing…
蓝牙加密握手未完成，请删除相机端旧遥控记录并重新配对。|Bluetooth security handshake failed. Remove the old remote from the camera and pair again.
正在注册 XIAO-Shutter 遥控器…|Registering the XIAO-Shutter remote…
相机缺少佳能遥控服务，请确认选择的是无线遥控器配对。|Canon remote service unavailable. Select wireless remote pairing on the camera.
相机的身份或快门服务不可用，当前协议可能不兼容。|Camera identity or shutter service unavailable. The protocol may be incompatible.
相机未接受遥控器身份，请重新进入相机的配对画面。|Camera rejected the remote. Reopen the camera pairing screen.
注册后相机断开了连接，请检查相机配对确认提示。|Camera disconnected after registration. Check the camera for a pairing prompt.
配对信息保存失败，请重启设备后重试。|Could not save pairing. Restart the device and retry.
相机已连接，可发送快门指令|Camera connected. Shutter ready.
相机未连接，未发送快门指令|Camera disconnected. Shutter command was not sent.
正在发送单次快门指令…|Sending a single shutter command…
快门释放未得到确认，已断开连接；请在相机上检查快门状态。|Shutter release unconfirmed. The camera was disconnected. Check the shutter on the camera.
快门写入未得到确认，已发送释放指令；请检查相机，勿自动重试。|Shutter command unconfirmed. Release was sent. Check the camera before retrying.
清除蓝牙配对记录失败，请重启后重试|Could not clear Bluetooth pairing. Restart and retry.
清除已保存相机信息失败|Could not clear saved camera information
本机配对已清除。若重配同一台相机，也请删除相机端的旧遥控记录。|Device pairing cleared. Remove the old remote from the camera before pairing it again.
没有可连接的相机，请先搜索或配对新相机|No camera available. Search for or pair a camera first.
无效 JSON 命令|Invalid JSON command
未知操作|Unknown operation
设备忙碌，请等待当前操作结束|Device busy. Wait for the current operation.
操作队列忙碌，请稍后重试|Command queue busy. Try again shortly.
初始化失败|Initialization failed
BLE任务创建失败|Could not start the Bluetooth task
命令过长|Command too long
移动中|Moving
已记录|Recorded
动作|Action
组内全部动作均有已保存轨迹，不跳过动作。|All actions are recorded.
已就绪|Ready
未就绪（本组无拍照点，不影响播放）|Not ready (no capture points in this group)
未就绪（此动作无拍照点，不影响播放）|Not ready (no capture points in this action)
`.trim().split("\n").map(line => line.split("|")));

  // Numeric fields and nested status messages retain their values in both languages.
  const patterns = [
    [/^拍摄组\s*(\d+)$/, n => `Group ${n}`],
    [/^动作\s*(\d+)$/, n => `Action ${n}`],
    [/^动作 (\d+) 名称$/, n => `Action ${n} name`],
    [/^删除动作 (\d+)$/, n => `Delete action ${n}`],
    [/^删除拍照点 (\d+)$/, n => `Delete capture point ${n}`],
    [/^(\d+) 动作$/, n => count(n, "action")],
    [/^(\d+) 个动作 · (\d+) 个已录制$/, (total, saved) => `${count(total, "action")} · ${saved} recorded`],
    [/^(\d+) 个拍照点(.*)$/, (n, suffix) => count(n, "capture point") + ({
      " · 空格记点并拍照": " · Space to mark and shoot", " · 仅记录位置": " · Record position only"
    }[suffix] || "")],
    [/^拍照点 (\d+) · ([\d.]+) 秒$/, (n, seconds) => `Capture point ${n} · ${seconds} s`],
    [/^已保存 · ([\d.]+) 秒 · (\d+) 点$/, (seconds, n) => `Saved · ${seconds} s · ${count(n, "sample")}`],
    [/^正在录制 · (\d+) 点$/, n => `Recording · ${count(n, "sample")}`],
    [/^已记录拍照点 (\d+)$/, n => `Capture point ${n} saved`],
    [/^播放已保存轨迹，拍摄 (\d+) 张照片$/, n => `Play the saved trajectory and take ${count(n, "photo")}`],
    [/^实时反馈 · 目标容差 ±([\d.]+)°$/, n => `Live feedback · Tolerance ±${n}°`],
    [/^(J\d+) 温度预警：([\d.]+)°C$/, (joint, value) => `${joint} temperature warning: ${value}°C`],
    [/^正在核对六轴 · 容差 ±([\d.]+)°$/, n => `Checking joint angles · Tolerance ±${n}°`],
    [/^六轴未全部进入 ±([\d.]+)° 容差，未触发拍照$/, n => `One or more joints were outside ±${n}°. Shutter not triggered.`],
    [/^开始按点回放？六轴进入 ±([\d.]+)° 容差后，按本组设置等待并拍照。最后一张完成后回 Home 并保持使能。请确认往返路线畅通、人员离开运动范围。$/, n => `Play capture points? Once all six joints are within ±${n}°, the arm waits as configured for this group, then takes the photo. After the final shot, it returns Home and stays enabled. Confirm both paths are clear and everyone is outside the workspace.`],
    [/^实时反馈 · Home 偏差小于 ([\d.]+)°$/, n => `Live feedback · Home error < ${n}°`],
    [/^已返回 Home · 等待 ([\d.]+) 秒$/, n => `At Home · Waiting ${n} s`],
    [/^实时反馈 · J2 ±([\d.]+)°，其余 ±([\d.]+)°$/, (j2, others) => `Live feedback · J2 ±${j2}°, others ±${others}°`],
    [/^目标 (.+)$/, value => `Target ${value}`],
    [/^偏差 (.+)$/, value => `Error ${value}`],
    [/^([\d.]+) 秒$/, n => `${n} s`],
    [/^固件 (.+)$/, version => `Firmware ${version}`],
    [/^原位保持诊断：([\d.]+)秒$/, n => `Position hold test: ${n} s`],
    [/^操作失败：(.+)。请核实设备状态后再操作。$/s, message => `Request failed: ${english(message)}. Check the device before retrying.`],
    [/^停止请求未确认：(.+)，请现场处理$/s, message => `Stop unconfirmed: ${english(message)}. Check the arm on site.`],
    [/^状态消息无法读取：(.+)$/s, message => `Unable to read status: ${english(message)}`],
    [/^(\/api\/[^：]+)：(.+)$/s, (path, message) => `${path}: ${english(message)}`],
    [/^请先连接蓝牙相机，再开始录制和打点。(.+)$/s, message => `Connect the Bluetooth camera before recording or marking points. ${english(message)}`],
    [/^删除「(.+)」及组内动作关联？此操作不可撤销。$/, name => `Delete “${displayName(name, true)}” and its action links? This cannot be undone.`],
    [/^(?:(.+) · )?将按顺序播放 (\d+) 个已保存动作，跳过 (\d+) 个空动作，共 (\d+) 张照片(；请先录制动作)?。$/, (hint, saved, empty, shots, missing) =>
      `${hint ? english(hint) + " · " : ""}Plays ${count(saved, "saved action")}, skips ${count(empty, "empty action")}, takes ${count(shots, "photo")}.${missing ? " Record an action first." : ""}`],
    [/^录制未保存：(.+)；(原有已保存轨迹仍保留，播放将使用原轨迹。|请调整姿态后重新录制。)$/s, (message, next) =>
      `Recording was not saved: ${english(message)}. ${next.startsWith("原有") ? "Playback will use the saved trajectory." : "Adjust the pose and record again."}`],
    [/^整组播放 · (\d+) \/ (\d+) · ([^·]+?)(?: · 剩余 (\d+) 秒)?$/, (index, total, phase, remaining) =>
      `Group playback · ${index} / ${total} · ${english(phase)}${remaining ? ` · ${remaining} s left` : ""}`],
    [/^拍照 ([\d—]+) \/ ([\d—]+) · ([^·]+?)(?: · (.+))?$/, (index, total, phase, detail) =>
      `Capture ${index} / ${total} · ${english(phase)}${detail ? " · " + english(detail) : ""}`],
    [/^ESP32 (在线|离线) · 蓝牙(已连接|未连接)(.*)$/, (usb, bluetooth, detail) =>
      `ESP32 ${usb === "在线" ? "online" : "offline"} · Bluetooth ${english(bluetooth).toLowerCase()}${detail.replace(/ · 处理中$/, " · Busy")}`],
    [/^回 Home 超时，六轴未在 ±([\d.]+)° 内连续保持 2 秒$/, tolerance => `Home timed out. The six joints did not hold within ±${tolerance}° for 2 s.`],
    [/^回 Home 超时，六轴偏差未能连续 2 秒小于 ([\d.]+)°$/, tolerance => `Home timed out. Joint errors did not stay below ${tolerance}° for 2 s.`],
    [/^回 Home 超时，六轴偏差未全部小于 ([\d.]+)°$/, tolerance => `Home timed out. One or more joint errors reached ${tolerance}° or more.`],
    [/^六轴未在 ±([\d.]+)° 内持续停稳2秒，未触发拍照$/, tolerance => `The six joints did not hold within ±${tolerance}° for 2 s. Shutter was not triggered.`],
    [/^相机桥连接失败：(.+)$/s, message => `Camera bridge connection failed: ${english(message)}`],
    [/^相机桥拒绝指令（(\d+)）$/, code => `Camera bridge rejected the command (HTTP ${code})`],
    [/^USB 未连接：(.+)$/s, message => `USB disconnected: ${english(message)}`],
    [/^正在连接 (.+)$/, name => `Connecting to ${name}`],
    [/^蓝牙认证失败（代码 (.+)），请检查相机配对画面。$/, code => `Bluetooth authentication failed (code ${code}). Check the camera pairing screen.`],
    [/^控制状态：(.+?) · 电机连接：(.+?) · 日志：(.+)$/, (mode, connection, log) =>
      `Control state: ${english(mode)} · Controller: ${english(connection)} · Log: ${english(log)}`],
    [/^负载配置：(.+?)( · 单动作按点拍照)? · 补偿比例：(.+) · 速度：(.+)$/, (profile, capture, ratio, speed) =>
      `Payload: ${english(profile)}${capture ? " · Capture at saved points" : ""} · Compensation: ${ratio} · Speed: ${speed}`],
    [/^按顺序播放「(.+)」的 (\d+) 个已保存动作？\n执行顺序：(.+)\n(.+)\n回放速度：([\d.]+ rad\/s|—)。本组将拍摄 (\d+) 张照片；相机状态：(.+)。\n动作间隔 ([\d.]+) 秒，完成后保持使能。$/,
      (name, actions, order, skip, speed, photos, camera, interval) =>
        `Play ${count(actions, "saved action")} in “${displayName(name, true)}”?\nOrder: ${order.split(" → ").map(english).join(" → ")}\n${english(skip)}\nSpeed: ${speed}. Captures: ${photos}. Camera: ${english(camera)}.\nInterval: ${interval} s. The arm stays enabled afterward.`],
    [/^(\d+\. )(.+?)(（本次录制失败，使用原有已保存轨迹）)?$/, (number, name, retained) =>
      number + displayName(name, true) + (retained ? " (using the previous recording)" : "")],
    [/^将跳过 (\d+) 个空动作：(.+)。$/, (n, names) =>
      `Skipping ${count(n, "empty action")}: ${names.split("、").map(name => displayName(name, true)).join(", ")}.`],
    [/^播放「(.+)」？机械臂将先移动到记录起点，再执行轨迹。\n回放速度：([\d.]+ rad\/s|—)。将拍摄 (\d+) 张照片；相机状态：([^\n]+)。([^]*)$/,
      (name, speed, photos, camera, retained) =>
        `Play “${displayName(name, true)}”? The arm will move to the recorded start and follow the trajectory.\nSpeed: ${speed}. Captures: ${photos}. Camera: ${english(camera)}.${retained ? "\nThe last recording was not saved. Playback will use the previous recording." : ""}`],
  ];

  const count = (n, singular) => `${n} ${singular}${Number(n) === 1 ? "" : "s"}`;

  function displayName(name, useEnglish) {
    return useEnglish ? String(name).replace(/^拍摄组\s*(\d+)$/, "Group $1").replace(/^动作\s*(\d+)$/, "Action $1") : name;
  }

  function english(value) {
    const text = value.trim();
    if (labels.has(text)) return labels.get(text);
    for (const [pattern, translate] of patterns) {
      const match = pattern.exec(text);
      if (match) return translate(...match.slice(1));
    }
    if (text.includes(" · ")) return text.split(" · ").map(english).join(" · ");
    return value;
  }

  return {english, displayName};
})();
