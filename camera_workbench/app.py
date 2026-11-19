"""Compatibility API for the original shooting UI; no legacy motor control code."""
import copy
import json
import logging
import math
import queue
import threading
import time
import uuid
from pathlib import Path
from gravity_console.shooting_groups import ShootingGroupStore, atomic_write_json
from gravity_console.camera_bridge import CameraBridgeClient
from .motor_temperature import STOP_C, WARNING_C

BACKEND = "new-rs-ros"
NAMES = [f"joint{i}" for i in range(1, 7)]
HOME_TOLERANCE_DEG = .5
PHOTO_POINT_TOLERANCE_DEG = .7
PHOTO_POINT_TOLERANCE_BY_JOINT_DEG = (PHOTO_POINT_TOLERANCE_DEG,) * len(NAMES)
MOTION_SPEED_RAD_S = 1.0
HOME_WAIT_S = 5.


def six(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("点位必须包含六轴角度")
    q = [float(x) for x in values]
    if not all(math.isfinite(x) for x in q):
        raise ValueError("点位包含无效角度")
    return q


class Workbench:
    def __init__(self, data_dir, home_path, backend, *, camera=None, launcher=None, temperature=None):
        self.data = Path(data_dir)
        self.data.mkdir(parents=True, exist_ok=True)
        self.home_path = Path(home_path)
        self.home = json.loads(self.home_path.read_text())
        self.home_q = six(self.home["q_rad"])
        self.backend, self.launcher = backend, launcher
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.cancel = threading.Event()
        self.epoch = 0
        self.worker = None
        self.recording = None
        self.record_started = 0.
        self.mark_lock = threading.Lock()
        self.last_mark = float("-inf")
        self.events, self.subscribers = [], []
        self.mode = "disabled"
        self.message = "旧工作台界面已恢复 · 新底层未启动"
        self.capture = self.playback = None
        self.revision = 0
        path = self.data / "teach_trajectories.json"
        self.trajectories = json.loads(path.read_text())["trajectories"] if path.exists() else []
        if not isinstance(self.trajectories, list):
            raise ValueError("动作记录文件格式无效")
        self.groups = ShootingGroupStore(self.data / "shooting_groups.json", self.trajectories)
        self.camera = camera or CameraBridgeClient(self.data / "photo_journal.jsonl")
        settings = self.data / "camera_settings.json"
        self.post_delay = float(json.loads(settings.read_text())["post_delay_s"]) if settings.exists() else 0.
        self.closed = threading.Event()
        self.temperature = temperature
        self.temperature_warnings = set()
        self.thermal_latch = self.data / "thermal_trip.json"
        self.thermal_cancel = threading.Event()
        self.thermal_thread = None
        self.thermal_trip = json.loads(self.thermal_latch.read_text()) if self.thermal_latch.exists() else None
        self.thermal_phase = "locked" if self.thermal_trip else None

    def permissions(self):
        p = self.data / "hardware_permission.json"
        return json.loads(p.read_text()) if p.exists() else {}

    def require_motion(self, *, manual=False):
        if self.thermal_trip:
            raise RuntimeError("温度保护已锁定，请先确认舵机冷却")
        permission = self.permissions()
        if permission.get("allow_motion") is not True:
            raise RuntimeError("界面已恢复；真机操作尚未开放，机械臂不会启动")
        if manual and permission.get("allow_manual_teach") is not True:
            raise RuntimeError("手动示教入口暂未开放：此前切换曾前倾，未改变控制模式")

    def compatible(self, trajectory):
        if trajectory.get("backend") != BACKEND:
            return False
        reference = trajectory.get("home_reference")
        return (reference == self.home["reference_id"]
                or reference in self.home.get("compatible_home_references", ()))

    def snapshot(self):
        with self.lock:
            hw = self.backend.snapshot()
            summaries, errors = [], {}
            for t in self.trajectories:
                points = [{k: p[k] for k in ("id", "sample_index", "t")} for p in t.get("photo_points", [])]
                summaries.append(dict(slot=t["slot"], label=t.get("label", "动作"),
                    sample_count=len(t.get("samples", [])), duration_s=t.get("duration_s", 0),
                    saved_ms=t.get("saved_ms", 0), camera_profile=t.get("camera_profile", "heavy"),
                    compatible=self.compatible(t), photo_points=points))
            mode = self.mode
            if not self.worker or not self.worker.is_alive():
                mode = "running" if self.recording else "holding" if hw["enabled"] else "disabled"
                if self.mode == "error":
                    mode = "error"
            if self.thermal_phase in ("stopping", "returning"):
                mode = "homing"
            temps = self.temperature.snapshot() if self.temperature else {}
            readings = [temps.get(axis, (None, False))[0] for axis in range(1, 7)]
            return dict(backend_kind=BACKEND, mode=mode, enabled=hw["enabled"], connected=hw["online"],
                drives_in_reset=bool(self.temperature and self.temperature.drives_in_reset()),
                dry_run=False, q_deg=[math.degrees(x) for x in hw["q_rad"]] if hw.get("q_rad") else [None]*6,
                home_q_deg=[math.degrees(x) for x in self.home_q],
                home_tolerance_deg=HOME_TOLERANCE_DEG, photo_point_tolerance_deg=PHOTO_POINT_TOLERANCE_DEG,
                photo_point_tolerance_by_joint_deg=list(PHOTO_POINT_TOLERANCE_BY_JOINT_DEG),
                motion_speed_rad_s=MOTION_SPEED_RAD_S,
                thermal=dict(temperature_c=readings, warning_axes=[i for i,v in enumerate(readings,1)
                             if v is not None and v >= WARNING_C],
                             trip=self.thermal_trip, phase=self.thermal_phase),
                home_reference=self.home["reference_id"], hardware_authorized=self.permissions().get("allow_motion", False),
                camera_profile="heavy", rated_camera_kg=.95,
                camera_profiles={"heavy": {"label": "相机负载 · 950 g", "total_camera_kg": .95,
                    "note": "六轴相机配置 · 新 RS 底层"}},
                joint_percent=[100]*6, camera={**self.camera.snapshot(), "post_delay_s": self.post_delay},
                shooting_groups=self.groups.snapshot(summaries), trajectories=summaries, recording_errors=errors,
                recording_slot=self.recording["slot"] if self.recording else None,
                recording_samples=len(self.recording["samples"]) if self.recording else 0,
                recording_photo_points=copy.deepcopy(self.recording["photo_points"]) if self.recording else [],
                group_playback=copy.deepcopy(self.playback), photo_capture=copy.deepcopy(self.capture),
                presets=[], message=self.message, updated_ms=int(time.time()*1000), revision=self.revision,
                log_path=None)

    def notify(self, message=None, level="info", *, update_message=True):
        with self.lock:
            if message:
                if update_message:
                    self.message = message
                event = dict(message=message, level=level, ts=time.strftime("%H:%M:%S"))
                self.events.append(event)
                self.events = self.events[-100:]
                for q in list(self.subscribers):
                    try:
                        q.put_nowait(dict(type="event", event=event))
                    except queue.Full:
                        pass
            self.revision += 1
        state = dict(type="status", status=self.snapshot())
        for q in list(self.subscribers):
            try:
                q.put_nowait(state)
            except queue.Full:
                pass

    def start_monitor(self):
        self.camera.start()
        if self.temperature:
            self.temperature.start()
        def monitor():
            seq = -1
            while not self.closed.wait(.1):
                hw = self.backend.snapshot()
                with self.lock:
                    rec = self.recording
                    if rec and hw["enabled"] and hw.get("q_rad") and hw["seq"] != seq:
                        seq = hw["seq"]
                        rec["samples"].append(dict(t=time.monotonic()-self.record_started, q_rad=hw["q_rad"]))
                self.check_temperature()
                self.notify()
        threading.Thread(target=monitor, daemon=True).start()

    def close(self):
        self.closed.set()
        self.cancel.set()
        self.thermal_cancel.set()
        if self.temperature:
            self.temperature.close()
        self.camera.cancel_photo()
        if self.worker and self.worker.is_alive():
            self.backend.stop()
            self.worker.join(timeout=2.)
        if self.thermal_thread:
            self.thermal_thread.join(timeout=2.)
        self.camera.close()
        self.backend.close()

    def check_temperature(self):
        if not self.temperature:
            return
        readings = self.temperature.snapshot()
        warnings = []
        with self.lock:
            for axis, (value, _) in readings.items():
                if value < WARNING_C - 2:
                    self.temperature_warnings.discard(axis)
                elif value >= WARNING_C and axis not in self.temperature_warnings:
                    self.temperature_warnings.add(axis)
                    warnings.append((axis, value))
            if readings and not self.thermal_trip:
                axis, (value, _) = max(readings.items(), key=lambda item: item[1][0])
                if value >= STOP_C:
                    self.thermal_trip = dict(axis=axis, temperature_c=value)
                    self.thermal_phase = "stopping"
                    self.thermal_cancel = threading.Event()
                    self.epoch += 1
                    self.cancel.set()
                    self.thermal_thread = threading.Thread(
                        target=self.thermal_shutdown, name="thermal-shutdown", daemon=True
                    )
                    self.thermal_thread.start()
        for axis, value in warnings:
            self.notify(f"J{axis} 温度预警：{value:.1f}°C", "warning", update_message=False)

    def thermal_shutdown(self):
        cancel = self.thermal_cancel
        try:
            self.camera.cancel_photo()
            try:
                atomic_write_json(self.thermal_latch, self.thermal_trip)
            except OSError:
                self.notify("温度保护状态未能保存", "error", update_message=False)
            self.notify("舵机达到 80°C · 已取消当前任务", "error", update_message=False)
            with self.command_lock:
                self.backend.stop()
                if self.worker and self.worker.is_alive():
                    self.worker.join(timeout=5.)
                    if self.worker.is_alive():
                        raise RuntimeError("previous operation still running")
                with self.lock:
                    self.recording = self.playback = self.capture = None
                hw = self.backend.snapshot()
                if not hw["online"] or hw.get("errors") or any(fault for _,fault in self.temperature.snapshot().values()):
                    raise RuntimeError("controller feedback or motor fault prevents Home")
                if hw["enabled"]:
                    self.thermal_phase = "returning"
            if hw["enabled"]:
                self.home_move(cancel, wait_at_home=False)
                if cancel.is_set():
                    raise RuntimeError("thermal return cancelled")
                self.backend.trigger("disable")
                deadline = time.monotonic() + 2.
                while True:
                    hw = self.backend.snapshot()
                    if hw["online"] and not hw["enabled"]:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("disable not confirmed")
                    self.wait(.05, cancel)
                self.thermal_phase = "disabled"
                self.notify("温度保护 · 已返回 Home 并失能", "error")
            else:
                self.thermal_phase = "locked"
                self.notify("温度保护 · 机械臂已失能", "error")
        except Exception:
            logging.exception("Temperature-triggered return could not be verified")
            self.thermal_phase = "failed"
            self.notify("温度保护 · 无法确认回 Home 并失能，请现场处理", "error")

    def begin(self, mode, function):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError("上一操作尚未结束")
            self.cancel = threading.Event()
            cancel, epoch = self.cancel, self.epoch
            self.mode = mode
            def run():
                try:
                    function(cancel)
                    with self.lock:
                        if epoch == self.epoch:
                            self.mode = "holding" if self.backend.snapshot()["enabled"] else "disabled"
                except Exception as exc:
                    with self.lock:
                        if epoch == self.epoch:
                            self.mode = "error"
                            self.notify(str(exc), "error")
                finally:
                    self.notify()
            self.worker = threading.Thread(target=run, daemon=True)
            self.worker.start()

    def wait(self, seconds, cancel):
        if cancel.wait(seconds):
            raise RuntimeError("操作已取消")

    def wait_for_position(self, target, cancel, timeout=8., *, home=False, hold_s=0.):
        started = time.monotonic()
        deadline = started + timeout + hold_s
        in_tolerance_since = None
        seq = self.backend.snapshot()["seq"]
        samples = []
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise RuntimeError("操作已取消")
            seq, q = self.backend.sample(after=seq, timeout=.5)
            now = time.monotonic()
            errors = [math.degrees(a-b) for a,b in zip(q,target)]
            if not home:
                samples.append(dict(elapsed_s=now-started, feedback_seq=seq, error_deg=errors))
            if home:
                good = max(map(abs, errors)) < HOME_TOLERANCE_DEG
            else:
                good = all(abs(error) <= limit for error, limit in zip(errors, PHOTO_POINT_TOLERANCE_BY_JOINT_DEG))
            if good:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                if now-in_tolerance_since >= hold_s:
                    return seq, q, time.time(), samples, in_tolerance_since-started
            else:
                in_tolerance_since = None
            self.wait(.05, cancel)
        if home:
            raise RuntimeError(f"回 Home 超时，六轴偏差未全部小于 {HOME_TOLERANCE_DEG:.1f}°")
        raise RuntimeError(f"六轴未全部进入 ±{PHOTO_POINT_TOLERANCE_DEG:.1f}° 容差，未触发拍照")

    def home_move(self, cancel, *, wait_at_home=True):
        self.notify("机械臂正在返回 Home")
        self.backend.move(self.home_q, cancel, speed=MOTION_SPEED_RAD_S, replay=True)
        self.wait_for_position(self.home_q, cancel, home=True)
        self.capture = None
        if wait_at_home:
            self.notify(f"已返回 Home · 等待 {HOME_WAIT_S:g} 秒")
            self.wait(HOME_WAIT_S, cancel)
        self.notify("已返回 Home · 保持使能")

    def photo(self, key, cancel):
        if cancel.is_set():
            raise RuntimeError("拍照已取消")
        op = self.camera.start_shutter(key, cancel_event=cancel)
        deadline = time.monotonic()+12
        while not op.done.wait(.05):
            if cancel.is_set() or time.monotonic() > deadline:
                op.cancel()
                raise RuntimeError("快门结果未确认，请检查相机，不会自动重拍")
        if not op.snapshot()["ok"]:
            raise RuntimeError(op.snapshot()["message"])

    def stop(self):
        with self.lock:
            self.epoch += 1
            self.cancel.set()
            self.thermal_cancel.set()
        self.camera.cancel_photo()
        self.backend.stop()
        with self.lock:
            if self.recording:
                self.save_recording()
            self.mode = "holding" if self.backend.snapshot()["enabled"] else "disabled"
            self.playback = None
        self.notify("操作已停止 · 请核实机械臂保持状态")

    def save_recording(self):
        rec = self.recording
        if not rec:
            return
        if not rec["samples"]:
            raise RuntimeError("未取得有效采样，原动作未覆盖")
        rec["duration_s"] = rec["samples"][-1]["t"]
        rec["saved_ms"] = int(time.time()*1000)
        updated = [t for t in self.trajectories if t["slot"] != rec["slot"]]+[rec]
        atomic_write_json(self.data / "teach_trajectories.json", dict(trajectories=updated))
        self.trajectories, self.recording = updated, None

    def trajectory(self, block):
        t = next((t for t in self.trajectories if t["slot"] == block["slot"]), None)
        if not t or not self.compatible(t):
            raise RuntimeError("该动作属于旧坐标或尚未录制，请在新基准下重新录制")
        if not t.get("photo_points"):
            raise RuntimeError("动作没有拍照点，请录制并按空格设置点位")
        # Full preflight before sending the first movement.
        for p in t["photo_points"]:
            six(p["q_rad"])
        return copy.deepcopy(t)

    def play(self, trajectories, interval, cancel, *, pre_shutter_delay_s=0.):
        self.backend.trigger("gravity_compensation/stop")
        for index, t in enumerate(trajectories):
            self.playback = dict(index=index+1, total=len(trajectories), phase="playing") if len(trajectories)>1 else None
            for p in t["photo_points"]:
                q = six(p["q_rad"])
                self.capture = dict(phase="moving", precision_reference_deg=[math.degrees(x) for x in q])
                self.notify("机械臂正在前往拍摄点")
                self.backend.move(q, cancel, speed=MOTION_SPEED_RAD_S, replay=True)
                self.capture["phase"] = "settling"
                self.notify(f"正在核对六轴 · 容差 ±{PHOTO_POINT_TOLERANCE_DEG:.1f}°")
                seq, actual, sampled_at, samples, in_tolerance_at = self.wait_for_position(
                    q, cancel, hold_s=pre_shutter_delay_s
                )
                key = f"replay:{uuid.uuid4()}:{p['id']}"
                if cancel.is_set():
                    raise RuntimeError("拍照已取消")
                record = dict(
                    key=key, point_id=p["id"], slot=t["slot"], label=t.get("label"),
                    home_reference=self.home["reference_id"], feedback_seq=seq,
                    sampled_at=sampled_at,
                    target_deg=[math.degrees(a-b) for a,b in zip(q,self.home_q)],
                    actual_deg=[math.degrees(a-b) for a,b in zip(actual,self.home_q)],
                    error_deg=[math.degrees(a-b) for a,b in zip(actual,q)],
                    tolerance_deg=list(PHOTO_POINT_TOLERANCE_BY_JOINT_DEG),
                    settle_samples=samples, stable_window_start_s=in_tolerance_at,
                    pre_shutter_delay_s=pre_shutter_delay_s,
                )
                with (self.data / "replay_shutter_angles.jsonl").open("a", encoding="utf-8") as log:
                    log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                self.capture["phase"] = "triggering"
                self.notify("已到达拍摄点 · 正在触发快门")
                self.photo(key, cancel)
                if self.post_delay > 0:
                    self.capture["phase"] = "post_delay"
                    self.notify("拍照完成 · 正在保持位置")
                    self.wait(self.post_delay, cancel)
                self.capture["phase"] = "complete"
            if index+1 < len(trajectories):
                self.mode = "group_waiting"
                self.playback["phase"] = "waiting"
                self.wait(interval, cancel)
                self.mode = "replaying"
            if self.playback is not None:
                self.playback["phase"] = "returning_home"
            self.home_move(cancel)
        self.playback = None

    def command(self, path, data):
        if self.thermal_trip and path in ("/api/home-hold", "/api/home-disable"):
            raise RuntimeError("温度保护已锁定，请先确认舵机冷却")
        urgent = path in ("/api/stop", "/api/home-hold", "/api/home-disable", "/api/emergency-disable")
        if urgent:
            try:
                self.stop()  # Cancellation precedes any command lock or pending camera RPC.
            except Exception:
                if path != "/api/emergency-disable" or not self.thermal_trip:
                    raise
            if path == "/api/stop":
                return self.snapshot()
        generation = self.epoch
        with self.command_lock:
            if not urgent and generation != self.epoch:
                raise RuntimeError("请求已被停止操作取消")
            if urgent and self.worker and self.worker.is_alive():
                self.worker.join(timeout=3.)
            return self._command(path, data)

    def _command(self, path, data):
        if path == "/api/thermal/reset":
            if data.get("confirmed_cooled") is not True:
                raise RuntimeError("请先确认所有舵机已冷却到 70°C 以下")
            with self.lock:
                temps = self.temperature.snapshot() if self.temperature else {}
                if (self.thermal_thread and self.thermal_thread.is_alive()) or self.backend.snapshot()["enabled"]:
                    raise RuntimeError("温度保护仍在处理，请先确认机械臂已失能")
                if any(value >= WARNING_C or fault for value, fault in temps.values()):
                    raise RuntimeError("请先确认所有舵机已冷却到 70°C 以下")
                self.thermal_latch.unlink(missing_ok=True)
                self.thermal_trip = self.thermal_phase = None
                self.temperature_warnings.clear()
                self.mode = "disabled"
            self.notify("温度保护已解除 · 等待启动机械臂")
            return self.snapshot()
        if path.startswith("/api/groups/") or path in ("/api/blocks/add", "/api/blocks/update", "/api/blocks/delete"):
            with self.lock:
                if self.recording or (self.worker and self.worker.is_alive()):
                    raise RuntimeError("录制或执行中不能编辑分组")
                g, b = data.get("group_id"), data.get("block_id")
                operations = {
                    "/api/groups/add": lambda: self.groups.add_group(),
                    "/api/groups/update": lambda: self.groups.update_group(
                        g, data.get("name"), data.get("interval_s"),
                        pre_shutter_delay_s=data.get("pre_shutter_delay_s"),
                    ),
                    "/api/groups/delete": lambda: self.groups.delete_group(g),
                    "/api/blocks/add": lambda: self.groups.add_block(g),
                    "/api/blocks/update": lambda: self.groups.update_block(g,b,data["name"]),
                    "/api/blocks/delete": lambda: self.groups.delete_block(g,b)}
                if path in operations:
                    operations[path]()
                    self.notify("拍摄组设置已保存")
                    return self.snapshot()
        if path.startswith("/api/camera/"):
            action = path.rsplit("/",1)[1]
            if action == "settings":
                delay = float(data["post_delay_s"])
                if not math.isfinite(delay) or not 0 <= delay <= 10:
                    raise ValueError("拍照停留须为0–10秒")
                atomic_write_json(self.data/"camera_settings.json",dict(version=1,post_delay_s=delay))
                self.post_delay = delay
            else:
                self.camera.command(action)
            self.notify("相机操作已提交")
            return self.snapshot()
        if path == "/api/blocks/photo-points/delete":
            with self.lock:
                block = self.groups.resolve_block(data["group_id"],data["block_id"])
                t = self.recording if self.recording and self.recording["slot"]==block["slot"] else next((t for t in self.trajectories if t["slot"]==block["slot"]),None)
                if not t:
                    raise ValueError("动作未录制")
                t["photo_points"] = [p for p in t.get("photo_points",[]) if p["id"]!=data["point_id"]]
                if t is not self.recording:
                    atomic_write_json(self.data/"teach_trajectories.json",dict(trajectories=self.trajectories))
            self.notify("拍照点已删除")
            return self.snapshot()
        if path == "/api/stop":
            self.stop()
            return self.snapshot()
        if path == "/api/emergency-disable":
            self.backend.trigger("emergency_disable")
            self.notify("紧急失能已提交，请现场确认")
            return self.snapshot()
        self.require_motion(manual=path=="/api/blocks/record")
        if path == "/api/start":
            self.notify("正在启动新六轴底层")
            def start(cancel):
                if not self.backend.snapshot()["online"]:
                    if not self.launcher:
                        raise RuntimeError("新底层启动入口未配置")
                    self.launcher()
                    deadline=time.monotonic()+20
                    while not self.backend.snapshot()["online"]:
                        if time.monotonic()>deadline:
                            raise RuntimeError("底层启动结果未确认，请核实状态，不会自动重试")
                        self.wait(.1,cancel)
                if cancel.is_set():
                    raise RuntimeError("启动已取消，请核实设备状态")
                if not self.backend.snapshot()["enabled"]:
                    self.backend.trigger("enable")
                self.home_move(cancel)
            self.begin("starting",start)
        elif path in ("/api/home-hold", "/api/home-disable"):
            def home(cancel):
                self.home_move(cancel)
                if path.endswith("disable"):
                    self.backend.trigger("disable")
                    self.notify("已返回 Home · 已失能")
            self.begin("homing",home)
        elif path == "/api/blocks/record":
            if self.recording or (self.worker and self.worker.is_alive()):
                raise RuntimeError("请先停止当前录制或动作")
            if not self.backend.snapshot()["enabled"] or not self.camera.snapshot()["ready"]:
                raise RuntimeError("请先启动机械臂并连接相机")
            with self.lock:
                block = self.groups.resolve_block(data["group_id"],data["block_id"])
                slot = block["slot"]
                if slot is None:
                    used = {t["slot"] for t in self.trajectories}
                    used.update(b["slot"] for g in self.groups.groups for b in g["blocks"] if b["slot"])
                    slot = next(i for i in range(1,401) if i not in used)
                    self.groups.assign_slot(data["group_id"],data["block_id"],slot)
            epoch = self.epoch
            self.backend.trigger("gravity_compensation/start")
            with self.lock:
                if epoch != self.epoch:
                    self.backend.stop()
                    raise RuntimeError("录制启动已取消，已请求恢复保持")
                self.cancel = threading.Event()
                self.recording = dict(slot=slot,label=block["name"],backend=BACKEND,
                    home_reference=self.home["reference_id"],camera_profile="heavy",samples=[],photo_points=[])
                self.record_started = time.monotonic()
            self.mode = "running"
            self.notify("正在录制 · 请手动摆位，空格记录六轴点位")
        elif path == "/api/blocks/stop-recording":
            self.backend.trigger("gravity_compensation/stop")
            with self.lock:
                self.save_recording()
            self.notify("录制已保存 · 保持当前位置")
        elif path == "/api/blocks/photo-points":
            with self.mark_lock:
                if not self.recording:
                    raise RuntimeError("请先开始录制")
                if time.monotonic()-self.last_mark < .25:
                    raise RuntimeError("请松开空格后再按")
                rec, epoch = self.recording,self.epoch
                block = self.groups.resolve_block(data["group_id"],data["block_id"])
                if rec["slot"]!=block["slot"]:
                    raise RuntimeError("只能对正在录制的动作打点")
                self.last_mark = time.monotonic()
                seq=self.backend.snapshot()["seq"]
                _,q=self.backend.sample(after=seq)
                with self.lock:
                    if rec is not self.recording or epoch != self.epoch:
                        raise RuntimeError("打点已取消")
                    t=time.monotonic()-self.record_started
                    rec["samples"].append(dict(t=t,q_rad=q))
                    p=dict(id=str(uuid.uuid4()),sample_index=len(rec["samples"])-1,t=t,q_rad=q)
                    rec["photo_points"].append(p)
                    self.capture = dict(phase="recorded", precision_reference_deg=[math.degrees(x) for x in q])
                self.notify("点位已记录 · 等待单次快门")
                self.photo(f"teach:{rec['slot']}:{p['id']}",self.cancel)
                self.notify("点位已记录 · 快门指令已完成")
        elif path in ("/api/blocks/play", "/api/groups/play"):
            if self.recording:
                raise RuntimeError("请先停止录制")
            if not self.backend.snapshot()["enabled"] or not self.camera.snapshot()["ready"]:
                raise RuntimeError("请先启动机械臂并连接相机")
            g = self.groups.resolve_group(data["group_id"])
            if path=="/api/blocks/play":
                blocks=[self.groups.resolve_block(data["group_id"],data["block_id"])]
                interval=0
            else:
                blocks=[b for b in g["blocks"] if b["slot"] is not None]
                interval=g["interval_s"]
            trajectories=[self.trajectory(b) for b in blocks]
            if not trajectories:
                raise RuntimeError("没有已录制的新点位")
            pre_shutter_delay = g.get("pre_shutter_delay_s", 0.)
            self.begin("replaying", lambda cancel: self.play(
                trajectories, interval, cancel, pre_shutter_delay_s=pre_shutter_delay,
            ))
        else:
            raise ValueError("此旧诊断接口未接入；工作台不会启动旧控制程序")
        self.notify()
        return self.snapshot()
