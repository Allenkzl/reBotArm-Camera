import copy
import json
import math
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from .app import BACKEND, Workbench
from .ros_backend import RosBackend
from .motor_temperature import FRAME, decode_temperature


class FakeBackend:
    def __init__(self):
        self.enabled = False
        self.q = [.1]*6
        self.seq = 1
        self.calls = []
        self.moves = []
        self.move_speeds = []
        self.replay_moves = []
        self.block_sample = None
    def snapshot(self):
        return dict(online=True, enabled=self.enabled, q_rad=list(self.q),
                    reference_rad=list(self.q), state="IDLE", errors=[], seq=self.seq)
    def sample(self, after=None, timeout=2):
        if self.block_sample:
            self.block_sample.wait(2)
        self.seq += 1
        return self.seq, list(self.q)
    def trigger(self, name, timeout=8):
        self.calls.append(name)
        if name=="enable":
            self.enabled=True
        elif name in ("disable","emergency_disable"):
            self.enabled=False
        return "ok"
    def move(self, q, cancel, speed=.075, *, replay=False):
        if cancel.is_set():
            raise RuntimeError("cancelled")
        self.moves.append(list(q))
        self.move_speeds.append(speed)
        self.replay_moves.append(replay)
        self.q=list(q)
    def stop(self):
        self.calls.append("stop")
        if self.block_sample:
            self.block_sample.set()
    def close(self):
        self.calls.append("close")


class FakeCamera:
    def __init__(self):
        self.shots=[]
        self.commands=[]
    def snapshot(self):
        return dict(ready=True,connected=True,device_online=True,bridge_online=True,busy=False)
    def start_shutter(self,key,cancel_event=None):
        if cancel_event and cancel_event.is_set():
            raise RuntimeError("cancelled")
        self.shots.append(key)
        done=threading.Event();done.set()
        return SimpleNamespace(done=done,snapshot=lambda:dict(ok=True),cancel=lambda:None)
    def cancel_photo(self):
        pass
    def command(self,action):
        self.commands.append(action)
    def start(self):
        pass
    def close(self):
        pass


class WorkbenchTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        self.home=self.root/"home.json"
        self.home.write_text(json.dumps(dict(reference_id="current-home",q_rad=[.1]*6)))
        self.data=self.root/"data";self.data.mkdir()
        self.legacy=dict(slot=1,camera_profile="heavy",samples=[dict(t=0,q_rad=[9.]*6)],
                         photo_points=[dict(id="old",sample_index=0,t=0,q_rad=[9.]*6)])
        (self.data/"teach_trajectories.json").write_text(json.dumps(dict(trajectories=[self.legacy])))
        self.backend,self.camera=FakeBackend(),FakeCamera()
        self.app=Workbench(self.data,self.home,self.backend,camera=self.camera)
        self.g=self.app.groups.groups[0]
    def tearDown(self):
        self.app.close();self.tmp.cleanup()
    def permit(self,manual=False):
        (self.data/"hardware_permission.json").write_text(json.dumps(dict(allow_motion=True,allow_manual_teach=manual)))
    def ids(self,index=0):
        return dict(group_id=self.g["id"],block_id=self.g["blocks"][index]["id"])
    def record(self):
        self.permit(True);self.backend.enabled=True
        self.app.command("/api/blocks/record",self.ids(1))
    def test_constructor_and_repeated_status_never_enable_or_shoot(self):
        for _ in range(5):
            s=self.app.snapshot()
            self.assertEqual(s["backend_kind"],BACKEND)
            self.assertEqual(s["home_reference"],"current-home")
            self.assertFalse(s["hardware_authorized"])
        self.assertEqual(self.backend.calls,[])
        self.assertEqual(self.camera.shots,[])
    def test_unapproved_motion_rejected_before_transport(self):
        for path in ("/api/start","/api/blocks/record","/api/blocks/play","/api/home-hold"):
            with self.assertRaises(RuntimeError):
                self.app.command(path,self.ids())
        self.assertFalse(any(x in self.backend.calls for x in ("enable","disable","gravity_compensation/start")))
        self.assertEqual(self.backend.moves,[])
    def test_group_edits_persist_without_motor_access(self):
        self.assertEqual(self.app.snapshot()["shooting_groups"][0]["pre_shutter_delay_s"], 0.)
        self.app.command("/api/groups/update",dict(group_id=self.g["id"],name="我的拍摄组",pre_shutter_delay_s=1.5))
        stored=json.loads((self.data/"shooting_groups.json").read_text())
        self.assertEqual(stored["groups"][0]["name"],"我的拍摄组")
        self.assertEqual(stored["groups"][0]["pre_shutter_delay_s"],1.5)
        with self.assertRaises(ValueError):
            self.app.command("/api/groups/update",dict(group_id=self.g["id"],pre_shutter_delay_s=61))
        self.app.command("/api/blocks/add",dict(group_id=self.g["id"]))
        self.assertEqual(len(self.app.groups.groups[0]["blocks"]),5)
        self.assertEqual(self.backend.calls,[])
    def test_historical_coordinates_cannot_be_replayed(self):
        self.permit();self.backend.enabled=True
        with self.assertRaisesRegex(RuntimeError,"旧坐标"):
            self.app.command("/api/blocks/play",self.ids())
        self.assertEqual(self.backend.moves,[])
        self.assertEqual(self.camera.shots,[])
        self.assertEqual(self.app.trajectories,[self.legacy])
    def test_manual_mode_has_separate_gate(self):
        self.permit(False);self.backend.enabled=True
        with self.assertRaisesRegex(RuntimeError,"手动示教"):
            self.app.command("/api/blocks/record",self.ids(1))
        self.assertEqual(self.backend.calls,[])
    def test_space_point_uses_new_sample_and_saves_new_home_binding(self):
        self.record()
        self.backend.q=[.2,.3,.4,.5,.6,.7]
        self.app.command("/api/blocks/photo-points",self.ids(1))
        self.app.command("/api/blocks/stop-recording",self.ids(1))
        new=next(t for t in self.app.trajectories if t["slot"]!=1)
        self.assertEqual(new["photo_points"][0]["q_rad"],self.backend.q)
        self.assertEqual(new["home_reference"],"current-home")
        self.assertEqual(new["backend"],BACKEND)
        self.assertEqual(len(self.camera.shots),1)
        self.assertEqual(self.app.trajectories[0],self.legacy)
    def test_stop_bypasses_pending_point_request_and_prevents_shutter(self):
        self.record();self.backend.block_sample=threading.Event()
        entered=threading.Event();original=self.backend.sample
        def blocked(*a,**kw):
            entered.set();return original(*a,**kw)
        self.backend.sample=blocked
        errors=[]
        def mark():
            try:self.app.command("/api/blocks/photo-points",self.ids(1))
            except RuntimeError as exc:errors.append(str(exc))
        thread=threading.Thread(target=mark);thread.start();self.assertTrue(entered.wait(1))
        # Provide an earlier valid sample so stopping can preserve the recording.
        self.app.recording["samples"].append(dict(t=0,q_rad=[.1]*6))
        self.app.command("/api/stop",{})
        thread.join(1)
        self.assertFalse(thread.is_alive());self.assertTrue(errors)
        self.assertEqual(self.camera.shots,[])
    def test_inaccurate_point_never_triggers_camera(self):
        t=dict(slot=2,photo_points=[dict(id="point",q_rad=[.2]*6)])
        with patch.object(self.app,"wait_for_position",side_effect=RuntimeError("未到位")):
            with self.assertRaises(RuntimeError):self.app.play([t],0,threading.Event())
        self.assertEqual(self.camera.shots,[])
    def test_home_strict_half_degree_is_independent_of_photo_tolerance(self):
        now = [0.]

        def advance(seconds, cancel):
            now[0] += seconds

        cancel = threading.Event()
        with patch("camera_workbench.app.time.monotonic", side_effect=lambda: now[0]), \
             patch.object(self.app, "wait", side_effect=advance), patch.object(self.backend, "move") as move:
            for axis in (1, 2):
                for error in (-.5, .5, .55):
                    self.backend.q = [0.]*6
                    self.backend.q[axis] = math.radians(error)
                    with self.assertRaisesRegex(RuntimeError, "小于 0.5°"):
                        self.app.wait_for_position([0.]*6, cancel, timeout=.2, home=True)
            self.backend.q = list(self.app.home_q)
            self.backend.q[2] += math.radians(.49)
            before = now[0]
            self.app.home_move(cancel)
            move.assert_called_once_with(self.app.home_q, cancel, speed=1.0, replay=True)
            self.assertEqual(now[0]-before, 5.)
            for axis in range(6):
                self.backend.q = list(self.app.home_q)
                self.backend.q[axis] -= math.radians(.69)
                _, _, _, samples, _ = self.app.wait_for_position(self.app.home_q, cancel)
                self.assertAlmostEqual(samples[-1]["error_deg"][axis], -.69)
                self.backend.q[axis] = self.app.home_q[axis] + math.radians(.71)
                with self.assertRaisesRegex(RuntimeError, "±0.7°"):
                    self.app.wait_for_position(self.app.home_q, cancel, timeout=.2)
    def test_play_uses_marked_angles_not_recorded_path_then_returns_home(self):
        t=dict(slot=2,samples=[dict(q_rad=[9.]*6)],photo_points=[dict(id="point",q_rad=[.2]*6)])
        actual = [.2 + math.radians(delta) for delta in (0., .25, -.4, .1, 0., -.2)]
        samples = [dict(elapsed_s=2., feedback_seq=42, error_deg=[0., .25, -.4, .1, 0., -.2])]
        with patch.object(self.app,"wait_for_position",side_effect=[(42, actual, 1234., samples, 0.), None]), \
             patch.object(self.app,"wait") as wait:
            self.app.play([t],0,threading.Event())
        wait.assert_called_once_with(5., unittest.mock.ANY)
        self.assertEqual(self.backend.moves,[[.2]*6,[.1]*6])
        self.assertEqual(self.backend.move_speeds, [1.0, 1.0])
        self.assertEqual(self.backend.replay_moves, [True, True])
        self.assertEqual(len(self.camera.shots),1)
        entry = json.loads((self.app.data / "replay_shutter_angles.jsonl").read_text())
        self.assertEqual(entry["key"], self.camera.shots[0])
        self.assertEqual(entry["feedback_seq"], 42)
        self.assertEqual(entry["sampled_at"], 1234.)
        self.assertEqual(entry["settle_samples"], samples)
        self.assertEqual(entry["stable_window_start_s"], 0.)
        self.assertEqual(entry["slot"], 2)
        self.assertEqual(entry["point_id"], "point")
        self.assertEqual([round(x, 4) for x in entry["error_deg"]], [0., .25, -.4, .1, 0., -.2])
        self.assertAlmostEqual(entry["actual_deg"][2], entry["target_deg"][2] - .4)

    def test_settle_log_preserves_out_of_tolerance_sample_before_stable_window(self):
        now = [0.]
        target = [0.]*6

        def advance(seconds, cancel):
            now[0] += seconds

        def sample(after=None, timeout=.5):
            error = .75 if now[0] < .2 else -.2
            return (after or 0) + 1, [0., 0., math.radians(error), 0., 0., 0.]

        with patch("camera_workbench.app.time.monotonic", side_effect=lambda: now[0]), \
             patch.object(self.app, "wait", side_effect=advance), \
             patch.object(self.backend, "sample", side_effect=sample):
            seq, _, _, samples, stable_start = self.app.wait_for_position(target, threading.Event())

        self.assertAlmostEqual(samples[0]["error_deg"][2], .75)
        self.assertAlmostEqual(stable_start, .2)
        self.assertAlmostEqual(samples[-1]["elapsed_s"], stable_start)
        self.assertEqual(samples[-1]["feedback_seq"], seq)
        self.assertAlmostEqual(samples[-1]["error_deg"][2], -.2)

    def test_group_replay_returns_home_and_waits_after_every_action(self):
        self.permit()
        self.backend.enabled = True
        for slots, expected in (
            ([1, 2, 3, 4], [1, 0, 2, 0, 3, 0, 4, 0]),
            ([None, 2, 3, 4], [2, 0, 3, 0, 4, 0]),
            ([1, 2, None, None], [1, 0, 2, 0]),
        ):
            with self.subTest(slots=slots):
                self.backend.moves.clear()
                self.camera.shots.clear()
                self.app.trajectories = []
                self.g["pre_shutter_delay_s"] = .7
                for block, slot in zip(self.g["blocks"], slots):
                    block["slot"] = slot
                    if slot is not None:
                        self.app.trajectories.append(dict(
                            slot=slot, backend=BACKEND, home_reference="current-home",
                            photo_points=[dict(id=f"point-{slot}", t=0, sample_index=0,
                                               q_rad=[float(slot)]*6)],
                        ))

                waits_before_shutter = []
                waits_at_home = []
                self.g["interval_s"] = 1.

                def steady(target, cancel, *, home=False, hold_s=0.):
                    waits_before_shutter.append(hold_s)
                    return 1, target, 1234., [], 0.

                def wait(seconds, cancel):
                    waits_at_home.append((seconds, self.backend.moves[-1] == self.app.home_q))

                with patch.object(self.app, "wait_for_position", side_effect=steady), \
                     patch.object(self.app, "wait", side_effect=wait):
                    self.app.command("/api/groups/play", self.ids())
                    self.app.worker.join(2)

                self.assertEqual(self.app.mode, "holding")
                self.assertEqual(self.backend.moves,
                                 [[float(n)]*6 if n else self.app.home_q for n in expected])
                self.assertEqual(len(self.camera.shots), sum(n != 0 for n in expected))
                self.assertEqual(waits_before_shutter, [.7 if n else 0. for n in expected])
                self.assertEqual(waits_at_home, [(1., False), (5., True)] * (len(expected)//2-1) + [(5., True)])

    def test_pre_shutter_wait_restarts_if_position_leaves_tolerance(self):
        now = [0.]

        def advance(seconds, cancel):
            now[0] += seconds

        def sample(after=None, timeout=.5):
            q = [.1]*6
            if .15 <= now[0] < .25:
                q[2] += math.radians(.71)
            return after + 1, q

        with patch("camera_workbench.app.time.monotonic", side_effect=lambda: now[0]), \
             patch.object(self.app, "wait", side_effect=advance), \
             patch.object(self.backend, "sample", side_effect=sample):
            _, _, _, samples, stable_start = self.app.wait_for_position([.1]*6, threading.Event(), hold_s=.3)
        self.assertAlmostEqual(stable_start, .25)
        self.assertGreaterEqual(samples[-1]["elapsed_s"] - stable_start, .3)
    def test_failed_home_does_not_disable(self):
        self.permit();self.backend.enabled=True
        self.backend.move=lambda *a,**kw:(_ for _ in ()).throw(RuntimeError("move failed"))
        self.app.command("/api/home-disable",{})
        self.app.worker.join(2)
        self.assertNotIn("disable",self.backend.calls)
        self.assertEqual(self.app.mode,"error")
    def test_emergency_disable_does_not_request_home(self):
        self.permit();self.backend.enabled=True
        self.app.command("/api/emergency-disable",{})
        self.assertIn("emergency_disable",self.backend.calls)
        self.assertEqual(self.backend.moves,[])
    def test_events_match_original_frontend_envelope(self):
        q=queue.Queue();self.app.subscribers.append(q)
        self.app.notify("hello")
        event,state=q.get(),q.get()
        self.assertEqual(event["event"]["message"],"hello")
        self.assertIn("home_q_deg",state["status"])
    def test_old_home_binding_stays_incompatible_even_for_new_backend(self):
        t=copy.deepcopy(self.legacy);t.update(backend=BACKEND,home_reference="previous-home")
        self.assertFalse(self.app.compatible(t))
        self.app.home["compatible_home_references"] = ["previous-home"]
        self.assertTrue(self.app.compatible(t))
        t["home_reference"] = "unknown-home"
        self.assertFalse(self.app.compatible(t))
    def test_camera_settings_and_connect_do_not_start_arm(self):
        self.app.command("/api/camera/settings",dict(post_delay_s=2))
        self.app.command("/api/camera/connect",{})
        self.assertEqual(self.camera.commands,["connect"])
        self.assertEqual(self.backend.calls,[])

    def test_robstride_temperature_frame_is_filtered_to_camera_axes(self):
        data = bytes.fromhex("7FE2800D985B0262")
        self.assertEqual(decode_temperature(FRAME.pack(0x828003FD, 8, data)), (3, 61.0, False))
        self.assertEqual(decode_temperature(FRAME.pack(0x828403FD, 8, data)), (3, 61.0, True))
        for can_id in (0x828007FD, 0x828003FE, 0x818003FD, 0x028003FD):
            self.assertIsNone(decode_temperature(FRAME.pack(can_id, 8, data)))

    def test_temperature_warning_is_once_per_heating_cycle(self):
        readings = {3: (70., False)}
        self.app.temperature = SimpleNamespace(snapshot=lambda: readings, close=lambda: None, drives_in_reset=lambda: False)
        for value in (70., 71., 69.9, 70.):
            readings[3] = (value, False)
            self.app.check_temperature()
        self.assertEqual(len(self.app.events), 1)
        readings[3] = (67., False)
        self.app.check_temperature()
        readings[3] = (70., False)
        self.app.check_temperature()
        self.assertEqual(len(self.app.events), 2)
        self.assertEqual(self.backend.calls, [])

    def test_temperature_trip_stops_returns_home_disables_and_requires_reset(self):
        readings = {3: (80., False)}
        self.app.temperature = SimpleNamespace(snapshot=lambda: readings, close=lambda: None, drives_in_reset=lambda: False)
        self.backend.enabled = True
        cancel = self.app.cancel
        with patch.object(self.app, "wait") as wait:
            self.app.check_temperature()
            self.app.thermal_thread.join(2.)
        self.assertTrue(cancel.is_set())
        self.assertEqual(self.backend.calls, ["stop", "disable"])
        self.assertEqual(self.backend.moves, [self.app.home_q])
        self.assertEqual(self.app.thermal_phase, "disabled")
        self.assertTrue(self.app.thermal_latch.exists())
        wait.assert_not_called()
        self.permit()
        with self.assertRaisesRegex(RuntimeError, "温度保护已锁定"):
            self.app.command("/api/start", {})
        with self.assertRaisesRegex(RuntimeError, "70°C"):
            self.app.command("/api/thermal/reset", {"confirmed_cooled": True})
        readings[3] = (69., False)
        self.app.command("/api/thermal/reset", {"confirmed_cooled": True})
        self.assertIsNone(self.app.thermal_trip)
        self.assertFalse(self.app.thermal_latch.exists())
        self.assertFalse(self.backend.enabled)

    def test_asserted_motor_overtemperature_fault_prevents_thermal_home(self):
        self.app.temperature = SimpleNamespace(snapshot=lambda: {3: (80., True)}, close=lambda: None, drives_in_reset=lambda: False)
        self.backend.enabled = True
        with self.assertLogs(level="ERROR"):
            self.app.check_temperature()
            self.app.thermal_thread.join(2.)
        self.assertEqual(self.backend.calls, ["stop"])
        self.assertEqual(self.backend.moves, [])
        self.assertEqual(self.app.thermal_phase, "failed")
    def test_ros_joint_feedback_rejects_seventh_axis(self):
        msg=SimpleNamespace(name=[f"joint{i}" for i in range(1,8)],position=[0.]*7)
        with self.assertRaises(ValueError):RosBackend.six(msg)


if __name__=="__main__":unittest.main()
