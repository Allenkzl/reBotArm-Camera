"""ROS transport only. Importing/constructing this class never enables hardware."""
import math
import threading
import time

TEACH_REPLAY_FRAME_ID = "rebotarm_teaching_replay"


class RosBackend:
    def __init__(self, namespace="rebot_arm_rs"):
        if namespace != "rebot_arm_rs":
            raise ValueError("only the six-axis camera hardware namespace is supported")
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from rebotarm_msgs.msg import ArmStatus
        from std_srvs.srv import Trigger
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self.rclpy, self.Trigger, self.Action = rclpy, Trigger, FollowJointTrajectory
        rclpy.init()
        self.node = rclpy.create_node("legacy_camera_workbench")
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.q, self.ref, self.seq, self.received = None, None, 0, 0.
        self.arm, self.arm_received, self.goal = {}, 0., None
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.subs = [self.node.create_subscription(JointState, f"/{namespace}/joint_states", self._joint, qos_profile_sensor_data),
                     self.node.create_subscription(JointState, f"/{namespace}/control_reference", self._reference, qos_profile_sensor_data),
                     self.node.create_subscription(ArmStatus, f"/{namespace}/arm_status", self._status,
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        self.clients = {name: self.node.create_client(Trigger, f"/{namespace}/{name}") for name in
                        ("enable", "disable", "emergency_disable", "hold_current",
                         "gravity_compensation/start", "gravity_compensation/stop")}
        self.action = ActionClient(self.node, FollowJointTrajectory, f"/{namespace}/follow_joint_trajectory")
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def six(msg):
        values = dict(zip(msg.name, msg.position))
        result = [float(values[f"joint{i}"]) for i in range(1, 7)]
        if len(msg.name) != 6 or not all(math.isfinite(x) for x in result):
            raise ValueError("invalid six-axis feedback")
        return result

    def _joint(self, msg):
        try:
            q = self.six(msg)
        except (ValueError, KeyError):
            return
        with self.condition:
            self.q, self.received = q, time.monotonic()
            self.seq += 1
            self.condition.notify_all()

    def _reference(self, msg):
        try:
            values = self.six(msg)
        except (ValueError, KeyError):
            return
        with self.lock:
            self.ref = values

    def _status(self, msg):
        if list(msg.joint_names) != [f"joint{i}" for i in range(1, 7)]:
            return
        with self.lock:
            self.arm = dict(enabled=msg.enabled, loop=msg.control_loop_active,
                            state=msg.state_machine, errors=list(msg.error_codes))
            self.arm_received = time.monotonic()

    def snapshot(self):
        with self.lock:
            fresh = self.q is not None and time.monotonic() - self.received < .5
            # An absent service means the latched status cannot establish liveness.
            online = fresh and self.clients["enable"].service_is_ready()
            return dict(online=online, enabled=bool(online and self.arm.get("enabled")),
                        q_rad=list(self.q) if fresh else None, reference_rad=self.ref,
                        state=self.arm.get("state"), errors=list(self.arm.get("errors", [])), seq=self.seq)

    def sample(self, after=None, timeout=2.):
        with self.condition:
            deadline = time.monotonic() + timeout
            while self.q is None or time.monotonic() - self.received >= .5 or (after is not None and self.seq <= after):
                left = deadline - time.monotonic()
                if left <= 0:
                    raise RuntimeError("没有收到新鲜六轴反馈，未记录点位")
                self.condition.wait(left)
            return self.seq, list(self.q)

    @staticmethod
    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("新底层响应结果未知，请核对状态；不要重复提交")
            time.sleep(.01)
        return future.result()

    def trigger(self, name, timeout=8.):
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=1.):
            raise RuntimeError("新六轴控制器未启动；工作台不会自动使能设备")
        reply = self.wait(client.call_async(self.Trigger.Request()), timeout)
        if not reply.success:
            raise RuntimeError(reply.message)
        return reply.message

    def move(self, target, cancel, speed=.075, *, replay=False):
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        _, current = self.sample()
        if len(target) != 6 or not all(math.isfinite(x) for x in target):
            raise ValueError("目标必须包含六个有限角度")
        max_speed = 1.0 if replay else .15
        if not .01 <= speed <= max_speed:
            raise ValueError("回放速度超出范围")
        if not self.action.wait_for_server(timeout_sec=1.):
            raise RuntimeError("新底层轨迹接口未连接")
        goal = self.Action.Goal()
        if replay:
            # Use the controller's replay trajectory profile without changing its general action limit.
            goal.trajectory.header.frame_id = TEACH_REPLAY_FRAME_ID
        goal.trajectory.joint_names = [f"joint{i}" for i in range(1, 7)]
        # Smooth zero-velocity endpoints; this duration bounds the cubic peak speed.
        duration = max(1., 1.6 * max(abs(a-b) for a,b in zip(current,target)) / speed)
        for q, seconds in ((current, 0.), (target, duration)):
            p = JointTrajectoryPoint()
            p.positions, p.velocities = list(q), [0.]*6
            p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1)*1e9))
            goal.trajectory.points.append(p)
        if cancel.is_set():
            raise RuntimeError("动作已取消")
        accepted = self.action.send_goal_async(goal)
        # A stop while goal acceptance is pending must cancel the late handle too.
        accepted.add_done_callback(lambda f: f.result().cancel_goal_async() if cancel.is_set() and f.result().accepted else None)
        handle = self.wait(accepted, 5.)
        if not handle.accepted:
            raise RuntimeError("新底层拒绝轨迹请求")
        with self.lock:
            self.goal = handle
        result = handle.get_result_async()
        deadline = time.monotonic() + duration + 8
        try:
            while not result.done():
                if cancel.is_set() or time.monotonic() >= deadline:
                    handle.cancel_goal_async()
                    raise RuntimeError("动作已取消或超时，已请求停止并保持")
                if not self.snapshot()["online"]:
                    handle.cancel_goal_async()
                    raise RuntimeError("六轴反馈中断，已请求停止")
                time.sleep(.02)
            reply = result.result()
            if reply.status != 4 or reply.result.error_code != 0:
                raise RuntimeError(reply.result.error_string or "轨迹未完成")
        finally:
            with self.lock:
                if self.goal is handle:
                    self.goal = None

    def stop(self):
        with self.lock:
            handle = self.goal
        if handle:
            self.wait(handle.cancel_goal_async(), 3.)
        if self.snapshot().get("state") == "GRAVITY_COMP":
            self.trigger("gravity_compensation/stop")
        if self.snapshot()["enabled"]:
            self.trigger("hold_current")

    def close(self):
        # Closing a web service is never an implicit Home/disable operation.
        self.executor.shutdown(timeout_sec=2)
        self.node.destroy_node()
        self.rclpy.shutdown()
