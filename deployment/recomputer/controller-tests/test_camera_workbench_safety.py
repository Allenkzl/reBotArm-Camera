
import threading,time
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from rebotarmcontroller.hardware_manager import HardwareManager

def bare():
 h=object.__new__(HardwareManager);h._cmd_lock=threading.RLock()
 h._robot=SimpleNamespace(has_gripper=False,connect=Mock(),disconnect=Mock(),
                         stop_control_loop=Mock(),control_loop_active=True)
 h._arm_group=SimpleNamespace(joint_names=[f"joint{i}" for i in range(1,7)],_mm={})
 return h

def test_connect_disabled_profile_never_starts_motor_output():
 h=bare();h._connected=False;h._startup_enabled=False;h._start_endpos_loop=Mock()
 h.connect()
 assert h._connected and not h._enabled
 h._start_endpos_loop.assert_not_called()

def test_explicit_valid_zero_replies_are_accepted():
 h=bare()
 for n in h.joint_names:h._arm_group._mm[n]=SimpleNamespace(robstride_get_param_f32=Mock(return_value=0.))
 np.testing.assert_equal(h._verify_startup_feedback(),np.zeros(6))
 for m in h._arm_group._mm.values():assert m.robstride_get_param_f32.call_count==2

def test_failed_feedback_never_reaches_enable_or_mode_switch():
 h=bare();h._enabled=False;h._verify_startup_feedback=Mock(side_effect=RuntimeError("no reply"))
 h._configure_groups_for_endpos=Mock()
 with pytest.raises(RuntimeError):h._start_endpos_loop()
 h._configure_groups_for_endpos.assert_not_called()

def test_shutdown_disabled_connection_never_homes_or_enables():
 h=bare();h._connected=True;h._enabled=False;h.safe_home=Mock()
 h.shutdown()
 h.safe_home.assert_not_called()
 h._robot.disconnect.assert_called_once()
 assert not h._connected

def test_urgent_hold_freezes_latest_pose_without_home_or_disable():
 h=bare();h._enabled=True;h._gravity_comp_active=False
 h._feedback_refreshed_at=time.monotonic();h._cached_arm_position=np.arange(6)/10
 h._endpos_ctrl=SimpleNamespace(_q_target=np.ones(6),_qd_target=np.ones(6))
 h._mit_stream_velocity=np.ones(6);h._mit_stream_acceleration=np.ones(6)
 h._mit_stream_target=np.ones(6);h._lowlevel_pos_target=np.ones(6)
 h._state_machine="LOWLEVEL_STREAMING";h.stop_motion=Mock();h.safe_home=Mock();h.disable=Mock()
 h.emergency_hold_current()
 np.testing.assert_equal(h._endpos_ctrl._q_target,h._cached_arm_position)
 assert h._mit_stream_target is None and h._state_machine=="IDLE"
 h.safe_home.assert_not_called();h.disable.assert_not_called()
