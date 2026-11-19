
"""Software Home behavior with no motor connection or motion."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
from rebotarmcontroller.hardware_config import _ensure_rebot_sdk_in_syspath
from rebotarmcontroller.hardware_manager import HardwareManager

_ensure_rebot_sdk_in_syspath()
from reBotArm_control_py.controllers import RebotArmEndPose

HOME = np.array([.02928,.00128,.00404,-.01036,-.00165,-.06035])

def controller(start, settled):
    obj = object.__new__(RebotArmEndPose)
    obj._running = True
    obj._n = 6
    obj._q_target = np.ones(6)
    obj._dt = .01
    calls = []
    def state():
        q = np.array(start if not calls else settled, dtype=float)
        calls.append(q)
        return q, np.zeros(6), np.zeros(6)
    obj.rebotarm = SimpleNamespace(get_state=state)
    return obj, calls

@pytest.mark.parametrize("configured", [None, HOME])
def test_final_hold_and_settle_use_same_home(configured):
    target = np.zeros(6) if configured is None else configured
    obj, reads = controller(target + .03, target)
    with patch("reBotArm_control_py.controllers.rebotarm_endpose_controller.time.sleep"):
        obj.safe_home(home_position=configured)
    np.testing.assert_allclose(obj._q_target, target)
    assert len(reads) == 2
    assert obj._vlim_override is None

def test_near_home_still_sets_exact_final_target():
    obj, _ = controller(HOME + .001, HOME)
    obj.safe_home(home_position=HOME)
    np.testing.assert_allclose(obj._q_target, HOME)

@pytest.mark.parametrize("invalid", [[0.] * 5, [float("nan")] * 6, [float("inf")] * 6])
def test_invalid_home_rejected_before_target_changes(invalid):
    obj, _ = controller(np.zeros(6), np.zeros(6))
    before = obj._q_target.copy()
    with pytest.raises(ValueError):
        obj.safe_home(home_position=invalid)
    np.testing.assert_equal(obj._q_target, before)

def test_disable_gate_uses_configured_home_not_encoder_zero():
    obj = object.__new__(HardwareManager)
    obj._home_position = HOME.copy()
    obj._arm_group = SimpleNamespace(joint_names=[f"joint{i}" for i in range(1,7)])
    obj.get_joint_state = lambda: (HOME.copy(), np.zeros(6), np.zeros(6))
    assert obj.is_near_safe_home()
    obj.get_joint_state = lambda: (np.zeros(6), np.zeros(6), np.zeros(6))
    assert not obj.is_near_safe_home()

def test_camera_profile_loads_captured_home_without_motor_io():
    import json
    from reBotArm_control_py.actuator import RebotArm
    root = Path(__file__).resolve().parents[4]
    profile = root / "deployment/recomputer/hardware-camera-j1-j6.yaml"
    expected = json.loads((root / "deployment/recomputer/home.json").read_text())
    with patch.object(RebotArm, "connect", side_effect=AssertionError("CAN forbidden")), \
         patch.object(RebotArm, "_setup_motors", side_effect=AssertionError("motor setup forbidden")):
        obj = HardwareManager(hardware_config=str(profile), model="rs", channel="can0")
    assert not obj.has_gripper and not obj.enabled
    np.testing.assert_allclose(obj._home_position, expected["q_rad"])
