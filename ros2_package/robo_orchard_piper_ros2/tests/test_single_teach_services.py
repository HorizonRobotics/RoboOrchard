# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

import sys
import types


def _install_stub_modules():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.node = types.ModuleType("rclpy.node")

    class FakeNode:
        def __init__(self, *args, **kwargs):
            self._parameters = {}
            self._created_services = []

        def declare_parameter(self, name, value):
            self._parameters[name] = value

        def get_parameter(self, name):
            value = self._parameters[name]
            parameter_value = types.SimpleNamespace(
                string_value=value if isinstance(value, str) else "",
                bool_value=bool(value) if isinstance(value, bool) else False,
                integer_value=value if isinstance(value, int) else 0,
                double_value=value if isinstance(value, float) else 0.0,
                double_array_value=(
                    value
                    if isinstance(value, list)
                    and all(isinstance(v, float) for v in value)
                    else []
                ),
                string_array_value=(
                    value
                    if isinstance(value, list)
                    and all(isinstance(v, str) for v in value)
                    else []
                ),
            )
            return types.SimpleNamespace(
                get_parameter_value=lambda: parameter_value
            )

        def get_logger(self):
            return types.SimpleNamespace(
                info=lambda *args, **kwargs: None,
                warn=lambda *args, **kwargs: None,
                error=lambda *args, **kwargs: None,
            )

        def create_publisher(self, *args, **kwargs):
            return types.SimpleNamespace(publish=lambda *a, **k: None)

        def create_service(self, service_type, name, callback):
            self._created_services.append(name)
            return types.SimpleNamespace(name=name, callback=callback)

        def create_subscription(self, *args, **kwargs):
            return types.SimpleNamespace()

        def create_timer(self, *args, **kwargs):
            return types.SimpleNamespace()

        def add_on_set_parameters_callback(self, callback):
            self._parameter_callback = callback
            return types.SimpleNamespace(callback=callback)

    fake_rclpy.node.Node = FakeNode
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_rclpy.node

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces.msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces.msg.SetParametersResult = type(
        "SetParametersResult",
        (),
        {
            "__init__": lambda self, successful=True, reason="": (
                setattr(self, "successful", successful),
                setattr(self, "reason", reason),
                None,
            )[-1]
        },
    )
    sys.modules["rcl_interfaces"] = rcl_interfaces
    sys.modules["rcl_interfaces.msg"] = rcl_interfaces.msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.PoseStamped = type("PoseStamped", (), {})
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs.msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs.srv = types.ModuleType("std_srvs.srv")
    std_srvs.srv.Trigger = type(
        "Trigger",
        (),
        {"Request": object, "Response": object},
    )
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs.srv

    piper_msg = types.ModuleType("robo_orchard_piper_msg_ros2")
    piper_msg.msg = types.ModuleType("robo_orchard_piper_msg_ros2.msg")
    piper_msg.msg.PiperStatusMsg = type("PiperStatusMsg", (), {})
    sys.modules["robo_orchard_piper_msg_ros2"] = piper_msg
    sys.modules["robo_orchard_piper_msg_ros2.msg"] = piper_msg.msg

    fake_bridge = types.ModuleType("robo_orchard_piper_ros2.ros_bridge")
    fake_bridge.GravityCompensationError = type(
        "GravityCompensationError", (RuntimeError,), {}
    )
    fake_bridge.resolve_deflection_calibration = lambda *args, **kwargs: (
        [],
        [],
        "",
    )
    # None = store has no friction/gravity section (fallback path)
    fake_bridge.resolve_friction_calibration = lambda *args, **kwargs: None
    fake_bridge.resolve_gravity_scale_calibration = lambda *args, **kwargs: (
        None
    )
    fake_bridge.PinocchioGravityCompensator = lambda *args, **kwargs: (
        types.SimpleNamespace(
            configure=lambda *a, **k: None,
            set_record_path=lambda *a, **k: None,
        )
    )
    fake_bridge.create_piper = lambda *args, **kwargs: types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(
            arm_status=types.SimpleNamespace(ctrl_mode=0x01, teach_status=0)
        )
    )
    fake_bridge.enable_arm_ctrl = lambda *args, **kwargs: None
    fake_bridge.get_arm_ee_pose = lambda *args, **kwargs: None
    fake_bridge.get_arm_state = lambda *args, **kwargs: None
    fake_bridge.get_arm_status = lambda *args, **kwargs: None
    fake_bridge.joint_control = lambda *args, **kwargs: None
    fake_bridge.joint_mit_control = lambda *args, **kwargs: None
    fake_bridge.reset_piper_ctrl_mode = lambda *args, **kwargs: True
    fake_bridge.switch_piper_ctrl_mode = lambda *args, **kwargs: True
    fake_bridge.set_ctrl_method = lambda *args, **kwargs: None
    sys.modules["robo_orchard_piper_ros2.ros_bridge"] = fake_bridge


_install_stub_modules()

sys.path.insert(0, "ros2_package/robo_orchard_piper_ros2")

import robo_orchard_piper_ros2.single as single_module  # noqa: E402
from robo_orchard_piper_ros2.single import PiperSingleControlNode  # noqa: E402


def _build_node(
    ctrl_mode: int,
    teach_status: int = 0,
    enable_mit_ctrl: bool = False,
):
    node = PiperSingleControlNode.__new__(PiperSingleControlNode)
    arm_status = types.SimpleNamespace(
        ctrl_mode=ctrl_mode, teach_status=teach_status
    )
    node.piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status)
    )
    node._arm_status = arm_status
    node.enable_mit_ctrl = enable_mit_ctrl
    node.mit_kp = 10.0
    node.mit_kd = 0.8
    node.mit_vel_ref = 45.0
    node.mit_torque_ref = 0.0
    node._enable_flag = False
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *args, **kwargs: None
    )
    return node


def _build_velocity_node(source: str = "position_delta"):
    node = PiperSingleControlNode.__new__(PiperSingleControlNode)
    node.mit_velocity_feedforward_source = source
    node.mit_velocity_feedforward_alpha = 1.0
    # 0 selects the legacy single-EMA path, so alpha=1.0 passes the raw
    # finite difference through unchanged.
    node.mit_velocity_feedforward_cutoff_hz = 0.0
    node.mit_velocity_feedforward_deadband = 0.0
    node.mit_velocity_feedforward_phase_lead = 0.0
    node._prev_cmd_pos = None
    node._prev_cmd_vel = None
    node._prev_cmd_time = None
    node._vel_lp1 = None
    node._vel_lp2 = None
    node._last_cmd_dt = None
    return node


def _joint_command(position, velocity=None, stamp_s: float = 1.0):
    sec = int(stamp_s)
    nanosec = int(round((stamp_s - sec) * 1e9))
    return types.SimpleNamespace(
        position=position,
        velocity=[] if velocity is None else velocity,
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec)
        ),
    )


def test_is_controlable_accepts_raw_sdk_can_mode_value():
    node = _build_node(ctrl_mode=0x01)
    node._enable_flag = True

    assert node.is_controlable() is True


def test_enable_arm_ctrl_in_active_teach_mode_does_not_attempt_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=1)

    calls = []

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("switch_ctrl_mode"),
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is False
    assert node._enable_flag is False
    assert calls == []


def test_enable_arm_ctrl_force_resets_active_teach_mode(monkeypatch):
    node = _build_node(ctrl_mode=0x02, teach_status=1)
    calls = []

    monkeypatch.setattr(
        single_module,
        "reset_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("reset") or True,
    )
    monkeypatch.setattr(
        single_module,
        "set_ctrl_method",
        lambda *args, **kwargs: calls.append("set_ctrl_method"),
    )

    assert node.enable_arm_ctrl(force_reset=True) is True
    assert node._enable_flag is True
    assert calls == ["reset", "set_ctrl_method"]


def test_enable_arm_ctrl_force_reset_failure_stays_disabled(monkeypatch):
    node = _build_node(ctrl_mode=0x02, teach_status=1)
    calls = []

    monkeypatch.setattr(
        single_module,
        "reset_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("reset") or False,
    )
    monkeypatch.setattr(
        single_module,
        "set_ctrl_method",
        lambda *args, **kwargs: calls.append("set_ctrl_method"),
    )

    assert node.enable_arm_ctrl(force_reset=True) is False
    assert node._enable_flag is False
    assert calls == ["reset"]


def test_enable_arm_ctrl_in_post_teach_mode_succeeds_after_ctrl_mode_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_switch(*args, **kwargs):
        calls.append("switch_ctrl_mode")
        node._arm_status.ctrl_mode = 0x01
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        fake_switch,
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["switch_ctrl_mode", "set_ctrl_method"]


def test_enable_arm_ctrl_in_post_teach_mode_does_not_fail_on_immediate_status(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_switch(*args, **kwargs):
        calls.append("switch_ctrl_mode")
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        fake_switch,
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["switch_ctrl_mode", "set_ctrl_method"]


def test_enable_arm_ctrl_in_fresh_power_on_does_not_fail_on_stale_status(
    monkeypatch,
):
    # Simulate a cold boot where the piper SDK's first GetArmStatus returns
    # a zero-initialised struct (ctrl_mode=0x00) because no CAN status frame
    # has arrived yet, and where the post-MotionCtrl_2 status read still
    # reflects the pre-command value.
    node = _build_node(ctrl_mode=0x00, teach_status=0)

    calls = []

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        # set_ctrl_method issues MotionCtrl_2(0x01, ...) but the status
        # frame has not refreshed yet; leave node._arm_status.ctrl_mode
        # at its stale value to exercise the race.
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("switch_ctrl_mode"),
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["enable", "set_ctrl_method"]


def test_enable_ctrl_service_fails_when_ctrl_mode_switch_times_out(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)
    logs = []
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        error=lambda message: logs.append(message),
    )

    def fake_switch(*args, **kwargs):
        raise TimeoutError("ctrl mode switch timed out")

    monkeypatch.setattr(single_module, "switch_piper_ctrl_mode", fake_switch)
    monkeypatch.setattr(
        single_module, "set_ctrl_method", lambda *args, **kwargs: None
    )

    response = types.SimpleNamespace(success=None, message=None)

    ret = node._enable_ctrl_service_callback(object(), response)

    assert ret is response
    assert response.success is False
    assert "unexpected error occurred" in response.message.lower()
    assert "ctrl mode switch timed out" in response.message
    assert node._enable_flag is False
    assert logs == ["Error while enabling arm: ctrl mode switch timed out"]


def test_enable_ctrl_service_retries_when_flag_set_but_not_controlable(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x00, teach_status=0)
    node._enable_flag = True
    calls = []
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    def fake_enable_arm_ctrl():
        calls.append("enable")
        node._arm_status.ctrl_mode = 0x01
        return True

    monkeypatch.setattr(node, "enable_arm_ctrl", fake_enable_arm_ctrl)

    response = types.SimpleNamespace(success=None, message=None)

    ret = node._enable_ctrl_service_callback(object(), response)

    assert ret is response
    assert response.success is True
    assert response.message == "Arm enabled successfully."
    assert calls == ["enable"]


def test_auto_enable_timeout_does_not_abort_node_startup(monkeypatch):
    base_node = PiperSingleControlNode.__mro__[1]
    original_declare_parameter = base_node.declare_parameter
    logs = []

    def fake_declare_parameter(self, name, value):
        if name == "auto_enable_arm_ctrl":
            value = True
        return original_declare_parameter(self, name, value)

    def fake_get_logger(self):
        return types.SimpleNamespace(
            info=lambda *args, **kwargs: None,
            warn=lambda message: logs.append(message),
            error=lambda *args, **kwargs: None,
        )

    def fake_enable_arm_ctrl(self):
        raise TimeoutError("enable timed out")

    monkeypatch.setattr(base_node, "declare_parameter", fake_declare_parameter)
    monkeypatch.setattr(base_node, "get_logger", fake_get_logger)
    monkeypatch.setattr(
        PiperSingleControlNode, "enable_arm_ctrl", fake_enable_arm_ctrl
    )

    node = PiperSingleControlNode()

    assert node._enable_flag is False
    assert node._created_services == ["enable_ctrl", "reset_ctrl"]
    assert logs == ["Auto enable timed out. enable timed out"]


def test_node_does_not_register_disable_ctrl_service():
    node = PiperSingleControlNode()

    assert node._created_services == ["enable_ctrl", "reset_ctrl"]


def test_velocity_feedforward_defaults_to_position_delta_source():
    node = _build_velocity_node()

    first = _joint_command([0.0] * 6, velocity=[99.0] * 6, stamp_s=1.0)
    second = _joint_command([0.1] * 6, velocity=[99.0] * 6, stamp_s=1.1)

    assert node._command_velocity(first) == [0.0] * 6
    velocity = node._command_velocity(second)

    assert all(abs(v - 1.0) < 1e-9 for v in velocity)


def test_velocity_feedforward_message_source_is_explicit_opt_in():
    node = _build_velocity_node(source="message")
    node.mit_velocity_feedforward_deadband = 0.02

    command = _joint_command(
        [0.0] * 6,
        velocity=[0.01, 0.04, 99.0, -99.0, 1.5, -1.5],
    )

    # Soft-threshold deadband: below-threshold values zero, the rest are
    # shrunk toward zero by the deadband before the +/-45 clamp.
    expected = [0.0, 0.02, 45.0, -45.0, 1.48, -1.48]
    velocity = node._command_velocity(command)
    assert all(
        abs(v - e) < 1e-9 for v, e in zip(velocity, expected, strict=True)
    ), velocity


def _build_phase_lead_node():
    node = _build_velocity_node()
    node.mit_velocity_feedforward_cutoff_hz = 10.0
    node.mit_velocity_feedforward_phase_lead = 1.0
    node._last_cmd_dt = 0.01
    return node


def test_phase_lead_predicts_v_des_forward_by_the_filter_delay():
    node = _build_phase_lead_node()

    velocity = [1.0, -0.5, 0.0, 0.0, 0.0, 0.0]
    accel = [10.0, 4.0, 0.0, 0.0, 0.0, 0.0]
    delay = 2.0 / (2.0 * 3.141592653589793 * 10.0) + 0.5 * 0.01

    led = node._phase_lead_velocity(velocity, accel)

    assert abs(led[0] - (1.0 + delay * 10.0)) < 1e-9
    assert abs(led[1] - (-0.5 + delay * 4.0)) < 1e-9
    assert led[2:] == [0.0] * 4


def test_phase_lead_output_stays_within_firmware_velocity_clamp():
    node = _build_phase_lead_node()

    led = node._phase_lead_velocity([44.9, -44.9], [1e5, -1e5])

    assert led == [45.0, -45.0]


def test_phase_lead_is_inert_when_disabled_or_inapplicable():
    velocity = [1.0] * 6
    accel = [10.0] * 6

    node = _build_phase_lead_node()
    node.mit_velocity_feedforward_phase_lead = 0.0
    assert node._phase_lead_velocity(velocity, accel) == velocity

    node = _build_phase_lead_node()
    assert node._phase_lead_velocity(velocity, None) == velocity

    # Legacy EMA path: group delay is alpha/rate-dependent, lead unmodeled.
    node = _build_phase_lead_node()
    node.mit_velocity_feedforward_cutoff_hz = 0.0
    assert node._phase_lead_velocity(velocity, accel) == velocity

    # Message-sourced velocity never went through the smoothing chain.
    node = _build_phase_lead_node()
    node.mit_velocity_feedforward_source = "message"
    assert node._phase_lead_velocity(velocity, accel) == velocity
