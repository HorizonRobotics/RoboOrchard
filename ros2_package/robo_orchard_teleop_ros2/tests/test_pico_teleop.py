# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

from __future__ import annotations
# ruff: noqa: E402
import math
import sys
import types
from dataclasses import dataclass
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


@dataclass
class _Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class _Pose:
    position: _Point
    orientation: _Quaternion


@dataclass
class _PoseStamped:
    pose: _Pose


@dataclass
class _Controller:
    status: int
    pose: _Pose


@dataclass
class _VRState:
    left_controller: _Controller
    right_controller: _Controller


class _LongPressIntent:
    def __init__(self, source_type, value_thresh=0.0, thresh=0.0):
        self.source_type = source_type
        self.value_thresh = value_thresh
        self.thresh = thresh

    def is_active(self, _msg):
        return False


class _ResetIntent:
    def __init__(self, source_type, thresh=0.0):
        self.source_type = source_type
        self.thresh = thresh

    def should_reset(self, _msg):
        return False


class _ResetNowIntent(_ResetIntent):
    def should_reset(self, _msg):
        return True


class _IkOptimizer:
    last_init_kwargs = None
    last_solve_pose = None
    last_solve_poses = []
    last_solve_orientation_modes = []
    last_solve_joint_state = None
    next_solution = [0.0] * 6
    next_solutions = None

    def __init__(self, *args, **kwargs):
        type(self).last_init_kwargs = dict(kwargs)

    def solve(self, pose, joint_state, orientation_mode="all"):
        type(self).last_solve_pose = pose
        type(self).last_solve_poses.append(pose)
        type(self).last_solve_orientation_modes.append(orientation_mode)
        type(self).last_solve_joint_state = joint_state
        if type(self).next_solutions is not None:
            return type(self).next_solutions.pop(0)
        return list(type(self).next_solution)


class _Matrix3:
    def __init__(self, rows):
        self.rows = [[float(value) for value in row] for row in rows]

    def __getitem__(self, key):
        row_idx, col_idx = key
        return self.rows[row_idx][col_idx]

    @property
    def T(self):  # noqa: N802
        return _Matrix3(zip(*self.rows, strict=False))

    def __matmul__(self, other):
        if isinstance(other, _Vector3):
            return _Vector3(
                sum(row[i] * other.values[i] for i in range(3))
                for row in self.rows
            )

        other_rows = other.rows
        result = []
        for row in self.rows:
            result_row = []
            for col_idx in range(3):
                value = 0.0
                for i in range(3):
                    value += row[i] * other_rows[i][col_idx]
                result_row.append(value)
            result.append(result_row)
        return _Matrix3(result)


class _Vector3:
    def __init__(self, values):
        self.values = [float(value) for value in values]

    def __iter__(self):
        return iter(self.values)

    def __sub__(self, other):
        return _Vector3(self.values[i] - other.values[i] for i in range(3))

    def __add__(self, other):
        return _Vector3(self.values[i] + other.values[i] for i in range(3))

    def __imul__(self, scale):
        self.values = [value * float(scale) for value in self.values]
        return self


class _Matrix4:
    def __init__(self, rows=None):
        if rows is None:
            rows = [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        self.rows = [[float(value) for value in row] for row in rows]

    def copy(self):
        return _Matrix4(self.rows)

    def __getitem__(self, key):
        row_sel, col_sel = key
        if row_sel == slice(None, 3, None) and col_sel == slice(None, 3, None):
            return _Matrix3(row[:3] for row in self.rows[:3])
        if row_sel == slice(None, 3, None) and col_sel == 3:
            return _Vector3(row[3] for row in self.rows[:3])
        raise TypeError(f"Unsupported slice {key!r}")

    def __setitem__(self, key, value):
        row_sel, col_sel = key
        if row_sel == slice(None, 3, None) and col_sel == slice(None, 3, None):
            rows = value.rows if isinstance(value, _Matrix3) else value
            for i in range(3):
                for j in range(3):
                    self.rows[i][j] = float(rows[i][j])
            return
        if row_sel == slice(None, 3, None) and col_sel == 3:
            values = value.values if isinstance(value, _Vector3) else value
            for i in range(3):
                if isinstance(values, list):
                    self.rows[i][3] = float(values[i])
                else:
                    self.rows[i][3] = float(list(values)[i])
            return
        raise TypeError(f"Unsupported assignment {key!r}")

    def __matmul__(self, other):
        result = []
        for row in self.rows:
            result_row = []
            for col_idx in range(4):
                value = 0.0
                for i in range(4):
                    value += row[i] * other.rows[i][col_idx]
                result_row.append(value)
            result.append(result_row)
        return _Matrix4(result)


def _identity4():
    return _Matrix4()


def _invert_transform(matrix):
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    rotation_t = rotation.T
    inv = _identity4()
    inv[:3, :3] = rotation_t
    inv[:3, 3] = _Vector3(
        -sum(rotation_t.rows[i][j] * translation.values[j] for j in range(3))
        for i in range(3)
    )
    return inv


def _norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def _quat_to_matrix(quat: _Quaternion):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return _Matrix3(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def _matrix_to_quat(matrix):
    m = matrix
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _Quaternion(x=x, y=y, z=z, w=w)


def _pose_msg_to_matrix(pose: _Pose):
    matrix = _identity4()
    matrix[:3, :3] = _quat_to_matrix(pose.orientation)
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _matrix_to_pose_msg(matrix):
    return _Pose(
        position=_Point(*matrix[:3, 3]),
        orientation=_matrix_to_quat(matrix[:3, :3]),
    )


geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.Pose = _Pose
geometry_msgs_msg.PoseStamped = _PoseStamped
geometry_msgs.msg = geometry_msgs_msg
sys.modules["geometry_msgs"] = geometry_msgs
sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

pico_msgs = types.ModuleType("robo_orchard_pico_msg_ros2")
pico_msgs_msg = types.ModuleType("robo_orchard_pico_msg_ros2.msg")
pico_msgs_msg.VRState = _VRState
pico_msgs.msg = pico_msgs_msg
sys.modules["robo_orchard_pico_msg_ros2"] = pico_msgs
sys.modules["robo_orchard_pico_msg_ros2.msg"] = pico_msgs_msg

intent_module = types.ModuleType("robo_orchard_teleop_ros2.bridge.pico.intent")
intent_module.LongPressIntent = _LongPressIntent
intent_module.ResetIntent = _ResetIntent
sys.modules["robo_orchard_teleop_ros2.bridge.pico.intent"] = intent_module

ik_module = types.ModuleType("robo_orchard_teleop_ros2.ik")
ik_module.IkOptimizer = _IkOptimizer
sys.modules["robo_orchard_teleop_ros2.ik"] = ik_module

msg_adaptor_module = types.ModuleType("robo_orchard_teleop_ros2.msg_adaptor")
msg_adaptor_module.pose_msg_to_matrix = _pose_msg_to_matrix
msg_adaptor_module.matrix_to_pose_msg = _matrix_to_pose_msg
sys.modules["robo_orchard_teleop_ros2.msg_adaptor"] = msg_adaptor_module

numpy_module = types.ModuleType("numpy")
numpy_module.eye = lambda _n: _identity4()
numpy_module.linalg = types.SimpleNamespace(inv=_invert_transform, norm=_norm)
_saved_numpy_module = sys.modules.get("numpy")
sys.modules["numpy"] = numpy_module

try:
    from robo_orchard_teleop_ros2.bridge.pico.teleop import VRTeleOp
finally:
    if _saved_numpy_module is None:
        sys.modules.pop("numpy", None)
    else:
        sys.modules["numpy"] = _saved_numpy_module


def _make_pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return _Pose(
        position=_Point(x=x, y=y, z=z),
        orientation=_Quaternion(x=qx, y=qy, z=qz, w=qw),
    )


def test_target_pose_separates_translation_and_orientation_delta():
    _IkOptimizer.last_init_kwargs = None
    _IkOptimizer.last_solve_pose = None
    _IkOptimizer.last_solve_poses = []
    _IkOptimizer.last_solve_orientation_modes = []
    _IkOptimizer.last_solve_joint_state = None
    _IkOptimizer.next_solution = [0.0] * 6
    _IkOptimizer.next_solutions = None
    teleop = VRTeleOp(
        source_type="left",
        urdf_path="robot.urdf",
        base_link_name="base_link",
        ee_link_name="link6",
        trigger_intent=_LongPressIntent("left"),
        reset_intent=_ResetIntent("X"),
    )
    teleop.control_state.is_active = True
    teleop.control_state.initial_vr_pose = _make_pose()
    teleop.control_state.initial_ee_pose = _make_pose(
        x=0.4,
        y=-0.1,
        z=0.2,
    )
    teleop.latest_vr_state = _VRState(
        left_controller=_Controller(
            status=1,
            pose=_make_pose(
                x=0.1,
                y=0.0,
                z=0.0,
                qz=math.sin(math.pi / 4.0),
                qw=math.cos(math.pi / 4.0),
            ),
        ),
        right_controller=_Controller(status=0, pose=_make_pose()),
    )
    teleop.control_state.current_joint_state = [0.1] * 6

    result = teleop()

    assert result is not None
    assert _IkOptimizer.last_solve_pose is not None
    assert _IkOptimizer.last_solve_joint_state == [0.1] * 6
    assert math.isclose(_IkOptimizer.last_solve_pose.position.x, 0.5)
    assert math.isclose(_IkOptimizer.last_solve_pose.position.y, -0.1)
    assert math.isclose(_IkOptimizer.last_solve_pose.position.z, 0.2)
    assert math.isclose(_IkOptimizer.last_solve_pose.orientation.x, 0.0)
    assert math.isclose(_IkOptimizer.last_solve_pose.orientation.y, 0.0)
    assert math.isclose(
        _IkOptimizer.last_solve_pose.orientation.z,
        math.sin(math.pi / 4.0),
    )
    assert math.isclose(
        _IkOptimizer.last_solve_pose.orientation.w,
        math.cos(math.pi / 4.0),
    )


def test_target_pose_low_pass_smooths_full_6d_target():
    _IkOptimizer.last_solve_pose = None
    _IkOptimizer.last_solve_poses = []
    _IkOptimizer.last_solve_orientation_modes = []
    _IkOptimizer.last_solve_joint_state = None
    _IkOptimizer.next_solution = [0.0] * 6
    _IkOptimizer.next_solutions = None
    teleop = VRTeleOp(
        source_type="left",
        urdf_path="robot.urdf",
        base_link_name="base_link",
        ee_link_name="link6",
        trigger_intent=_LongPressIntent("left"),
        reset_intent=_ResetIntent("X"),
        pose_low_pass_alpha=0.25,
    )
    teleop.control_state.is_active = True
    teleop.control_state.initial_vr_pose = _make_pose()
    teleop.control_state.initial_ee_pose = _make_pose()
    teleop.control_state.current_joint_state = [0.1] * 6

    teleop.latest_vr_state = _VRState(
        left_controller=_Controller(status=1, pose=_make_pose()),
        right_controller=_Controller(status=0, pose=_make_pose()),
    )
    first_result = teleop()
    assert first_result is not None
    assert _IkOptimizer.last_solve_pose is not None
    assert math.isclose(_IkOptimizer.last_solve_pose.position.x, 0.0)
    assert math.isclose(_IkOptimizer.last_solve_pose.orientation.w, 1.0)

    teleop.latest_vr_state = _VRState(
        left_controller=_Controller(
            status=1,
            pose=_make_pose(
                x=2.0,
                qz=math.sin(math.pi / 2.0),
                qw=math.cos(math.pi / 2.0),
            ),
        ),
        right_controller=_Controller(status=0, pose=_make_pose()),
    )
    second_result = teleop()

    assert second_result is not None
    assert _IkOptimizer.last_solve_pose is not None
    assert math.isclose(_IkOptimizer.last_solve_pose.position.x, 0.5)
    assert math.isclose(_IkOptimizer.last_solve_pose.position.y, 0.0)
    assert math.isclose(_IkOptimizer.last_solve_pose.position.z, 0.0)
    assert math.isclose(_IkOptimizer.last_solve_pose.orientation.x, 0.0)
    assert math.isclose(_IkOptimizer.last_solve_pose.orientation.y, 0.0)
    assert math.isclose(
        _IkOptimizer.last_solve_pose.orientation.z, 0.31622776601683794
    )
    assert math.isclose(
        _IkOptimizer.last_solve_pose.orientation.w, 0.9486832980505138
    )


def test_target_pose_filter_state_clears_on_reset_deactivate_and_invalid_vr():
    teleop = VRTeleOp(
        source_type="left",
        urdf_path="robot.urdf",
        base_link_name="base_link",
        ee_link_name="link6",
        trigger_intent=_LongPressIntent("left"),
        reset_intent=_ResetNowIntent("X"),
        pose_low_pass_alpha=0.25,
    )

    teleop.control_state.is_active = True
    teleop.control_state.filtered_target_ee_pose = _make_pose(x=1.0)
    reset_action = teleop.update_vr_state(
        _VRState(
            left_controller=_Controller(status=1, pose=_make_pose()),
            right_controller=_Controller(status=0, pose=_make_pose()),
        )
    )
    assert reset_action is not None
    assert reset_action.name == "RESET"
    assert teleop.control_state.filtered_target_ee_pose is None

    teleop.control_state.reset_intent = _ResetIntent("X")
    teleop.control_state.is_active = True
    teleop.control_state.filtered_target_ee_pose = _make_pose(x=2.0)
    deactivate_action = teleop.update_vr_state(
        _VRState(
            left_controller=_Controller(status=1, pose=_make_pose()),
            right_controller=_Controller(status=0, pose=_make_pose()),
        )
    )
    assert deactivate_action is not None
    assert deactivate_action.name == "DEACTIVE"
    assert teleop.control_state.filtered_target_ee_pose is None

    teleop.control_state.is_active = True
    teleop.control_state.filtered_target_ee_pose = _make_pose(x=3.0)
    invalid_action = teleop.update_vr_state(
        _VRState(
            left_controller=_Controller(status=0, pose=_make_pose()),
            right_controller=_Controller(status=0, pose=_make_pose()),
        )
    )
    assert invalid_action is not None
    assert invalid_action.name == "INVALID_VR_MSG"
    assert teleop.control_state.filtered_target_ee_pose is None


def test_ik_failure_falls_back_to_position_only():
    _IkOptimizer.last_solve_pose = None
    _IkOptimizer.last_solve_poses = []
    _IkOptimizer.last_solve_orientation_modes = []
    _IkOptimizer.next_solution = [0.0] * 6
    _IkOptimizer.next_solutions = [
        None,
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    ]
    teleop = VRTeleOp(
        source_type="left",
        urdf_path="robot.urdf",
        base_link_name="base_link",
        ee_link_name="link6",
        trigger_intent=_LongPressIntent("left"),
        reset_intent=_ResetIntent("X"),
        pose_low_pass_alpha=1.0,
    )
    teleop.control_state.is_active = True
    teleop.control_state.initial_vr_pose = _make_pose()
    teleop.control_state.initial_ee_pose = _make_pose()
    teleop.control_state.current_ee_pose = _make_pose(
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
    )
    teleop.control_state.current_joint_state = [0.1] * 6
    teleop.latest_vr_state = _VRState(
        left_controller=_Controller(
            status=1,
            pose=_make_pose(
                x=0.01,
                qz=math.sin(math.pi / 4.0),
                qw=math.cos(math.pi / 4.0),
            ),
        ),
        right_controller=_Controller(status=0, pose=_make_pose()),
    )

    result = teleop()

    assert result is not None
    assert result.solution == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert len(_IkOptimizer.last_solve_poses) == 2
    assert _IkOptimizer.last_solve_orientation_modes == ["all", None]


if __name__ == "__main__":
    test_target_pose_separates_translation_and_orientation_delta()
    print("ok")
