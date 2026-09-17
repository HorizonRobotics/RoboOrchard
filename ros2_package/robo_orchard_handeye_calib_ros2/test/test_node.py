# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

import json
from datetime import datetime, timezone
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")

from robo_orchard_handeye_calib_ros2 import node as node_module  # noqa: E402


def make_node(tmp_path, publish_tf=False):
    node = SimpleNamespace(
        config=SimpleNamespace(
            mode="eye_in_hand",
            end_effector_frame_name="tool",
            camera_frame_name="camera",
            result_file=str(tmp_path / "result.json"),
            output_root=None,
            publish_tf=publish_tf,
        ),
        aruco_poses_list=[object() for _ in range(3)],
        ee_poses_list=[object() for _ in range(3)],
        record_data_cnt=3,
        cur_aruco_pose=object(),
        cur_ee_pose=object(),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _publish_tf=lambda *args: None,
    )
    node._clear_data = lambda: node_module.CalibrationNode._clear_data(node)
    return node


def save(node):
    return node_module.CalibrationNode._save_data_service_callback(
        node, None, SimpleNamespace(success=False, message="")
    )


def test_successful_save_does_not_publish_tf_by_default(tmp_path, monkeypatch):
    node = make_node(tmp_path)
    published = []
    node._publish_tf = lambda *args: published.append(args)
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        lambda *args: ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
    )
    response = save(node)
    assert response.success
    assert str(tmp_path / "result.json") in response.message
    assert (tmp_path / "result.json").exists()
    assert not published
    assert node.aruco_poses_list == node.ee_poses_list == []
    assert node.record_data_cnt == 0
    assert node.cur_aruco_pose is node.cur_ee_pose is None
    assert not save(node).success


def test_successful_save_publishes_tf_when_enabled(tmp_path, monkeypatch):
    node = make_node(tmp_path, publish_tf=True)
    published = []
    node._publish_tf = lambda *args: published.append(args)
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        lambda *args: ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
    )

    assert save(node).success
    assert len(published) == 1
    assert node.aruco_poses_list == node.ee_poses_list == []
    assert node.record_data_cnt == 0
    assert node.cur_aruco_pose is node.cur_ee_pose is None


@pytest.mark.parametrize("failure", ["solver", "write", "publish"])
def test_failed_save_preserves_samples(tmp_path, monkeypatch, failure):
    node = make_node(tmp_path, publish_tf=failure == "publish")
    original_samples = list(node.aruco_poses_list)

    def fail(*args):
        raise RuntimeError("save failed")

    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        fail
        if failure == "solver"
        else lambda *args: ([1, 2, 3], [0, 0, 0, 1]),
    )
    if failure == "write":
        node.config.result_file = str(tmp_path / "missing" / "result.json")
    elif failure == "publish":
        node._publish_tf = fail
    response = save(node)
    assert not response.success
    if failure != "solver":
        assert node.config.result_file in response.message
    if failure == "publish":
        assert "saved" in response.message
        assert "TF publication failed" in response.message
        assert (tmp_path / "result.json").exists()
    assert node.aruco_poses_list == original_samples
    assert len(node.ee_poses_list) == node.record_data_cnt == 3
    assert node.cur_aruco_pose is not None
    assert node.cur_ee_pose is not None


def test_reset_data_clears_samples_and_preserves_result(tmp_path):
    node = make_node(tmp_path)
    result = tmp_path / "result.json"
    result.write_text("previous calibration")
    published = []
    node._publish_tf = lambda *args: published.append(args)
    for _ in range(2):
        response = node_module.CalibrationNode._reset_data_service_callback(
            node, None, SimpleNamespace(success=False, message="")
        )
        assert response.success
        assert node.aruco_poses_list == node.ee_poses_list == []
        assert node.record_data_cnt == 0
        assert node.cur_aruco_pose is node.cur_ee_pose is None
    assert result.read_text() == "previous calibration"
    assert not published


@pytest.mark.parametrize("suffix", [".json", ""])
def test_save_rejects_existing_file_without_renaming(
    tmp_path, monkeypatch, suffix
):
    existing = tmp_path / f"result{suffix}"
    existing.write_text("previous calibration")
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        lambda *args: ([1, 2, 3], [0, 0, 0, 1]),
    )
    node = make_node(tmp_path, publish_tf=True)
    node.config.result_file = str(existing)
    node._publish_tf = Mock()
    response = save(node)

    assert not response.success
    assert str(existing) in response.message
    assert node.config.result_file == str(existing)
    assert existing.read_text() == "previous calibration"
    assert list(tmp_path.iterdir()) == [existing]
    assert len(node.aruco_poses_list) == len(node.ee_poses_list) == 3
    assert node.record_data_cnt == 3
    assert node.cur_aruco_pose is not None
    assert node.cur_ee_pose is not None
    node._publish_tf.assert_not_called()


def test_save_allocates_a_utc_directory_for_each_session(
    tmp_path, monkeypatch
):
    output_root = tmp_path / "new" / "calibrations"
    instants = [
        datetime(2026, 9, 17, 8, 30, 25, microsecond, tzinfo=timezone.utc)
        for microsecond in (123456, 123457)
    ]
    clock = Mock()
    clock.now.side_effect = instants
    monkeypatch.setattr(node_module, "datetime", clock)
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        lambda *args: ([1, 2, 3], [0, 0, 0, 1]),
    )

    for instant in instants:
        node = make_node(tmp_path)
        node.config.result_file = None
        node.config.output_root = str(output_root)
        response = save(node)
        result = (
            output_root / instant.strftime("%Y%m%dT%H%M%S.%fZ") / "result.json"
        )

        assert response.success
        assert str(result) in response.message
        assert json.loads(result.read_text()) == {
            "parent_frame": "tool",
            "child_frame": "camera",
            "result": {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]},
        }
        assert node.config.result_file is None
        assert node.config.output_root == str(output_root)
        assert node.aruco_poses_list == node.ee_poses_list == []
        assert node.record_data_cnt == 0
        assert node.cur_aruco_pose is node.cur_ee_pose is None

    assert len(list(output_root.iterdir())) == 2
    assert clock.now.call_count == 2
    clock.now.assert_called_with(timezone.utc)


@pytest.mark.parametrize("has_result", [True, False])
def test_save_rejects_existing_timestamp_directory(
    tmp_path, monkeypatch, has_result
):
    instant = datetime(2026, 9, 17, 8, 30, 25, 123456, tzinfo=timezone.utc)
    directory = tmp_path / instant.strftime("%Y%m%dT%H%M%S.%fZ")
    directory.mkdir()
    result = directory / "result.json"
    if has_result:
        result.write_text("previous calibration")
    clock = Mock()
    clock.now.return_value = instant
    monkeypatch.setattr(node_module, "datetime", clock)
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        lambda *args: ([1, 2, 3], [0, 0, 0, 1]),
    )
    node = make_node(tmp_path)
    node.config.result_file = None
    node.config.output_root = str(tmp_path)
    response = save(node)

    assert not response.success
    assert str(directory) in response.message
    assert list(tmp_path.iterdir()) == [directory]
    if has_result:
        assert result.read_text() == "previous calibration"
    else:
        assert not result.exists()
    assert len(node.aruco_poses_list) == len(node.ee_poses_list) == 3
    assert node.record_data_cnt == 3
    assert node.cur_aruco_pose is not None
    assert node.cur_ee_pose is not None


@pytest.mark.parametrize("failure", ["insufficient_samples", "solver"])
def test_failed_calibration_does_not_allocate_output_directory(
    tmp_path, monkeypatch, failure
):
    output_root = tmp_path / "calibrations"
    node = make_node(tmp_path)
    node.config.result_file = None
    node.config.output_root = str(output_root)
    if failure == "insufficient_samples":
        node.aruco_poses_list.clear()
    monkeypatch.setattr(
        node_module,
        "run_eye_in_hand_calibration",
        Mock(side_effect=RuntimeError("solver failed")),
    )

    assert not save(node).success
    assert not output_root.exists()
