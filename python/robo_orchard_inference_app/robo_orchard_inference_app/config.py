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

import os
from pathlib import Path
from typing import Dict, List, Literal
from urllib.parse import urlencode

import pydantic

__all__ = [
    "LaunchCfg",
    "MITControlTuningCfg",
    "ScriptedMotionCfg",
    "TaskCfg",
]


def _resolve_default_gravity_urdf_path() -> str:
    """The dual-arm gravity URDF path.

    Written exactly once, in configs/piper_urdf_path.txt (see
    calibration/paths.py); resolved here via the PIPER_GRAVITY_URDF_PATH
    env override, then the repo the package runs from (editable install
    or source tree), then the container mount.
    """
    env = os.environ.get("PIPER_GRAVITY_URDF_PATH")
    if env:
        return env
    candidates = [
        parent / "configs" / "piper_urdf_path.txt"
        for parent in Path(__file__).resolve().parents
    ]
    candidates.append(Path("/opt/roboorchard/configs/piper_urdf_path.txt"))
    for candidate in candidates:
        try:
            return candidate.read_text().strip()
        except OSError:
            continue
    raise RuntimeError(
        "Cannot resolve the default gravity URDF path: set "
        "PIPER_GRAVITY_URDF_PATH or make configs/piper_urdf_path.txt "
        "reachable (repo checkout or /opt/roboorchard mount)."
    )


DEFAULT_GRAVITY_URDF_PATH = _resolve_default_gravity_urdf_path()


class FoxgloveCfg(pydantic.BaseModel):
    """Configuration for Foxglove visualization settings."""

    host: str = "https://app.foxglove.dev"
    """Base host URL for Foxglove."""

    remote_file_layout_id: str | None = None
    """Layout ID for remote file view."""

    remote_file_endpoint_template: str = "{host}/view?{query}"
    """A format string for the remote file endpoint."""

    websocket_layout_id: str | None = None
    """Layout ID for websocket view."""

    websocket_ds: str = "foxglove-websocket"
    """websocket ds."""

    websocket_url: str = "ws://localhost:8765"
    """Websocket connection URL."""

    width: int | None = None
    """Display width in pixels."""

    height: int | None = 960
    """Display height in pixels."""

    display_type: Literal["iframe", "link_button"] = "iframe"
    """Display type."""

    def get_websocket_url(self) -> str:
        """Generates a URL for websocket connection to Foxglove.

        Returns:
            str: Complete URL with query parameters for websocket connection
        """

        param = {
            "ds": self.websocket_ds,
            "ds.url": self.websocket_url,
        }
        if self.websocket_layout_id is not None:
            param["layout"] = self.websocket_layout_id
        url = "{}/?{}".format(self.host, urlencode(param))
        return url

    def get_remote_file_url(self, remote_file: str) -> str:
        """Generates a URL for viewing a remote file in Foxglove.

        Args:
            remote_file (str): URL or path to the remote file

        Returns:
            str: Complete URL with query parameters for remote file viewing
        """

        param = {
            "ds": "remote-file",
            "ds.url": remote_file,
        }
        if self.remote_file_layout_id is not None:
            param["layout"] = self.remote_file_layout_id
        url = self.remote_file_endpoint_template.format(
            host=self.host, query=urlencode(param)
        )
        return url


class MITControlTuningCfg(pydantic.BaseModel):
    master_param_node_names: list[str] = pydantic.Field(default_factory=list)
    follower_param_node_names: list[str] = pydantic.Field(default_factory=list)
    default_master_enabled: bool = True
    default_follower_enabled: bool = True
    default_master_kp: float = 10.0
    default_master_kd: float = 0.8
    default_master_vel_ref: float = 45.0
    default_master_torque_ref: float = 0.0
    # When enabled the UI forces master kp/kd to 0 and lets the node feed the
    # rnea(q, 0, 0) gravity torque as the MIT feedforward torque, so the master
    # arm floats (gravity-compensated, fully backdrivable).
    default_master_gravity_compensation_enabled: bool = False
    default_master_gravity_compensation_urdf_path: str = (
        DEFAULT_GRAVITY_URDF_PATH
    )
    default_master_gravity_compensation_scale: float = 1.0
    # Per-joint multiplier applied on top of the global scale.
    default_master_gravity_compensation_scale_per_joint: list[float] = (
        pydantic.Field(default_factory=lambda: [1.0] * 6)
    )
    default_master_gravity_compensation_max_t_ref: float = 8.0
    # Master friction compensation (open_manipulator-style): a per-joint torque
    # added in the direction of measured motion to cancel Coulomb/stiction so
    # the floating master backdrives smoothly. Disabled by default.
    default_master_friction_compensation_enabled: bool = False
    default_master_friction_compensation_scale: list[float] = pydantic.Field(
        default_factory=lambda: [0.0] * 6
    )
    default_master_friction_compensation_load_scale: float = 0.0
    default_master_friction_compensation_min_velocity: float = 0.02
    default_master_friction_compensation_taper_velocity: float = 2.0
    # The deflection tables in the calibration store
    # (calibration/deflection_calibrations.json) are calibrated at
    # mit_kp=25; keep this in sync (see calibration/CALIBRATION.md).
    default_follower_kp: float = 25.0
    default_follower_kd: float = 0.8
    default_follower_vel_ref: float = 45.0
    default_follower_torque_ref: float = 0.0
    # Opt-in via the app's "Velocity feedforward (follower)" checkbox.
    # Hardware-characterized 2026-07-06: halves
    # follower phase lag when enabled. Default on since 2026-07-08:
    # without it the follower lags the command by ~50 ms vs ~16 ms.
    default_follower_velocity_feedforward: bool = True
    default_follower_gravity_compensation_alpha: float = 0.0
    default_follower_gravity_compensation_enabled: bool = False
    default_follower_gravity_compensation_urdf_path: str = (
        DEFAULT_GRAVITY_URDF_PATH
    )
    default_follower_gravity_compensation_scale: float = 1.0
    # Per-joint multiplier applied on top of the global scale.
    default_follower_gravity_compensation_scale_per_joint: list[float] = (
        pydantic.Field(default_factory=lambda: [1.0] * 6)
    )
    default_follower_gravity_compensation_max_t_ref: float = 8.0


class ScriptedMotionCfg(pydantic.BaseModel):
    command: list[str] = pydantic.Field(
        default_factory=lambda: [
            "ros2",
            "run",
            "robo_orchard_teleop_ros2",
            "scripted_joint_master",
        ]
    )
    left_command_topic: str = "/left_algo_cmd"
    right_command_topic: str = "/right_algo_cmd"
    left_state_topic: str = "/puppet/joint_left"
    right_state_topic: str = "/puppet/joint_right"
    publish_left: bool = True
    publish_right: bool = True
    use_current_state: bool = True
    mirror_right: bool = False
    rate_hz: float = 100.0
    start_delay_s: float = 1.0
    recording_command_min_subscribers: int = 2
    """Required command-topic subscribers before starting a recorded
    prelaunched scripted motion after its trigger file appears. This lets the
    recorder discover command publishers before the first command sample."""
    recording_command_subscriber_wait_timeout_s: float = 2.0
    reset_position: list[float] = pydantic.Field(default_factory=list)
    """Reset pose (7 joint values) used when stopping scripted motion;
    empty list falls back to the standard arm reset pose."""
    scenario_directory: str = ""
    """Optional override for the packaged robot-eval scenario registry."""


class ROSBridgeCfg(pydantic.BaseModel):
    host: str = "localhost"

    port: int = 9090

    # take over
    takeover_service_name: list[str] = pydantic.Field(default_factory=list)
    release_service_name: list[str] = pydantic.Field(default_factory=list)
    stop_service_name: list[str] = pydantic.Field(default_factory=list)

    # inference
    enable_inference_service_name: list[str] = pydantic.Field(
        default_factory=list
    )
    disable_inference_service_name: list[str] = pydantic.Field(
        default_factory=list
    )

    inference_node_candidates: list[str] = pydantic.Field(default_factory=list)

    # arm control
    mit_control: MITControlTuningCfg = pydantic.Field(
        default_factory=MITControlTuningCfg
    )
    enable_arm_service_name: list[str] = pydantic.Field(default_factory=list)
    reset_arm_service_name: list[str] = pydantic.Field(default_factory=list)
    reset_position: list[float] = pydantic.Field(default_factory=list)
    """Reset pose (7 joint values) used by the standard arm reset; empty
    list means the controllers' default (zeros)."""

    # recorder
    recorder_name: str = "/mcap_recorder_service"

    # hand-eye calib
    record_handeye_calib_service_name: str | None = None
    save_handeye_calib_service_name: str | None = None
    static_transform_service_name: str | None = None


class UIControlCfg(pydantic.BaseModel):
    start_keyboard: str | None = "shift+s"
    stop_keyboard: str | None = "shift+f"


class LaunchCfg(pydantic.BaseModel):
    """Configuration for launch settings."""

    workspace: str = ".workspace/"
    """Working directory."""

    foxglove: FoxgloveCfg = pydantic.Field(default_factory=FoxgloveCfg)
    """Foxglove configuration."""

    app_cache_directory: str = ".cache/robo_orchard_inference_app"
    """Directory for application cache."""

    ros_bridge: ROSBridgeCfg = pydantic.Field(
        default_factory=ROSBridgeCfg,
    )
    """ROS bridge config."""

    file_server_uri: str | None = None
    """URI for file server."""

    ui_control: UIControlCfg = pydantic.Field(default_factory=UIControlCfg)
    """UIControl configuration."""

    scripted_motion: ScriptedMotionCfg = pydantic.Field(
        default_factory=ScriptedMotionCfg
    )
    """Scripted motion launch configuration."""


class TaskCfg(pydantic.BaseModel):
    """Configuration for task settings."""

    available_collectors: List[str] = pydantic.Field(default_factory=list)
    """List of available data collectors."""

    available_tasks: Dict[str, List[str]] = pydantic.Field(
        default_factory=dict
    )
    """Dictionary mapping task names to instruction lists."""

    candidate_tf_directories: List[str] = pydantic.Field(default_factory=list)

    metas: dict[str, list[str]] = pydantic.Field(default_factory=dict)
    """Dictionary mapping customize key to values."""
