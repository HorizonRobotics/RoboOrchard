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

from typing import Dict, List, Literal
from urllib.parse import urlencode

import pydantic

__all__ = ["LaunchCfg", "TaskCfg"]


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
    enable_arm_service_name: list[str] = pydantic.Field(default_factory=list)
    disable_arm_service_name: list[str] = pydantic.Field(default_factory=list)
    reset_arm_service_name: list[str] = pydantic.Field(default_factory=list)

    # recorder
    recorder_name: str = "/mcap_recorder_service"

    # hand-eye calib
    record_handeye_calib_service_name: str | None = None
    save_handeye_calib_service_name: str | None = None


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


class TaskCfg(pydantic.BaseModel):
    """Configuration for task settings."""

    available_collectors: List[str] = pydantic.Field(default_factory=list)
    """List of available data collectors."""

    available_tasks: Dict[str, List[str]] = pydantic.Field(
        default_factory=dict
    )
    """Dictionary mapping task names to instruction lists."""

    metas: dict[str, list[str]] = pydantic.Field(default_factory=dict)
    """Dictionary mapping customize key to values."""
