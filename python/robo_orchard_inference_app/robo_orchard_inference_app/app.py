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


import argparse
import os
import subprocess
from typing import Tuple

import streamlit as st

from robo_orchard_inference_app.components import (
    FoxgloveVisualComponent,
    LogComponent,
    MainControlComponent,
    SideBarComponent,
    TaskConfigComponent,
)
from robo_orchard_inference_app.config import LaunchCfg, TaskCfg
from robo_orchard_inference_app.logger import Logger
from robo_orchard_inference_app.state import AppState, CollectingState
from robo_orchard_inference_app.utils import (
    create_ros_bridge_client,
    start_process,
    time_str_now,
)


@st.cache_resource
def file_server() -> str:
    """Starts a file server on a free port and returns its URL.

    Returns:
        str: URL of the started file server (e.g., "http://localhost:port")

    Raises:
        subprocess.CalledProcessError: If the file server fails to start
    """

    from robo_orchard_core.utils.network import find_free_port

    port: int = find_free_port()

    try:
        start_process(
            f"robo-orchard file-server --port {port} --dir /",
            shell=True,
            executable="/bin/bash",
        )
        return f"http://localhost:{port}"
    except subprocess.CalledProcessError as e:
        print(
            "Failed to launch file server at port {}. Error msgs:\n{}\n".format(  # noqa: E501
                port, e.stderr.decode("utf-8")
            )
        )
        raise


@st.cache_resource
def get_app_start_timestr() -> str:
    return time_str_now()


def parse_args() -> Tuple[LaunchCfg, TaskCfg]:
    """Parses command-line arguments and loads configuration files.

    Returns:
        Tuple[LaunchCfg, TaskCfg]: Tuple containing launch and task
            configurations.

    Raises:
        FileNotFoundError: If required configuration files are not found.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--launch-config", type=str, required=True, help="app launch config"
    )
    parser.add_argument("--task-config", type=str, default=None)
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Whether in DEBUG mode.",
    )
    args = parser.parse_args()

    if not os.path.exists(args.launch_config):
        raise FileNotFoundError(
            f"Cannot find launch config: {args.launch_config}"
        )
    with open(args.launch_config, "r") as fr:
        launch_cfg: LaunchCfg = LaunchCfg.model_validate_json(fr.read())

    if args.debug:
        launch_cfg.app_cache_directory = os.path.join(
            launch_cfg.app_cache_directory, "DEBUG"
        )

    if args.task_config is None:
        latest_cfg = os.path.join(
            launch_cfg.app_cache_directory, "task_configs", "latest.json"
        )
        if os.path.exists(latest_cfg):
            with open(latest_cfg, "r") as fr:
                app_task_cfg: TaskCfg = TaskCfg.model_validate_json(fr.read())
        else:
            app_task_cfg = TaskCfg()
    else:
        if not os.path.exists(args.task_config):
            raise FileNotFoundError(
                f"task config: {args.task_config} does not exists!"
            )
        with open(args.task_config, "r") as fh:
            app_task_cfg: TaskCfg = TaskCfg.model_validate_json(fh.read())

    if args.debug:
        if not launch_cfg.workspace.endswith("DEBUG"):
            launch_cfg.workspace = os.path.join(launch_cfg.workspace, "DEBUG")

    return launch_cfg, app_task_cfg


def _st_init():
    """Initializes Streamlit session state with application components.

    Notes:
        - Only runs if 'st_is_initialize' is not in session state
        - Sets up all necessary components and configurations
    """

    if "st_is_initialize" in st.session_state:
        return

    launch_cfg, task_cfg = parse_args()

    st.session_state.launch_cfg = launch_cfg
    st.session_state.task_cfg = task_cfg
    st.session_state.collecting_state = CollectingState(
        session_time_str=get_app_start_timestr()
    )
    st.session_state.app_state = AppState()
    st.session_state.logger = Logger()

    st.session_state.ros_client = create_ros_bridge_client(
        host=st.session_state.launch_cfg.ros_bridge.host,
        port=st.session_state.launch_cfg.ros_bridge.port,
    )

    if st.session_state.launch_cfg.file_server_uri is None:
        st.session_state.file_server_uri = file_server()
    else:
        st.session_state.file_server_uri = (
            st.session_state.launch_cfg.file_server_uri
        )

    st.session_state.task_config_panel = TaskConfigComponent()
    st.session_state.log_panel = LogComponent()
    st.session_state.main_control_panel = MainControlComponent()
    st.session_state.side_bar_panel = SideBarComponent()
    st.session_state.foxglove_panel = FoxgloveVisualComponent()

    st.session_state.st_is_initialize = True


def main_page():
    """Sets up and renders the main Streamlit application page.

    Notes:
        - Configures page layout and title
        - Initializes session state
        - Renders all UI components in a two-column layout
    """
    st.set_page_config(
        layout="wide",
        page_title="Horizon Robotics Lab Inference App",
        initial_sidebar_state="collapsed",
    )

    _st_init()

    st.session_state.side_bar_panel()

    cols = st.columns([4, 2])
    with cols[0]:
        st.session_state.task_config_panel()
        st.session_state.foxglove_panel()
    with cols[1]:
        st.session_state.log_panel()
        st.session_state.main_control_panel()


if __name__ == "__main__":
    main_page()
