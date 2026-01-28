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

from abc import ABCMeta, abstractmethod

import roslibpy
import streamlit as st

from robo_orchard_inference_app.config import LaunchCfg, TaskCfg
from robo_orchard_inference_app.logger import Logger
from robo_orchard_inference_app.state import AppState, CollectingState


class ComponentBase(metaclass=ABCMeta):
    """Abstract base class for UI components."""

    def __init__(self, key_prefix: str | None = None):
        if key_prefix is None:
            key_prefix = self.__class__.__name__
        self.key_prefix = key_prefix

    @abstractmethod
    def __call__(self):
        """Abstract method to render the component."""
        pass

    # --- Configs ---
    @property
    def launch_cfg(self) -> LaunchCfg:
        return st.session_state.launch_cfg

    @property
    def app_cache_directory(self) -> str:
        return self.launch_cfg.app_cache_directory

    @property
    def task_cfg(self) -> TaskCfg:
        return st.session_state.task_cfg

    @task_cfg.setter
    def task_cfg(self, value: TaskCfg):  # type: ignore
        st.session_state.task_cfg = value

    # --- States ---
    @property
    def collecting_state(self) -> CollectingState:
        return st.session_state.collecting_state

    @property
    def app_state(self) -> AppState:
        return st.session_state.app_state

    # --- Global Objects ----
    @property
    def ros_client(self) -> roslibpy.Ros:
        return st.session_state.ros_client

    @property
    def logger(self) -> Logger:
        return st.session_state.logger

    @property
    def file_server_uri(self) -> str:
        return st.session_state.file_server_uri
