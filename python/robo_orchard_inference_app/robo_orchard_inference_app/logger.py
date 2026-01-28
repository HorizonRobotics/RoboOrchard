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

from typing import Literal

import streamlit as st

from robo_orchard_inference_app.state import AppState, LogMessage

__all__ = ["Logger"]


class Logger:
    def __init__(self, max_size: int = 128):
        self.max_size = max_size

    @property
    def app_state(self) -> AppState:
        return st.session_state.app_state

    def log(self, level: Literal["info", "warning", "error"], msg: str):
        if len(self.app_state.logs) >= self.max_size:
            self.app_state.logs.pop(
                0
            )  # Prune the oldest log to maintain max size

        log_entry = LogMessage(level=level, message=msg)
        self.app_state.logs.append(log_entry)

        # Provide immediate transient feedback
        icon = {"info": "✅", "warning": "⚠️", "error": "🚨"}.get(level)
        st.toast(msg, icon=icon)

    def info(self, msg: str):
        return self.log("info", msg)

    def warning(self, msg: str):
        return self.log("warning", msg)

    def warn(self, msg: str):
        return self.log("warning", msg)

    def error(self, msg: str):
        return self.log("error", msg)
