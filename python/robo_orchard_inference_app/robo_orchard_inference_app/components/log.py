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

import streamlit as st

from robo_orchard_inference_app.components.mixin import ComponentBase


class LogComponent(ComponentBase):
    def __call__(self):
        """Renders the log panel UI."""
        with st.expander("📝 Log Panel", expanded=False):
            # Button to clear the logs
            if st.button("Clear Logs", use_container_width=True):
                self.app_state.logs.clear()
                st.rerun()

            # Create a container for the logs with a
            # specific height and scrollbar
            log_container = st.container(height=256)

            # Display logs in reverse chronological order (newest first)
            for log in reversed(self.app_state.logs):
                timestamp = log.timestamp.strftime("%H:%M:%S")
                message = f"[{timestamp}] {log.message}"

                if log.level == "info":
                    log_container.info(message, icon="✅")
                elif log.level == "warning":
                    log_container.warning(message, icon="⚠️")
                else:  # error
                    log_container.error(message, icon="🚨")
