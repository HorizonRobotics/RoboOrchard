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
from streamlit.components.v1 import iframe

from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.config import FoxgloveCfg


class FoxgloveVisualComponent(ComponentBase):
    """Component for displaying Foxglove."""

    @property
    def cfg(self) -> FoxgloveCfg:
        return self.launch_cfg.foxglove

    def _render_link_button(self):
        css_html = """
        <style>
            /* Target the container for the columns */
            div[data-testid="stHorizontalBlock"] {
                /* Make the columns themselves align to the center vertically */
                align-items: center;
            }

            /* Optional: if you still want some space on top */
            .st-emotion-cache-z5fcl4 {
                padding-top: 2rem;
            }

            /* Style for the button, now without border and with a link-like look */
            .custom-button {
                border: none;
                padding: 0.5rem 1rem;
                border-radius: 0.25rem;
                background-color: transparent;
                color: #0068c9;
                text-align: center;
                text-decoration: none;
                font-size: 1rem;
                font-weight: 600;
                cursor: pointer;
                transition: background-color 0.3s;
            }
            .custom-button:hover {
                background-color: #f0f2f6;
            }
        </style>
        """  # noqa: E501
        st.markdown(css_html, unsafe_allow_html=True)

        button_html = f"""
        <div style="display: flex; justify-content: center;">
            <a href="{self.cfg.get_websocket_url()}" target="_blank" class="custom-button">
                🚀 Launch Visualization
            </a>
        </div>
        """  # noqa: E501
        st.markdown(button_html, unsafe_allow_html=True)

    def __call__(self):
        """Renders the Foxglove iframe."""
        with st.spinner("Loading Foxglove"):
            if self.cfg.display_type == "iframe":
                iframe(
                    src=self.cfg.get_websocket_url(),
                    width=self.cfg.width,
                    height=self.cfg.height,
                )
            elif self.cfg.display_type == "link_button":
                self._render_link_button()
            else:
                raise NotImplementedError(
                    f"Invalid display type: {self.cfg.display_type}"
                )
