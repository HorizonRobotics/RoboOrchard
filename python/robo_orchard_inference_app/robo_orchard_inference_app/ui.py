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

from typing import Hashable

import pydantic
import streamlit as st


class StatusConfig(pydantic.BaseModel):
    text: str
    color: str


def multi_status_indicator(
    current_status: Hashable,
    status_config: dict[Hashable, StatusConfig],
    default_config: StatusConfig | None = None,
    size_px: int = 20,
):
    """Displays a status indicator for multiple, named states.

    This function is configuration-driven. You define all possible states
    and their corresponding text and color in the `status_map` dictionary.

    Args:
        current_status (str): The key of the current status to display.
                              Must exist as a key in `status_map`.
        status_config (dict[Hashable, StatusConfig]):
            A dictionary that maps status keys to their properties.
        default_config: StatusConfig | None:
            Default status config for fallback.
        size_px (int): The diameter of the indicator circle in pixels.
    """

    if default_config is None:
        default_config = StatusConfig(text="Unknown", color="grey")

    config = status_config.get(current_status, default_config)

    html_content = f"""
    <div style="display: flex; align-items: center; gap: 8px;">
        <div style="
            width: {size_px}px;
            height: {size_px}px;
            background-color: {config.color};
            border-radius: 50%;
        "></div>
        <span style="font-size: 1.1rem; font-weight: 600;">{config.text}</span>
    </div>
    """
    st.markdown(html_content, unsafe_allow_html=True)
