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

import streamlit as st
from streamlit_tags import st_tags

from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.utils import time_str_now


class TaskConfigComponent(ComponentBase):
    """Component for managing task configuration."""

    def _dump_task_cfg_to_disk(self):
        """Saves the current task configuration to the cache directory."""
        save_dir = os.path.join(
            self.app_cache_directory, "task_configs", time_str_now()
        )
        os.makedirs(save_dir, exist_ok=True)
        json_str = self.task_cfg.model_dump_json(indent=4)

        with open(os.path.join(save_dir, "app_task.json"), "w") as f:
            f.write(json_str)
        with open(
            os.path.join(
                self.app_cache_directory, "task_configs", "latest.json"
            ),
            "w",
        ) as f:
            f.write(json_str)

    def sync_dict_keys(self, data_dict: dict, new_keys: list[str]):
        # 1. remove key
        keys_to_remove = [k for k in data_dict.keys() if k not in new_keys]
        for k in keys_to_remove:
            del data_dict[k]

        # 2. add key
        for k in new_keys:
            if k not in data_dict:
                data_dict[k] = []

        return data_dict

    def _render_username(self):
        new_collectors = st_tags(
            label="Available Users",
            text="Enter to add user name...",
            value=self.task_cfg.available_collectors,
            suggestions=[],
            key=f"{self.key_prefix}_render_username",
        )
        if new_collectors != self.task_cfg.available_collectors:
            self.task_cfg.available_collectors = new_collectors
            self._dump_task_cfg_to_disk()
            st.rerun()

    def _render_tasks(self):
        current_task_keys = list(self.task_cfg.available_tasks.keys())

        updated_task_keys = st_tags(
            label="Available Tasks",
            text="Enter to add task...",
            value=current_task_keys,
            suggestions=[],
            key=f"{self.key_prefix}_render_tasks",
        )

        if updated_task_keys != current_task_keys:
            self.sync_dict_keys(
                self.task_cfg.available_tasks, updated_task_keys
            )
            self._dump_task_cfg_to_disk()
            st.rerun()

    def _render_instructions(self):
        task_options = list(self.task_cfg.available_tasks.keys())

        if task_options:
            cols = st.columns([0.2, 0.8])

            with cols[0]:
                st.write("Select Task")
                selected_task = st.selectbox(
                    "Select Task",
                    options=task_options,
                    label_visibility="collapsed",
                )

            with cols[1]:
                updated_instrs = st_tags(
                    label=f"Available instructions for task [{selected_task}]",
                    text="Enter instruction...",
                    value=self.task_cfg.available_tasks[selected_task],
                    suggestions=[],
                    key=f"{self.key_prefix}_render_{selected_task}_instructions",
                )
                if (
                    updated_instrs
                    != self.task_cfg.available_tasks[selected_task]
                ):
                    self.task_cfg.available_tasks[selected_task] = (
                        updated_instrs
                    )
                    self._dump_task_cfg_to_disk()
                    st.rerun()
        else:
            st.write("Please add **task** first.")

    def _render_meta_keys(self):
        current_meta_keys = list(self.task_cfg.metas.keys())

        updated_meta_keys = st_tags(
            label="Available Metas",
            text="Enter meta key...",
            value=current_meta_keys,
            suggestions=[],
            key=f"{self.key_prefix}_render_meta_keys",
        )

        if updated_meta_keys != current_meta_keys:
            self.sync_dict_keys(self.task_cfg.metas, updated_meta_keys)
            self._dump_task_cfg_to_disk()
            st.rerun()

    def _render_metas(self):
        meta_options = list(self.task_cfg.metas.keys())

        if meta_options:
            cols = st.columns([0.2, 0.8])

            with cols[0]:
                st.write("Select Meta Key")
                selected_meta_key = st.selectbox(
                    "Select Meta Key",
                    options=meta_options,
                    label_visibility="collapsed",
                )
            with cols[1]:
                updated_meta_vals = st_tags(
                    label=f"Available values for meta key [{selected_meta_key}]",  # noqa: E501
                    text="Enter value...",
                    value=self.task_cfg.metas[selected_meta_key],
                    suggestions=[],
                    key=f"{self.key_prefix}_render_{selected_meta_key}_values",
                )
                if updated_meta_vals != self.task_cfg.metas[selected_meta_key]:
                    self.task_cfg.metas[selected_meta_key] = updated_meta_vals
                    self._dump_task_cfg_to_disk()
                    st.rerun()
        else:
            st.write("Please add **meta key** first")

    def __call__(self):
        """Renders the task configuration management UI."""
        with st.expander("⚙️ Task Configuration", expanded=True):
            self._render_username()
            self._render_tasks()
            self._render_meta_keys()
            st.divider()
            self._render_instructions()
            self._render_metas()
