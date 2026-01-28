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

import uuid
from dataclasses import dataclass
from typing import Callable, List, Optional

import streamlit as st

from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.state import EpisodeMeta


@dataclass
class MetaRow:
    """Data class representing a single metadata row in the UI."""

    unique_id: str | None = None
    meta_key: str | None = None

    def __post_init__(self):
        if self.unique_id is None:
            self.unique_id = str(uuid.uuid4())


class EditEpisodeMetaComponent(ComponentBase):
    """Component for editing episode metadata in the RoboOrchard inference app.

    This component provides a UI for editing various metadata fields associated
    with an episode, including user name, task name, instruction, and custom
    metadata key-value pairs. It integrates with Streamlit and maintains
    consistency with the task configuration.

    Args:
        episode_meta (EpisodeMeta): The episode metadata to edit.
        rerun_callback (Callable, optional): Function to call when metadata
            changes. If None, defaults to st.rerun.
        *args: Additional positional arguments passed to ComponentBase.
        **kwargs: Additional keyword arguments passed to ComponentBase.
    """

    def __init__(
        self,
        episode_meta: EpisodeMeta,
        rerun_callback: Optional[Callable] = None,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.episode_meta = episode_meta
        self.meta_rows = [
            MetaRow(meta_key=key) for key in self.episode_meta.metas.keys()
        ]
        if rerun_callback is None:
            rerun_callback = st.rerun
        self.rerun_callback = rerun_callback

    def _render_user_name(self):
        """Render the user name selection UI component.

        Displays a selectbox for choosing the collector's user name from available
        options in the task configuration. Handles cases where the previously
        selected user is no longer available by resetting the selection and
        showing a warning.

        Updates the episode_meta.user_name when a new selection is made and
        triggers a rerun of the Streamlit app.
        """  # noqa: E501
        cols = st.columns([1, 3])
        with cols[0]:
            st.write("UserName")
        with cols[1]:
            options = list(self.task_cfg.available_collectors)
            if self.episode_meta.user_name:
                # handle the case that episode meta is set
                # but modify the task config
                if self.episode_meta.user_name in options:
                    index = options.index(self.episode_meta.user_name)
                else:
                    self.logger.warn(
                        "Available Collectors have been changed. Reset user name."  # noqa: E501
                    )
                    index = None
                    self.episode_meta.user_name = ""
            else:
                index = None
            user_name = st.selectbox(
                "Select collecting user",
                options=options,
                index=index,
                label_visibility="collapsed",
            )
            if (
                user_name is not None
                and user_name != self.episode_meta.user_name
            ):
                self.episode_meta.user_name = user_name
                self.rerun_callback()

    def _render_task_name(self):
        """Render the task name selection UI component.

        Displays a selectbox for choosing the task name from available tasks
        in the task configuration. When a task is selected, it resets the
        instruction field. Handles cases where the previously selected task
        is no longer available by resetting both task name and instruction
        and showing a warning.

        Updates the episode_meta.task_name and resets episode_meta.instruction
        when a new selection is made, then triggers a rerun of the Streamlit app.
        """  # noqa: E501
        cols = st.columns([1, 3])
        with cols[0]:
            st.write("TaskName")
        with cols[1]:
            options = list(self.task_cfg.available_tasks.keys())
            if self.episode_meta.task_name:
                # handle the case that episode meta is set
                # but modify the task config
                if self.episode_meta.task_name in options:
                    index = options.index(self.episode_meta.task_name)
                else:
                    self.logger.warn(
                        "Available Tasks have been changed. Reset task name and instruction."  # noqa: E501
                    )
                    index = None
                    self.episode_meta.task_name = ""
                    self.episode_meta.instruction = ""
            else:
                index = None
            task_name = st.selectbox(
                "Select task name",
                options=options,
                index=index,
                label_visibility="collapsed",
            )
            if (
                task_name is not None
                and task_name != self.episode_meta.task_name
            ):
                self.episode_meta.task_name = task_name
                self.episode_meta.instruction = ""  # reset instruction
                self.rerun_callback()

    def _render_instruction(self):
        """Render the instruction selection UI component.

        Displays a selectbox for choosing the instruction from available
        instructions for the currently selected task. The available options
        are dynamically determined based on the selected task name. Handles
        cases where the previously selected instruction is no longer available
        by resetting the selection and showing a warning.

        Updates the episode_meta.instruction when a new selection is made
        and triggers a rerun of the Streamlit app.
        """  # noqa: E501
        cols = st.columns([1, 3])
        with cols[0]:
            st.write("Instruction")
        with cols[1]:
            if self.episode_meta.task_name:
                options = self.task_cfg.available_tasks[
                    self.episode_meta.task_name
                ]
                if self.episode_meta.instruction in options:
                    index = options.index(self.episode_meta.instruction)
                elif self.episode_meta.instruction == "":
                    index = None
                else:
                    self.logger.warn(
                        "Available Instructions have been changed. Reset instruction."  # noqa: E501
                    )
                    index = None
                    self.episode_meta.instruction = ""
            else:
                options = []
                index = None
            instruction = st.selectbox(
                "Select instruction",
                options=options,
                index=index,
                label_visibility="collapsed",
                key="{}_select_{}_instruction".format(
                    self.key_prefix, self.collecting_state.task_name
                ),
            )
            if (
                instruction is not None
                and instruction != self.episode_meta.instruction
            ):
                self.episode_meta.instruction = instruction
                self.rerun_callback()

    def _get_meta_key_options(self, key: Optional[str]) -> List[str]:
        """Get available metadata key options for selection.

        Returns a list of metadata keys that can be selected, excluding keys
        that are already in use by other rows (to prevent duplication). If the
        current key is not in the available options but exists in the task
        configuration, it is included to maintain consistency.

        Args:
            key (Optional[str]): The current metadata key for this row, which
                should be preserved in options if valid.

        Returns:
            List[str]: Available metadata key options for selection.
        """  # noqa: E501
        all_meta_keys = []
        for meta_row in self.meta_rows:
            if meta_row.meta_key is not None:
                all_meta_keys.append(meta_row.meta_key)

        options = list(set(self.task_cfg.metas.keys()) - set(all_meta_keys))

        if key is not None and key not in options:
            if key in self.task_cfg.metas:
                options.append(key)

        return options

    def _render_metas(self):
        """Render the custom metadata key-value pairs UI component.

        Displays a dynamic interface for adding, editing, and removing custom
        metadata key-value pairs. Each row consists of a selectbox for choosing
        a metadata key, a multiselect for choosing values, and a delete button.
        Prevents duplicate keys across rows and validates against the task
        configuration. Handles cases where configured metadata keys or values
        have changed by discarding invalid entries and showing warnings.

        Provides an "Add meta" button to create new metadata rows and ensures
        proper cleanup when rows are deleted.
        """  # noqa: E501
        if st.button("Add meta", type="primary", use_container_width=True):
            self.meta_rows.append(MetaRow(meta_key=None))
            self.rerun_callback()

        # handle the case that metas and meta value have been modified!
        render_rows = []
        for meta_row in self.meta_rows:
            if meta_row.meta_key is not None:
                if meta_row.meta_key in self.task_cfg.metas:
                    render_rows.append(meta_row)
                    if meta_row.meta_key in self.episode_meta.metas:
                        current_meta_vals = []
                        for val in self.episode_meta.metas[meta_row.meta_key]:
                            if val in self.task_cfg.metas[meta_row.meta_key]:
                                current_meta_vals.append(val)
                            else:
                                self.logger.warn(
                                    f"Available values for meta key {meta_row.meta_key} have been changed. Discard the meta value: {val}"  # noqa: E501
                                )
                        self.episode_meta.metas[meta_row.meta_key] = (
                            current_meta_vals
                        )
                else:
                    self.logger.warn(
                        f"Available meta keys have been changed. Discard meta {meta_row.meta_key}"  # noqa: E501
                    )

        for idx, meta_row in enumerate(self.meta_rows):
            with st.container(border=True):
                cols = st.columns([1.5, 3, 1])
                row_uuid = meta_row.unique_id

                with cols[0]:
                    options = self._get_meta_key_options(meta_row.meta_key)
                    if meta_row.meta_key in options:
                        index = options.index(meta_row.meta_key)
                    else:
                        index = None
                    meta_key = st.selectbox(
                        "Select meta key",
                        options=options,
                        index=index,
                        key=f"{self.key_prefix}_select_meta_key_{row_uuid}",
                        label_visibility="collapsed",
                    )
                    if meta_key is not None:
                        if meta_key not in self.episode_meta.metas:
                            self.episode_meta.metas[meta_key] = []
                        if meta_key != meta_row.meta_key:
                            meta_row.meta_key = meta_key
                            self.rerun_callback()

                with cols[1]:
                    if meta_key is None:
                        options = []
                        default = None
                    else:
                        options = self.task_cfg.metas[meta_key]
                        default = self.episode_meta.metas[meta_key]
                    meta_vals = st.multiselect(
                        "Select meta values",
                        options=options,
                        default=default,
                        key=f"{self.key_prefix}_select_{meta_key}_value_{row_uuid}",
                        label_visibility="collapsed",
                    )
                    if (
                        meta_key is not None
                        and meta_vals != self.episode_meta.metas[meta_key]
                    ):
                        self.episode_meta.metas[meta_key] = meta_vals
                        self.rerun_callback()

                with cols[2]:
                    if st.button(
                        "🗑️",
                        key=f"{self.key_prefix}_del_btn_{row_uuid}",
                        help="Delete",
                    ):
                        self.meta_rows.pop(idx)
                        if meta_key is not None:
                            self.episode_meta.metas.pop(meta_key)
                        self.rerun_callback()

    def __call__(self):
        """Render the complete episode metadata editing interface.

        Calls all individual rendering methods to display the full UI for
        editing episode metadata, including user name, task name, instruction,
        and custom metadata key-value pairs.
        """
        self._render_user_name()
        self._render_task_name()
        self._render_instruction()
        self._render_metas()
