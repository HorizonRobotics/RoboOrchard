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
from dataclasses import dataclass

import streamlit as st

from robo_orchard_inference_app.components.edit_episode_meta import (
    EditEpisodeMetaComponent,
    EpisodeMeta,
)
from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.config import FoxgloveCfg
from robo_orchard_inference_app.utils import remove_path


@dataclass
class EditInfo:
    episode_meta_file: str
    panel: EditEpisodeMetaComponent

    @property
    def episode_meta(self) -> EpisodeMeta:
        return self.panel.episode_meta


class SideBarComponent(ComponentBase):
    """Sidebar component for managing recorded files."""

    def __init__(
        self,
    ):
        super().__init__()
        self._selected_uri: str | None = None
        self._delete_flags = dict()
        self._edit_info: EditInfo | None = None

    @property
    def foxglove_cfg(self) -> FoxgloveCfg:
        return self.launch_cfg.foxglove

    def _delete_callback(self, uri: str):
        if self._delete_flags.get(uri, False):
            self.logger.warn(f":red[Already tried deleting {uri}]")
        try:
            with st.spinner("Deleting"):
                remove_path(uri)
            self._delete_flags[uri] = True
            self.collecting_state.episode_counter.sub()
            self._selected_uri = None  # reset selected state
        except FileNotFoundError:
            self.logger.error(f"Cannot found recording uri: {uri}")

    @st.dialog("Update Episode Meta", width="large", dismissible=False)
    def _update_meta_panel(self, edit_info: EditInfo):
        self._edit_info = None

        def rerun_callback():
            self._edit_info = edit_info
            st.rerun()

        edit_info.panel()

        cols = st.columns(2)
        if cols[0].button("Confirm", use_container_width=True, type="primary"):
            with open(edit_info.episode_meta_file, "w") as fh:
                fh.write(edit_info.episode_meta.model_dump_json(indent=4))
            st.rerun()
        if cols[1].button("Cancel", use_container_width=True):
            st.rerun()

    def _if_has_episode_meta(self, episode_meta_file: str):
        with open(episode_meta_file, "r") as fh:
            episode_meta: EpisodeMeta = EpisodeMeta.model_validate_json(
                fh.read()
            )

        edit_info = EditInfo(
            episode_meta_file=episode_meta_file,
            panel=EditEpisodeMetaComponent(
                episode_meta=episode_meta,
                rerun_callback=None,
                key_prefix=f"{self.key_prefix}_configure",
            ),
        )

        def _rerun_callback():
            self._edit_info = edit_info
            st.rerun()

        edit_info.panel.rerun_callback = _rerun_callback

        self._update_meta_panel(edit_info)

    def _render_recording_files_manage_panel(self):
        st.header("📁 Data Management")

        st.sidebar.markdown(f"User: {self.collecting_state.user_name}")
        st.sidebar.markdown(f"Task: {self.collecting_state.task_name}")

        if not self.collecting_state.is_configured or not os.path.exists(
            self.collecting_state.data_root
        ):
            return

        all_collecting_data = list(
            sorted(os.listdir(self.collecting_state.data_root))
        )

        st.sidebar.markdown(f"Total episodes: {len(all_collecting_data)}")

        for idx, uri in enumerate(all_collecting_data):
            with st.container():
                if st.button(uri):
                    if self._selected_uri == uri:
                        self._selected_uri = None
                    else:
                        self._selected_uri = uri

                if self._selected_uri == uri:
                    data_uri = os.path.join(
                        self.collecting_state.data_root, uri
                    )

                    cols = st.columns(2)

                    with cols[0]:
                        st.link_button(
                            "Visualize",
                            url=self.foxglove_cfg.get_remote_file_url(
                                remote_file="{}/{}".format(
                                    self.file_server_uri,
                                    os.path.abspath(
                                        os.path.join(data_uri, f"{uri}_0.mcap")
                                    ),
                                )
                            ),
                        )

                        st.link_button(
                            "Browse",
                            url="{}/{}".format(
                                self.file_server_uri, os.path.abspath(data_uri)
                            ),
                        )

                    with cols[1]:
                        if st.button(
                            "Delete",
                            key=f"{self.key_prefix}_delete_{idx}",
                        ):
                            self._delete_callback(data_uri)
                            st.rerun()

                        if st.button(
                            "Edit Meta",
                            key=f"{self.key_prefix}_update_meta_{idx}",
                        ):
                            episode_meta_file = os.path.join(
                                data_uri, "episode_meta.json"
                            )
                            if os.path.exists(episode_meta_file):
                                self._if_has_episode_meta(episode_meta_file)
                            else:
                                self.logger.error(
                                    "No episode meta file found!"
                                )
                                st.rerun()

    def __call__(self):
        """Renders the sidebar UI."""

        with st.sidebar:
            self._render_recording_files_manage_panel()

            if self._edit_info is not None:
                self._update_meta_panel(self._edit_info)
                self._edit_info = None
