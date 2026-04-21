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

import sys
import types

roslibpy = types.SimpleNamespace(
    Service=None,
    ServiceRequest=lambda data=None: types.SimpleNamespace(data=data),
    Ros=object,
    Topic=object,
    core=types.SimpleNamespace(RosTimeoutError=RuntimeError),
)
sys.modules["roslibpy"] = roslibpy
sys.modules["robo_orchard_inference_app.version"] = types.SimpleNamespace(
    __full_version__="0.0.0",
    __git_hash__="test",
    __version__="0.0.0",
)
sys.modules["streamlit"] = types.SimpleNamespace(
    session_state=types.SimpleNamespace(
        app_state=types.SimpleNamespace(logs=[])
    ),
    toast=lambda *args, **kwargs: None,
    cache_resource=lambda fn: fn,
)
sys.modules["psutil"] = types.SimpleNamespace(
    signal=types.SimpleNamespace(SIGINT=2, SIGKILL=9),
    pid_exists=lambda pid: False,
    Process=object,
    NoSuchProcess=RuntimeError,
)

from robo_orchard_inference_app.config import ROSBridgeCfg  # noqa: E402
from robo_orchard_inference_app.ros_bridge import (  # noqa: E402
    RosServiceHelper,
)
from robo_orchard_inference_app.state import (  # noqa: E402
    EpisodeMeta,
    InferenceState,
)


class DummyLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, msg: str):
        self.infos.append(msg)

    def error(self, msg: str):
        self.errors.append(msg)


class DummyRosClient:
    is_connected = True

    def get_services(self):
        return ["/set_static_transforms"]


def _make_helper(monkeypatch, calls, service_name="/set_static_transforms"):
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    class DummyService:
        def __init__(self, ros_client, service_name, service_type):
            self.service_name = service_name
            self.service_type = service_type

        def call(self, request, timeout=0):
            calls.append(
                {
                    "service_name": self.service_name,
                    "service_type": self.service_type,
                    "request": request.data,
                }
            )
            return {"success": True, "message": "ok"}

    monkeypatch.setattr(ros_bridge_module.roslibpy, "Service", DummyService)
    monkeypatch.setattr(
        ros_bridge_module.roslibpy,
        "ServiceRequest",
        lambda data=None: types.SimpleNamespace(data=data),
        raising=False,
    )
    return RosServiceHelper(
        ros_client=DummyRosClient(),
        ros_bridge_cfg=ROSBridgeCfg(
            static_transform_service_name=service_name
        ),
        inference_state=InferenceState(),
        logger=DummyLogger(),
    )


def test_sync_static_transforms_skips_when_directory_empty(monkeypatch):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    episode_meta = EpisodeMeta(tf_directory="")

    result = helper.sync_static_transforms(episode_meta)

    assert result is True
    assert calls == []


def test_sync_static_transforms_calls_service_on_first_sync(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    (tmp_path / "left.json").write_text("{}")
    episode_meta = EpisodeMeta(tf_directory=str(tmp_path))

    result = helper.sync_static_transforms(episode_meta)

    assert result is True
    assert len(calls) == 1
    assert calls[0]["service_type"] == (
        "robo_orchard_data_msg_ros2/srv/SetStaticTransforms"
    )
    assert calls[0]["request"] == {"directory": str(tmp_path)}


def test_sync_static_transforms_skips_when_fingerprint_unchanged(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    (tmp_path / "left.json").write_text("{}")
    episode_meta = EpisodeMeta(tf_directory=str(tmp_path))

    helper.sync_static_transforms(episode_meta)
    helper.sync_static_transforms(episode_meta)

    assert len(calls) == 1


def test_sync_static_transforms_calls_service_on_fingerprint_change(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    (tmp_path / "left.json").write_text("first")
    episode_meta = EpisodeMeta(tf_directory=str(tmp_path))

    helper.sync_static_transforms(episode_meta)
    (tmp_path / "left.json").write_text("updated content that changes mtime")
    helper.sync_static_transforms(episode_meta)

    assert len(calls) == 2


def test_sync_static_transforms_calls_service_when_directory_switches(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    dir_a = tmp_path / "a"
    dir_b = tmp_path / "b"
    dir_a.mkdir()
    dir_b.mkdir()
    (dir_a / "left.json").write_text("{}")
    (dir_b / "right.json").write_text("{}")

    episode_meta = EpisodeMeta(tf_directory=str(dir_a))
    helper.sync_static_transforms(episode_meta)

    episode_meta.tf_directory = str(dir_b)
    helper.sync_static_transforms(episode_meta)

    assert len(calls) == 2
    assert calls[0]["request"]["directory"] == str(dir_a)
    assert calls[1]["request"]["directory"] == str(dir_b)


def test_sync_static_transforms_triggers_after_cache_invalidate(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls)
    (tmp_path / "left.json").write_text("{}")
    episode_meta = EpisodeMeta(tf_directory=str(tmp_path))

    helper.sync_static_transforms(episode_meta)
    helper.invalidate_static_transform_cache()
    helper.sync_static_transforms(episode_meta)

    assert len(calls) == 2


def test_sync_static_transforms_resyncs_when_only_directory_path_changes(
    monkeypatch, tmp_path
):
    """Two dirs with identical file content must not collide in cache."""
    calls = []
    helper = _make_helper(monkeypatch, calls)
    dir_a = tmp_path / "a"
    dir_b = tmp_path / "b"
    dir_a.mkdir()
    dir_b.mkdir()
    content = "{}"
    (dir_a / "same.json").write_text(content)
    (dir_b / "same.json").write_text(content)
    # Force identical mtime/size so the file-set portion matches.
    import os

    shared_stat = (dir_a / "same.json").stat()
    os.utime(
        dir_b / "same.json",
        ns=(shared_stat.st_atime_ns, shared_stat.st_mtime_ns),
    )

    episode_meta = EpisodeMeta(tf_directory=str(dir_a))
    helper.sync_static_transforms(episode_meta)

    episode_meta.tf_directory = str(dir_b)
    helper.sync_static_transforms(episode_meta)

    assert len(calls) == 2
    assert calls[0]["request"]["directory"] == str(dir_a)
    assert calls[1]["request"]["directory"] == str(dir_b)


def test_sync_static_transforms_fails_when_service_name_not_configured(
    monkeypatch, tmp_path
):
    calls = []
    helper = _make_helper(monkeypatch, calls, service_name=None)
    (tmp_path / "left.json").write_text("{}")
    episode_meta = EpisodeMeta(tf_directory=str(tmp_path))

    result = helper.sync_static_transforms(episode_meta)

    assert result is False
    assert calls == []
    assert any(
        "static_transform_service_name" in msg
        for msg in helper.logger.errors
    )
