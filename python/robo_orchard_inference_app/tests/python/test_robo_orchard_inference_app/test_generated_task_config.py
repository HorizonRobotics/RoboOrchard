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

import importlib.util
import io
import pathlib
import sys
import types

sys.path.insert(0, "python/robo_orchard_inference_app")
version = types.ModuleType("robo_orchard_inference_app.version")
version.__version__ = "0.0.0"
version.__full_version__ = "0.0.0"
version.__git_hash__ = "test"
sys.modules.setdefault("robo_orchard_inference_app.version", version)

from robo_orchard_inference_app.config import TaskCfg


def _find_repo_file(relative_path: str) -> pathlib.Path:
    current = pathlib.Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return candidate
    raise FileNotFoundError(relative_path)


def _load_module():
    module_path = _find_repo_file(
        "projects/HoloBrain/app/gen_inference_app_task_config.py"
    )
    spec = importlib.util.spec_from_file_location(
        "generated_task_cfg_module", module_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_generated_task_cfg_adds_default_tf_directory(monkeypatch):
    module = _load_module()
    cached_cfg = TaskCfg(
        available_collectors=["haichao"],
        available_tasks={"pick": ["cube"]},
        candidate_tf_directories=["/custom/tf"],
        metas={"lighting": ["bright"]},
    )

    monkeypatch.setattr(module, "_project_root", lambda: "/project/HoloBrain")
    monkeypatch.setattr(module.os.path, "exists", lambda path: True)

    written = {}

    class _ReadCapture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return None

    class _WriteCapture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            written["content"] = self.getvalue()

    def fake_open(path, mode="r", *args, **kwargs):
        if "r" in mode:
            return _ReadCapture(cached_cfg.model_dump_json())
        return _WriteCapture()

    monkeypatch.setattr("builtins.open", fake_open)

    module.main()

    rendered = TaskCfg.model_validate_json(written["content"])
    assert rendered.available_collectors == ["haichao"]
    assert rendered.available_tasks == {"pick": ["cube"]}
    assert rendered.metas == {"lighting": ["bright"]}
    assert rendered.candidate_tf_directories == [
        "/custom/tf",
        "/project/HoloBrain/data/tf/default_static",
    ]


def test_generated_task_cfg_does_not_duplicate_default_tf_directory(
    monkeypatch,
):
    module = _load_module()
    default_tf_directory = "/project/HoloBrain/data/tf/default_static"
    cached_cfg = TaskCfg(candidate_tf_directories=[default_tf_directory])

    monkeypatch.setattr(module, "_project_root", lambda: "/project/HoloBrain")
    monkeypatch.setattr(module.os.path, "exists", lambda path: True)

    written = {}

    class _ReadCapture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return None

    class _WriteCapture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            written["content"] = self.getvalue()

    def fake_open(path, mode="r", *args, **kwargs):
        if "r" in mode:
            return _ReadCapture(cached_cfg.model_dump_json())
        return _WriteCapture()

    monkeypatch.setattr("builtins.open", fake_open)

    module.main()

    rendered = TaskCfg.model_validate_json(written["content"])
    assert rendered.candidate_tf_directories == [default_tf_directory]
