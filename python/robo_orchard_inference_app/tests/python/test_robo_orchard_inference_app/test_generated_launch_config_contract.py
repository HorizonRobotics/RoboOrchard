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


def _find_repo_file(relative_path: str) -> pathlib.Path:
    current = pathlib.Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return candidate
    raise FileNotFoundError(relative_path)


def _render_generated_launch_cfg(monkeypatch) -> str:
    module_path = _find_repo_file(
        "projects/HoloBrain/app/gen_inference_app_launch_config.py"
    )
    spec = importlib.util.spec_from_file_location(
        "generated_launch_cfg_module", module_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    written = {}

    class _Capture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            written["content"] = self.getvalue()

    monkeypatch.setattr("builtins.open", lambda *args, **kwargs: _Capture())
    module.main()
    return written["content"]


def test_generated_launch_cfg_omits_disable_ctrl_services(monkeypatch):
    rendered = _render_generated_launch_cfg(monkeypatch)

    assert "disable_arm_service_name" not in rendered
    assert "disable_ctrl" not in rendered
