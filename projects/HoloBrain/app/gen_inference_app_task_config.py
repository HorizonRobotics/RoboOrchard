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

import os

from robo_orchard_inference_app.config import TaskCfg


def _project_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _default_tf_directory() -> str:
    return os.path.join(_project_root(), "data", "tf", "default_static")


def _latest_task_config_path() -> str:
    return os.path.join(
        _project_root(),
        ".cache",
        "robo_orchard_inference_app",
        "task_configs",
        "latest.json",
    )


def _load_task_cfg() -> TaskCfg:
    latest_task_config = _latest_task_config_path()
    if not os.path.exists(latest_task_config):
        return TaskCfg()

    with open(latest_task_config, "r") as f:
        return TaskCfg.model_validate_json(f.read())


def main():
    config = _load_task_cfg()
    default_tf_directory = _default_tf_directory()
    if default_tf_directory not in config.candidate_tf_directories:
        config.candidate_tf_directories.append(default_tf_directory)

    with open(
        os.path.join(os.path.dirname(__file__), "inference_app_task_cfg.json"),
        "w",
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
