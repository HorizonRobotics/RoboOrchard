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

import argparse
import os
import subprocess


def get_root():
    current_file_path = os.path.abspath(__file__)
    root_path = os.path.dirname(current_file_path)
    for _ in range(2):
        root_path = os.path.dirname(root_path)
    return root_path


def python_lint(root_path: str, auto_format: bool = False):
    # run external python file to lint python

    current_file_path = os.path.abspath(__file__)
    cur_folder = os.path.dirname(current_file_path)

    subprocess.check_call(
        " ".join(
            [
                "bash",
                (f"{cur_folder}/code_lint_py.sh"),
                f"{root_path}/",
                "true" if auto_format else "false",
            ]
        ),
        shell=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="check format.")
    parser.add_argument(
        "--auto_format", action="store_true", help="auto format python"
    )
    parser = parser.parse_args()
    root_path = get_root()
    # cpp_lint(root_path) # not available yet
    python_lint(root_path, parser.auto_format)
