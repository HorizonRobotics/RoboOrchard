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
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "robo_orchard_wuji_glove_ros2"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (
            os.path.join("share", PACKAGE_NAME, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=[
        "numpy",
        "setuptools",
        "wuji-sdk==2026.7.21",
    ],
    zip_safe=False,
    maintainer="xuewu.lin",
    maintainer_email="xuewu.lin@horizon.auto",
    description="Wuji Glove state publisher and Wuji Hand retargeting nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wuji_glove_driver_node = "
            "robo_orchard_wuji_glove_ros2.driver_node:main",
            "wuji_glove_retarget_node = "
            "robo_orchard_wuji_glove_ros2.retarget_node:main",
        ],
    },
)
