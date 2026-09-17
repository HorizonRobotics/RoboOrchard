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

from setuptools import find_packages, setup

PACKAGE_NAME = "robo_orchard_control_manager_ros2"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["pydantic>=2", "PyYAML", "setuptools"],
    zip_safe=True,
    maintainer="Yiwei Jin",
    maintainer_email="yiwei.jin@horizon.auto",
    description="Global command routing and control lifecycle manager.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_manager_node = "
            "robo_orchard_control_manager_ros2.node:main",
        ],
    },
)
