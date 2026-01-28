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
from glob import glob

from setuptools import find_packages, setup

package_name = "robo_orchard_teleop_ros2"
PYTHON_BASE_DIR = os.path.abspath(os.path.dirname(__file__))


def data_files_generator():
    """Generate data files for package.

    For python package, we can use `package_data` to specify data files.
    However, ROS2 package is not a python package, you can also specify
    `data_files` to install data to custom path.

    """
    data_files = [
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ]

    for root, _, files in os.walk("resource"):
        for file in files:
            data_files.append(
                (f"share/{package_name}/{root}", [os.path.join(root, file)])
            )

    return data_files


def set_manifest():
    """Set MANIFEST.in file."""
    file_name = os.path.join(os.path.dirname(__file__), "MANIFEST.in")
    manifest_file = open(file_name, "w")
    manifest_file.write(
        f"""
recursive-include {package_name} *.py
recursive-exclude {package_name} *.pyc
recursive-include resource *.json
exclude resource/{package_name}
"""
    )
    manifest_file.close()


def generate_packages() -> tuple:
    """Generate packages and package_dir for setup."""
    packages = []
    package_dir = {}
    # add packages
    for pkg in [
        package_name,
    ]:
        packages.append(pkg)
        # package_dir.update({pkg: pkg})
        packages.extend(
            [
                f"{pkg}.{module}"
                for module in find_packages(os.path.join(PYTHON_BASE_DIR, pkg))
            ]
        )

    # add resources
    packages.append(f"{package_name}_resource")
    package_dir.update({f"{package_name}_resource": "resource"})

    if os.path.exists(os.path.join(PYTHON_BASE_DIR, "resource")):
        resource_path = os.path.join(PYTHON_BASE_DIR, "resource")
    else:
        resource_path = os.path.join(
            PYTHON_BASE_DIR, f"{package_name}_resource"
        )

    packages.extend(
        [
            f"{package_name}_resource.{module}"
            for module in os.listdir(resource_path)
            if os.path.isdir(os.path.join(PYTHON_BASE_DIR, "resource", module))
        ]
    )

    set_manifest()
    return packages, package_dir


packages, package_dir = generate_packages()

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files_generator(),
    install_requires=["pytorch_kinematics", "numpy", "scipy", "sshkeyboard"],
    zip_safe=True,
    maintainer="hongyu.xie",
    maintainer_email="hongyu.xie@horizon.auto",
    description="The ros2 package of pico data bridge.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pico_bridge = robo_orchard_teleop_ros2.bridge.pico.node:main",  # noqa: E501
            "take_over = robo_orchard_teleop_ros2.take_over.node:main",  # noqa: E501
            "pico_vr_trigger = robo_orchard_teleop_ros2.take_over.trigger.pico_vr:main",  # noqa: E501
            "keyboard_trigger = robo_orchard_teleop_ros2.take_over.trigger.keyboard:main",  # noqa: E501
            "piper_pico_vr_teleop = robo_orchard_teleop_ros2.robot.piper.pico_vr:main",  # noqa: E501
        ],
    },
)
