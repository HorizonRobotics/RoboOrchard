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
import shutil
from pathlib import Path

current_dir = Path(__file__).parent
project_root = current_dir.parent.parent
root_path = project_root.parent.parent

files_to_link = [
    (root_path / "LICENSE", current_dir / "LICENSE"),
    (project_root / "CHANGELOG.md", current_dir / "link" / "CHANGELOG.md"),
    (project_root / "README.md", current_dir / "link" / "README.md"),
]

for src, dst in files_to_link:
    if src.exists():
        shutil.copy2(src, dst)

project = "HoloBrain"
copyright = "2026, Horizon Robotics"
author = "Horizon Robotics"
release = "0.0.0"

extensions = [
    "sphinx.ext.doctest",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    "myst_parser",
    "sphinx_design",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

html_theme = "furo"
html_theme_options = {
    # Set the name of the project to appear in the sidebar
    "navigation_with_keys": True,
    "sidebar_hide_name": False,
    "light_logo": "logo/logo_light.png",
    "dark_logo": "logo/logo_dark.png",
    "footer_icons": [
        {
            "name": "GitHub",
            "url": "https://github.com/HorizonRobotics/RoboOrchard",
            "html": """
                <svg stroke="currentColor" fill="currentColor" stroke-width="0" viewBox="0 0 16 16">
                    <path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"></path>
                </svg>
            """,  # noqa
            "class": "",
        },
    ],
}
if os.environ.get("DOC_ANNOUNCEMENT", None) is not None:
    html_theme_options.update(
        dict(announcement=os.environ["DOC_ANNOUNCEMENT"])
    )

html_static_path = ["_static"]
