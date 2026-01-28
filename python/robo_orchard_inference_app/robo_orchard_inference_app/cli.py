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
import sys

import typer
from streamlit.web import cli as stcli

app = typer.Typer(help="Inference App.")


@app.command(
    name="launch",
    context_settings={
        "allow_extra_args": True,
        "ignore_unknown_options": True,
    },
)
def launch(ctx: typer.Context):
    """Launch the inference app."""

    app_path = os.path.join(os.path.dirname(__file__), "app.py")
    if not os.path.exists(app_path):
        typer.echo(f"Error: Could not find app.py at {app_path}", err=True)
        raise typer.Exit(code=1)

    try:
        launch_index = sys.argv.index("launch")
        raw_args = sys.argv[launch_index + 1 :]
    except ValueError:
        raw_args = ctx.args

    sys.argv = ["streamlit", "run", app_path] + raw_args
    sys.exit(stcli.main())


if __name__ == "__main__":
    app()
