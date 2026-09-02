# AGENTS.md

## Repository Scope

- RoboOrchard is an open-source real-robot integration repository containing
  Python packages, ROS 2 packages, C++ components, and project integrations.
- Base decisions on checked-in code and public repository documentation. Do
  not add private infrastructure, internal-only projects, credentials,
  personal filesystem paths, or non-public URLs to tracked files.
- Treat checked-out git submodules as independent repositories. Follow their
  own `AGENTS.md` files when working inside them, and do not modify submodule
  contents unless the task explicitly includes them.

## Sources of Truth

- Use `README.md` for the repository overview and supported environment.
- Use the root `Makefile`, package-local build files, and `pyproject.toml` for
  commands and tooling configuration rather than inventing new workflows.
- Prefer source files over generated outputs. Do not modify `build/`, install
  outputs, caches, vendored code, or third-party code unless explicitly asked.

## Working Rules

- Inspect the relevant package and its tests before changing behavior.
- Keep changes focused on the requested package and avoid unrelated cleanup.
- Fix root causes rather than adding compatibility branches or temporary
  workarounds without a demonstrated need.
- Prefer the simplest design that fits existing repository patterns. Do not
  introduce interfaces, base classes, configuration layers, or dependencies
  solely for hypothetical future use.
- Preserve public interfaces unless the task requires changing them; document
  and test observable contract changes.
- Keep comments and documentation aligned with behavior. Do not remove useful
  context or leave stale descriptions after changing an implementation.

## Python Development

- Support Python 3.10 or newer.
- Follow the root `pyproject.toml` for files covered by its Ruff configuration:
  79-character lines, double-quoted strings, Google-style docstrings, and
  absolute imports.
- Add type annotations to new or changed public interfaces and to internal
  boundaries where types are not otherwise obvious.
- Document public APIs when their inputs, outputs, units, shapes, ordering,
  side effects, or ownership are not clear from the signature.
- Use the repository license header on new Python source files, matching the
  current year format accepted by `scm/qac/check_license_header.py`.
- In ROS 2 nodes and long-running services, use the provided logger rather
  than `print` for runtime status and errors.

## ROS 2 and C++ Development

- Follow the conventions of the affected ROS 2 package and its neighboring
  source files; this repository contains both `ament_python` and CMake-based
  packages.
- Keep robot-specific topic names, message types, frame names, joint layouts,
  and hardware settings in parameters or configuration when the surrounding
  design supports it.
- When adding a dependency, update the owning package's `package.xml` and its
  corresponding Python or CMake build metadata.
- Do not edit generated ROS interfaces, build outputs, installed artifacts, or
  third-party sources when the source definition can be changed instead.

## Tests and Validation

- Add or update tests for behavior changes and regressions when the affected
  package has an established test structure.
- Start with the smallest relevant package or test target. Expand to broader
  checks only when the change crosses package boundaries.
- Use `make check-lint` for the root Python lint and license checks.
- Use `make test` for the root Python application test target when applicable.
- Use `make ros2-build` and `make ros2-test` for workspace-wide ROS 2
  validation when the environment is prepared and the change warrants it.
- Report validation that could not be run because of missing ROS 2 setup,
  hardware, services, dependencies, or other environment constraints.

## Dependencies and Security

- Reuse existing dependencies when practical. Add a new dependency only when
  it is necessary and declare it in the correct package manifest.
- Never hardcode credentials, tokens, passwords, private endpoints, or
  machine-specific paths. Avoid logging sensitive request or robot data.
- Do not inspect or modify `.git/`, editor history, local environments, caches,
  or ignored runtime data unless the task explicitly requires it.

## Git Workflow

- Follow `CONTRIBUTING.md` for commit and MR/PR formatting.
- Before performing Git operations or preparing commit or MR/PR content, read
  `.agents/instructions/git.md`.

## Deploy Node

- For changes to `ros2_package/robo_orchard_deploy_ros2/`, read
  `.agents/references/deploy-node-boundary.md` first.
