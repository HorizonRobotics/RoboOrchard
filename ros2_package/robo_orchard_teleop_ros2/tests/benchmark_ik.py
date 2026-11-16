# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

"""Offline regression/benchmark with external inputs and baseline IK source.

No ROS node, driver, network request, or hardware is involved.
"""

from __future__ import annotations
import argparse
import hashlib
import importlib
import importlib.util
import json
import os
import sys
from pathlib import Path
from time import perf_counter
from types import ModuleType, SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation


def pose(values):
    """Make an inert xyz/xyzw ROS-shaped value object."""
    return SimpleNamespace(
        position=SimpleNamespace(**dict(zip("xyz", values[:3], strict=True))),
        orientation=SimpleNamespace(
            **dict(zip("xyzw", values[3:], strict=True))
        ),
    )


def load_source(path, name="baseline_ik"):
    """Load a baseline mathematical module, with a type-only ROS stub."""
    sys.modules.setdefault("geometry_msgs.msg", SimpleNamespace(Pose=object))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.IkOptimizer


def load_ikpy_math() -> None:
    """Load unchanged IKPy math without its unrelated initializer.

    Exclude the analytics thread and JAX availability probe in the reference
    package initializer. FK, residuals, and the SciPy baseline are unchanged.
    """
    if "ikpy" in sys.modules:
        return
    spec = importlib.util.find_spec("ikpy")
    if spec is None:
        raise ImportError("Install the baseline IKPy dependency for replay")
    package = ModuleType("ikpy")
    package.__path__ = list(spec.submodule_search_locations)
    sys.modules["ikpy"] = package
    package.__version__ = importlib.import_module("ikpy._version").__version__


def percentiles(values):
    """Return P50/P95/P99/max."""
    return np.percentile(values, [50, 95, 99, 100]).tolist()


def main():
    """Compare each recorded measured seed, never using original q as seed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--left-ee", required=True)
    parser.add_argument("--right-ee", required=True)
    parser.add_argument("--baseline-source", type=Path, required=True)
    parser.add_argument(
        "--current-source",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "robo_orchard_teleop_ros2/ik.py",
    )
    parser.add_argument("--extension-dir", type=Path)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--rounds", type=int, default=1)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.extension_dir:
        sys.path.insert(0, str(args.extension_dir))
        ik_module = importlib.import_module("_ik")
    else:
        ik_module = importlib.import_module("robo_orchard_teleop_ros2._ik")
    baseline_class = load_source(args.baseline_source)
    load_ikpy_math()
    sys.modules["robo_orchard_teleop_ros2._ik"] = ik_module
    current_class = load_source(args.current_source, "current_ik")
    inputs = np.load(args.inputs, allow_pickle=False)
    report = {
        "metadata": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "scipy": sys.modules["scipy"].__version__,
            "ikpy": sys.modules["ikpy"].__version__,
            "affinity": sorted(os.sched_getaffinity(0)),
            "threads": {
                key: os.environ.get(key)
                for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")
            },
            "sha256": {
                name: hashlib.sha256(path.read_bytes()).hexdigest()
                for name, path in (
                    ("inputs", args.inputs),
                    ("urdf", args.urdf),
                    ("baseline_source", args.baseline_source),
                    ("current_source", args.current_source),
                    ("ik_binary", Path(ik_module.__file__)),
                )
            },
            "ik_source_sha256": {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(
                    (Path(__file__).resolve().parents[1] / "src/ik").glob(
                        "*.*"
                    )
                )
                if path.suffix in (".cpp", ".hpp")
            },
        }
    }
    raw_output = {}
    rng = np.random.default_rng(2026)
    for side, ee in (("left", args.left_ee), ("right", args.right_ee)):
        solver = ik_module.Ik(str(args.urdf), args.base, ee)
        baseline = baseline_class(str(args.urdf), args.base, ee)
        current = current_class(str(args.urdf), args.base, ee)
        seeds = inputs[side + "_seed_q"]
        targets = inputs[side + "_target"]
        count = min(len(seeds), args.limit or len(seeds))
        assert solver.joint_names == baseline.get_joint_names()
        timing = {"ik_ms": [], "baseline_ms": []}
        maximum, fk_maximum, rejected = 0.0, 0.0, 0
        joint_maximum = np.zeros(solver.num_joints)
        historical_maximum = np.zeros(solver.num_joints)
        historical = inputs.get(side + "_historical_q")
        raw_indices, raw_q = [], []
        out_of_bounds = 0
        for index in range(min(5, count)):
            target = targets[index]
            values = [
                *target[:3, 3],
                *Rotation.from_matrix(target[:3, :3]).as_quat(),
            ]
            for optimizer in (current, baseline):
                optimizer.solve(pose(values), seeds[index].tolist())
        for _ in range(args.rounds):
            for index in rng.permutation(count):
                seed = seeds[index].tolist()
                target = targets[index]
                quaternion = Rotation.from_matrix(target[:3, :3]).as_quat()
                values = [*target[:3, 3], *quaternion]
                target_pose = pose(values)
                solutions = {}
                for name in rng.permutation(["ik", "baseline"]):
                    start = perf_counter()
                    if name == "ik":
                        solutions[name] = current.solve(target_pose, seed)
                    else:
                        solutions[name] = baseline.solve(target_pose, seed)
                    timing[name + "_ms"].append(
                        1000 * (perf_counter() - start)
                    )
                old, new = solutions["baseline"], solutions["ik"]
                assert (old is None) == (new is None), (side, int(index))
                if old is None:
                    rejected += 1
                else:
                    delta = np.max(
                        np.abs(np.rad2deg(np.asarray(new) - np.asarray(old)))
                    )
                    maximum = max(maximum, float(delta))
                    joint_maximum = np.maximum(
                        joint_maximum,
                        np.abs(np.rad2deg(np.asarray(new) - np.asarray(old))),
                    )
                    assert delta < 0.01, (side, int(index), float(delta))
                    outside = np.any(np.asarray(new) < solver.lower) or np.any(
                        np.asarray(new) > solver.upper
                    )
                    out_of_bounds += int(outside)
                    assert not outside
                    if historical is not None:
                        difference = np.abs(
                            np.rad2deg(np.asarray(new) - historical[index])
                        )
                        historical_maximum = np.maximum(
                            historical_maximum, difference
                        )
                        assert np.max(difference) < 0.01
                    raw_indices.append(index)
                    raw_q.append(new)
                old_fk = baseline.chain.forward_kinematics(
                    baseline._clamp_seed(seed)
                )
                fk_delta = np.max(
                    np.abs(solver.fk(solver.clamp(seed)) - old_fk)
                )
                fk_maximum = max(fk_maximum, float(fk_delta))
                assert fk_delta < 1e-10
        report[side] = {
            "samples_per_round": count,
            "rounds": args.rounds,
            "max_joint_difference_deg": maximum,
            "per_joint_max_difference_deg": joint_maximum.tolist(),
            "historical_per_joint_max_difference_deg": (
                historical_maximum.tolist() if historical is not None else None
            ),
            "max_fk_matrix_difference": fk_maximum,
            "rejected": rejected,
            "out_of_bounds": out_of_bounds,
            **{name: percentiles(value) for name, value in timing.items()},
        }
        print(json.dumps({side: report[side]}, indent=2), flush=True)
        raw_output[side + "_index"] = np.asarray(raw_indices)
        raw_output[side + "_q"] = np.asarray(raw_q)
        for name, values in timing.items():
            raw_output[side + "_" + name] = np.asarray(values)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
        np.savez_compressed(args.report.with_suffix(".npz"), **raw_output)


if __name__ == "__main__":
    main()
