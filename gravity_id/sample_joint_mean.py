#!/usr/bin/env python3
"""Sample a JointState topic and print per-joint mean/std over a window.

Companion tool for kp_offset_consistency_check.md. Passive: subscribes only.

    python3 gravity_id/sample_joint_mean.py --topic /puppet/joint_left \
        --seconds 2.0

Prints mean and std for the first 6 joints, plus sample count, as both a
human-readable table and a single JSON line (for pasting into the check
spreadsheet or diffing runs).
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class _Sampler(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("gravity_check_joint_sampler")
        self.samples: list[list[float]] = []
        self.create_subscription(JointState, topic, self._on_msg, 50)

    def _on_msg(self, msg: JointState) -> None:
        if len(msg.position) >= 6:
            self.samples.append([float(p) for p in msg.position[:6]])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/puppet/joint_left")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument(
        "--settle", type=float, default=0.0,
        help="Extra wait before the sampling window starts.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = _Sampler(args.topic)
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.samples.clear()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        raise SystemExit(
            f"No JointState messages on {args.topic} in {args.seconds}s."
        )

    n = len(samples)
    means = [sum(s[j] for s in samples) / n for j in range(6)]
    stds = [
        math.sqrt(sum((s[j] - means[j]) ** 2 for s in samples) / n)
        for j in range(6)
    ]
    print(f"topic={args.topic} samples={n} window={args.seconds}s")
    for j in range(6):
        print(f"  joint{j + 1}: mean={means[j]:+.5f} rad  std={stds[j]:.5f}")
    print(json.dumps({
        "topic": args.topic,
        "samples": n,
        "mean": [round(m, 6) for m in means],
        "std": [round(s, 6) for s in stds],
    }))


if __name__ == "__main__":
    main()
