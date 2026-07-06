#!/usr/bin/env python3
"""Independent gravity-torque check for the left Piper chain.

Parses the dual-arm URDF directly and computes, for a given q, the actuator
torque each left joint must apply to hold the arm static under gravity:

    tau_k = -axis_k . sum_{links distal to k} (p_com_i - p_k) x (m_i * g)

This is the same quantity pin.rnea(q, 0, 0) returns, computed with nothing
but FK, so it cross-checks the controller's recorded rnea values.
"""

import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = "/data/holobrain/urdf/piper_x_description_dualarm_v2.urdf"
G = np.array([0.0, 0.0, -9.81])


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    n = np.linalg.norm(axis)
    if n == 0:
        return np.eye(3)
    x, y, z = axis / n
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def parse(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        mass, com = 0.0, np.zeros(3)
        if inertial is not None:
            m = inertial.find("mass")
            if m is not None:
                mass = float(m.get("value"))
            o = inertial.find("origin")
            if o is not None:
                com = np.array(
                    [float(v) for v in (o.get("xyz") or "0 0 0").split()]
                )
        links[link.get("name")] = (mass, com)
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array(
            [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        ) if o is not None else np.zeros(3)
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] \
            if o is not None else [0, 0, 0]
        a = j.find("axis")
        axis = np.array(
            [float(v) for v in (a.get("xyz") or "1 0 0").split()]
        ) if a is not None else np.array([1.0, 0, 0])
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz,
            "R0": rpy_to_mat(*rpy),
            "axis": axis,
        })
    return links, joints


def fk(links, joints, qmap):
    children = {}
    for j in joints:
        children.setdefault(j["parent"], []).append(j)
    all_children = {j["child"] for j in joints}
    roots = [name for name in links if name not in all_children]
    world = {r: (np.eye(3), np.zeros(3)) for r in roots}
    joint_frames = {}  # joint name -> (axis_world, origin_world)
    subtree = {}       # joint name -> set of link names below it
    stack = list(roots)
    while stack:
        parent = stack.pop()
        Rp, pp = world[parent]
        for j in children.get(parent, []):
            R = Rp @ j["R0"]
            p = pp + Rp @ j["xyz"]
            q = qmap.get(j["name"], 0.0)
            if j["type"] in ("revolute", "continuous"):
                joint_frames[j["name"]] = (R @ j["axis"], p)
                R = R @ axis_angle(j["axis"], q)
            elif j["type"] == "prismatic":
                p = p + R @ (j["axis"] * q)
            world[j["child"]] = (R, p)
            stack.append(j["child"])

    def collect(link):
        out = {link}
        for j in children.get(link, []):
            out |= collect(j["child"])
        return out

    for j in joints:
        if j["name"] in joint_frames:
            subtree[j["name"]] = collect(j["child"])
    return world, joint_frames, subtree


def gravity_torques(urdf_path, q6, prefix="left_joint"):
    links, joints = parse(urdf_path)
    qmap = {f"{prefix}{i + 1}": q6[i] for i in range(6)}
    world, joint_frames, subtree = fk(links, joints, qmap)
    taus = []
    for i in range(1, 7):
        jn = f"{prefix}{i}"
        axis_w, p_j = joint_frames[jn]
        t = np.zeros(3)
        for ln in subtree[jn]:
            mass, com = links[ln]
            if mass == 0.0:
                continue
            Rl, pl = world[ln]
            p_com = pl + Rl @ com
            t += np.cross(p_com - p_j, mass * G)
        taus.append(-float(axis_w @ t))
    return taus


def fk_report(urdf_path, q6, prefix="left_joint"):
    links, joints = parse(urdf_path)
    qmap = {f"{prefix}{i + 1}": q6[i] for i in range(6)}
    world, joint_frames, subtree = fk(links, joints, qmap)
    print(f"pose {q6}:")
    for i in range(1, 7):
        axis_w, p_j = joint_frames[f"{prefix}{i}"]
        print(f"  {prefix}{i}: origin=({p_j[0]:+.3f},{p_j[1]:+.3f},{p_j[2]:+.3f}) axis=({axis_w[0]:+.2f},{axis_w[1]:+.2f},{axis_w[2]:+.2f})")
    # subtree COM below joint 3 (forearm + wrist + gripper)
    for jn in (f"{prefix}2", f"{prefix}3"):
        m_tot, moment = 0.0, np.zeros(3)
        for ln in subtree[jn]:
            mass, com = links[ln]
            if mass == 0.0:
                continue
            Rl, pl = world[ln]
            m_tot += mass
            moment += mass * (pl + Rl @ com)
        com_w = moment / m_tot
        _, p_j = joint_frames[jn]
        d = com_w - p_j
        print(f"  subtree({jn}): mass={m_tot:.3f} kg  COM-rel=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})  horiz-lever={math.hypot(d[0],d[1]):.3f} m")

if __name__ == "__main__":
    poses = {
        "test pose [0,1,-1,0,0,0]": [0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
        "phase1-ish [0,1,-1.4,0,0,0]": [0.0, 1.0, -1.4, 0.0, 0.0, 0.0],
        "arm fwd    [0,0.6,-0.6,0,0,0]": [0.0, 0.6, -0.6, 0.0, 0.0, 0.0],
        "horizontal [0,1.57,0,0,0,0]": [0.0, 1.57, 0.0, 0.0, 0.0, 0.0],
        "straight up[0,0,0,0,0,0]": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    for label, q in poses.items():
        taus = gravity_torques(URDF, q)
        print(label, " tau:", ["%+.4f" % t for t in taus])
    print()
    fk_report(URDF, [0.0, 1.0, -1.0, 0.0, 0.0, 0.0])
    print()
    fk_report(URDF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print()
    for q in ([0.0, 1.57, -1.4, 0, 0, 0], [0.0, 2.0, -1.4, 0, 0, 0]):
        print(q, "tau:", ["%+.3f" % t for t in gravity_torques(URDF, q)])
