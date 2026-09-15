#!/usr/bin/env python3
"""Stage 5 gate: load the MuJoCo model, verify it matches the frozen contract
(docs/architecture.md §1, firmware messages.hpp JointId), step physics, and
confirm actuators actually move their joints. Run: python validate.py"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Expected actuated joints in JointId order (messages.hpp). TorsoLift is id 16.
EXPECTED_JOINTS = [
    "base_left_wheel", "base_right_wheel",
    "head_pan", "head_tilt",
    "left_shoulder_pan", "left_shoulder_lift", "left_elbow",
    "left_wrist_flex", "left_wrist_roll", "left_gripper",
    "right_shoulder_pan", "right_shoulder_lift", "right_elbow",
    "right_wrist_flex", "right_wrist_roll", "right_gripper",
    "torso_lift",
]


def check(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        check.failed += 1
check.failed = 0


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
    data = mujoco.MjData(model)
    print("Nori MuJoCo model — Stage 5 validation\n")

    # 1. contract: every expected joint exists and each has one actuator
    print("Contract (17 actuated DOF):")
    joint_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                   for i in range(model.njnt)}
    for j in EXPECTED_JOINTS:
        check(j in joint_names, f"joint '{j}' present")
    check(model.nu == len(EXPECTED_JOINTS),
          f"actuator count == 17 (got {model.nu})")
    # base free joint = 6 DOF + 17 actuated = 23 qpos-velocity DOF (free adds 6)
    check(model.nv == 6 + len(EXPECTED_JOINTS),
          f"nv == 23 (free base 6 + 17) (got {model.nv})")

    # 2. physics is stable: settle under gravity, robot shouldn't explode
    print("\nPhysics:")
    for _ in range(500):
        mujoco.mj_step(model, data)
    check(np.all(np.isfinite(data.qpos)), "qpos finite after 500 steps")
    base_z = data.body("base").xpos[2]
    check(0.0 < base_z < 0.3, f"base rests on wheels (z={base_z:.3f} m)")

    # 3. actuators move joints: command the lift up, confirm it rises
    print("\nActuation:")
    lift_act = model.actuator("a_torso_lift").id
    lift_adr = model.joint("torso_lift").qposadr[0]
    data.ctrl[lift_act] = 0.25  # command 0.25 m
    for _ in range(2000):
        mujoco.mj_step(model, data)
    lift_pos = data.qpos[lift_adr]
    check(abs(lift_pos - 0.25) < 0.03, f"torso_lift reached 0.25 m (got {lift_pos:.3f})")

    # command an arm joint and confirm it tracks
    elbow_act = model.actuator("a_left_elbow").id
    elbow_adr = model.joint("left_elbow").qposadr[0]
    data.ctrl[elbow_act] = 1.0
    for _ in range(2000):
        mujoco.mj_step(model, data)
    elbow_pos = data.qpos[elbow_adr]
    check(abs(elbow_pos - 1.0) < 0.1, f"left_elbow reached 1.0 rad (got {elbow_pos:.3f})")

    print()
    if check.failed:
        print(f"FAILED: {check.failed} check(s)")
        sys.exit(1)
    print(f"All checks passed. Total mass = {sum(model.body_mass):.2f} kg, "
          f"{model.nu} actuators, {model.nv} DOF.")


if __name__ == "__main__":
    main()
