#!/usr/bin/env python3
"""Replay a ROS-recorded joint log in the native MuJoCo GUI (macOS: mjpython).

The motion in demo_log.npz was produced by the real ROS 2 control stack driving
the MuJoCo-backed hardware plugin (see ros/record_demo.py). This plays those
joint angles back kinematically so you can watch it on the Mac:

  .venv/bin/mjpython simulation/mujoco/replay.py

The base pose is reconstructed from the recorded wheel angles via differential-
drive odometry (the same kinematic model diff_drive_controller uses), so the
chassis drives and turns as the wheels roll. Arms/head/lift move exactly as ROS
commanded. Loops until you close it.
"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

WHEEL_RADIUS = 0.06   # m  (matches nori.xml drive wheels)
TRACK = 0.26          # m  (wheel separation, y = ±0.13)
BASE_Z = 0.06         # m  (base spawn height; kept level — no tipping in replay)


def integrate_odometry(names, positions):
    """Return base (x, y, yaw) per frame from left/right wheel angle deltas."""
    li, ri = names.index("base_left_wheel"), names.index("base_right_wheel")
    n = positions.shape[0]
    xyt = np.zeros((n, 3))  # x, y, yaw
    x = y = th = 0.0
    for k in range(1, n):
        d_l = WHEEL_RADIUS * (positions[k, li] - positions[k - 1, li])
        d_r = WHEEL_RADIUS * (positions[k, ri] - positions[k - 1, ri])
        d = 0.5 * (d_l + d_r)
        dth = (d_r - d_l) / TRACK
        x += d * np.cos(th + 0.5 * dth)
        y += d * np.sin(th + 0.5 * dth)
        th += dth
        xyt[k] = (x, y, th)
    return xyt


def main():
    log = np.load(os.path.join(HERE, "demo_log.npz"))
    names = [str(n) for n in log["names"]]
    times = log["times"]
    positions = log["positions"]
    print(f"replaying {len(times)} frames, {times[-1]:.1f}s, {len(names)} joints")

    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
    data = mujoco.MjData(model)
    # map each logged joint name -> its qpos address
    adr = {}
    for n in names:
        try:
            adr[n] = model.joint(n).qposadr[0]
        except KeyError:
            pass
    # base free joint: qpos[base:base+3] = xyz, [base+3:base+7] = quat (w,x,y,z)
    base = model.joint("base_free").qposadr[0]
    # prefer the base pose recorded from ROS /odom; else reconstruct from wheels
    if "base_xyt" in log:
        odom = log["base_xyt"]
        print("base pose: recorded ROS odometry")
    else:
        odom = integrate_odometry(names, positions)
        print("base pose: reconstructed from wheel angles")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t0 = time.time()
            for k in range(len(times)):
                if not viewer.is_running():
                    break
                for j, n in enumerate(names):
                    if n in adr:
                        data.qpos[adr[n]] = positions[k, j]
                x, y, th = odom[k]
                data.qpos[base:base + 3] = (x, y, BASE_Z)
                data.qpos[base + 3:base + 7] = (np.cos(th / 2), 0.0, 0.0, np.sin(th / 2))
                mujoco.mj_forward(model, data)  # kinematics only, no gravity
                viewer.sync()
                # keep wall-clock pace with the recording
                dt = (times[k + 1] - times[k]) if k + 1 < len(times) else 0.03
                time.sleep(max(0.0, dt))
            time.sleep(0.5)  # pause between loops


if __name__ == "__main__":
    main()
