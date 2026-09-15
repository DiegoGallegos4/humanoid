#!/usr/bin/env python3
"""Interactive viewer that sweeps every joint so you can eyeball the model.
On macOS the MuJoCo GUI needs mjpython:  mjpython view.py
(headless validation lives in validate.py and needs no display.)"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
    data = mujoco.MjData(model)
    print(f"loaded: {model.nu} actuators, {model.nv} DOF — animating (Ctrl-C to stop)")
    # sweep target = midpoint ± 70% of each driven joint's range (wheels have no range)
    tgt = []
    for a in range(model.nu):
        jid = model.actuator_trnid[a, 0]
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            tgt.append(((lo + hi) / 2, (hi - lo) / 2))
        else:
            tgt.append((0.0, 0.0))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        last = 0.0
        while viewer.is_running():
            t = time.time() - t0
            for a in range(2, model.nu):  # position joints
                mid, amp = tgt[a]
                data.ctrl[a] = mid + 0.7 * amp * np.sin(2.2 * t + 0.6 * a)
            # wheels (velocity cmd, rad/s): gentle counter-rotate so it rocks in
            # place without tipping the raised, top-heavy torso. Bump the gain or
            # flip one sign for a faster spin / straight-line roll.
            spin = 1.2 * np.sin(0.4 * t)
            data.ctrl[0] = spin      # base_left_wheel
            data.ctrl[1] = -spin     # base_right_wheel
            mujoco.mj_step(model, data)
            viewer.sync()
            if t - last > 2.0:  # heartbeat to the log so progress is visible headless
                print(f"t={t:5.1f}s  base_z={data.body('base').xpos[2]:.3f}  "
                      f"lift={data.qpos[model.joint('torso_lift').qposadr[0]]:.3f}")
                last = t
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
