# Nori — MuJoCo model (Stage 5, Sim Level C)

Full-robot physics model: differential base, prismatic torso lift, pan/tilt head,
two SO-101 5-DOF arms + grippers. **17 actuated DOF**, joint names and order match
the frozen contract (`docs/architecture.md` §1/§6, `firmware/.../messages.hpp`).

## Files
- `nori.xml` — the robot: body tree, joints, 17 actuators, IMU/camera sensors.
- `scene.xml` — floor + light + visuals, `include`s `nori.xml`. **Load this one.**
- `validate.py` — headless Stage-5 gate (no display needed).
- `view.py` — interactive viewer that sweeps the joints.

## Setup & run
```bash
# from repo root — one-time
python3 -m venv .venv && .venv/bin/pip install mujoco numpy

# headless validation (contract + physics + actuation)
.venv/bin/python simulation/mujoco/validate.py

# interactive GUI (macOS needs mjpython, shipped with the mujoco wheel)
.venv/bin/mjpython simulation/mujoco/view.py
```

## Model notes
- REP-103 frame (x fwd, y left, z up); geometry/mass from `architecture.md` §8b.
- Base has a free joint so it stands on its wheels + fore/aft casters (nv = 6 + 17 = 23).
- Actuators: wheels are velocity-controlled; servos are `position` (kp≈30, models the
  STS3215 onboard PID); the lift is a stiff `position` drive (models a non-backdrivable
  lead screw holding the upper body against gravity).
- Grippers are modeled as one moving finger vs a fixed one → 1 DOF each, as specified.

Refine link shapes/inertia and swap in SO-101 CAD meshes later without changing joint
names or the DOF set — the contract is what downstream stages depend on.

## Not in this stage
URDF export, `robot_state_publisher`, TF and RViz are **Stage 6** (ROS integration).
Stage 5 is the physics model + direct-command validation.
