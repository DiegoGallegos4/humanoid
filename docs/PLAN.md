# Nori — Simulation-First Development Plan

> **Principle:** Simulation is not a throwaway prototype. It is *another
> implementation of the same interfaces* as the physical robot. We build the
> entire stack virtually first, then swap simulated modules for hardware one at
> a time without redesigning the architecture.

See `docs/project-summary.md` for the full vision. This document is the
executable roadmap: ordered stages, each with a concrete deliverable and a
"done when" test. We develop entirely on the Mac (native + Docker/Ubuntu) until
Stage 8.

---

## North star

**Autonomous bimanual mobile pick-and-place: load dishes into a rack, then into
a dishwasher.** This is the long-term capability target that all Stage-7 work
aims at. It is long-horizon, contact-rich, dual-arm manipulation combined with
mobile-base navigation and an articulated appliance (the dishwasher door + rack).

Strategy — **de-risk manipulation early.** The mobile base is comparatively
solved (odometry done, SLAM/Nav2 are well-trodden); the manipulation +
perception stack is greenfield and the true long pole, so we stand up a static
pick-and-place milestone before combining with navigation.

**The one decision that keeps us from undershooting:** object pose is a
*swappable interface* (`PoseSource`), mirroring the sim/hardware interface
philosophy already in the repo. v1 = ground-truth pose from the sim (exact,
free); v2 = estimate from the head depth camera — swapped in without touching
the manipulation code. Likewise MoveIt uses a **dual-arm planning group from day
one**, and task sequencing uses **BehaviorTree.CPP** (the same engine Nav2 uses).

---

## Locked decisions

- **Simulation is the product (for now).** We are **not** buying any hardware.
  The goal is a complete, fully simulated Nori digital twin. Physical hardware
  (Stage 8) is explicitly **future / out of scope** — the architecture keeps the
  door open, but no components are purchased.
- **All three sim levels are first-class**, not optional. In particular **Renode
  (Sim Level B) is a core deliverable**, not a pre-hardware gate: it is how the
  *actual compiled firmware* runs against a virtual STM32G4 + peripherals, so the
  real firmware is exercised without any board.
- **Workflow:** step-by-step — each stage is completed, tested, and reviewed
  before the next begins.
- **Primary simulator:** MuJoCo (native on Mac, strong contact/manipulation
  physics). Gazebo optional later for heavier ROS 2 integration.
- **Firmware language:** C++17, abstract HAL via interfaces, GoogleTest units,
  builds with arm-none-eabi-g++.
- **Direct-serial transport (arch v2):** the Pi drives the 14 arm/head Feetech
  smart servos directly over their serial bus; a single Base MCU (STM32G4) owns
  the 2 DC wheels + lift. No per-arm MCUs, no CAN backbone (like the real Nori).
  The Stage-4 message set stays the transport-agnostic contract. See
  `docs/architecture.md` §4.
- **17 DOF:** added a vertical **torso lift column** (prismatic) for household
  reach — the real Nori's headline capability. (Was 16.)

## Guiding constraints

- **£500 total BOM** for the reference robot (drives all component choices).
- **Modular, replaceable layers** — every layer talks to the layer below through
  an interface, so sim and hardware are interchangeable.
- **The MCU/Linux boundary is fixed:** Linux/ROS 2 does high-level robotics; MCUs
  do deterministic low-level control. micro-ROS is optional and must not dictate
  firmware architecture.
- **Everything hardware-dependent is a thin layer** (HAL). Control, protocol,
  state machines and safety are hardware-independent and unit-testable on the Mac.

---

## The three simulation levels (targets, built incrementally)

| Level | Tool | Answers | Introduced |
|-------|------|---------|-----------|
| A. Native software sim | plain C++ on Mac | Does our controller behave correctly? | Stage 3 |
| B. MCU emulation | Renode + STM32G4 ELF | What does the *real firmware* execute? | Stage 6 |
| C. Full robot sim | MuJoCo | What does the physical robot do? | Stage 5 |

All three are core. The end state is a single closed loop with **no hardware**:

```
MuJoCo robot (C) ⇄ ROS 2 ⇄ CAN (sim transport) ⇄ Renode: firmware ELF (B) ⇄ virtual actuators (A)
```

---

## Stages

### Stage 0 — Repo scaffold + dev environment  ✅ (in progress)
Deliverable: directory tree, git repo, Docker/Ubuntu image with ROS 2 + toolchain,
build system skeleton (CMake).
Done when: `docker compose run dev` gives a shell with ROS 2, arm-none-eabi-gcc,
cmake/ninja; native firmware tests build on the Mac.

### Stage 1 — Freeze the component architecture  ← START HERE
Deliverable: `docs/architecture.md` fixing the interface contract:
number of joints & DOF, actuator types per joint, battery voltage, current
budget, MCU topology, CAN topology, sensor list, SBC choice, power architecture,
safety architecture. These are *interfaces*, not final part numbers.
Done when: every downstream stage can be built against these numbers without
guessing. This is a decision doc, reviewed before code depends on it.

### Stage 2 — Firmware core + HAL
Deliverable: `firmware/` with a HAL interface (PWM, ADC, encoder, CAN, GPIO,
timer, watchdog) and **two implementations**: `hal/sim` (runs on Mac) and
`hal/stm32` (stub/build-only for now). Language: C++17, unit-tested with the
native compiler.
Done when: firmware core builds for native (sim HAL) and cross-compiles for
STM32G4 (stub HAL); HAL contract has passing unit tests against the sim impl.

### Stage 3 — Actuator simulation + control loops  (Sim Level A)
Deliverable: motor → gearbox → encoder physics model in `hal/sim`; PID position
and velocity controllers in `firmware/control`.
Done when: closed loop `PID → sim motor → sim encoder → PID` holds a position
setpoint and tracks a velocity ramp in a unit test, with plots/logs.

### Stage 4 — CAN protocol definition
Deliverable: `firmware/protocol` — the ROS 2 ↔ MCU ↔ actuator contract. Message
set for command (position/velocity/torque/mode) and state (position/velocity/
current/temperature/faults), plus a transport abstraction with an in-process sim
transport. Single source of truth (e.g. a schema) shared by firmware and ROS.
Done when: a host process can command the firmware sim over the sim transport and
read back state; round-trip test passes.

### Stage 5 — Simulated robot model  (Sim Level C)
Deliverable: full URDF/robot description (base, torso, head, 2 arms, 2 grippers,
cameras, IMU) in `ros/nori_description`; MuJoCo and/or Gazebo scene in
`simulation/`.
Done when: robot loads in the simulator, joints move via direct sim commands, and
`robot_state_publisher` + RViz show correct TF.

### Stage 6 — Connect ROS 2 end-to-end
Deliverable: `ros2_control` hardware interface (`ros/nori_hardware`) that speaks
the CAN abstraction → firmware sim → actuator sim → robot sim. Controllers in
`ros/nori_control`.
Done when: a ROS 2 command (e.g. joint trajectory) drives the simulated robot
through the *real* firmware control loops (initially the native `hal/sim`).

### Stage 6b — Firmware under Renode  (Sim Level B, core deliverable)
Deliverable: run the **actual compiled STM32G4 firmware ELF** under Renode against
virtual peripherals + a virtual CAN, behind the same CAN transport as Stage 6.
This replaces the native `hal/sim` loop with the true embedded binary — the
highest-fidelity firmware sim we can reach without a board.
Done when: the full loop `MuJoCo ⇄ ROS 2 ⇄ CAN ⇄ Renode(firmware ELF) ⇄ virtual
actuators` closes, and fault injection (stall, encoder failure, CAN loss, battery
sag, thermal, watchdog, overcurrent, disconnected actuator) behaves correctly.
- **Known issue to resolve here:** Renode ships x86_64-only portable builds; it
  does not run on the arm64 (Apple Silicon) dev container. Options: mono-based
  build on arm64, a dedicated emulated x86_64 Renode sidecar container, or a
  native arm64 release if available. Not needed before Stage 6b.

### Stage 7 — Higher-level robotics in sim  (toward the north star)
Higher-level autonomy, built as a milestone ladder aimed at *dishes → rack →
dishwasher*. Each milestone is a testable deliverable; we complete and review one
before the next. The base track (7A) and the manipulation track (7B–7D) can
progress independently and converge at 7E.

**Locked-now decisions (so we don't undershoot):**
- Object pose is a swappable `PoseSource` interface — v1 sim ground-truth,
  v2 depth-camera estimate — in a new `ros/nori_perception`.
- MoveIt 2 is the arm planning stack, with a **dual-arm** planning group from the
  start (`ros/nori_moveit_config`, `ros/nori_manipulation`).
- Task sequencing uses BehaviorTree.CPP (shared with Nav2) in `ros/nori_bt`.
- World assets (kitchen bench, dish rack, dishwasher w/ hinged door, dish set)
  are first-class and versioned under `simulation/mujoco/assets/`.
- Head camera gains a **depth** stream; gripper finger friction tuned for holding
  smooth crockery.

**7A — Mobile base autonomy** *(in progress)*
odometry ✅ → teleop ✅ → 2D lidar (LaserScan) ✅ → SLAM (slam_toolbox) ✅ →
Nav2 goal-to-pose. The lidar is a `mj_ray` fan cast from the `lidar` MJCF site
inside the nori_hardware plugin, masked to geom group 2 (mappable environment);
scene.xml gains a 6×6 m room so there is something to scan. SLAM lives in
`ros/nori_navigation` (slam_toolbox async, run as a lifecycle node auto-driven
to active); the scan is stamped with the node's ROS clock, not the steady clock
controller_manager hands `write()`, so it lines up with the TF tree.
Done when: the sim robot maps a room and drives to a commanded pose.

**7B — Manipulation foundation** *(the long pole — de-risk early)*
MoveIt 2 up on the arms; single arm picks a rigid object (mug) from a **known**
pose and places it at a target; gripper driven through ros2_control.
Done when: plan → execute pick-and-place succeeds repeatably in MuJoCo.

**7C — Perception-in-the-loop**
Head depth camera → 6-DOF object pose module; swap the ground-truth `PoseSource`
for the estimate behind the same interface.
Done when: 7B pick-and-place works from an *estimated* pose.

**7D — Rack loading** *(static base)*
Sequence 3+ dishes into slotted rack targets; introduce the behavior tree and a
simple world model; collision-aware placement.
Done when: 3+ dishes racked without collision, driven by one behavior tree.

**7E — Mobile manipulation**
Combine 7A + 7D: navigate to the bench, then rack a dish, all in one behavior
tree (base + arm coordination, torso lift for reach).
Done when: the base drives to the bench and the arm racks a dish autonomously.

**7F — North star: dishwasher**
Articulated dishwasher door (open/close, bimanual), pull the rack, load dishes,
close. Long-horizon task plan.
Done when: a full dish-to-dishwasher cycle completes in sim.

### Stage 8 — Minimum hardware bring-up  (FUTURE / OUT OF SCOPE)
Not planned now — we are not buying components. Kept only to show the
architecture supports it: when/if desired, buy an STM32G4 board, CAN transceiver,
one actuator, PSU and debugger, and swap `hal/sim`/Renode for real `hal/stm32` on
one joint. Because sim and hardware share interfaces, the upper stack is unchanged.

---

## Repository layout

```
nori/
├── firmware/      core, hal/{stm32,sim}, drivers, control, safety, protocol, tests
├── ros/           nori_description, nori_hardware, nori_bringup, nori_navigation,
│                  nori_perception, nori_moveit_config, nori_manipulation, nori_bt
├── simulation/    mujoco, gazebo
├── hardware/      electronics, actuators, mechanical
└── docs/          project-summary.md, PLAN.md, architecture.md
```

## Status tracker

- [x] Stage 0 — scaffold + dev env (container builds; ROS 2 Jazzy + cmake/ninja + arm-gcc + gtest verified, firmware builds & tests pass inside it; Renode arm64 deferred to 6b)
- [x] Stage 1 — freeze architecture (`docs/architecture.md` FROZEN v2: direct-serial topology, 17 DOF incl. lift)
- [x] Stage 2 — firmware core + HAL (native build + 11 HAL tests pass; stm32 stub compiles)
- [x] Stage 3 — actuator sim + control (DC-motor+gearbox+encoder model, PID + pos/vel controllers; closed-loop tests pass, 19 total)
- [x] Stage 4 — actuator protocol (message set + codec, `docs/can-protocol.md`; round-trip + bus loopback tests pass, 27 total incl. 17-DOF/lift check)
- [x] Stage 5 — simulated robot model (MuJoCo `simulation/mujoco/`: 17-DOF robot, base+lift+head+2 SO-101 arms; `validate.py` gate passes — contract, stable physics, joints track commands)
- [x] Stage 6 — ROS 2 end-to-end (`ros/`: URDF + ros2_control; `nori_hardware/NoriMujocoSystem` plugin drives MuJoCo in-process; `test_bringup.sh mujoco` passes — trajectory → controllers → physics → `/joint_states` tracks with realistic dynamics)
- [ ] Stage 6b — firmware ELF under Renode (Sim Level B)
- [~] Stage 7 — higher-level autonomy toward *dishes → rack → dishwasher* (north star)
  - [~] 7A mobile base: odometry ✅, teleop ✅, 2D lidar ✅ (`/scan`, `test_lidar.sh`), SLAM ✅ (slam_toolbox, `/map`, `test_slam.sh`), next: Nav2
  - [ ] 7B manipulation foundation (MoveIt 2, single-arm pick-and-place)
  - [ ] 7C perception-in-the-loop (depth cam → swappable object pose)
  - [ ] 7D rack loading (behavior tree, multi-dish)
  - [ ] 7E mobile manipulation (Nav2 + manipulation)
  - [ ] 7F dishwasher (articulated door, full cycle)
- [ ] Stage 8 — minimum hardware (FUTURE / out of scope)
