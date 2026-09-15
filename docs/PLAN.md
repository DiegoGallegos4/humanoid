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

### Stage 7 — Higher-level robotics in sim
Deliverable: navigation (Nav2), SLAM, and manipulation (MoveIt 2) bring-up in
`ros/nori_navigation` and `ros/nori_manipulation`, all against the simulated robot.
Done when: the sim robot navigates a mapped environment and executes a planned
arm motion to a pose.

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
├── ros/           nori_description, nori_hardware, nori_control, nori_navigation, nori_manipulation
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
- [ ] Stage 7 — nav / SLAM / manipulation
- [ ] Stage 8 — minimum hardware (FUTURE / out of scope)
