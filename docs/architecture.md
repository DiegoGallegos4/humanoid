# Nori — Architecture Contract (Stage 1)

> This is the **frozen interface contract** every downstream stage builds
> against. It fixes *interfaces and topology* (joint count, actuator classes,
> voltage domain, compute/actuator I/O layout, sensors, safety), not final part
> numbers. Because sim and hardware share these interfaces, nothing here assumes
> we buy anything — it's equally the spec for the MuJoCo/Renode digital twin.
>
> Status: **FROZEN v2** (v1 approved 2026-09-07; v2 2026-09-07). Changes require
> a version bump. Decisions locked: SO-101 5-DOF+gripper arms; **vertical lift
> column (17 DOF total)**; **direct-serial transport** — the Pi drives the smart
> servos directly over the Feetech bus, and a single Base MCU handles the DC
> wheels + lift (§4); reference geometry set (§8b).
>
> **v2 changes** (from v1's 16-DOF, 3-MCU/CAN-backbone design): (1) added the
> torso lift joint → 17 DOF; (2) dropped the per-arm MCUs and the robot-wide
> CAN-FD backbone — arms/head are driven straight from the Pi over the Feetech
> serial bus (like the real Nori), and only the wheeled base keeps an MCU.

---

## 1. Degrees of freedom

Humanoid-ish wheeled base, two 5-DOF arms + grippers, pan/tilt head.

| Module | Joint | Actuated | Notes |
|--------|-------|:--------:|-------|
| Base | left wheel | ✓ | differential drive |
| Base | right wheel | ✓ | differential drive |
| Base | caster(s) | – | passive |
| Torso | lift column (prismatic) | ✓ | vertical reach — SO-101 arms + head ride the lift |
| Head | pan (yaw) | ✓ | |
| Head | tilt (pitch) | ✓ | |
| Left arm | shoulder pan (yaw) | ✓ | SO-101 layout |
| Left arm | shoulder lift (pitch) | ✓ | |
| Left arm | elbow | ✓ | |
| Left arm | wrist flex (pitch) | ✓ | |
| Left arm | wrist roll | ✓ | |
| Left arm | gripper | ✓ | parallel, 1-DOF |
| Right arm | (mirror of left) | ✓×6 | |

**Actuated DOF total = 17** — 2 base + 1 lift + 2 head + 6 per arm ×2.
(A vertical **lift column** gives household reach — floor → counter → high shelf —
and is the real Nori's headline capability. Torso *yaw* remains a future add-on.)

Rationale: adopts the **SO-101** (SO-ARM/LeRobot) 5-DOF + gripper layout — the
open-source community standard — for maximal CAD + learning-stack reuse (see
`docs/arm-dof-comparison.md`). Includes **wrist roll** for gripper orientation.
6/7-DOF full-pose parity with the real Nori is a documented sim/upgrade path.

---

## 2. Actuator classes

Three abstract actuator classes behind one **smart-actuator interface** (§6). The
upper stack never sees the physical type or which driver moves it.

| Class | Used for | Physical (reference) | Driven by | Feedback |
|-------|----------|----------------------|-----------|----------|
| **Serial-bus servo** | head (2), arms + grippers (12) | Feetech STS3215-class, half-duplex TTL bus, ~11.1 V | **Pi directly** (Feetech bus) | position, velocity, load/current, voltage, temperature |
| **DC gearmotor + encoder** | base wheels (2) | 12 V gearmotor + quadrature encoder, H-bridge driver | Base MCU | position (ticks), velocity, current (shunt) |
| **Linear lift actuator** | torso lift (1) | stepper + lead screw (non-backdrivable) | Base MCU | position (steps), homing switch |

The servos are *already* smart actuators (onboard position PID). A Pi-side
driver reads/writes their Feetech registers and presents them on the interface
in §6 — no MCU in the arm path. Only the "dumb" actuators (DC wheels, lift) need
the MCU's real-time control loops (Stage 3). Swapping to Dynamixel / BLDC later
changes only a driver, not ROS 2.

---

## 3. Power domain

Single **3S LiPo (11.1 V nominal, 12.6 V full, ~9 V empty)** — chosen because the
serial-bus servos and 12 V gearmotors both run directly off it, avoiding a
separate motor supply.

```
3S LiPo ─ main fuse ─ e-stop cutoff ─┬─ ACTUATOR RAIL (servos + motor driver)   [fused]
 (+ 3S BMS/protection)               ├─ 5 V buck ─ SBC + logic + servo logic     [fused]
                                     └─ MCU 3.3 V (on-board LDO from 5 V/rail)
```

- **E-stop cuts the actuator rail only** — logic/SBC/MCUs stay alive to report
  state and hold safe. (Modeled in sim as an actuator-rail power flag.)
- Current sensing on the actuator rail (shunt / INA219-class) feeds the safety layer.
- Rough current budget: 14 servos (running ~0.5–1 A ea, stall ~2.5 A), 2 motors
  (~1–2 A ea), Pi ~3–5 A @5 V → design for **~25–30 A peak**; ~5000 mAh 3S,
  ≥30 A discharge.

---

## 4. Compute + actuator I/O topology

Direct-serial (mirrors the real Nori): the Pi talks the smart servos itself; one
Base MCU owns the real-time base + lift. No per-arm MCUs, no CAN backbone.

```
        Raspberry Pi 5 (8 GB) — ROS 2 Jazzy, SLAM, nav, IK, coordination
          │
          ├─ USB → Feetech bus adapter ─ half-duplex TTL servo bus
          │         └─ 14× STS3215-class servos, daisy-chained:
          │            head pan/tilt · left arm ×6 · right arm ×6
          │
          └─ USB-serial → Base MCU (STM32G4, FreeRTOS)
                           ├ 2× DC wheel motor  (PWM + quadrature encoder)
                           ├ 1× lift actuator   (stepper + lead screw, homing switch)
                           ├ IMU (I²C/SPI)
                           └ actuator-rail current sense + e-stop status
```

- **SBC:** Raspberry Pi 5 (8 GB). Runs the servo-bus driver + high-level stack;
  heavy inference (VLA/ACT/VLM) offloaded off-board (see §4b). No onboard GPU.
- **MCU:** 1× STM32G4 (Cortex-M4F) for the wheeled base — real-time DC-motor
  control (our HAL + PID, Stage 3), encoders, the lift, IMU, and safety. Links
  to the Pi over USB-serial.
- **Servo bus:** the 14 arm/head servos hang off one Feetech TTL bus straight
  from the Pi — no MCU in that path. (A second bus/adapter can split arms L/R if
  bandwidth demands; single bus is the baseline.)
- **Bus framing:** the base link carries the same actuator command/state
  messages (`docs/can-protocol.md`) — transport-agnostic, so it rides USB-serial
  now and could ride CAN unchanged if more MCU nodes are ever added.

---

## 4b. Compute split (where AI runs)

Like the real Nori, the robot is a **thin client** — no onboard GPU.

| Tier | Runs on | Work |
|------|---------|------|
| Real-time (≤1 kHz) | Base MCU | DC-motor + lift PID, encoders, safety, watchdog |
| On-robot (Pi 5, CPU) | Raspberry Pi 5 | servo-bus driver, ROS 2, SLAM, nav, IK, state, camera capture, safety supervisor |
| Off-board (GPU) | dev box / cloud | heavy policies — VLA / ACT / VLM inference, training |

The off-board tier is optional and only for learned manipulation; teleop and
scripted/IK motion work with just the Pi. In sim, the "off-board" tier is just
another process — no network needed until we choose to split it.

---

## 5. Sensors

| Sensor | Location | Interface | Owner |
|--------|----------|-----------|-------|
| RGB camera | head | USB / CSI | Pi |
| Depth (RGB-D) | head | USB | Pi — **stretch** (cost line); sim always models it |
| IMU | base/torso | I²C or SPI | Base MCU → Pi |
| Wheel encoders | base | quadrature | Base MCU |
| Lift position | torso | step count + homing switch | Base MCU |
| Joint feedback | arm/head servos | Feetech serial bus | Pi (servo driver) |
| Microphone | head | USB | Pi |

---

## 6. Smart-actuator interface (the contract)

Every actuated joint, sim or real, exposes:

```
COMMAND   mode ∈ {idle, position, velocity, torque/current}
          target_position   [rad]
          target_velocity   [rad/s]
          target_effort     [A or Nm]

STATE     position [rad]   velocity [rad/s]   effort/current [A]
          temperature [°C] voltage [V]        fault_flags (bitfield)
```

Fault flags (bitfield): overcurrent, overtemperature, position-limit,
velocity-limit, comm-timeout, encoder-fault, driver-fault, under-voltage.

This maps 1:1 onto `ros2_control` joint interfaces and the actuator message set
(Stage 4), **regardless of transport** — Feetech register R/W for the servos
(Pi-side driver), USB-serial frames for the Base MCU (wheels + lift). Joint
naming: `<side>_<segment>` e.g. `left_shoulder_pan`, `right_gripper`,
`head_pan`, `base_left_wheel`, `torso_lift`.

---

## 7. Safety architecture

| Layer | Mechanism |
|-------|-----------|
| Hardware | latching e-stop cuts actuator rail; main fuse; per-rail fuses |
| MCU | independent watchdog (IWDG); wheel/lift velocity/current limits; safe-state on fault |
| Comms | heartbeat Pi→Base MCU (serial) and Pi→servo bus; on timeout, MCU ramps the base/lift to a safe hold and the servo driver commands hold/torque-off |
| Feedback | over-current / over-temp / under-voltage shutdown from servo + rail telemetry |

All of these are **modeled in simulation** (fault injection is a Stage 6b goal),
so safety logic is tested without hardware.

---

## 8. Indicative BOM (sub-£500 target)

Not a purchase list (we're not buying) — a feasibility check that the contract
fits the budget.

| Item | Qty | ~£ |
|------|----:|---:|
| Serial-bus servos (STS3215-class) | 14 | 196 |
| Feetech USB bus adapter (Pi → servo bus) | 1 | 6 |
| DC gearmotor + encoder | 2 | 30 |
| Motor driver (dual H-bridge) | 1 | 8 |
| STM32G4 board (WeAct-class, base) | 1 | 8 |
| Lift: NEMA17 stepper + lead screw + driver | 1 | 22 |
| Raspberry Pi 5 (8 GB) | 1 | 75 |
| microSD / storage | 1 | 15 |
| RGB camera | 1 | 20 |
| IMU (BNO085-class) | 1 | 15 |
| USB microphone | 1 | 8 |
| 3S LiPo + BMS/protection | 1 | 38 |
| 5 V buck converter | 1 | 8 |
| Power distribution + fuses + e-stop | 1 | 20 |
| Chassis (3D print + hardware, wheels/casters) | 1 | 40 |
| Wiring / connectors | – | 20 |
| **Total (indicative)** | | **~529** |

**Trim levers to reach <£500** (decide later; doesn't affect sim): cheaper SBC
(Pi 4 / Orange Pi, −£30–40); 5→4 DOF per arm (−2 servos ≈ −£28); a single
higher-torque servo lift instead of the stepper stage. Direct-serial already
removed ~£45 of MCU/CAN parts vs v1. Depth camera (~£70) is a stretch item,
always excluded from the base target.

---

## 8b. Reference geometry & mass (defaults for URDF)

Sensible defaults so Stage 5 (MuJoCo/URDF) is unblocked. Refine later without
changing the contract. Coordinate convention: REP-103 (x fwd, y left, z up).

| Segment | Dimension | Value |
|---------|-----------|------:|
| Base footprint | L × W | 0.30 × 0.30 m |
| Base height (top plate) | z | 0.15 m |
| Drive wheel diameter | ⌀ | 0.12 m |
| Wheel separation (track) | y | 0.26 m |
| Torso height (base top → shoulder), lift retracted | z | 0.35 m |
| **Lift travel (stroke)** | z | **0.30 m** |
| Shoulder height (floor → shoulder), lift retracted | z | 0.50 m |
| Shoulder lateral offset (each side) | y | ±0.13 m |
| Head height (floor → camera), lift retracted | z | 0.65 m |
| Upper arm (shoulder→elbow) | L | 0.12 m |
| Forearm (elbow→wrist) | L | 0.12 m |
| Wrist→gripper base | L | 0.05 m |
| Gripper finger length / stroke | — | 0.05 / 0.05 m |
| Arm reach (shoulder→gripper) | — | ~0.29 m |

Heights above are with the lift **retracted**; full extension adds up to
+0.30 m (shoulder → 0.80 m, head → 0.95 m).

| Body | Mass |
|------|-----:|
| Base (incl. battery) | ~2.5 kg |
| Lift column + carriage | ~0.4 kg |
| Torso | ~1.0 kg |
| Each arm | ~0.5 kg |
| Head | ~0.3 kg |
| **Total** | **~5.7 kg** |

Joint limits (defaults): revolute arm joints ±1.9 rad, elbow 0–2.4 rad, head pan
±1.5 rad / tilt ±0.7 rad, gripper 0–0.05 m, **torso lift 0–0.30 m (prismatic)**.
Wheels continuous.

## 9. What is frozen vs open

**Frozen (code may depend on it):** 17 actuated DOF and their names (incl.
`torso_lift`); three actuator classes + the smart-actuator interface (§6);
direct-serial topology — Pi drives the servo bus, one Base MCU drives wheels +
lift (§4); 3S power domain; sensor set; safety layers; reference geometry (§8b).

**Open (does not block sim):** exact part numbers, precise current budget,
depth-camera inclusion, single-vs-split servo bus, lift drive (stepper vs servo),
whether the base link is USB-serial or CAN.
