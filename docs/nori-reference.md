# Reference: the real Nori robot (Nori Robotics)

Background on the product our project is modeled on. Not our design — a reference.
Two confidence tiers below: **official** (norirobotics.com) vs **secondary**
(Launch HN, press, the open-source SDK).

## Official (norirobotics.com, A3)
- **NORI A3**, **$1,688** full price, **ships fall 2026**, assembled in San Francisco (YC-backed).
- Bimanual (two-armed) robot. **Arms: 7+1 DOF, 1.5 kg payload per arm.**
- **LiDAR:** 12 m range, 8–12 Hz, 0.72° angular resolution @ 10 Hz.
- **Cameras:** 4× 720p RGB, up to 30 fps, on grippers / head / neck.
- Speaker + microphone for spoken commands. **6–8 h battery.**
- **Nori Lab** laptop app (train/operate/manage). **Skills Marketplace.**
- Tasks: kitchen help, tidying, fetching from fridge, loading dishes, folding clothes, pouring.

## Secondary (HN / press / SDK — medium confidence)
- **Form:** wheeled differential-drive base (~45×45 cm) + three-stage telescoping lift column
  (~69→145 cm) + pan-tilt head. ~20 kg. **Not a biped.** Models: L1, L2, L2 Grande, A3.
- **Actuators:** Feetech **STS3215** serial-bus servos (no QDD motors, no force-torque, no tactile);
  servo current used as a grip-force proxy. 3D-printed TPU gripper fingers.
- **Compute:** single **Raspberry Pi 5 (4 GB)** — deliberate "thin client"; Jetson rejected for cost.
- **Split brain:** Pi runs SLAM + safety watchdog + on-board IK; heavy **ACT** / **VLA** policies run
  off-board (PC/LAN or server/WAN).
- **Middleware:** custom "nori-protocol" over **WebRTC** (aiortc/GStreamer, Supabase signaling);
  **ROS 2 gateway** for interop (REP-103). ROS 2 is interop, not the core runtime.
- **Open-source lineage:** Hugging Face **LeRobot** ecosystem; arms derive from **SO-100/SO-101**;
  design inspired by **XLeRobot**; imitation learning via **ACT**; data pipeline on Hugging Face.
  Apache-2.0 SDK: `github.com/Nori-Robotics/nori-sdk-py`. Some hardware + 3D-print files open.
- **No evidence of Nav2 / MoveIt** — navigation appears custom and, per SDK v1.1.0 docs, "unproven on
  hardware"; live `perceive()` returns null. Reliable use today ≈ teleop + narrow trained skills.

## Sources
- https://www.norirobotics.com/
- https://www.ycombinator.com/companies/noril1
- https://news.ycombinator.com/item?id=49525153 (Launch HN)
- https://www.humanoidsdaily.com/news/the-1-688-appliance-bet-nori-robotics-launches-wheeled-bimanual-manipulator
- https://newatlas.com/robotics/household-robot-chores-nori-a3/
- https://github.com/Nori-Robotics/nori-sdk-py

## Implications for our build (see docs/architecture.md)
- ✅ We already match: Feetech STS3215 class servos, Raspberry Pi SBC, ROS 2, wheeled bimanual.
- 🔀 We add an **STM32G4 + CAN-FD real-time firmware layer** the real Nori doesn't have (they drive
  servos straight off the Pi). Deliberate — deterministic low-level control — but extra vs cheapest path.
- 🔀 Real Nori pushes ACT/VLA **off-board** (thin client); our plan leans on on-SBC robotics. Decide at Stage 7+.
- 💡 Reuse opportunity: **SO-101** arm geometry for our Stage 5 URDF; **LeRobot + ACT** for the learning layer.
- Their arms are **7+1 DOF**; our frozen design is **5+gripper**. Reconsider if we want parity.
