# Nori Robotics — Project Summary

Open-source, low-cost Nori-like household robot. Target scope is **end-to-end
parity** (not a progressively simplified mobile base). Initial target: a
humanoid-form-factor, wheeled robot with two arms/grippers, built from
commodity/open-source components, **sub-£500 BOM**, for the hobbyist market.

**Philosophy:** Build the complete robotics stack, but make every layer modular
and replaceable.

## Long-term vision
An open robotics platform (open hardware + open software) where the £500 robot is
the developer/hobbyist reference. Hardware upgrades over time (actuators → cameras
→ compute → manipulation → autonomy) toward a consumer-grade robot, while the
**software architecture stays largely unchanged**.

## Target robot (V1)
Wheeled mobile base, humanoid-ish torso, head, two arms, two grippers, multiple
arm joints, cameras, IMU, wheel encoders, joint feedback, onboard compute,
battery + BMS, motor/actuator controllers, wireless, e-stop/hardware safety,
real-time firmware, ROS 2, SLAM/localization, navigation, object perception,
kinematics, manipulation, voice/UI, AI/task planning. Exact DOF and parts are
derived from the £500 constraint.

## Layered architecture
```
L7 APPLICATION   Household tasks / UI / voice / skills
L6 COGNITION     LLM / VLM / task planning / world model
L5 ROBOTICS      Perception / SLAM / navigation / manipulation / IK
L4 ROBOT RUNTIME ROS 2 / TF / controllers / hardware interfaces
L3 REAL-TIME     MCU / PID / trajectories / actuator control / safety
L2 HARDWARE      Motors / encoders / cameras / IMU / grippers
L1 POWER+PHYS    Battery / BMS / regulators / distribution / CAN
```
Crucial boundary: **Linux/ROS 2 = high-level robotics; MCUs = deterministic
low-level hardware control.**

## Physical modules (interfaces, not a monolith)
POWER (battery, BMS, distribution, DC/DC, safety) · COMPUTE (SBC, storage,
network) · CONTROL (MCU, CAN, actuator controllers) · BASE (L/R drive, encoders)
· LEFT ARM (joints, gripper) · RIGHT ARM (joints, gripper) · HEAD (camera, depth
camera, IMU, microphone).

## Electrical architecture
Battery → BMS → power distribution → {motor power → actuators, 12/9V →
peripherals, 5/3.3V → compute/MCU}. Centralized power management, fusing, current
monitoring, hardware e-stop, regulated rails, clean motor/logic power separation.
Battery voltage TBD from actuator selection.

## Communication
Robot-wide bus: **CAN/CAN-FD**. SBC → CAN-FD → CAN backbone → {Base MCU, Arm MCU,
Arm MCU}. Higher-level software talks to an abstract actuator interface regardless
of the underlying actuator (hobby servo / Dynamixel / DC / BLDC / custom).

## Smart actuator abstraction
Motor → gearbox → encoder → MCU/driver → CAN, reporting current/temp/position/
velocity/fault. Interface: COMMAND {position, velocity, torque/current, mode};
STATE {position, velocity, current, temperature, faults}. Lets actuator hardware
evolve without rewriting the stack.

## MCU choice
**STM32G4 + FreeRTOS + CAN-FD** (control-oriented: advanced timers/PWM, encoder
interfaces, ADCs, FPU, CAN-FD, DMA, watchdogs, UART/SPI/I²C). Several MCUs
(Base + Left-arm + Right-arm), not one per actuator — £500 constraint. Final
actuator architecture derived from cost.

## Firmware architecture
`HAL {stm32, simulator}` · `drivers {motor, encoder, IMU, CAN}` · `control {PID,
velocity, position}` · `safety {watchdog, limits, e-stop}` · `protocol {commands,
telemetry}`. Rule: **hardware-dependent code is a thin layer**; control, protocol,
state machines and safety are testable without hardware.

## ROS 2
Runs on the SBC: robot_state_publisher, ros2_control, Nav2, SLAM, MoveIt 2,
perception, task planner. The MCU does not run ROS 2. Boundary: ROS 2 → commands
→ CAN → MCU {PID, PWM, encoder, safety} → physical actuator. micro-ROS optional
later.

## Development without hardware
Develop on the Mac: VS Code, Git, CMake/Ninja, ARM toolchain, Docker → Ubuntu →
ROS 2. GPU/cloud only later (VLM inference, training, large sim, RL, datasets).

## Firmware simulation — three levels
A. **Native software sim** — hardware-independent firmware on the Mac (PID ↔ sim
motor ↔ sim encoder); fast unit/integration testing.
B. **MCU emulation** — Renode runs the actual compiled firmware ELF against a
virtual STM32 + peripherals.
C. **Hardware-in-the-loop** — real STM32 over CAN drives a simulated robot /
physics model; test failures (stall, encoder failure, CAN loss, battery sag,
thermal, watchdog, overcurrent, disconnected actuator) before the physical robot.

## Full robot simulation
Nori digital twin (base dynamics, arm joints, sensor cameras) → ROS 2 → **same
robot software** → {simulation | hardware}. Gazebo/MuJoCo: what does the physical
robot do? Renode: what does the firmware execute? Motor/encoder sim: does the
controller behave? We want all three.

## Key architectural principle
```
        SIMULATION
            │  same interfaces
            ▼
        Nori APIs
       ┌────┴────┐
   Firmware    ROS 2
   HAL/API    robotics
       └────┬────┘
         HARDWARE
```
Simulation is another implementation of the same interfaces as the physical
robot. Path: £0 software dev → £100 hardware test rig → £500 complete open robot
→ increasingly capable consumer platform, without redesigning the architecture at
each stage.
```

## First development environment
```
Mac → Docker/Ubuntu → { ROS 2, Gazebo or MuJoCo, CMake, ARM GCC, FreeRTOS, Renode }
```
