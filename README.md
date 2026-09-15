# Nori

Open-source, low-cost (sub-£500) Nori-like household robot: wheeled humanoid base
with two arms/grippers. Built simulation-first — every layer is modular so
simulated modules can be swapped for hardware one at a time.

- **Vision:** [`docs/project-summary.md`](docs/project-summary.md)
- **Roadmap:** [`docs/PLAN.md`](docs/PLAN.md)
- **Architecture contract:** [`docs/architecture.md`](docs/architecture.md) (Stage 1)

## Layout
```
firmware/   real-time control: core, hal/{stm32,sim}, drivers, control, safety, protocol, tests
ros/        ROS 2: description, hardware, control, navigation, manipulation
simulation/ mujoco, gazebo scenes
hardware/   electronics, actuators, mechanical
docs/       vision, plan, architecture
```

## Dev environment
Everything is developed on the Mac via a Docker/Ubuntu container (ROS 2 Jazzy +
build/cross toolchains). No hardware needed until Stage 8.

```bash
docker compose build dev
docker compose run --rm dev          # shell in /nori with ROS 2 + toolchains

# Inside the container — build & test firmware natively (sim HAL):
cmake -S firmware -B firmware/build -G Ninja
cmake --build firmware/build
ctest --test-dir firmware/build --output-on-failure
```

## Status
See the status tracker in [`docs/PLAN.md`](docs/PLAN.md). Currently: Stage 0/1.
