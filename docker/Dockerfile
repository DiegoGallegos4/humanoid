# Nori development environment.
# Ubuntu 24.04 + ROS 2 Jazzy + native/cross build toolchains.
# Works on Apple Silicon (arm64) and x86_64.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      curl \
      ca-certificates \
      cmake \
      ninja-build \
      git \
      gdb \
      clang \
      clang-format \
      # Cross toolchain for STM32G4 (Cortex-M4F)
      gcc-arm-none-eabi \
      # Unit test / simulation deps
      libgtest-dev \
      python3-pip \
      python3-colcon-common-extensions \
      # ROS 2 sim + control + viz (installed lazily; keep image lean if unused)
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      # Mapping / navigation (Stage 7A): SLAM + Nav2
      ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# --- MuJoCo (Sim Level C): physics engine, C lib + headers -------------------
# The pip wheel ships libmujoco.so + include/mujoco/*.h; the nori_hardware
# ros2_control plugin links against these to drive the sim in-process. Pinned to
# match simulation/mujoco (Stage 5). Ubuntu 24.04's python is externally managed,
# hence --break-system-packages (this is the container's only python).
RUN pip install --no-cache-dir --break-system-packages mujoco==3.12.0

# --- Renode (Sim Level B): run the real firmware ELF on a virtual STM32G4 ------
# Core to the sim-first strategy (no hardware). Used at Stage 6b.
# NOTE: the official portable releases are x86_64-only. On amd64 we install it
# here; on arm64 (Apple Silicon) it is NOT installed — the portable binary fails
# under emulation. arm64 Renode is a Stage 6b task (options: mono-based build,
# a dedicated emulated x86_64 sidecar, or a newer native arm64 release). See
# docs/PLAN.md Stage 6b.
ARG RENODE_VERSION=1.15.3
RUN set -eux; \
    arch="$(dpkg --print-architecture)"; \
    if [ "$arch" = "amd64" ]; then \
      url="https://github.com/renode/renode/releases/download/v${RENODE_VERSION}/renode-${RENODE_VERSION}.linux-portable-dotnet.tar.gz"; \
      curl -fL "$url" -o /tmp/renode.tar.gz; \
      mkdir -p /opt/renode; \
      tar -xzf /tmp/renode.tar.gz -C /opt/renode --strip-components=1; \
      rm /tmp/renode.tar.gz; \
      ln -s /opt/renode/renode /usr/local/bin/renode; \
    else \
      echo "Renode skipped on ${arch}: no official portable build. Resolve at Stage 6b."; \
    fi

# Auto-source ROS 2 for interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /nori
CMD ["/bin/bash"]
