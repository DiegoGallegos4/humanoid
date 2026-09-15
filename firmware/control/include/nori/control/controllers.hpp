#pragma once

// Position and velocity controllers — thin, typed wrappers over Pid that fix the
// semantics used by the actuator abstraction (docs/architecture.md §6):
//   * work in SI units (rad, rad/s)
//   * emit a normalized effort in [-1, 1] that the driver maps to voltage/PWM
#include "nori/control/pid.hpp"

namespace nori::control {

class PositionController {
 public:
  explicit PositionController(PidGains gains) : pid_(gains) {
    pid_.set_output_limits(-1.0f, 1.0f);
    pid_.set_integral_limits(-1.0f, 1.0f);
  }
  void set_gains(PidGains g) { pid_.set_gains(g); }
  void reset() { pid_.reset(); }

  // effort in [-1, 1]
  float update(float target_rad, float measured_rad, float dt) {
    return pid_.update(target_rad, measured_rad, dt);
  }

 private:
  Pid pid_;
};

class VelocityController {
 public:
  explicit VelocityController(PidGains gains) : pid_(gains) {
    pid_.set_output_limits(-1.0f, 1.0f);
    pid_.set_integral_limits(-1.0f, 1.0f);
  }
  void set_gains(PidGains g) { pid_.set_gains(g); }
  void reset() { pid_.reset(); }

  // effort in [-1, 1]
  float update(float target_rad_s, float measured_rad_s, float dt) {
    return pid_.update(target_rad_s, measured_rad_s, dt);
  }

 private:
  Pid pid_;
};

}  // namespace nori::control
