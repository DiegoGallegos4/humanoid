#pragma once

// Generic PID controller for the Nori firmware control layer.
//
// Hardware-independent and unit-tested (see docs/PLAN.md Stage 3). Features that
// matter for real actuators:
//   * derivative-on-measurement  — avoids a derivative "kick" on setpoint steps
//   * output clamping             — respects actuator effort limits ([-1, 1])
//   * integral clamping (anti-windup) — integral can't run away while saturated
#include <algorithm>

namespace nori::control {

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

class Pid {
 public:
  explicit Pid(PidGains gains) : gains_(gains) {}

  void set_gains(PidGains gains) { gains_ = gains; }
  void set_output_limits(float lo, float hi) { out_lo_ = lo; out_hi_ = hi; }
  void set_integral_limits(float lo, float hi) { i_lo_ = lo; i_hi_ = hi; }

  void reset() {
    integral_ = 0.0f;
    prev_measurement_ = 0.0f;
    has_prev_ = false;
  }

  // dt in seconds (> 0). Returns the clamped control effort.
  float update(float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;

    integral_ += gains_.ki * error * dt;
    integral_ = std::clamp(integral_, i_lo_, i_hi_);

    float derivative = 0.0f;
    if (has_prev_ && dt > 0.0f) {
      // derivative on measurement (note the sign)
      derivative = -gains_.kd * (measurement - prev_measurement_) / dt;
    }
    prev_measurement_ = measurement;
    has_prev_ = true;

    const float output = gains_.kp * error + integral_ + derivative;
    return std::clamp(output, out_lo_, out_hi_);
  }

  float integral() const { return integral_; }

 private:
  PidGains gains_;
  float out_lo_ = -1.0f;
  float out_hi_ = 1.0f;
  float i_lo_ = -1.0f;
  float i_hi_ = 1.0f;
  float integral_ = 0.0f;
  float prev_measurement_ = 0.0f;
  bool has_prev_ = false;
};

}  // namespace nori::control
