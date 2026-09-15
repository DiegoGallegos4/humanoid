#pragma once

// Actuator physics for Sim Level A: motor -> gearbox -> encoder.
//
// A lumped-parameter brushed-DC-motor model with a gear reduction, integrated
// with semi-implicit Euler. It is deliberately hardware-independent physics —
// controllers drive it, and it produces the encoder ticks the controllers read
// back, closing the loop entirely in software (docs/PLAN.md Stage 3).
//
// Electrical (inductance neglected):  i = (V - Ke*w) / R
// Torque:                             tau = Kt * i
// Mechanics (motor shaft):            J*dw/dt = tau - b*w - tau_load/gear
// Output shaft:                       theta_out = theta_motor / gear,
//                                     w_out = w_motor / gear
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nori::hal::sim {

struct DcMotorParams {
  float R = 1.2f;                     // winding resistance [ohm]
  float Kt = 0.018f;                  // torque constant [Nm/A]
  float Ke = 0.018f;                  // back-EMF constant [V/(rad/s)]
  float J = 5.0e-6f;                  // rotor inertia (motor shaft) [kg m^2]
  float b = 2.0e-6f;                  // viscous friction [Nm/(rad/s)]
  float gear_ratio = 50.0f;           // reduction (>1)
  float supply_v = 12.0f;             // bus voltage [V]
  int32_t counts_per_rev_out = 4096;  // encoder counts per output revolution
};

class DcMotorModel {
 public:
  explicit DcMotorModel(DcMotorParams p = {}) : p_(p) {}

  // Advance the model by dt seconds with a normalized effort in [-1, 1]
  // (effort * supply_v = applied terminal voltage).
  void step(float effort, float dt) {
    const float voltage =
        std::clamp(effort, -1.0f, 1.0f) * p_.supply_v;
    step_voltage(voltage, dt);
  }

  // Advance with an explicit terminal voltage [V].
  void step_voltage(float voltage, float dt) {
    voltage = std::clamp(voltage, -p_.supply_v, p_.supply_v);
    current_ = (voltage - p_.Ke * w_motor_) / p_.R;
    const float tau = p_.Kt * current_;
    const float tau_load_motor = load_torque_out_ / p_.gear_ratio;
    const float domega = (tau - p_.b * w_motor_ - tau_load_motor) / p_.J;
    w_motor_ += domega * dt;          // semi-implicit: velocity first
    theta_motor_ += w_motor_ * dt;    // then position from new velocity
  }

  // External load on the OUTPUT shaft [Nm] (e.g. gravity, contact). Default 0.
  void set_load_torque(float tau_out) { load_torque_out_ = tau_out; }

  // --- output-shaft state ---
  float position_rad() const { return theta_motor_ / p_.gear_ratio; }
  float velocity_rad_s() const { return w_motor_ / p_.gear_ratio; }
  float current_a() const { return current_; }

  int32_t encoder_count() const {
    return static_cast<int32_t>(std::lround(
        position_rad() / (2.0f * static_cast<float>(M_PI)) *
        p_.counts_per_rev_out));
  }

  const DcMotorParams& params() const { return p_; }

  // Convenience: ticks -> output radians (for controllers reading an encoder).
  static float counts_to_rad(int32_t counts, int32_t counts_per_rev) {
    return static_cast<float>(counts) / static_cast<float>(counts_per_rev) *
           2.0f * static_cast<float>(M_PI);
  }

 private:
  DcMotorParams p_;
  float w_motor_ = 0.0f;      // motor-shaft angular velocity [rad/s]
  float theta_motor_ = 0.0f;  // motor-shaft angle [rad]
  float current_ = 0.0f;      // last winding current [A]
  float load_torque_out_ = 0.0f;
};

}  // namespace nori::hal::sim
