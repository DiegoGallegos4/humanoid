// Stage 3 — actuator physics + PID closed-loop tests (Sim Level A).
// The controller reads position via the HAL Encoder interface and drives the
// DC-motor model; the loop closes entirely in software.
#include <gtest/gtest.h>

#include <cmath>

#include "nori/control/controllers.hpp"
#include "nori/control/pid.hpp"
#include "nori/hal/sim/sim_actuator.hpp"
#include "nori/hal/sim/sim_hal.hpp"

using namespace nori::control;
using namespace nori::hal::sim;

namespace {
constexpr float kDt = 0.001f;  // 1 kHz control loop
}

// ---------------------------------------------------------------------------
// PID unit behaviour
// ---------------------------------------------------------------------------
TEST(Pid, OutputIsClamped) {
  Pid pid({/*kp=*/100.0f, 0.0f, 0.0f});
  pid.set_output_limits(-1.0f, 1.0f);
  EXPECT_FLOAT_EQ(pid.update(10.0f, 0.0f, kDt), 1.0f);   // huge +error -> +1
  EXPECT_FLOAT_EQ(pid.update(-10.0f, 0.0f, kDt), -1.0f);  // huge -error -> -1
}

TEST(Pid, IntegralIsAntiWindupClamped) {
  Pid pid({0.0f, /*ki=*/10.0f, 0.0f});
  pid.set_integral_limits(-0.5f, 0.5f);
  for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, kDt);
  EXPECT_LE(pid.integral(), 0.5f + 1e-6f);
  EXPECT_GE(pid.integral(), -0.5f - 1e-6f);
}

// ---------------------------------------------------------------------------
// Open-loop plant sanity
// ---------------------------------------------------------------------------
TEST(DcMotor, RestStaysAtRest) {
  DcMotorModel motor;
  for (int i = 0; i < 1000; ++i) motor.step(0.0f, kDt);
  EXPECT_NEAR(motor.velocity_rad_s(), 0.0f, 1e-6f);
  EXPECT_NEAR(motor.position_rad(), 0.0f, 1e-6f);
}

TEST(DcMotor, ReachesSteadyStateVelocity) {
  DcMotorModel motor;
  // Analytic steady-state output velocity at full effort ~13.2 rad/s.
  for (int i = 0; i < 1000; ++i) motor.step(1.0f, kDt);  // 1 s
  EXPECT_GT(motor.velocity_rad_s(), 12.0f);
  EXPECT_LT(motor.velocity_rad_s(), 14.0f);
}

TEST(DcMotor, EncoderTracksPosition) {
  DcMotorModel motor;
  for (int i = 0; i < 200; ++i) motor.step(1.0f, kDt);
  const float from_counts = DcMotorModel::counts_to_rad(
      motor.encoder_count(), motor.params().counts_per_rev_out);
  EXPECT_NEAR(from_counts, motor.position_rad(), 0.01f);
}

// ---------------------------------------------------------------------------
// Closed-loop position control
// ---------------------------------------------------------------------------
TEST(PositionControl, ReachesSetpoint) {
  DcMotorModel motor;
  SimEncoder enc;
  PositionController ctrl({/*kp=*/8.0f, /*ki=*/2.0f, /*kd=*/0.6f});

  const float target = 1.0f;  // rad
  const int cpr = motor.params().counts_per_rev_out;
  float measured = 0.0f;
  for (int i = 0; i < 3000; ++i) {  // 3 s
    enc.set_count(motor.encoder_count());
    measured = DcMotorModel::counts_to_rad(enc.count(), cpr);
    const float effort = ctrl.update(target, measured, kDt);
    motor.step(effort, kDt);
  }
  EXPECT_NEAR(measured, target, 0.02f);
}

TEST(PositionControl, HoldsAgainstConstantLoad) {
  DcMotorModel motor;
  motor.set_load_torque(0.05f);  // steady output-shaft load [Nm]
  SimEncoder enc;
  PositionController ctrl({12.0f, 6.0f, 0.8f});

  const float target = 0.5f;
  const int cpr = motor.params().counts_per_rev_out;
  float measured = 0.0f;
  for (int i = 0; i < 4000; ++i) {
    enc.set_count(motor.encoder_count());
    measured = DcMotorModel::counts_to_rad(enc.count(), cpr);
    motor.step(ctrl.update(target, measured, kDt), kDt);
  }
  EXPECT_NEAR(measured, target, 0.05f);  // integral rejects the load
}

// ---------------------------------------------------------------------------
// Closed-loop velocity control
// ---------------------------------------------------------------------------
TEST(VelocityControl, TracksTarget) {
  DcMotorModel motor;
  VelocityController ctrl({/*kp=*/0.2f, /*ki=*/4.0f, /*kd=*/0.0f});

  const float target = 5.0f;  // rad/s (within ~13 rad/s max)
  float measured = 0.0f;
  for (int i = 0; i < 2000; ++i) {  // 2 s
    measured = motor.velocity_rad_s();
    motor.step(ctrl.update(target, measured, kDt), kDt);
  }
  EXPECT_NEAR(measured, target, 0.2f);
}
