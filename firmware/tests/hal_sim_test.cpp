// Stage 2 — HAL contract tests against the sim implementation.
// Verifies each sim peripheral honours the HAL interface behaviour that the
// rest of the firmware will rely on.
#include <gtest/gtest.h>

#include "nori/hal/hal.hpp"
#include "nori/hal/sim/sim_hal.hpp"
#include "nori/version.hpp"

using namespace nori::hal;
using namespace nori::hal::sim;

TEST(Version, IsSet) {
  EXPECT_EQ(nori::kVersionMajor, 0);
  EXPECT_STREQ(nori::kVersionString, "0.1.0");
}

TEST(SimGpio, WriteReadRoundTrip) {
  SimGpio gpio;
  EXPECT_EQ(gpio.read(), PinState::Low);
  gpio.write(PinState::High);
  EXPECT_EQ(gpio.read(), PinState::High);
}

TEST(SimPwm, DutyIsClamped) {
  SimPwm pwm;
  pwm.set_duty(0.5f);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.5f);
  pwm.set_duty(1.7f);
  EXPECT_FLOAT_EQ(pwm.duty(), 1.0f);
  pwm.set_duty(-0.3f);
  EXPECT_FLOAT_EQ(pwm.duty(), 0.0f);
}

TEST(SimPwm, FrequencySettable) {
  SimPwm pwm;
  pwm.set_frequency(25000);
  EXPECT_EQ(pwm.frequency(), 25000u);
}

TEST(SimAdc, VoltageToRawAndBack) {
  SimAdc adc(3.3f, 4095);
  adc.set_voltage(1.65f);  // half scale
  EXPECT_NEAR(adc.read_raw(), 2048, 1);
  EXPECT_FLOAT_EQ(adc.read_voltage(), 1.65f);
}

TEST(SimAdc, VoltageClampedForRaw) {
  SimAdc adc(3.3f, 4095);
  adc.set_voltage(5.0f);
  EXPECT_EQ(adc.read_raw(), 4095);
}

TEST(SimEncoder, AdvanceAndReset) {
  SimEncoder enc;
  EXPECT_EQ(enc.count(), 0);
  enc.advance(100);
  enc.advance(-30);
  EXPECT_EQ(enc.count(), 70);
  enc.reset();
  EXPECT_EQ(enc.count(), 0);
}

TEST(SimCanBus, InjectThenReceive) {
  SimCanBus bus;
  CanFrame in;
  EXPECT_FALSE(bus.receive(in));  // empty

  CanFrame f;
  f.id = 0x123;
  f.dlc = 2;
  f.data[0] = 0xAB;
  f.data[1] = 0xCD;
  bus.inject(f);

  EXPECT_TRUE(bus.receive(in));
  EXPECT_EQ(in.id, 0x123u);
  EXPECT_EQ(in.dlc, 2);
  EXPECT_EQ(in.data[0], 0xAB);
  EXPECT_FALSE(bus.receive(in));  // drained
}

TEST(SimCanBus, LoopbackBetweenNodes) {
  SimCanBus a, b;
  SimCanBus::connect(a, b);

  CanFrame f;
  f.id = 0x7FF;
  f.dlc = 1;
  f.data[0] = 42;
  EXPECT_TRUE(a.send(f));

  CanFrame got;
  EXPECT_TRUE(b.receive(got));  // arrived at the peer
  EXPECT_EQ(got.id, 0x7FFu);
  EXPECT_EQ(got.data[0], 42);
}

TEST(SimClockWatchdog, ExpiresWithoutKick) {
  SimClock clock;
  SimWatchdog wdt(clock);
  wdt.enable(100);  // 100 ms timeout

  clock.advance_ms(50);
  EXPECT_FALSE(wdt.expired());
  wdt.kick();

  clock.advance_ms(80);
  EXPECT_FALSE(wdt.expired());  // kicked at 50 ms, only 80 ms since

  clock.advance_ms(50);
  EXPECT_TRUE(wdt.expired());   // 130 ms since last kick > 100 ms
}

TEST(SimClock, MonotonicAdvance) {
  SimClock clock;
  EXPECT_EQ(clock.now_us(), 0u);
  clock.advance_us(1500);
  clock.advance_ms(1);
  EXPECT_EQ(clock.now_us(), 2500u);
}
