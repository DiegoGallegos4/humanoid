#pragma once

// STM32G4 implementations of the Nori HAL.
//
// BUILD-ONLY STUB for now (Stage 2): the classes exist and satisfy the HAL
// interfaces so the firmware cross-compiles for the target, but the peripheral
// bodies are not yet wired to real registers / the STM32 HAL/LL drivers. Real
// implementations land alongside first hardware bring-up (Stage 8, future) or
// when running under Renode (Stage 6b), whichever we do first.
//
// This file is compiled only for the `stm32g4` build target, never natively.

#include "nori/hal/hal.hpp"

namespace nori::hal::stm32 {

class Stm32Gpio final : public Gpio {
 public:
  void write(PinState state) override;
  PinState read() const override;
};

class Stm32Pwm final : public Pwm {
 public:
  void set_duty(float duty) override;
  float duty() const override;
  void set_frequency(uint32_t hz) override;
  uint32_t frequency() const override;
};

class Stm32Adc final : public Adc {
 public:
  uint16_t read_raw() override;
  float read_voltage() override;
};

class Stm32Encoder final : public Encoder {
 public:
  int32_t count() const override;
  void reset() override;
};

class Stm32CanBus final : public CanBus {
 public:
  bool send(const CanFrame& frame) override;
  bool receive(CanFrame& out) override;
};

class Stm32Clock final : public Clock {
 public:
  uint64_t now_us() const override;
};

class Stm32Watchdog final : public Watchdog {
 public:
  void enable(uint32_t timeout_ms) override;
  void kick() override;
  bool expired() const override;
};

}  // namespace nori::hal::stm32
