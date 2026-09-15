#pragma once

// Nori firmware Hardware Abstraction Layer (HAL).
//
// These are the ONLY hardware-facing interfaces the rest of the firmware
// (drivers, control, safety, protocol) is allowed to depend on. Every concrete
// build provides an implementation:
//   * hal/sim   — pure software model, runs on the host, used for unit tests
//                 and the native software sim (Sim Level A).
//   * hal/stm32 — real STM32G4 peripherals (build-only stub for now).
//
// Keeping this layer thin and abstract is what lets simulation and hardware be
// interchangeable. See docs/architecture.md §6 and docs/PLAN.md Stage 2.

#include <array>
#include <cstdint>

namespace nori::hal {

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
enum class PinState : uint8_t { Low = 0, High = 1 };

class Gpio {
 public:
  virtual ~Gpio() = default;
  virtual void write(PinState state) = 0;
  virtual PinState read() const = 0;
};

// ---------------------------------------------------------------------------
// PWM — unidirectional duty in [0, 1]. Direction (for motors) is handled by a
// separate GPIO / driver, matching how H-bridges work.
// ---------------------------------------------------------------------------
class Pwm {
 public:
  virtual ~Pwm() = default;
  virtual void set_duty(float duty) = 0;      // clamped to [0, 1]
  virtual float duty() const = 0;
  virtual void set_frequency(uint32_t hz) = 0;
  virtual uint32_t frequency() const = 0;
};

// ---------------------------------------------------------------------------
// ADC — analog input (current shunt, battery voltage, thermistor, ...).
// ---------------------------------------------------------------------------
class Adc {
 public:
  virtual ~Adc() = default;
  virtual uint16_t read_raw() = 0;    // raw counts (e.g. 12-bit on STM32G4)
  virtual float read_voltage() = 0;   // converted volts at the pin
};

// ---------------------------------------------------------------------------
// Encoder — accumulated quadrature ticks (wheel / motor position).
// ---------------------------------------------------------------------------
class Encoder {
 public:
  virtual ~Encoder() = default;
  virtual int32_t count() const = 0;  // signed accumulated ticks
  virtual void reset() = 0;
};

// ---------------------------------------------------------------------------
// CAN / CAN-FD — the robot-wide bus. Frames carry up to 64 bytes (FD).
// receive() is non-blocking.
// ---------------------------------------------------------------------------
struct CanFrame {
  uint32_t id = 0;
  bool extended = false;          // 29-bit vs 11-bit id
  bool fd = false;                // CAN-FD frame
  uint8_t dlc = 0;                // number of valid data bytes
  std::array<uint8_t, 64> data{};
};

class CanBus {
 public:
  virtual ~CanBus() = default;
  virtual bool send(const CanFrame& frame) = 0;      // false if tx queue full
  virtual bool receive(CanFrame& out) = 0;           // false if nothing waiting
};

// ---------------------------------------------------------------------------
// Clock — monotonic microsecond time source for control loops / timing.
// ---------------------------------------------------------------------------
class Clock {
 public:
  virtual ~Clock() = default;
  virtual uint64_t now_us() const = 0;
};

// ---------------------------------------------------------------------------
// Watchdog — independent watchdog. kick() must be called within the timeout or
// the system is considered hung (real HW resets; sim exposes expired()).
// ---------------------------------------------------------------------------
class Watchdog {
 public:
  virtual ~Watchdog() = default;
  virtual void enable(uint32_t timeout_ms) = 0;
  virtual void kick() = 0;
  virtual bool expired() const = 0;
};

}  // namespace nori::hal
