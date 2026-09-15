#pragma once

// Software (host) implementations of the Nori HAL — Sim Level A.
//
// Each class is fully inspectable/controllable from tests: you can drive inputs
// (encoder counts, ADC voltages, incoming CAN frames, elapsed time) and observe
// outputs (PWM duty, GPIO state, sent CAN frames). No hardware involved.

#include <deque>

#include "nori/hal/hal.hpp"

namespace nori::hal::sim {

class SimGpio final : public Gpio {
 public:
  void write(PinState state) override { state_ = state; }
  PinState read() const override { return state_; }

 private:
  PinState state_ = PinState::Low;
};

class SimPwm final : public Pwm {
 public:
  void set_duty(float duty) override {
    duty_ = duty < 0.0f ? 0.0f : (duty > 1.0f ? 1.0f : duty);
  }
  float duty() const override { return duty_; }
  void set_frequency(uint32_t hz) override { freq_ = hz; }
  uint32_t frequency() const override { return freq_; }

 private:
  float duty_ = 0.0f;
  uint32_t freq_ = 20000;  // 20 kHz default
};

// ADC driven by a test-settable voltage; raw counts derived from resolution.
class SimAdc final : public Adc {
 public:
  explicit SimAdc(float vref = 3.3f, uint16_t max_counts = 4095)
      : vref_(vref), max_counts_(max_counts) {}

  void set_voltage(float v) { voltage_ = v; }

  uint16_t read_raw() override {
    float clamped = voltage_ < 0.0f ? 0.0f : (voltage_ > vref_ ? vref_ : voltage_);
    return static_cast<uint16_t>((clamped / vref_) * max_counts_ + 0.5f);
  }
  float read_voltage() override { return voltage_; }

 private:
  float vref_;
  uint16_t max_counts_;
  float voltage_ = 0.0f;
};

// Encoder whose count can be set/advanced by a motor model or a test.
class SimEncoder final : public Encoder {
 public:
  int32_t count() const override { return count_; }
  void reset() override { count_ = 0; }

  void set_count(int32_t c) { count_ = c; }
  void advance(int32_t delta) { count_ += delta; }

 private:
  int32_t count_ = 0;
};

// In-memory CAN bus. A test (or a second SimCanBus) plays the role of the peer:
//   node.send(f)      -> lands in this bus's tx queue
//   bus.inject(f)     -> a frame arriving from the peer, readable via receive()
// For loopback tests, connect two buses with connect().
class SimCanBus final : public CanBus {
 public:
  bool send(const CanFrame& frame) override {
    tx_.push_back(frame);
    if (peer_) peer_->rx_.push_back(frame);
    return true;
  }
  bool receive(CanFrame& out) override {
    if (rx_.empty()) return false;
    out = rx_.front();
    rx_.pop_front();
    return true;
  }

  // Test helpers.
  void inject(const CanFrame& frame) { rx_.push_back(frame); }
  bool tx_empty() const { return tx_.empty(); }
  std::size_t tx_size() const { return tx_.size(); }
  CanFrame pop_tx() {
    CanFrame f = tx_.front();
    tx_.pop_front();
    return f;
  }
  static void connect(SimCanBus& a, SimCanBus& b) {
    a.peer_ = &b;
    b.peer_ = &a;
  }

 private:
  std::deque<CanFrame> tx_;
  std::deque<CanFrame> rx_;
  SimCanBus* peer_ = nullptr;
};

// Monotonic clock advanced explicitly by the sim harness (deterministic tests).
class SimClock final : public Clock {
 public:
  uint64_t now_us() const override { return now_us_; }
  void advance_us(uint64_t dt) { now_us_ += dt; }
  void advance_ms(uint64_t dt) { now_us_ += dt * 1000; }

 private:
  uint64_t now_us_ = 0;
};

// Watchdog tied to a SimClock: expires if not kicked within the timeout.
class SimWatchdog final : public Watchdog {
 public:
  explicit SimWatchdog(const SimClock& clock) : clock_(clock) {}

  void enable(uint32_t timeout_ms) override {
    enabled_ = true;
    timeout_us_ = static_cast<uint64_t>(timeout_ms) * 1000;
    last_kick_us_ = clock_.now_us();
  }
  void kick() override { last_kick_us_ = clock_.now_us(); }
  bool expired() const override {
    if (!enabled_) return false;
    return (clock_.now_us() - last_kick_us_) > timeout_us_;
  }

 private:
  const SimClock& clock_;
  bool enabled_ = false;
  uint64_t timeout_us_ = 0;
  uint64_t last_kick_us_ = 0;
};

}  // namespace nori::hal::sim
