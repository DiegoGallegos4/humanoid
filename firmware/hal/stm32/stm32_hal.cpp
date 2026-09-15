// STM32G4 HAL — build-only stub (see stm32_hal.hpp).
// Bodies return safe defaults so the firmware links for the target. Replace with
// real STM32 HAL/LL register access at hardware / Renode bring-up.
#include "nori/hal/stm32/stm32_hal.hpp"

namespace nori::hal::stm32 {

void Stm32Gpio::write(PinState) {}
PinState Stm32Gpio::read() const { return PinState::Low; }

void Stm32Pwm::set_duty(float) {}
float Stm32Pwm::duty() const { return 0.0f; }
void Stm32Pwm::set_frequency(uint32_t) {}
uint32_t Stm32Pwm::frequency() const { return 0; }

uint16_t Stm32Adc::read_raw() { return 0; }
float Stm32Adc::read_voltage() { return 0.0f; }

int32_t Stm32Encoder::count() const { return 0; }
void Stm32Encoder::reset() {}

bool Stm32CanBus::send(const CanFrame&) { return false; }
bool Stm32CanBus::receive(CanFrame&) { return false; }

uint64_t Stm32Clock::now_us() const { return 0; }

void Stm32Watchdog::enable(uint32_t) {}
void Stm32Watchdog::kick() {}
bool Stm32Watchdog::expired() const { return false; }

}  // namespace nori::hal::stm32
