#pragma once

// Nori CAN protocol — message set (the ROS 2 <-> MCU <-> actuator contract).
//
// This is the single source of truth shared by firmware and (later) the ROS 2
// hardware interface. It mirrors the smart-actuator interface in
// docs/architecture.md §6.
//
// Frame budget: command and state payloads are <= 8 bytes so they work on BOTH
// classic CAN and CAN-FD. Multi-byte fields are little-endian (STM32 native),
// fixed-point int16 with the scales below.
//
// CAN ID layout (11-bit standard):
//   bits [10:8] = message class (see MsgClass)
//   bits  [7:0] = joint id (JointId), or 0 for bus-wide frames
#include <array>
#include <cstdint>

namespace nori::protocol {

// --- addressing --------------------------------------------------------------
enum class MsgClass : uint32_t {
  JointCommand = 0x1,  // SBC -> MCU, per joint
  JointState   = 0x2,  // MCU -> SBC, per joint
  Heartbeat    = 0x3,  // SBC -> all MCUs (safety liveness)
  EstopStatus  = 0x4,  // e-stop / safety state broadcast
};

// 17 actuated joints (docs/architecture.md §1). Values are the wire joint id.
// TorsoLift is appended (id 16) so the base/head/arm ids stay stable.
enum class JointId : uint8_t {
  BaseLeftWheel = 0, BaseRightWheel = 1,
  HeadPan = 2, HeadTilt = 3,
  LeftShoulderPan = 4, LeftShoulderLift = 5, LeftElbow = 6,
  LeftWristFlex = 7, LeftWristRoll = 8, LeftGripper = 9,
  RightShoulderPan = 10, RightShoulderLift = 11, RightElbow = 12,
  RightWristFlex = 13, RightWristRoll = 14, RightGripper = 15,
  TorsoLift = 16,
  Count = 17,
};

inline uint32_t make_id(MsgClass cls, uint8_t joint = 0) {
  return (static_cast<uint32_t>(cls) << 8) | joint;
}
inline MsgClass id_class(uint32_t id) {
  return static_cast<MsgClass>((id >> 8) & 0x7);
}
inline uint8_t id_joint(uint32_t id) { return static_cast<uint8_t>(id & 0xFF); }

// --- fixed-point scales (value = raw * scale) --------------------------------
inline constexpr float kPositionScale = 0.001f;  // rad/LSB   -> ±32.767 rad
inline constexpr float kVelocityScale = 0.001f;  // rad/s/LSB -> ±32.767 rad/s
inline constexpr float kCurrentScale  = 0.001f;  // A/LSB     -> ±32.767 A

// --- fault bitfield (matches sim + safety layer) -----------------------------
namespace fault {
inline constexpr uint8_t Overcurrent   = 1u << 0;
inline constexpr uint8_t Overtemp       = 1u << 1;
inline constexpr uint8_t PositionLimit  = 1u << 2;
inline constexpr uint8_t VelocityLimit  = 1u << 3;
inline constexpr uint8_t CommTimeout    = 1u << 4;
inline constexpr uint8_t EncoderFault    = 1u << 5;
inline constexpr uint8_t DriverFault     = 1u << 6;
inline constexpr uint8_t UnderVoltage    = 1u << 7;
}  // namespace fault

// --- message structs (SI units; codec handles fixed-point on the wire) -------
enum class JointMode : uint8_t { Idle = 0, Position = 1, Velocity = 2, Torque = 3 };

struct JointCommand {
  uint8_t joint_id = 0;
  JointMode mode = JointMode::Idle;
  float position_rad = 0.0f;
  float velocity_rad_s = 0.0f;
  float effort_a = 0.0f;   // torque/current setpoint
  uint8_t seq = 0;
};

struct JointState {
  uint8_t joint_id = 0;
  float position_rad = 0.0f;
  float velocity_rad_s = 0.0f;
  float current_a = 0.0f;
  int8_t temperature_c = 0;
  uint8_t faults = 0;      // OR of fault:: flags
};

struct Heartbeat {
  uint32_t seq = 0;        // increments each tick; MCU times out if it stops
  uint8_t flags = 0;
};

struct EstopStatus {
  bool tripped = false;    // true => actuator rail cut
};

}  // namespace nori::protocol
