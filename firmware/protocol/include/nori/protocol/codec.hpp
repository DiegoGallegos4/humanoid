#pragma once

// Nori CAN protocol codec — pack/unpack message structs to/from HAL CanFrames.
// Header-only, hardware-independent, shared by firmware and the ROS 2 bridge.
#include <algorithm>
#include <cmath>

#include "nori/hal/hal.hpp"
#include "nori/protocol/messages.hpp"

namespace nori::protocol {

// --- little-endian fixed-point helpers ---------------------------------------
inline int16_t to_i16(float value, float scale) {
  float raw = std::lround(value / scale);
  raw = std::clamp(raw, -32768.0f, 32767.0f);
  return static_cast<int16_t>(raw);
}
inline float from_i16(int16_t raw, float scale) {
  return static_cast<float>(raw) * scale;
}
inline void put_i16(hal::CanFrame& f, int i, int16_t v) {
  f.data[i] = static_cast<uint8_t>(v & 0xFF);
  f.data[i + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline int16_t get_i16(const hal::CanFrame& f, int i) {
  return static_cast<int16_t>(f.data[i] |
                              (static_cast<uint16_t>(f.data[i + 1]) << 8));
}
inline void put_u32(hal::CanFrame& f, int i, uint32_t v) {
  for (int b = 0; b < 4; ++b) f.data[i + b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
}
inline uint32_t get_u32(const hal::CanFrame& f, int i) {
  uint32_t v = 0;
  for (int b = 0; b < 4; ++b) v |= static_cast<uint32_t>(f.data[i + b]) << (8 * b);
  return v;
}

// --- JointCommand: [mode][pos:i16][vel:i16][eff:i16][seq] = 8 bytes ----------
inline hal::CanFrame encode_command(const JointCommand& c) {
  hal::CanFrame f;
  f.id = make_id(MsgClass::JointCommand, c.joint_id);
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(c.mode);
  put_i16(f, 1, to_i16(c.position_rad, kPositionScale));
  put_i16(f, 3, to_i16(c.velocity_rad_s, kVelocityScale));
  put_i16(f, 5, to_i16(c.effort_a, kCurrentScale));
  f.data[7] = c.seq;
  return f;
}
inline bool decode_command(const hal::CanFrame& f, JointCommand& out) {
  if (id_class(f.id) != MsgClass::JointCommand || f.dlc < 8) return false;
  out.joint_id = id_joint(f.id);
  out.mode = static_cast<JointMode>(f.data[0]);
  out.position_rad = from_i16(get_i16(f, 1), kPositionScale);
  out.velocity_rad_s = from_i16(get_i16(f, 3), kVelocityScale);
  out.effort_a = from_i16(get_i16(f, 5), kCurrentScale);
  out.seq = f.data[7];
  return true;
}

// --- JointState: [pos:i16][vel:i16][cur:i16][temp:i8][faults] = 8 bytes -------
inline hal::CanFrame encode_state(const JointState& s) {
  hal::CanFrame f;
  f.id = make_id(MsgClass::JointState, s.joint_id);
  f.dlc = 8;
  put_i16(f, 0, to_i16(s.position_rad, kPositionScale));
  put_i16(f, 2, to_i16(s.velocity_rad_s, kVelocityScale));
  put_i16(f, 4, to_i16(s.current_a, kCurrentScale));
  f.data[6] = static_cast<uint8_t>(s.temperature_c);
  f.data[7] = s.faults;
  return f;
}
inline bool decode_state(const hal::CanFrame& f, JointState& out) {
  if (id_class(f.id) != MsgClass::JointState || f.dlc < 8) return false;
  out.joint_id = id_joint(f.id);
  out.position_rad = from_i16(get_i16(f, 0), kPositionScale);
  out.velocity_rad_s = from_i16(get_i16(f, 2), kVelocityScale);
  out.current_a = from_i16(get_i16(f, 4), kCurrentScale);
  out.temperature_c = static_cast<int8_t>(f.data[6]);
  out.faults = f.data[7];
  return true;
}

// --- Heartbeat: [seq:u32][flags] --------------------------------------------
inline hal::CanFrame encode_heartbeat(const Heartbeat& h) {
  hal::CanFrame f;
  f.id = make_id(MsgClass::Heartbeat);
  f.dlc = 5;
  put_u32(f, 0, h.seq);
  f.data[4] = h.flags;
  return f;
}
inline bool decode_heartbeat(const hal::CanFrame& f, Heartbeat& out) {
  if (id_class(f.id) != MsgClass::Heartbeat || f.dlc < 5) return false;
  out.seq = get_u32(f, 0);
  out.flags = f.data[4];
  return true;
}

// --- EstopStatus: [tripped] --------------------------------------------------
inline hal::CanFrame encode_estop(const EstopStatus& e) {
  hal::CanFrame f;
  f.id = make_id(MsgClass::EstopStatus);
  f.dlc = 1;
  f.data[0] = e.tripped ? 1 : 0;
  return f;
}
inline bool decode_estop(const hal::CanFrame& f, EstopStatus& out) {
  if (id_class(f.id) != MsgClass::EstopStatus || f.dlc < 1) return false;
  out.tripped = f.data[0] != 0;
  return true;
}

}  // namespace nori::protocol
