// Stage 4 — CAN protocol round-trip + loopback tests.
#include <gtest/gtest.h>

#include "nori/hal/sim/sim_hal.hpp"
#include "nori/protocol/codec.hpp"

using namespace nori::protocol;
using nori::hal::CanFrame;
using nori::hal::sim::SimCanBus;

namespace {
constexpr float kQuant = 0.0011f;  // one LSB of tolerance for fixed-point
}

TEST(Addressing, IdEncodesClassAndJoint) {
  const uint32_t id = make_id(MsgClass::JointState,
                              static_cast<uint8_t>(JointId::LeftElbow));
  EXPECT_EQ(id_class(id), MsgClass::JointState);
  EXPECT_EQ(id_joint(id), static_cast<uint8_t>(JointId::LeftElbow));
}

TEST(JointMap, HasSeventeenJointsIncludingLift) {
  EXPECT_EQ(static_cast<int>(JointId::Count), 17);
  EXPECT_EQ(static_cast<int>(JointId::TorsoLift), 16);
  // lift addresses like any other joint (prismatic, driven by the Base MCU)
  const uint32_t id = make_id(MsgClass::JointCommand,
                              static_cast<uint8_t>(JointId::TorsoLift));
  EXPECT_EQ(id_joint(id), 16u);
}

TEST(JointCommandCodec, RoundTrip) {
  JointCommand c;
  c.joint_id = static_cast<uint8_t>(JointId::RightShoulderPan);
  c.mode = JointMode::Position;
  c.position_rad = 1.234f;
  c.velocity_rad_s = -2.5f;
  c.effort_a = 0.75f;
  c.seq = 42;

  CanFrame f = encode_command(c);
  EXPECT_EQ(f.dlc, 8);
  EXPECT_EQ(id_class(f.id), MsgClass::JointCommand);

  JointCommand out;
  ASSERT_TRUE(decode_command(f, out));
  EXPECT_EQ(out.joint_id, c.joint_id);
  EXPECT_EQ(out.mode, JointMode::Position);
  EXPECT_NEAR(out.position_rad, c.position_rad, kQuant);
  EXPECT_NEAR(out.velocity_rad_s, c.velocity_rad_s, kQuant);
  EXPECT_NEAR(out.effort_a, c.effort_a, kQuant);
  EXPECT_EQ(out.seq, 42);
}

TEST(JointStateCodec, RoundTripWithFaults) {
  JointState s;
  s.joint_id = static_cast<uint8_t>(JointId::HeadTilt);
  s.position_rad = -0.42f;
  s.velocity_rad_s = 3.14f;
  s.current_a = 1.5f;
  s.temperature_c = 57;
  s.faults = fault::Overcurrent | fault::CommTimeout;

  CanFrame f = encode_state(s);
  JointState out;
  ASSERT_TRUE(decode_state(f, out));
  EXPECT_EQ(out.joint_id, s.joint_id);
  EXPECT_NEAR(out.position_rad, s.position_rad, kQuant);
  EXPECT_NEAR(out.velocity_rad_s, s.velocity_rad_s, kQuant);
  EXPECT_NEAR(out.current_a, s.current_a, kQuant);
  EXPECT_EQ(out.temperature_c, 57);
  EXPECT_TRUE(out.faults & fault::Overcurrent);
  EXPECT_TRUE(out.faults & fault::CommTimeout);
  EXPECT_FALSE(out.faults & fault::Overtemp);
}

TEST(Codec, RejectsWrongClass) {
  JointState s;
  CanFrame f = encode_state(s);  // a state frame
  JointCommand cmd;
  EXPECT_FALSE(decode_command(f, cmd));  // must not decode as a command
}

TEST(HeartbeatCodec, RoundTrip) {
  Heartbeat h{123456u, 0x03};
  CanFrame f = encode_heartbeat(h);
  Heartbeat out;
  ASSERT_TRUE(decode_heartbeat(f, out));
  EXPECT_EQ(out.seq, 123456u);
  EXPECT_EQ(out.flags, 0x03);
}

TEST(EstopCodec, RoundTrip) {
  CanFrame f = encode_estop({true});
  EstopStatus out;
  ASSERT_TRUE(decode_estop(f, out));
  EXPECT_TRUE(out.tripped);
}

// End-to-end: SBC node sends a command over the bus; MCU node receives, decodes,
// and replies with state. Exercises the real HAL CanBus loopback (Stage 4 gate).
TEST(BusLoopback, CommandThenStateReply) {
  SimCanBus sbc, mcu;
  SimCanBus::connect(sbc, mcu);

  JointCommand c;
  c.joint_id = static_cast<uint8_t>(JointId::LeftGripper);
  c.mode = JointMode::Velocity;
  c.velocity_rad_s = 1.0f;
  ASSERT_TRUE(sbc.send(encode_command(c)));

  // MCU side
  CanFrame rx;
  ASSERT_TRUE(mcu.receive(rx));
  JointCommand got_cmd;
  ASSERT_TRUE(decode_command(rx, got_cmd));
  EXPECT_EQ(got_cmd.joint_id, static_cast<uint8_t>(JointId::LeftGripper));
  EXPECT_EQ(got_cmd.mode, JointMode::Velocity);

  JointState reply;
  reply.joint_id = got_cmd.joint_id;
  reply.velocity_rad_s = got_cmd.velocity_rad_s;
  ASSERT_TRUE(mcu.send(encode_state(reply)));

  // SBC side
  ASSERT_TRUE(sbc.receive(rx));
  JointState got_state;
  ASSERT_TRUE(decode_state(rx, got_state));
  EXPECT_EQ(got_state.joint_id, static_cast<uint8_t>(JointId::LeftGripper));
  EXPECT_NEAR(got_state.velocity_rad_s, 1.0f, kQuant);
}
