# Nori Actuator Protocol (Stage 4)

The actuator command/state contract. Source of truth is the code:
`firmware/protocol/include/nori/protocol/{messages,codec}.hpp`. The future ROS 2
hardware interface (Stage 6) encodes/decodes the same frames.

**Transport-agnostic** (architecture v2, direct-serial §4): the same messages
carry over the **Base-MCU link** (USB-serial for wheels + lift; CAN if MCU nodes
are ever added). The arm/head **Feetech servos are driven straight from the Pi**
over their own register protocol; the Pi's servo driver maps them onto the same
`JointCommand`/`JointState` fields, so the message set below is the single
logical contract even though two wire formats exist.

Mirrors the smart-actuator interface (`docs/architecture.md` §6). Payloads are
≤ 8 bytes so they fit **classic CAN and CAN-FD** frames. Multi-byte fields are
**little-endian**, fixed-point `int16` with the scales below.

## CAN ID (11-bit standard)
```
bits [10:8] = message class     bits [7:0] = joint id (0 for bus-wide)
```
| Class | Value | Direction |
|-------|:-----:|-----------|
| JointCommand | 0x1 | SBC → MCU (per joint) |
| JointState   | 0x2 | MCU → SBC (per joint) |
| Heartbeat    | 0x3 | SBC → all MCUs (liveness) |
| EstopStatus  | 0x4 | safety broadcast |

Example: `LeftElbow` (joint 7) state → id `(0x2<<8)|7 = 0x207`.

## Joint IDs
0 base_left_wheel · 1 base_right_wheel · 2 head_pan · 3 head_tilt ·
4–9 left arm (shoulder pan, shoulder lift, elbow, wrist flex, wrist roll, gripper) ·
10–15 right arm (same order) · 16 torso_lift. 17 joints total. (SO-101 arms + lift.)

## Scales
| Field | Scale | Range |
|-------|-------|-------|
| position | 0.001 rad/LSB | ±32.767 rad |
| velocity | 0.001 rad/s/LSB | ±32.767 rad/s |
| current/effort | 0.001 A/LSB | ±32.767 A |

## Payloads
**JointCommand (8 B):** `mode:u8` `pos:i16` `vel:i16` `eff:i16` `seq:u8`
(mode: 0 idle, 1 position, 2 velocity, 3 torque)

**JointState (8 B):** `pos:i16` `vel:i16` `cur:i16` `temp:i8(°C)` `faults:u8`

**Heartbeat (5 B):** `seq:u32` `flags:u8` — MCU enters safe state if it stops.

**EstopStatus (1 B):** `tripped:u8` — 1 ⇒ actuator rail cut.

## Fault bitfield (JointState.faults)
| bit | flag | bit | flag |
|:--:|------|:--:|------|
| 0 | overcurrent | 4 | comm_timeout |
| 1 | overtemp | 5 | encoder_fault |
| 2 | position_limit | 6 | driver_fault |
| 3 | velocity_limit | 7 | under_voltage |

Note: per-rail bus **voltage** is not in per-joint state (8-byte budget); it is
reported separately at the rail level (added when the safety/telemetry layer needs it).
