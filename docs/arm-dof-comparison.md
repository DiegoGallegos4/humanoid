# Arm DOF — Design Comparison

Decision doc for the arm kinematics before Stage 5. Compares three options against
our constraints: **sim-first** (DOF costs complexity, not £, until we buy),
**sub-£500 BOM**, **open-source reuse**, and **eventual parity with the real Nori**.

Background: 6 DOF are needed to place the gripper at an **arbitrary position AND
orientation**. 5 DOF reaches any position but gives up one orientation axis.
7 DOF = 6 + 1 *redundant* joint (reposition the elbow without moving the hand →
dodge obstacles/joint-limits/singularities). "+1" always means the gripper
(open/close), not a positioning DOF.

## The three options

**A — Current frozen design** (5-DOF + gripper)
shoulder pitch · shoulder roll · shoulder yaw · elbow · wrist pitch · [gripper]
- 3-DOF shoulder, but **no wrist roll** → can't spin the gripper about its approach
  axis (poor for knobs, pouring, reorienting a grasped object).

**B — SO-101 arrangement** (5-DOF + gripper) ← the open-source standard
shoulder pan · shoulder lift · elbow · wrist flex · **wrist roll** · [gripper]
- Same servo count/cost as A, but the joint layout the LeRobot/SO-arm community
  actually uses; **has wrist roll**. What the real Nori itself started from.

**C — Real-Nori parity** (7-DOF + gripper)
+2 joints over B (full 6-DOF pose **+ 1 redundant**) · [gripper]
- Best dexterity for cluttered home manipulation; matches Nori's "7+1 DOF".

## Comparison matrix

| Criterion | A: current 5-DOF | B: SO-101 5-DOF | C: 7-DOF parity |
|-----------|:---:|:---:|:---:|
| Positioning DOF | 5 | 5 | 7 |
| Arbitrary 6-DOF pose | No (−1 axis) | No (−1 axis) | **Yes** |
| Wrist roll (spin gripper) | ✗ | ✓ | ✓ |
| Redundancy (obstacle/singularity avoidance) | ✗ | ✗ | ✓ |
| Servos per arm (incl. gripper) | 6 | 6 | 8 |
| BOM delta vs A | — | £0 | ~+£56 (4 servos) |
| Total robot actuated DOF | 16 | 16 | 20 |
| **Open-source reuse** (CAD, URDF, calibration) | low | **high (SO-101)** | medium |
| **Learning ecosystem** (LeRobot, ACT, datasets) | low | **high** | medium¹ |
| Sim modelling / IK / tuning effort | med | med | high |
| Parity with real Nori | partial | partial | **full** |

¹ Community datasets/policies are largely captured on SO-101 5-DOF arms; a 7-DOF
arm can still use ACT/LeRobot tooling but off-the-shelf datasets won't transfer directly.

## Recommendation — **Option B (SO-101 5-DOF + gripper)** for V1, with a documented 7-DOF path

Why B is the best fit for *our* case:
- **Same cost and DOF count as our current design** (no BOM change, stays 16 DOF).
- **Unlocks the whole SO-101 open-source stack** — arm CAD/URDF for Stage 5, and
  LeRobot + ACT policies/datasets/calibration for Stage 7 — which is the single
  biggest accelerator available to us. Building on the community standard beats an
  ad-hoc 5-DOF layout of equal cost.
- **Wrist roll** is more useful than our current shoulder-roll for real manipulation.
- It is literally the arm the real Nori was built up from, so it's the natural
  parity baseline.

Because we are **sim-first, DOF is free to experiment with**: keep the reference
BOM at SO-101 5-DOF, but carry an **optional 7-DOF "parity" variant in simulation**
(add wrist pitch + one redundant joint). The smart-actuator + CAN abstraction makes
adding joints a config change, not a rewrite — so we can evaluate 6/7-DOF dexterity
in MuJoCo at zero hardware cost and decide later whether the extra servos are worth it.

Not recommended:
- **A** — no advantage over B at equal cost, and forgoes open-source reuse + wrist roll.
- **C as the *only* target** — real dexterity/parity win, but +complexity now and
  weaker off-the-shelf dataset reuse; better as a sim variant/upgrade than the V1 baseline.

## What choosing B changes
- `docs/architecture.md` §1: rename arm joints to the SO-101 layout (pan, lift,
  elbow, wrist flex, wrist roll, gripper). **Total DOF stays 16.**
- `firmware/protocol/messages.hpp`: rename the `JointId` arm entries (values unchanged).
- `docs/can-protocol.md`: update joint-name list.
- Stage 5 URDF: base it on SO-101 CAD.
- No firmware/control/codec logic changes — only names.
