// Leader -> follower joint mapping, identical to what `lerobot-teleoperate` does for the SO-101
// (body joints in lerobot's DEGREES mode, gripper in RANGE_0_100), plus two safety additions:
//  * the leader tracks its joints continuously, so crossing the encoder's 0/4095 seam reads as a small move
//    past the end of the range instead of a jump of a full turn;
//  * follower targets are kept kSeamMargin ticks away from that seam, so the follower is never sent the long
//    way round.
// Calibration comes straight from the motors: lerobot-calibrate stores range_min/range_max in each servo's
// Min/Max_Position_Limit registers.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace teleop {

constexpr int kNumJoints = 6;  // shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
constexpr int kWristRoll = 4;
constexpr int kGripper = 5;
constexpr int32_t kMaxRes = 4095;  // STS3215 positions run 0..4095
constexpr int32_t kTurn = 4096;
constexpr int32_t kSeamMargin = 100;  // ~9 degrees

inline const char* jointName(int j) {
  static const char* const names[kNumJoints] = {"shoulder_pan", "shoulder_lift", "elbow_flex",
                                                "wrist_flex",   "wrist_roll",    "gripper"};
  return names[j];
}

struct Range {
  int32_t min;
  int32_t max;
};

inline float mid(const Range& r) { return (r.min + r.max) * 0.5f; }
inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Rejects limits that can't come from lerobot-calibrate. On failure, writes the reason into `why`.
inline bool plausibleCalibration(const Range* r, char* why, size_t why_len) {
  for (int j = 0; j < kNumJoints; j++) {
    if (r[j].min < 0 || r[j].max > kMaxRes || r[j].max - r[j].min < 200) {
      snprintf(why, why_len, "%s limits [%ld, %ld] are not a calibrated range", jointName(j), (long)r[j].min,
               (long)r[j].max);
      return false;
    }
    if (j != kWristRoll && r[j].min == 0 && r[j].max == kMaxRes) {
      snprintf(why, why_len, "%s has no calibration (limits 0-4095): run lerobot-calibrate", jointName(j));
      return false;
    }
  }
  return true;
}

// Leader side ---------------------------------------------------------------------------------------------

// Continuous position of one leader joint, in raw ticks that may run past 0..4095.
class Unwrapper {
 public:
  void reset() { started_ = false; }

  int32_t update(int32_t raw, const Range& r, bool full_turn) {
    if (!started_) {
      value_ = raw;
      if (!full_turn) {
        // A joint with end stops sits inside its calibrated range, so take the reading nearest to it.
        const float m = mid(r);
        if (fabsf(raw + kTurn - m) < fabsf(value_ - m)) value_ = raw + kTurn;
        if (fabsf(raw - kTurn - m) < fabsf(value_ - m)) value_ = raw - kTurn;
      }
      started_ = true;
    } else {
      int32_t step = raw - last_;
      if (step > kTurn / 2) step -= kTurn;
      if (step < -kTurn / 2) step += kTurn;
      value_ += step;
    }
    last_ = raw;
    return value_;
  }

 private:
  bool started_ = false;
  int32_t last_ = 0;
  int32_t value_ = 0;
};

// Angle from the middle of the calibrated range in 0.01 degree (lerobot's DEGREES), saturated to int16.
inline int16_t toCentidegrees(int32_t pos, const Range& r) {
  float cdeg = (pos - mid(r)) * 36000.0f / kMaxRes;
  if (cdeg > 32767.0f) cdeg = 32767.0f;
  if (cdeg < -32767.0f) cdeg = -32767.0f;
  return (int16_t)lroundf(cdeg);
}

// Opening as 0..10000 (0.01 %) of the calibrated range (lerobot's RANGE_0_100).
inline uint16_t toCentipercent(int32_t pos, const Range& r) {
  const int32_t p = clampi(pos, r.min, r.max);
  return (uint16_t)lroundf((p - r.min) * 10000.0f / (r.max - r.min));
}

// Follower side -------------------------------------------------------------------------------------------

inline int32_t safeMin(const Range& r) { return r.min > kSeamMargin ? r.min : kSeamMargin; }
inline int32_t safeMax(const Range& r) { return r.max < kMaxRes - kSeamMargin ? r.max : kMaxRes - kSeamMargin; }

inline int32_t goalFromCentidegrees(int16_t cdeg, const Range& r) {
  const float pos = cdeg * (float)kMaxRes / 36000.0f + mid(r);
  return clampi((int32_t)lroundf(pos), safeMin(r), safeMax(r));
}

inline int32_t goalFromCentipercent(uint16_t cpct, const Range& r) {
  const float frac = (cpct > 10000 ? 10000 : cpct) / 10000.0f;
  return clampi((int32_t)lroundf(r.min + frac * (r.max - r.min)), safeMin(r), safeMax(r));
}

// Over-the-air message, leader -> follower (both ends are little-endian).
constexpr uint8_t kPacketMagic = 0xA5;
// The same pose, sent by a program on a PC rather than by the leader arm. A follower that knows it lets a
// program steer while it's sending and holds the leader back meanwhile; one that doesn't simply ignores
// it. The marker, not the route a packet took, is what tells them apart: a leader on Wi-Fi sends its own
// poses over Wi-Fi too.
constexpr uint8_t kProgramMagic = 0xA7;
// What a follower can do, reported in its status so a PC knows which kind of pose to send. 2: understands
// kProgramMagic. (A status without this byte is version 1.)
constexpr uint8_t kProtocolVersion = 2;

struct __attribute__((packed)) LeaderPacket {
  uint8_t magic;       // kPacketMagic
  uint8_t seq;         // increments every packet
  int16_t body[5];     // shoulder_pan .. wrist_roll, toCentidegrees()
  uint16_t gripper;    // toCentipercent()
};
static_assert(sizeof(LeaderPacket) == 14, "LeaderPacket must fit a default 20-byte BLE notification");

inline LeaderPacket makePacket(uint8_t seq, const int32_t* pos, const Range* r) {
  LeaderPacket p;
  p.magic = kPacketMagic;
  p.seq = seq;
  for (int j = 0; j < kGripper; j++) p.body[j] = toCentidegrees(pos[j], r[j]);
  p.gripper = toCentipercent(pos[kGripper], r[kGripper]);
  return p;
}

// Follower -> leader status, so the follower can be diagnosed without plugging USB into it (which its
// board's supply interferes with). The text lives here so both boards agree on the wording.
constexpr uint8_t kStatusMagic = 0x5B;

enum FollowerState : uint8_t { kStChecking = 0, kStWaiting, kStGliding, kStFollowing, kStFrozen, kStRecording };
enum FollowerFault : uint8_t { kFaultNone = 0, kFaultNoReply, kFaultCalibration, kFaultOutOfRange, kFaultWriteRefused };

struct __attribute__((packed)) FollowerStatus {
  uint8_t magic;   // kStatusMagic
  uint8_t state;   // FollowerState
  uint8_t fault;   // FollowerFault, only meaningful while checking
  uint8_t joint;   // joint the fault refers to, 0..5
  uint8_t poses;   // leader poses received since the last report, capped at 255
  int16_t pos[kNumJoints];        // where the arm actually is, in encoder steps
  int16_t range_min[kNumJoints];  // and how far it can go, so a PC can work in the same units
  int16_t range_max[kNumJoints];
  uint8_t version; // kProtocolVersion
};
static_assert(sizeof(FollowerStatus) == 42, "FollowerStatus must stay small");
constexpr size_t kStatusV1Size = 41;  // before `version`: still accepted, from a follower not yet reflashed

inline const char* stateText(uint8_t state) {
  switch (state) {
    case kStChecking: return "checking arm";
    case kStWaiting: return "waiting for leader";
    case kStGliding: return "gliding to leader pose";
    case kStFollowing: return "following";
    case kStFrozen: return "holding (no leader data)";
    case kStRecording: return "recording joint ranges: sweep every joint, then press S";
  }
  return "?";
}

inline const char* faultText(uint8_t fault) {
  switch (fault) {
    case kFaultNoReply: return "no reply from servo";
    case kFaultCalibration: return "no usable calibration in servo";
    case kFaultOutOfRange: return "outside its calibrated range";
    case kFaultWriteRefused: return "would not accept torque-on";
  }
  return "";
}

inline void goalsFromPacket(const LeaderPacket& p, const Range* r, int32_t* goals) {
  for (int j = 0; j < kGripper; j++) goals[j] = goalFromCentidegrees(p.body[j], r[j]);
  goals[kGripper] = goalFromCentipercent(p.gripper, r[kGripper]);
}

}  // namespace teleop
