// Re-recording joint ranges without a PC.
//
// lerobot-calibrate stores each joint's travel in the servo's Min/Max_Position_Limit registers, and the
// servo enforces them: a joint swept too little during calibration can no longer move beyond what was
// recorded. This records a fresh sweep and writes it back, leaving the homing offsets (the joint zero
// points) untouched, so it fixes a bad range without redoing a full calibration.
#pragma once

#include <Arduino.h>

#include "feetech.h"
#include "teleop_math.h"

namespace teleop {

constexpr uint32_t kEepromWriteMs = 30;  // a servo stops answering while it commits a register to EEPROM
constexpr uint8_t kEepromAttempts = 8;

// Centres each joint's travel on the encoder, exactly as lerobot's calibration does: with the arm held
// mid-range, the homing offset is set so this pose reads half a turn (2047). Without it a joint whose
// travel straddles the encoder's 0/4095 seam reports a jump of a full turn part way through its range,
// and no recorded range can describe it. Limits are opened up so the following sweep isn't clamped.
inline bool setHome(feetech::Bus& bus, const uint8_t* ids, Range* cal, char* msg, size_t msg_len) {
  for (int j = 0; j < kNumJoints; j++) {
    const uint8_t id = ids[j];
    bool ok = bus.write8(id, feetech::kLock, 0);
    delay(kEepromWriteMs);
    ok = ok && bus.write16(id, feetech::kHomingOffset, 0);
    delay(kEepromWriteMs);
    ok = ok && bus.write16(id, feetech::kMinPositionLimit, 0);
    delay(kEepromWriteMs);
    ok = ok && bus.write16(id, feetech::kMaxPositionLimit, (uint16_t)kMaxRes);
    delay(kEepromWriteMs);
    uint16_t actual = 0;
    ok = ok && bus.read16(id, feetech::kPresentPosition, &actual, kEepromAttempts);
    if (!ok) {
      snprintf(msg, msg_len, "servo %d (%s) did not answer while being re-homed", id, jointName(j));
      return false;
    }
    const int32_t offset = (int32_t)feetech::decodePosition(actual) - kMaxRes / 2;
    ok = bus.write16(id, feetech::kHomingOffset, feetech::encodeHoming(offset));
    delay(kEepromWriteMs);
    bus.write8(id, feetech::kLock, 1);
    delay(kEepromWriteMs);
    uint16_t written = 0;
    ok = ok && bus.read16(id, feetech::kHomingOffset, &written, kEepromAttempts);
    if (!ok || feetech::decodeHoming(written) != offset) {
      snprintf(msg, msg_len, "servo %d (%s) did not keep its new centre (%ld, wanted %ld)", id, jointName(j),
               (long)feetech::decodeHoming(written), (long)offset);
      return false;
    }
    cal[j].min = 0;
    cal[j].max = kMaxRes;
  }
  snprintf(msg, msg_len, "centred; now sweep every joint and save the ranges");
  return true;
}

class RangeRecorder {
 public:
  bool active() const { return active_; }

  void start(const int32_t* present) {
    for (int j = 0; j < kNumJoints; j++) {
      min_[j] = max_[j] = present[j];
    }
    active_ = true;
  }

  void update(const int32_t* present) {
    for (int j = 0; j < kNumJoints; j++) {
      if (present[j] < min_[j]) min_[j] = present[j];
      if (present[j] > max_[j]) max_[j] = present[j];
    }
  }

  void cancel() { active_ = false; }

  int32_t span(int joint) const { return max_[joint] - min_[joint]; }

  // Writes the swept ranges to the servos. wrist_roll keeps the full circle, as lerobot gives it.
  // Refuses if any other joint was not swept far enough, so a half-finished sweep can't be saved, or swept
  // across the encoder's seam (see plausibleCalibration). A refusal leaves the recording going: sweep on, or
  // cancel, then save again.
  //
  // The limits live in EEPROM, which a servo only writes while its Lock register is clear; otherwise the
  // new values apply until the power goes off and are then forgotten. lerobot clears that lock as part of
  // disabling torque, so servos it has driven are usually left locked. Each range is read back afterwards
  // to prove it stuck.
  bool save(feetech::Bus& bus, const uint8_t* ids, Range* cal, char* msg, size_t msg_len) {
    for (int j = 0; j < kNumJoints; j++) {
      if (j == kWristRoll) continue;
      if (span(j) < kMinUsefulSpan) {
        snprintf(msg, msg_len, "%s only moved %ld steps: sweep every joint to both stops", jointName(j),
                 (long)span(j));
        return false;
      }
      if (span(j) > kMaxUsefulSpan) {
        snprintf(msg, msg_len, "%s swept %ld steps, nearly a full turn: it crosses the encoder seam. Cancel, centre (h/H), redo", jointName(j), (long)span(j));
        return false;
      }
    }
    active_ = false;  // only now: a refusal above leaves the recording going
    for (int j = 0; j < kNumJoints; j++) {
      const int32_t lo = (j == kWristRoll) ? 0 : min_[j];
      const int32_t hi = (j == kWristRoll) ? kMaxRes : max_[j];
      bool ok = bus.write8(ids[j], feetech::kLock, 0);  // let this servo write its EEPROM
      delay(kEepromWriteMs);
      ok = ok && bus.write16(ids[j], feetech::kMinPositionLimit, (uint16_t)lo);
      delay(kEepromWriteMs);  // the servo stops answering while it commits; don't crowd the next write
      ok = ok && bus.write16(ids[j], feetech::kMaxPositionLimit, (uint16_t)hi);
      delay(kEepromWriteMs);
      bus.write8(ids[j], feetech::kLock, 1);  // lock again, as lerobot leaves them
      delay(kEepromWriteMs);
      uint16_t read_lo = 0, read_hi = 0;
      ok = ok && bus.read16(ids[j], feetech::kMinPositionLimit, &read_lo, kEepromAttempts) &&
           bus.read16(ids[j], feetech::kMaxPositionLimit, &read_hi, kEepromAttempts);
      if (!ok || read_lo != lo || read_hi != hi) {
        snprintf(msg, msg_len, "servo %d (%s) did not keep its new range ([%u,%u] after writing [%ld,%ld])",
                 ids[j], jointName(j), read_lo, read_hi, (long)lo, (long)hi);
        return false;
      }
      cal[j].min = lo;
      cal[j].max = hi;
    }
    snprintf(msg, msg_len, "saved");
    return true;
  }

 private:
  bool active_ = false;
  int32_t min_[kNumJoints] = {0};
  int32_t max_[kNumJoints] = {0};
};

// Commands the leader sends to the follower over the radio, typed into the leader's Serial Monitor.
constexpr uint8_t kCommandMagic = 0x5C;
enum Command : uint8_t { kCmdRecordRanges = 1, kCmdSaveRanges = 2, kCmdCancelRecording = 3, kCmdSetHome = 4 };

struct __attribute__((packed)) CommandPacket {
  uint8_t magic;
  uint8_t command;
};

}  // namespace teleop
