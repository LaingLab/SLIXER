// Minimal bus master for Feetech STS-series servos (the STS3215s in the SO-101 arms).
// Protocol 0, little-endian, 1 Mbps. Only what the teleop firmware needs: ping, read, write, sync read,
// sync write. The byte transport is abstract so the same code runs on the XIAO boards and in a desktop test.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace feetech {

// STS3215 control table (subset).
constexpr uint8_t kMinPositionLimit = 9;   // 2 bytes, EEPROM: lerobot stores the calibrated range_min here
constexpr uint8_t kMaxPositionLimit = 11;  // 2 bytes, EEPROM: ... and range_max here
constexpr uint8_t kHomingOffset = 31;      // 2 bytes, EEPROM, sign-magnitude (bit 11): Present = Actual - Homing
constexpr uint8_t kTorqueEnable = 40;      // 1 byte
constexpr uint8_t kAcceleration = 41;      // 1 byte
constexpr uint8_t kGoalPosition = 42;      // 2 bytes
constexpr uint8_t kLock = 55;              // 1 byte: 0 = EEPROM writable, 1 = EEPROM write-locked
constexpr uint8_t kPresentPosition = 56;   // 2 bytes, sign-magnitude (bit 15)

constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstSyncRead = 0x82;
constexpr uint8_t kInstSyncWrite = 0x83;

constexpr uint8_t kMaxServos = 8;
constexpr size_t kMaxPacket = 64;
constexpr uint32_t kReplyTimeoutUs = 3000;  // servos answer within ~0.1 ms (Return_Delay_Time = 0)
// A reply can be lost when the radio blocks the UART interrupt, so every read is retried, as lerobot does
// on these buses. Failed attempts are spaced out by the reply timeout above.
constexpr uint8_t kAttempts = 4;
constexpr uint32_t kByteTimeoutUs = 500;
constexpr int kMaxSkippedBytes = 32;

class Port {
 public:
  virtual ~Port() = default;
  virtual void write(const uint8_t* data, size_t len) = 0;  // returns once the bytes have left the UART
  virtual int readByte(uint32_t timeout_us) = 0;             // next received byte, or -1 on timeout
  virtual void discardInput() = 0;
};

// Instruction packet: FF FF id len instruction params... checksum, with len = nparams + 2 and
// checksum = ~(id + len + instruction + params) & 0xFF. Returns the packet length.
inline size_t buildPacket(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t nparams, uint8_t* out) {
  uint8_t sum = id + (nparams + 2) + instruction;
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = id;
  out[3] = nparams + 2;
  out[4] = instruction;
  for (uint8_t i = 0; i < nparams; i++) {
    out[5 + i] = params[i];
    sum += params[i];
  }
  out[5 + nparams] = ~sum;
  return nparams + 6;
}

inline int32_t decodePosition(uint16_t raw) {  // sign-magnitude, sign in bit 15
  int32_t v = raw & 0x7FFF;
  return (raw & 0x8000) ? -v : v;
}

inline uint16_t encodeHoming(int32_t v) {  // sign-magnitude with the sign in bit 11
  if (v > 2047) v = 2047;
  if (v < -2047) v = -2047;
  return v < 0 ? (uint16_t)(((-v) & 0x7FF) | 0x800) : (uint16_t)(v & 0x7FF);
}

inline int32_t decodeHoming(uint16_t raw) {
  const int32_t mag = raw & 0x7FF;
  return (raw & 0x800) ? -mag : mag;
}

inline uint16_t encodePosition(int32_t v) {
  return v < 0 ? (uint16_t)((-v & 0x7FFF) | 0x8000) : (uint16_t)(v & 0x7FFF);
}

class Bus {
 public:
  explicit Bus(Port& port) : port_(port) {}

  bool ping(uint8_t id, uint8_t attempts = kAttempts) {
    for (uint8_t i = 0; i < attempts; i++) {
      port_.discardInput();
      send(id, kInstPing, nullptr, 0);
      if (receive(id, nullptr, 0)) return true;
    }
    return false;
  }

  bool read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out, uint8_t attempts = kAttempts) {
    const uint8_t params[2] = {addr, len};
    for (uint8_t i = 0; i < attempts; i++) {
      port_.discardInput();
      send(id, kInstRead, params, 2);
      if (receive(id, out, len)) return true;
    }
    return false;
  }

  bool read16(uint8_t id, uint8_t addr, uint16_t* value, uint8_t attempts = kAttempts) {
    uint8_t b[2];
    if (!read(id, addr, 2, b, attempts)) return false;
    *value = b[0] | (b[1] << 8);
    return true;
  }

  // Waits for the servo's status reply, so a false return means the write may not have happened.
  // The servo acknowledges a write, so a lost reply can make a successful write look failed; harmless
  // here because every write this firmware makes can be repeated.
  bool write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len, uint8_t attempts = kAttempts) {
    uint8_t params[kMaxPacket];
    params[0] = addr;
    for (uint8_t i = 0; i < len; i++) params[1 + i] = data[i];
    for (uint8_t i = 0; i < attempts; i++) {
      port_.discardInput();
      send(id, kInstWrite, params, len + 1);
      if (receive(id, nullptr, 0)) return true;
    }
    return false;
  }

  bool write8(uint8_t id, uint8_t addr, uint8_t value) { return write(id, addr, &value, 1); }

  bool write16(uint8_t id, uint8_t addr, uint16_t value) {
    const uint8_t b[2] = {(uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    return write(id, addr, b, 2);
  }

  // Reads `len` bytes at `addr` from every servo in one transaction; out holds n * len bytes in id order.
  bool syncRead(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, uint8_t* out,
                uint8_t attempts = kAttempts) {
    uint8_t params[2 + kMaxServos];
    params[0] = addr;
    params[1] = len;
    for (uint8_t i = 0; i < n; i++) params[2 + i] = ids[i];
    for (uint8_t a = 0; a < attempts; a++) {
      port_.discardInput();
      send(kBroadcastId, kInstSyncRead, params, n + 2);
      bool ok = true;
      for (uint8_t i = 0; i < n && ok; i++) ok = receive(ids[i], out + i * len, len);
      if (ok) return true;
    }
    return false;
  }

  // Writes `len` bytes per servo at `addr` in one broadcast packet. Servos don't reply to this.
  void syncWrite(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, const uint8_t* data) {
    uint8_t params[kMaxPacket];
    params[0] = addr;
    params[1] = len;
    uint8_t k = 2;
    for (uint8_t i = 0; i < n; i++) {
      params[k++] = ids[i];
      for (uint8_t j = 0; j < len; j++) params[k++] = data[i * len + j];
    }
    send(kBroadcastId, kInstSyncWrite, params, k);
  }

  bool readPositions(const uint8_t* ids, uint8_t n, int32_t* positions, uint8_t attempts = kAttempts) {
    uint8_t b[2 * kMaxServos];
    if (!syncRead(kPresentPosition, 2, ids, n, b, attempts)) return false;
    for (uint8_t i = 0; i < n; i++) positions[i] = decodePosition(b[2 * i] | (b[2 * i + 1] << 8));
    return true;
  }

  void writeGoals(const uint8_t* ids, uint8_t n, const int32_t* goals) {
    uint8_t b[2 * kMaxServos];
    for (uint8_t i = 0; i < n; i++) {
      const uint16_t raw = encodePosition(goals[i]);
      b[2 * i] = raw & 0xFF;
      b[2 * i + 1] = raw >> 8;
    }
    syncWrite(kGoalPosition, 2, ids, n, b);
  }

  uint8_t lastError() const { return last_error_; }  // error bits from the most recent status packet

 private:
  void send(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t nparams) {
    uint8_t packet[kMaxPacket];
    port_.write(packet, buildPacket(id, instruction, params, nparams, packet));
  }

  // Status packet: FF FF id len error params... checksum, with len = nparams + 2.
  bool receive(uint8_t expect_id, uint8_t* params, uint8_t nparams) {
    int ff = 0;
    int b = -1;
    for (int skipped = 0;; skipped++) {
      b = port_.readByte(skipped == 0 ? kReplyTimeoutUs : kByteTimeoutUs);
      if (b < 0 || skipped > kMaxSkippedBytes) return false;
      if (b == 0xFF) {
        ff++;
      } else if (ff >= 2) {
        break;  // b is the id
      } else {
        ff = 0;
      }
    }
    const int id = b;
    const int len = port_.readByte(kByteTimeoutUs);
    const int err = port_.readByte(kByteTimeoutUs);
    if (len < 0 || err < 0 || id != expect_id || len != nparams + 2) return false;
    uint8_t sum = id + len + err;
    for (uint8_t i = 0; i < nparams; i++) {
      const int v = port_.readByte(kByteTimeoutUs);
      if (v < 0) return false;
      params[i] = (uint8_t)v;
      sum += (uint8_t)v;
    }
    const int checksum = port_.readByte(kByteTimeoutUs);
    if (checksum < 0 || (uint8_t)~sum != checksum) return false;
    last_error_ = (uint8_t)err;
    return true;
  }

  Port& port_;
  uint8_t last_error_ = 0;
};

}  // namespace feetech
