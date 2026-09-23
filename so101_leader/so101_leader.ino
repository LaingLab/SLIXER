// SO-101 wireless teleoperation, LEADER board: a XIAO ESP32-C3 on the leader arm's Seeed Bus Servo Driver
// Board. Reads the six leader joints 50 times a second and broadcasts them to the follower board over
// ESP-NOW. Only reads the servos; never enables torque or writes to them.
//
// Flash this folder to the leader's XIAO; flash so101_follower to the other one.
// Board: Tools > Board > esp32 > XIAO_ESP32C3, with USB CDC On Boot enabled (the default).
//
// Calibration is read from the servos at start-up (lerobot-calibrate stores it there), so recalibrating
// never needs a re-flash.

#include "feetech.h"
#include "link.h"
#include "ranges.h"
#include "serial_port.h"
#include "teleop_math.h"

#if !defined(ARDUINO_XIAO_ESP32C3) && !defined(ARDUINO_XIAO_ESP32S3) && !defined(ARDUINO_XIAO_ESP32C6) \
    && !defined(ARDUINO_XIAO_ESP32C5)
#error "Wrong board: pick Tools > Board > esp32 > XIAO_ESP32C3. The IDE auto-detects 'Ozobot circuit kit' for this USB id, which is a different chip."
#endif

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled: otherwise status messages would go out on the servo bus"
#endif

namespace {

constexpr uint32_t kPeriodMs = 20;  // 50 Hz
constexpr uint32_t kRestartAfterFailuresMs = 500;
const uint8_t kIds[teleop::kNumJoints] = {1, 2, 3, 4, 5, 6};

SerialPort<HardwareSerial> bus_port(Serial0);
feetech::Bus bus(bus_port);
teleop::Range cal[teleop::kNumJoints];
teleop::Unwrapper unwrap[teleop::kNumJoints];

teleop::RangeRecorder recorder;  // 'r'/'s' re-record this arm's joint ranges; 'R'/'S' the follower's
bool arm_ready = false;
uint8_t seq = 0;
uint32_t next_tick = 0, last_good_read = 0, last_report = 0, last_check = 0;
uint32_t sent = 0, read_failures = 0, follower_reports = 0;
uint32_t last_follower_ms = 0;
int32_t last_pos[teleop::kNumJoints];

// The follower reports itself here, so its state is readable on this board's USB while it runs headless.
void onFollowerStatus(const uint8_t* data, int len) {
  // Either size: a follower from before the version byte is still worth hearing from.
  if (len != sizeof(teleop::FollowerStatus) && len != (int)teleop::kStatusV1Size) return;
  teleop::FollowerStatus status = {};
  memcpy(&status, data, len);
  if (status.magic != teleop::kStatusMagic) return;
  follower_reports++;
  last_follower_ms = millis();
  static uint32_t last_printed = 0;
  if (last_follower_ms - last_printed < 2000) return;  // it reports many times a second; log one
  last_printed = last_follower_ms;
  Serial.printf("[follower] %s", teleop::stateText(status.state));
  if (status.fault != teleop::kFaultNone && status.joint < teleop::kNumJoints) {
    Serial.printf(": %s %d (%s)", teleop::faultText(status.fault), status.joint + 1, teleop::jointName(status.joint));
  }
  Serial.printf(" | %u poses received\n", status.poses);
}

void sendCommand(uint8_t command) {
  const teleop::CommandPacket packet = {teleop::kCommandMagic, command};
  net::send(&packet, sizeof packet);
}

// Keys typed in the Serial Monitor. Lower case acts on this arm, upper case on the follower's.
void handleKey(int key) {
  switch (key) {
    case 'w':
    case 'W':
      net::scanNetworks();
      break;
    case 'r': {
      int32_t present[teleop::kNumJoints];
      if (!arm_ready || !bus.readPositions(kIds, teleop::kNumJoints, present, 8)) {
        Serial.println("[leader] can't read this arm yet");
        return;
      }
      recorder.start(present);
      Serial.println("[leader] recording ranges: sweep every joint to both stops, then press s. Poses are not sent while recording.");
      break;
    }
    case 's': {
      if (!recorder.active()) return;
      char msg[112];
      if (recorder.save(bus, kIds, cal, msg, sizeof msg)) {
        for (auto& u : unwrap) u.reset();
        Serial.print("[leader] new ranges saved:");
        for (int j = 0; j < teleop::kNumJoints; j++) {
          Serial.printf(" %s[%ld,%ld]", teleop::jointName(j), (long)cal[j].min, (long)cal[j].max);
        }
        Serial.println();
      } else {
        Serial.printf("[leader] not saved: %s\n", msg);
      }
      break;
    }
    case 'h': {
      if (!arm_ready) {
        Serial.println("[leader] can't read this arm yet");
        return;
      }
      char msg[112];
      Serial.println("[leader] centring this arm on the pose it is in now...");
      if (teleop::setHome(bus, kIds, cal, msg, sizeof msg)) {
        for (auto& u : unwrap) u.reset();
        Serial.printf("[leader] %s\n", msg);
      } else {
        Serial.printf("[leader] not centred: %s\n", msg);
      }
      break;
    }
    case 'H': sendCommand(teleop::kCmdSetHome); Serial.println("[leader] told the follower to centre on its current pose"); break;
    case 'x': recorder.cancel(); Serial.println("[leader] recording cancelled"); break;
    case 'R': sendCommand(teleop::kCmdRecordRanges); Serial.println("[leader] told the follower to record its ranges"); break;
    case 'S': sendCommand(teleop::kCmdSaveRanges); Serial.println("[leader] told the follower to save its ranges"); break;
    case 'X': sendCommand(teleop::kCmdCancelRecording); Serial.println("[leader] told the follower to cancel"); break;
    case '?':
      Serial.println("[leader] keys (lower case = this arm, upper = follower):");
      Serial.println("[leader]   h/H centre the arm on its current mid-range pose (do this first if a joint's");
      Serial.println("[leader]       travel crosses the encoder seam), then r/R sweep, then s/S save, x/X cancel");
      break;
    default: break;
  }
}

// Pings the six servos and loads their calibration.
bool checkArm() {
  char why[112] = "";
  for (int j = 0; j < teleop::kNumJoints && !why[0]; j++) {
    uint16_t lo, hi;
    if (!bus.read16(kIds[j], feetech::kMinPositionLimit, &lo, 8) ||
        !bus.read16(kIds[j], feetech::kMaxPositionLimit, &hi, 8)) {
      snprintf(why, sizeof why, "no reply from servo %d (%s): arm powered? board in XIAO mode?", kIds[j],
               teleop::jointName(j));
    } else {
      cal[j].min = lo;
      cal[j].max = hi;
    }
  }
  if (!why[0]) teleop::plausibleCalibration(cal, why, sizeof why);
  if (why[0]) {
    static char last_complaint[112] = "";
    static uint32_t last_complained = 0;
    if (strcmp(why, last_complaint) != 0 || millis() - last_complained > 15000) {  // don't flood the log
      snprintf(last_complaint, sizeof last_complaint, "%s", why);
      last_complained = millis();
      Serial.printf("[leader] waiting: %s\n", why);
    }
    return false;
  }
  for (auto& u : unwrap) u.reset();
  Serial.print("[leader] arm ready, calibration:");
  for (int j = 0; j < teleop::kNumJoints; j++) {
    Serial.printf(" %s[%ld,%ld]", teleop::jointName(j), (long)cal[j].min, (long)cal[j].max);
  }
  Serial.println();
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);      // USB, for status messages
  Serial.setTxTimeoutMs(0);  // never stall the control loop when a PC is plugged in but not reading
  Serial0.begin(1000000, SERIAL_8N1, D7, D6);  // servo bus: RX = D7, TX = D6

  if (!net::begin(onFollowerStatus)) {
    Serial.println("[leader] radio failed to start");
  }
  Serial.printf("[leader] link up (%s), this board is %s\n", net::status(), WiFi.macAddress().c_str());
}

void loop() {
  const uint32_t now = millis();
  if ((int32_t)(now - next_tick) < 0) {
    delay(1);
    return;
  }
  next_tick = now + kPeriodMs;
  net::poll();
  while (Serial.available()) handleKey(Serial.read());

  if (!arm_ready) {
    if (now - last_check >= 1000) {
      last_check = now;
      arm_ready = checkArm();
      last_good_read = now;
    }
    return;
  }

  int32_t raw[teleop::kNumJoints];
  if (!bus.readPositions(kIds, teleop::kNumJoints, raw)) {
    read_failures++;
    // Lost the arm for a while (power, cable): start over, re-reading calibration and joint positions.
    if (now - last_good_read > kRestartAfterFailuresMs) arm_ready = false;
    return;
  }
  last_good_read = now;
  for (int j = 0; j < teleop::kNumJoints; j++) {
    last_pos[j] = (j == teleop::kGripper) ? raw[j] : unwrap[j].update(raw[j], cal[j], j == teleop::kWristRoll);
  }
  // While recording, hold the follower still rather than making it mirror the sweep.
  if (recorder.active()) {
    recorder.update(raw);
    if (now - last_report >= 1000) {
      last_report = now;
      Serial.print("[leader] recording, swept so far:");
      for (int j = 0; j < teleop::kNumJoints; j++) {
        Serial.printf(" %s %ld", teleop::jointName(j), (long)recorder.span(j));
      }
      Serial.println(" (press s to save)");
    }
    return;
  }

  const teleop::LeaderPacket packet = teleop::makePacket(seq++, last_pos, cal);
  net::send(&packet, sizeof packet);
  sent++;

  if (now - last_report >= 2000) {
    last_report = now;
    const bool heard = last_follower_ms != 0 && now - last_follower_ms < 6000;
    Serial.printf("[leader] %s on %s, %lu poses sent, %lu follower reports, %lu read failures | deg:",
                  heard ? "follower reporting" : "no follower heard", net::status(), (unsigned long)sent,
                  (unsigned long)follower_reports, (unsigned long)read_failures);
    for (int j = 0; j < teleop::kGripper; j++) Serial.printf(" %.1f", packet.body[j] / 100.0f);
    Serial.printf(" | gripper %.0f%%\n", packet.gripper / 100.0f);
  }
}
