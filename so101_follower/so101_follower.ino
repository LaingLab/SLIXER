// SO-101 wireless teleoperation, FOLLOWER board: a XIAO ESP32-C3 on the follower arm's Seeed Bus Servo
// Driver Board. Receives the leader's pose over ESP-NOW and drives the follower's six servos to match.
//
// Flash this folder to the follower's XIAO; flash so101_leader to the other one.
// Board: Tools > Board > esp32 > XIAO_ESP32C3, with USB CDC On Boot enabled (the default).
//
// Safety behaviour:
//  * torque stays off until all six servos answer, their calibration looks valid, every joint rests inside
//    its calibrated range, and fresh leader data is arriving;
//  * when torque comes on, the follower glides from where it is to the leader's pose instead of jumping;
//  * if leader data stops (out of range, leader switched off) it freezes in place, torque on, and glides
//    back once data returns;
//  * if its own servos stop answering or restart (power dip), it starts over from the checks.
//
// This board has no built-in LED. Wire one from D10 through a ~330 ohm resistor to GND for status at a
// glance; otherwise watch this board's state on the leader's USB, which it reports over the radio.

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

constexpr uint32_t kPeriodMs = 20;         // 50 Hz goal updates
constexpr uint32_t kStaleMs = 250;         // no leader data for this long -> freeze
constexpr uint32_t kHealthPeriodMs = 50;   // read the servos this often while moving
constexpr uint32_t kTelemetryPeriodMs = 50;  // report position this often: a PC steering the arm needs it
constexpr int kMaxHealthFailures = 10;     // ~1 s of failed reads -> start over
constexpr float kGlideDegPerSec = 45.0f;   // speed of the start-up / resume glide
constexpr int32_t kRangeTolerance = 150;   // a joint may rest this far (~13 deg) outside its range
constexpr int32_t kNoGlideTicks = 60;      // ~5 deg: closer than this, start following without gliding
constexpr uint8_t kCheckAttempts = 8;      // calibration reads gate everything, so retry them harder
constexpr int kStatusLed = D10;            // optional external LED, on when driven high
const uint8_t kIds[teleop::kNumJoints] = {1, 2, 3, 4, 5, 6};

enum class State : uint8_t { kChecking, kWaitingForLeader, kGliding, kFollowing, kFrozen, kRecording };

SerialPort<HardwareSerial> bus_port(Serial0);
feetech::Bus bus(bus_port);
teleop::Range cal[teleop::kNumJoints];

// Latest pose, written from the radio callback and the Wi-Fi poll.
portMUX_TYPE packet_mux = portMUX_INITIALIZER_UNLOCKED;
teleop::LeaderPacket latest;
uint32_t latest_ms = 0;
bool have_packet = false;
bool latest_from_program = false;  // whether the newest accepted pose came from a program or the leader
uint32_t program_pose_ms = 0;      // when a program's pose last arrived; 0 = never
uint32_t packets = 0, poses_since_report = 0, stale_poses = 0, leader_yielded = 0;
bool following_program = false;    // which the arm is following, so that a change of hands glides

State state = State::kChecking;
uint32_t next_tick = 0, last_check = 0, last_health = 0, last_torque_check = 0, last_report = 0;
uint32_t last_telemetry = 0;
uint32_t glide_start_ms = 0, glide_ms = 1000;
int health_failures = 0;
uint8_t torque_check_joint = 0;
bool check_failed = false;              // a check has failed since the last success: LED red
uint8_t fault = teleop::kFaultNone;     // what failed, for the LED blink code and the leader's log
uint8_t fault_joint = 0;
int32_t glide_from[teleop::kNumJoints];
int32_t goals[teleop::kNumJoints];
int32_t present_pos[teleop::kNumJoints] = {0};  // last known actual positions, reported to the PC
char problem[112] = "starting up";
teleop::RangeRecorder recorder;         // driven by R/S typed into the leader's Serial Monitor
volatile uint8_t pending_command = 0;

void onLeaderPose(const uint8_t* data, int len) {
  if (len == sizeof(teleop::CommandPacket) && data[0] == teleop::kCommandMagic) {
    pending_command = data[1];  // acted on in loop(), not in the radio callback
    return;
  }
  if (len != sizeof(teleop::LeaderPacket)) return;
  const bool from_program = data[0] == teleop::kProgramMagic;
  if (!from_program && data[0] != teleop::kPacketMagic) return;
  teleop::LeaderPacket incoming;
  memcpy(&incoming, data, sizeof incoming);
  const uint32_t arrived = millis();
  portENTER_CRITICAL(&packet_mux);
  // Two senders can't share the arm: each counts its own sequence numbers, so the newest-only filter below
  // would pick between them by whose counter happens to be higher, and the arm would lurch between two
  // poses. So while a program is sending, the leader waits its turn, whichever way its packets arrive, and
  // a quarter of a second after the program stops, the leader has the arm back.
  //
  // The claim is made by a program's pose arriving, not by one being accepted: while the leader's counter
  // is ahead, the program's are the poses the filter would throw away, so waiting for an accepted one
  // would wait for ever.
  if (from_program) {
    program_pose_ms = arrived;
  } else if (program_pose_ms != 0 && arrived - program_pose_ms < kStaleMs) {
    leader_yielded++;
    portEXIT_CRITICAL(&packet_mux);
    return;
  }
  // The same pose can arrive more than once -- by radio and by Wi-Fi -- and copies can overtake each other.
  // Acting on an older one makes the arm jump back and then forward again, which reads as twitching, so
  // only accept poses newer than the last. After a gap, take whatever arrives: the sequence may have moved
  // on or restarted. And when the pose comes from the other sender, its number means nothing next to the
  // last one's.
  const uint8_t newer_by = (uint8_t)(incoming.seq - latest.seq);
  const bool other_sender = have_packet && from_program != latest_from_program;
  if (!have_packet || other_sender || arrived - latest_ms > 500 || (newer_by != 0 && newer_by < 128)) {
    latest = incoming;
    latest_ms = arrived;
    latest_from_program = from_program;
    have_packet = true;
    packets++;
    poses_since_report++;
  } else {
    stale_poses++;
  }
  portEXIT_CRITICAL(&packet_mux);
}

// Copies the newest pose and says who sent it; true if it arrived less than kStaleMs ago.
bool freshPacket(teleop::LeaderPacket* out, bool* from_program) {
  portENTER_CRITICAL(&packet_mux);
  const bool have = have_packet;
  const uint32_t at = latest_ms;
  *out = latest;
  *from_program = latest_from_program;
  portEXIT_CRITICAL(&packet_mux);
  return have && millis() - at < kStaleMs;
}

const char* stateName() {
  switch (state) {
    case State::kRecording: return "recording ranges";
    case State::kChecking: return "checking arm";
    case State::kWaitingForLeader: return "waiting for leader";
    case State::kGliding: return "gliding to leader pose";
    case State::kFollowing: return "following";
    case State::kFrozen: return "holding (no leader data)";
  }
  return "?";
}

uint8_t stateCode() {
  switch (state) {
    case State::kRecording: return teleop::kStRecording;
    case State::kChecking: return teleop::kStChecking;
    case State::kWaitingForLeader: return teleop::kStWaiting;
    case State::kGliding: return teleop::kStGliding;
    case State::kFollowing: return teleop::kStFollowing;
    case State::kFrozen: return teleop::kStFrozen;
  }
  return teleop::kStChecking;
}

// While checking, the LED repeats the fault as a blink count: 1 = no reply from a servo, 2 = no usable
// calibration, 3 = a joint outside its range, 4 = torque refused. Steady on = following.
void updateLed(uint32_t now) {
  bool on = false;
  switch (state) {
    case State::kChecking: {
      const uint32_t blinks = fault == teleop::kFaultNone ? 1 : fault;
      const uint32_t cycle = now % (blinks * 500 + 1000);
      on = check_failed && cycle < blinks * 500 && (cycle / 250) % 2 == 0;
      break;
    }
    case State::kRecording: on = (now / 60) % 2; break;          // flicker: sweep the joints
    case State::kWaitingForLeader: on = (now / 500) % 2; break;  // slow blink: looking for the leader
    case State::kFrozen: on = (now / 125) % 2; break;            // fast blink: lost the leader, holding
    case State::kGliding: on = (now / 250) % 2; break;
    case State::kFollowing: on = true; break;
  }
  digitalWrite(kStatusLed, on ? HIGH : LOW);
}

void fail(uint32_t now, const char* why) {
  if (why != problem) snprintf(problem, sizeof problem, "%s", why);
  check_failed = true;
  Serial.printf("[follower] %s\n", problem);
  state = State::kChecking;
  last_check = now;
}

// Servos present, calibration plausible, every joint resting inside its calibrated range.
bool checkArm() {
  for (int j = 0; j < teleop::kNumJoints; j++) {
    uint16_t lo, hi;
    if (!bus.read16(kIds[j], feetech::kMinPositionLimit, &lo, kCheckAttempts) ||
        !bus.read16(kIds[j], feetech::kMaxPositionLimit, &hi, kCheckAttempts)) {
      snprintf(problem, sizeof problem, "no reply from servo %d (%s): arm powered? board in XIAO mode?", kIds[j],
               teleop::jointName(j));
      fault = teleop::kFaultNoReply;
      fault_joint = j;
      return false;
    }
    cal[j].min = lo;
    cal[j].max = hi;
  }
  if (!teleop::plausibleCalibration(cal, problem, sizeof problem)) {
    fault = teleop::kFaultCalibration;
    return false;
  }
  int32_t present[teleop::kNumJoints];
  if (!bus.readPositions(kIds, teleop::kNumJoints, present, kCheckAttempts)) {
    snprintf(problem, sizeof problem, "could not read joint positions");
    fault = teleop::kFaultNoReply;
    return false;
  }
  memcpy(present_pos, present, sizeof present_pos);
  for (int j = 0; j < teleop::kNumJoints; j++) {
    if (j == teleop::kWristRoll) continue;  // turns freely; its range is the whole circle
    if (present[j] < cal[j].min - kRangeTolerance || present[j] > cal[j].max + kRangeTolerance) {
      snprintf(problem, sizeof problem, "%s is at %ld, outside its calibrated range [%ld, %ld]: move it by hand",
               teleop::jointName(j), (long)present[j], (long)cal[j].min, (long)cal[j].max);
      fault = teleop::kFaultOutOfRange;
      fault_joint = j;
      return false;
    }
  }
  problem[0] = '\0';
  fault = teleop::kFaultNone;
  return true;
}

// Turns torque on without moving anything, then glides from the present pose to the leader's.
void startGlide(uint32_t now, const teleop::LeaderPacket& packet) {
  int32_t present[teleop::kNumJoints];
  if (!bus.readPositions(kIds, teleop::kNumJoints, present, kCheckAttempts)) {
    return fail(now, "could not read joint positions");
  }
  for (int j = 0; j < teleop::kNumJoints; j++) {
    const bool ok = bus.write8(kIds[j], feetech::kAcceleration, 254) &&  // as lerobot configures it
                    bus.write16(kIds[j], feetech::kGoalPosition, feetech::encodePosition(present[j])) &&
                    bus.write8(kIds[j], feetech::kTorqueEnable, 1);
    if (!ok) {  // servos already switched on just hold where they are
      snprintf(problem, sizeof problem, "servo %d (%s) did not accept torque-on", kIds[j], teleop::jointName(j));
      fault = teleop::kFaultWriteRefused;
      fault_joint = j;
      return fail(now, problem);
    }
  }
  int32_t target[teleop::kNumJoints];
  teleop::goalsFromPacket(packet, cal, target);
  int32_t farthest = 0;
  for (int j = 0; j < teleop::kNumJoints; j++) {
    glide_from[j] = present[j];
    goals[j] = present[j];
    const int32_t d = abs(target[j] - present[j]);
    if (d > farthest) farthest = d;
  }
  health_failures = 0;
  // Already where the leader is? Just follow. Gliding every time the link hiccups would make the arm
  // stutter, and a glide is only worth its slowness when the arm has somewhere to travel.
  if (farthest < kNoGlideTicks) {
    state = State::kFollowing;
    return;
  }
  const float seconds = farthest * 360.0f / teleop::kTurn / kGlideDegPerSec;
  glide_ms = (uint32_t)(seconds * 1000);
  if (glide_ms < 500) glide_ms = 500;
  if (glide_ms > 5000) glide_ms = 5000;
  glide_start_ms = now;
  state = State::kGliding;
  Serial.printf("[follower] torque on, gliding to the leader's pose over %lu ms\n", (unsigned long)glide_ms);
}

// While torque is on: servos must keep answering, and none may have restarted (a power dip turns torque off).
void checkHealth(uint32_t now) {
  if (now - last_health < kHealthPeriodMs) return;
  last_health = now;
  int32_t present[teleop::kNumJoints];
  if (!bus.readPositions(kIds, teleop::kNumJoints, present)) {
    if (++health_failures > kMaxHealthFailures) {
      fault = teleop::kFaultNoReply;
      fail(now, "follower servos stopped answering (power?)");
    }
    return;
  }
  health_failures = 0;
  memcpy(present_pos, present, sizeof present_pos);
  if (now - last_torque_check >= 1000) {
    last_torque_check = now;
    uint8_t torque = 1;
    const uint8_t id = kIds[torque_check_joint];
    torque_check_joint = (torque_check_joint + 1) % teleop::kNumJoints;
    if (bus.read(id, feetech::kTorqueEnable, 1, &torque) && torque == 0) {
      char why[72];
      snprintf(why, sizeof why, "servo %d lost torque (power dip?), restarting", id);
      fault = teleop::kFaultNoReply;
      fail(now, why);
    }
  }
}

// Keeps the reported position true while nothing else is reading the servos. Waiting for the leader, torque
// is off and the arm can be moved by hand; anything steering the arm starts from what's reported here, and a
// stale report would have it glide the arm back to where it used to be.
void refreshPositions(uint32_t now) {
  if (now - last_health < kHealthPeriodMs) return;
  last_health = now;
  int32_t present[teleop::kNumJoints];
  if (bus.readPositions(kIds, teleop::kNumJoints, present)) memcpy(present_pos, present, sizeof present_pos);
}

// Recording and re-centring both mean moving the arm by hand, so torque goes off first. After a fault the
// servos can still be holding -- fail() deliberately doesn't let a loaded arm drop -- and re-homing a servo
// that is holding a goal would drive it somewhere else entirely.
bool torqueOff() {
  bool ok = true;
  for (int j = 0; j < teleop::kNumJoints; j++) ok = bus.write8(kIds[j], feetech::kTorqueEnable, 0) && ok;
  return ok;
}

// Typing 'L' in the Serial Monitor runs this. Take the XIAO OFF its servo board, join D6 to D7 with a wire
// and power it from USB: it proves whether this board's serial port and those two pins work at all. On the
// servo board it would always fail, because that board mutes the receive line while transmitting.
void loopbackTest() {
  const uint8_t probe[] = {0x55, 0xAA, 0x0F, 0xF0, 0x12, 0x34};
  Serial.println("[follower] loopback test: XIAO off its board, D6 wired to D7...");
  while (Serial0.available()) Serial0.read();
  Serial0.write(probe, sizeof probe);
  Serial0.flush();
  uint8_t got[sizeof probe];
  size_t n = 0;
  const uint32_t deadline = millis() + 200;
  while (n < sizeof probe && (int32_t)(millis() - deadline) < 0) {
    if (Serial0.available()) got[n++] = (uint8_t)Serial0.read();
  }
  if (n == sizeof probe && memcmp(probe, got, n) == 0) {
    Serial.println("[follower] loopback PASSED: serial port and D6/D7 are fine");
  } else {
    Serial.printf("[follower] loopback FAILED: sent 6 bytes, got %u back\n", (unsigned)n);
  }
}

// R/S/X typed into the leader's Serial Monitor arrive here over the radio.
void handleCommand(uint32_t now, uint8_t command) {
  int32_t present[teleop::kNumJoints];
  switch (command) {
    case teleop::kCmdRecordRanges:
      if (state != State::kChecking && state != State::kWaitingForLeader) {
        Serial.println("[follower] can't record while the arm is being driven");
        return;
      }
      if (!torqueOff()) {
        Serial.println("[follower] can't record: not every servo would switch its torque off");
        return;
      }
      Serial.println("[follower] torque off: support the arm");
      if (!bus.readPositions(kIds, teleop::kNumJoints, present, kCheckAttempts)) {
        fail(now, "could not read joint positions");
        return;
      }
      recorder.start(present);
      state = State::kRecording;
      fault = teleop::kFaultNone;
      Serial.println("[follower] recording ranges: sweep every joint to both stops");
      break;
    case teleop::kCmdSaveRanges: {
      if (state != State::kRecording) return;
      char msg[112];
      if (recorder.save(bus, kIds, cal, msg, sizeof msg)) {
        Serial.println("[follower] new ranges saved");
        state = State::kChecking;  // re-reads them from the servos and checks they are usable
        check_failed = false;
        fault = teleop::kFaultNone;
        last_check = now - 1000;
      } else {
        Serial.printf("[follower] not saved: %s\n", msg);
        fault = teleop::kFaultCalibration;  // shows up in the leader's log
      }
      break;
    }
    case teleop::kCmdSetHome: {
      if (state != State::kChecking && state != State::kWaitingForLeader && state != State::kRecording) {
        Serial.println("[follower] can't re-centre while the arm is being driven");
        return;
      }
      if (!torqueOff()) {
        Serial.println("[follower] can't re-centre: not every servo would switch its torque off");
        return;
      }
      Serial.println("[follower] torque off: support the arm");
      char msg[112];
      if (teleop::setHome(bus, kIds, cal, msg, sizeof msg)) {
        Serial.printf("[follower] %s\n", msg);
        int32_t here[teleop::kNumJoints];
        if (bus.readPositions(kIds, teleop::kNumJoints, here, kCheckAttempts)) recorder.start(here);
        state = State::kRecording;
        fault = teleop::kFaultNone;
      } else {
        Serial.printf("[follower] not centred: %s\n", msg);
        fault = teleop::kFaultNoReply;
      }
      break;
    }
    case teleop::kCmdCancelRecording:
      recorder.cancel();
      if (state == State::kRecording) state = State::kChecking;
      break;
    default: break;
  }
}

// Position goes out many times a second, because anything steering this arm needs to see where it is.
void sendTelemetry(uint32_t now) {
  if (now - last_telemetry < kTelemetryPeriodMs) return;
  last_telemetry = now;
  const uint8_t poses = poses_since_report > 255 ? 255 : (uint8_t)poses_since_report;
  teleop::FollowerStatus status = {teleop::kStatusMagic, stateCode(), fault, fault_joint, poses,
                                   {0}, {0}, {0}, teleop::kProtocolVersion};
  for (int j = 0; j < teleop::kNumJoints; j++) {
    status.pos[j] = (int16_t)present_pos[j];
    status.range_min[j] = (int16_t)cal[j].min;
    status.range_max[j] = (int16_t)cal[j].max;
  }
  net::send(&status, sizeof status);  // read by the leader over the radio, and by a PC over Wi-Fi
}

void report(uint32_t now) {
  if (now - last_report < 2000) return;
  last_report = now;
  poses_since_report = 0;
  // The link is worth repeating rather than printing once at startup: at boot Wi-Fi has not had time to
  // join yet, so the first line always says "connecting" whether or not it ever succeeds. Saying it every
  // couple of seconds is the difference between knowing the follower is unreachable and guessing.
  Serial.printf("[follower] %s%s%s on %s | %lu poses (%lu stale ignored, %lu leader poses held back for a program)"
                " | goals:", stateName(), state == State::kChecking ? ": " : "",
                state == State::kChecking ? problem : "", net::status(), (unsigned long)packets,
                (unsigned long)stale_poses, (unsigned long)leader_yielded);
  for (int j = 0; j < teleop::kNumJoints; j++) Serial.printf(" %ld", (long)goals[j]);
  Serial.println();
}

}  // namespace

void setup() {
  pinMode(kStatusLed, OUTPUT);
  digitalWrite(kStatusLed, LOW);
  Serial.begin(115200);      // USB, for status messages
  Serial.setTxTimeoutMs(0);  // never stall the control loop when a PC is plugged in but not reading
  Serial0.begin(1000000, SERIAL_8N1, D7, D6);  // servo bus: RX = D7, TX = D6

  if (!net::begin(onLeaderPose)) {
    Serial.println("[follower] radio failed to start");
  }
  Serial.printf("[follower] link up (%s), this board is %s\n", net::status(), WiFi.macAddress().c_str());
}

void loop() {
  const uint32_t now = millis();
  updateLed(now);
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'l' || c == 'L') loopbackTest();
    if (c == 'w' || c == 'W') net::scanNetworks();
  }
  if ((int32_t)(now - next_tick) < 0) {
    delay(1);
    return;
  }
  next_tick = now + kPeriodMs;
  net::poll();
  sendTelemetry(now);
  report(now);
  if (pending_command != 0) {
    const uint8_t command = pending_command;
    pending_command = 0;
    handleCommand(now, command);
  }

  teleop::LeaderPacket packet;
  bool from_program = false;
  const bool fresh = freshPacket(&packet, &from_program);

  switch (state) {
    case State::kChecking:
      if (now - last_check >= 1000) {
        last_check = now;
        check_failed = !checkArm();
        if (!check_failed) {
          Serial.println("[follower] arm ready, waiting for the leader");
          state = State::kWaitingForLeader;
        } else {
          static char last_complaint[112] = "";
          static uint32_t last_complained = 0;
          if (strcmp(problem, last_complaint) != 0 || now - last_complained > 15000) {
            snprintf(last_complaint, sizeof last_complaint, "%s", problem);
            last_complained = now;
            Serial.printf("[follower] waiting: %s\n", problem);
          }
          if (fault == teleop::kFaultNoReply) {
            Serial.println("[follower] (type L here with the XIAO off its board and D6 wired to D7 to test its serial port)");
          }
        }
      }
      break;

    case State::kRecording: {
      int32_t present[teleop::kNumJoints];
      if (bus.readPositions(kIds, teleop::kNumJoints, present)) {
        recorder.update(present);
        memcpy(present_pos, present, sizeof present_pos);
      }
      break;
    }

    case State::kWaitingForLeader:
      if (fresh) {
        if (checkArm()) {
          following_program = from_program;
          startGlide(now, packet);  // the arm may have been moved by hand while waiting
        } else {
          check_failed = true;
          fail(now, problem);
        }
      } else {
        refreshPositions(now);
      }
      break;

    case State::kFrozen:
      if (fresh) {
        following_program = from_program;
        startGlide(now, packet);
      } else {
        checkHealth(now);  // torque is on: keep checking the servos answer, and report where the arm is
      }
      break;

    case State::kGliding:
    case State::kFollowing: {
      if (!fresh) {
        state = State::kFrozen;
        Serial.println("[follower] leader data stopped: holding position");
        break;
      }
      if (from_program != following_program) {
        // A program has just taken over from the leader, or handed back to it. The two were almost
        // certainly asking for different poses, and following the new one directly would be a lunge at
        // full servo speed; glide there instead, exactly as after any other interruption.
        following_program = from_program;
        Serial.printf("[follower] %s has the arm now\n", from_program ? "a program" : "the leader");
        startGlide(now, packet);
        break;
      }
      int32_t target[teleop::kNumJoints];
      teleop::goalsFromPacket(packet, cal, target);
      if (state == State::kGliding) {
        float u = (now - glide_start_ms) / (float)glide_ms;
        if (u >= 1.0f) {
          u = 1.0f;
          state = State::kFollowing;
        }
        const float s = u * u * (3.0f - 2.0f * u);  // ease in and out
        for (int j = 0; j < teleop::kNumJoints; j++) {
          goals[j] = glide_from[j] + lroundf((target[j] - glide_from[j]) * s);
        }
      } else {
        memcpy(goals, target, sizeof goals);
      }
      bus.writeGoals(kIds, teleop::kNumJoints, goals);
      checkHealth(now);
      break;
    }
  }
}
