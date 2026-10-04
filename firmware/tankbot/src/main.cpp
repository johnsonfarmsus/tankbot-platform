// TankBot firmware v3: generic sensor table + directional reflexes.
//
// The robot's hardware description (name, drive type, motor/lidar pins, and a list of sensors with
// slot, position, yaw, tilt, role and thresholds) lives in flash as JSON and is edited from the
// brain's Bot page (or, with no brain, from /setup). Drivers are created from the table, so any
// number of bumpers / rangers facing any direction are just entries.
//
// UDP: 5601 lidar (docs/protocol.md), 5602 motion, 5603 sensors ("TSSUB" -> "TCAP1"+hardware JSON
// once, then "TSN1"+readings at 20 Hz). HTTP: /api/hardware (GET, POST JSON -> save + restart),
// /api/sensors, /tof/calibrate?id=..., /setup (fallback page).

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <math.h>
#include "esp_timer.h"
#include "secrets.h"
#include "web_ui.h"

static const char *FW_VERSION = "3.0";

// ================= hardware description =================
enum SType { ST_BUMPER, ST_TOF, ST_ULTRASONIC, ST_IMU, ST_LIDAR, ST_CAMERA, ST_OTHER };
enum SRole { ROLE_OBSTACLE, ROLE_CLIFF, ROLE_BUMP, ROLE_ORIENTATION, ROLE_MAPPING, ROLE_NONE };
enum Dir { DIR_FRONT = 0, DIR_LEFT = 1, DIR_BACK = 2, DIR_RIGHT = 3 };

struct Sensor {
  char id[16] = "";
  char name[24] = "";
  SType type = ST_OTHER;
  char slot[10] = "";        // BUMP1 BUMP2 TOF US1 US2 I2C LIDAR CUSTOM NONE
  int pinA = -1, pinB = -1;  // resolved from slot (or custom): bumper pin / trig,echo / rx,tx
  SRole role = ROLE_NONE;
  bool enabled = true;
  float yawDeg = 0;          // 0 front, +left
  bool floorTilt = false;    // pointed at the floor (cliff sensing)
  int stopMm = 150;          // obstacle role: block when closer
  int floorMm = 0;           // cliff role: calibrated floor reading (0 = not calibrated)
  int backoffMs = 150;       // bump role: reverse this long after a hit (0 = just stop)
  float left = 0, front = 0, height = 0, width = 0; // placement for the brain (mm)
  // live
  int value = -1;            // mm, or 1/0 for bumpers
  bool ok = false;
  uint32_t lastMs = 0;
  // ultrasonic timing
  volatile uint32_t echoStart = 0, echoUs = 0; volatile bool echoDone = false; uint32_t lastTrigMs = 0;
};

static const int MAX_SENSORS = 12;
Sensor sensors[MAX_SENSORS];
int nSensors = 0;

struct Hardware {
  char name[24] = "TankBot";
  char drive[12] = "tank";
  int in1 = 16, in2 = 17, in3 = 18, in4 = 19, ena = 25, enb = 26;
  int lidarRx = 4, lidarTx = 27;
  bool lidar = true;
  int speed = 255, trim = 0;
} hw;

const char *typeName(SType t) {
  switch (t) { case ST_BUMPER: return "bumper"; case ST_TOF: return "tof"; case ST_ULTRASONIC: return "ultrasonic";
    case ST_IMU: return "imu"; case ST_LIDAR: return "lidar"; case ST_CAMERA: return "camera"; default: return "other"; }
}
SType typeFrom(const char *s) {
  if (!strcmp(s, "bumper")) return ST_BUMPER; if (!strcmp(s, "tof")) return ST_TOF; if (!strcmp(s, "ultrasonic")) return ST_ULTRASONIC;
  if (!strcmp(s, "imu")) return ST_IMU; if (!strcmp(s, "lidar")) return ST_LIDAR; if (!strcmp(s, "camera")) return ST_CAMERA; return ST_OTHER;
}
const char *roleName(SRole r) {
  switch (r) { case ROLE_OBSTACLE: return "obstacle"; case ROLE_CLIFF: return "cliff"; case ROLE_BUMP: return "bump";
    case ROLE_ORIENTATION: return "orientation"; case ROLE_MAPPING: return "mapping"; default: return "none"; }
}
SRole roleFrom(const char *s) {
  if (!strcmp(s, "obstacle")) return ROLE_OBSTACLE; if (!strcmp(s, "cliff")) return ROLE_CLIFF; if (!strcmp(s, "bump")) return ROLE_BUMP;
  if (!strcmp(s, "orientation")) return ROLE_ORIENTATION; if (!strcmp(s, "mapping")) return ROLE_MAPPING; return ROLE_NONE;
}
Dir dirOf(float yawDeg) {
  float y = fmodf(yawDeg + 540.0f, 360.0f) - 180.0f;
  if (fabsf(y) <= 45) return DIR_FRONT;
  if (fabsf(y) >= 135) return DIR_BACK;
  return y > 0 ? DIR_LEFT : DIR_RIGHT;
}
const char *dirName(Dir d) { return d == DIR_FRONT ? "front" : d == DIR_LEFT ? "left" : d == DIR_BACK ? "back" : "right"; }

/// Standard-board slots -> pins. docs/wiring.md
void resolveSlot(Sensor &s) {
  if (!strcmp(s.slot, "BUMP1")) { s.pinA = 13; s.pinB = -1; }
  else if (!strcmp(s.slot, "BUMP2")) { s.pinA = 23; s.pinB = -1; }
  else if (!strcmp(s.slot, "TOF")) { s.pinA = 32; s.pinB = 33; }      // ToF T -> 32 (rx), R <- 33 (tx)
  else if (!strcmp(s.slot, "US1")) { s.pinA = 14; s.pinB = 34; }      // trig, echo
  else if (!strcmp(s.slot, "US2")) { s.pinA = 2; s.pinB = 35; }
  else if (!strcmp(s.slot, "I2C")) { s.pinA = 21; s.pinB = 22; }
  else if (!strcmp(s.slot, "LIDAR")) { s.pinA = hw.lidarRx; s.pinB = hw.lidarTx; }
  // CUSTOM keeps the pins given; NONE (phone camera etc.) has no pins
}

Preferences prefs;

String hardwareJson(bool withLive);

void defaultSensors() {
  nSensors = 0;
  Sensor &l = sensors[nSensors++];
  strcpy(l.id, "lidar"); strcpy(l.name, "RPLidar"); l.type = ST_LIDAR; strcpy(l.slot, "LIDAR"); l.role = ROLE_MAPPING;
  l.left = 92; l.front = 40; l.height = 200;
  resolveSlot(l);
}

bool loadSensorsJson(const String &json) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) return false;
  JsonArray arr = doc["sensors"].as<JsonArray>();
  if (arr.isNull()) return false;
  nSensors = 0;
  for (JsonObject o : arr) {
    if (nSensors >= MAX_SENSORS) break;
    Sensor &s = sensors[nSensors];
    s = Sensor();
    strlcpy(s.id, o["id"] | "", sizeof(s.id));
    if (!s.id[0]) continue;
    strlcpy(s.name, o["name"] | s.id, sizeof(s.name));
    s.type = typeFrom(o["type"] | "other");
    strlcpy(s.slot, o["slot"] | "NONE", sizeof(s.slot));
    s.pinA = o["pinA"] | -1; s.pinB = o["pinB"] | -1;
    s.role = roleFrom(o["role"] | "none");
    s.enabled = o["enabled"] | true;
    s.yawDeg = o["yawDeg"] | 0.0f;
    s.floorTilt = o["floorTilt"] | false;
    s.stopMm = o["stopMm"] | 150;
    s.floorMm = o["floorMm"] | 0;
    s.backoffMs = constrain((int)(o["backoffMs"] | 150), 0, 1000);
    s.left = o["left"] | 0.0f; s.front = o["front"] | 0.0f; s.height = o["height"] | 0.0f; s.width = o["width"] | 0.0f;
    if (strcmp(s.slot, "CUSTOM")) resolveSlot(s);
    nSensors++;
  }
  if (doc["name"].is<const char *>()) strlcpy(hw.name, doc["name"], sizeof(hw.name));
  if (doc["drive"].is<const char *>()) strlcpy(hw.drive, doc["drive"], sizeof(hw.drive));
  JsonObject pins = doc["pins"];
  if (!pins.isNull()) {
    hw.in1 = pins["in1"] | hw.in1; hw.in2 = pins["in2"] | hw.in2; hw.in3 = pins["in3"] | hw.in3; hw.in4 = pins["in4"] | hw.in4;
    hw.ena = pins["ena"] | hw.ena; hw.enb = pins["enb"] | hw.enb; hw.lidarRx = pins["lidarRx"] | hw.lidarRx; hw.lidarTx = pins["lidarTx"] | hw.lidarTx;
  }
  hw.lidar = false;
  for (int i = 0; i < nSensors; i++) if (sensors[i].type == ST_LIDAR && sensors[i].enabled) hw.lidar = true;
  return true;
}

void saveHardware() {
  prefs.begin("tankbot", false);
  prefs.putString("hw", hardwareJson(false));
  prefs.putInt("speed", hw.speed); prefs.putInt("trim", hw.trim);
  prefs.end();
}

void loadHardware() {
  prefs.begin("tankbot", true);
  String json = prefs.getString("hw", "");
  hw.speed = prefs.getInt("speed", 255); hw.trim = prefs.getInt("trim", 0);
  bool ok = json.length() > 0 && loadSensorsJson(json);
  if (!ok) {
    // migrate v2 settings (fixed slots) into the table
    String n = prefs.getString("name", hw.name); n.toCharArray(hw.name, sizeof(hw.name));
    String d = prefs.getString("drive", hw.drive); d.toCharArray(hw.drive, sizeof(hw.drive));
    hw.in1 = prefs.getInt("in1", hw.in1); hw.in2 = prefs.getInt("in2", hw.in2); hw.in3 = prefs.getInt("in3", hw.in3); hw.in4 = prefs.getInt("in4", hw.in4);
    hw.ena = prefs.getInt("ena", hw.ena); hw.enb = prefs.getInt("enb", hw.enb);
    hw.lidarRx = prefs.getInt("lidarRx", hw.lidarRx); hw.lidarTx = prefs.getInt("lidarTx", hw.lidarTx);
    defaultSensors();
    if (prefs.getBool("sTof", false)) { Sensor &s = sensors[nSensors++]; strcpy(s.id, "tof"); strcpy(s.name, "ToF (floor)"); s.type = ST_TOF; strcpy(s.slot, "TOF"); s.role = ROLE_CLIFF; s.floorTilt = true; s.floorMm = prefs.getInt("floorMm", 0); s.left = 92; s.front = 10; s.height = -20; resolveSlot(s); }
    if (prefs.getBool("sUs", false)) { Sensor &s = sensors[nSensors++]; strcpy(s.id, "us1"); strcpy(s.name, "Ultrasonic"); s.type = ST_ULTRASONIC; strcpy(s.slot, "US1"); s.role = ROLE_OBSTACLE; s.stopMm = prefs.getInt("usStop", 150); s.left = 92; s.front = 5; s.height = -20; resolveSlot(s); }
    if (prefs.getBool("sBumpL", false)) { Sensor &s = sensors[nSensors++]; strcpy(s.id, "bump1"); strcpy(s.name, "Front bumper"); s.type = ST_BUMPER; strcpy(s.slot, "BUMP1"); s.role = ROLE_BUMP; s.left = 92; s.front = -20; s.height = -30; s.width = 160; resolveSlot(s); }
    if (prefs.getBool("sBumpR", false)) { Sensor &s = sensors[nSensors++]; strcpy(s.id, "bump2"); strcpy(s.name, "Right bumper"); s.type = ST_BUMPER; strcpy(s.slot, "BUMP2"); s.role = ROLE_BUMP; s.left = 140; s.front = -20; s.height = -30; resolveSlot(s); }
    hw.lidar = prefs.getBool("sLidar", true);
    sensors[0].enabled = hw.lidar;
  }
  prefs.end();
  if (!ok) saveHardware();
}

String hardwareJson(bool withLive) {
  JsonDocument doc;
  doc["name"] = hw.name; doc["fw"] = FW_VERSION; doc["drive"] = hw.drive;
  JsonObject pins = doc["pins"].to<JsonObject>();
  pins["in1"] = hw.in1; pins["in2"] = hw.in2; pins["in3"] = hw.in3; pins["in4"] = hw.in4; pins["ena"] = hw.ena; pins["enb"] = hw.enb;
  pins["lidarRx"] = hw.lidarRx; pins["lidarTx"] = hw.lidarTx;
  JsonArray slots = doc["slots"].to<JsonArray>();
  for (const char *n : {"BUMP1", "BUMP2", "TOF", "US1", "US2", "I2C", "LIDAR", "CUSTOM", "NONE"}) slots.add(n);
  JsonArray arr = doc["sensors"].to<JsonArray>();
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    JsonObject o = arr.add<JsonObject>();
    o["id"] = s.id; o["name"] = s.name; o["type"] = typeName(s.type); o["slot"] = s.slot;
    o["pinA"] = s.pinA; o["pinB"] = s.pinB; o["role"] = roleName(s.role); o["enabled"] = s.enabled;
    o["yawDeg"] = s.yawDeg; o["floorTilt"] = s.floorTilt; o["stopMm"] = s.stopMm; o["floorMm"] = s.floorMm; o["backoffMs"] = s.backoffMs;
    o["left"] = s.left; o["front"] = s.front; o["height"] = s.height; o["width"] = s.width;
    if (withLive) { o["value"] = s.value; o["ok"] = s.ok; }
  }
  String out; serializeJson(doc, out); return out;
}

// ================= shared =================
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
static const uint16_t LIDAR_PORT = 5601, MOTION_PORT = 5602, SENSOR_PORT = 5603;
static const char *HOSTNAME = "tankbot";
static const char *AP_SSID = "TankBot";
static const char *AP_PASS = "tankbot2025";
static const uint32_t UDP_CMD_TIMEOUT_MS = 300, WEB_CMD_TIMEOUT_MS = 500;

WebServer server(80);
DNSServer dnsServer;
WiFiUDP udpLidar, udpMotion, udpSensor;
bool apMode = false;
uint32_t apSinceMs = 0;

// ================= REFLEX state =================
bool blockDir[4] = {false, false, false, false};
char blockReason[4][24] = {"", "", "", ""};
uint32_t backoffUntilMs = 0;
uint32_t reflexEvents = 0;

// ================= MOTION =================
enum CmdSource { SRC_NONE, SRC_WEB, SRC_UDP, SRC_REFLEX };
float curLeft = 0, curRight = 0;
CmdSource cmdSrc = SRC_NONE;
uint32_t lastCmdMs = 0, watchdogTrips = 0, udpCommands = 0, blockedCommands = 0;

const char *srcName(CmdSource s) { return s == SRC_WEB ? "web" : s == SRC_UDP ? "udp" : s == SRC_REFLEX ? "reflex" : "none"; }

void stopMotorsRaw() {
  digitalWrite(hw.in1, LOW); digitalWrite(hw.in2, LOW); digitalWrite(hw.in3, LOW); digitalWrite(hw.in4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0); ledcWrite(PWM_CHANNEL_B, 0);
  curLeft = curRight = 0;
}

void setupMotors() {
  pinMode(hw.in1, OUTPUT); pinMode(hw.in2, OUTPUT); pinMode(hw.in3, OUTPUT); pinMode(hw.in4, OUTPUT);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(hw.ena, PWM_CHANNEL_A);
  ledcAttachPin(hw.enb, PWM_CHANNEL_B);
  stopMotorsRaw();
}

void applyMotors(float left, float right) {
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  if (fabsf(left) < 0.05f && fabsf(right) < 0.05f) { stopMotorsRaw(); return; }
  int leftSpeed = abs((int)(left * hw.speed));
  int rightSpeed = abs((int)(right * hw.speed));
  if (hw.trim < 0) leftSpeed = constrain(leftSpeed + (int)(hw.trim * fabsf(left)), 0, 255);
  else if (hw.trim > 0) rightSpeed = constrain(rightSpeed - (int)(hw.trim * fabsf(right)), 0, 255);
  if (left >= 0) { digitalWrite(hw.in1, HIGH); digitalWrite(hw.in2, LOW); }
  else           { digitalWrite(hw.in1, LOW);  digitalWrite(hw.in2, HIGH); }
  if (right >= 0) { digitalWrite(hw.in3, LOW);  digitalWrite(hw.in4, HIGH); }
  else            { digitalWrite(hw.in3, HIGH); digitalWrite(hw.in4, LOW); }
  ledcWrite(PWM_CHANNEL_A, leftSpeed);
  ledcWrite(PWM_CHANNEL_B, rightSpeed);
  curLeft = left; curRight = right;
}

void stopAll() { stopMotorsRaw(); cmdSrc = SRC_NONE; }

/// Does this command move the robot into a blocked direction?
bool vetoed(float l, float r, const char **why) {
  float fwd = l + r, spin = r - l;
  if (fwd > 0.05f && blockDir[DIR_FRONT]) { *why = blockReason[DIR_FRONT]; return true; }
  if (fwd < -0.05f && blockDir[DIR_BACK]) { *why = blockReason[DIR_BACK]; return true; }
  if (fabsf(fwd) < 0.1f) { // turning on the spot: the side we swing toward
    if (spin > 0.05f && blockDir[DIR_LEFT]) { *why = blockReason[DIR_LEFT]; return true; }
    if (spin < -0.05f && blockDir[DIR_RIGHT]) { *why = blockReason[DIR_RIGHT]; return true; }
  }
  return false;
}

void commandMotors(float left, float right, CmdSource src) {
  if (isnan(left) || isnan(right)) { stopAll(); return; }
  if (src != SRC_REFLEX && millis() < backoffUntilMs) return;
  const char *why = "";
  if (src != SRC_REFLEX && vetoed(left, right, &why)) {
    blockedCommands++;
    if (curLeft != 0 || curRight != 0) stopAll();
    lastCmdMs = millis();
    return;
  }
  applyMotors(left, right);
  bool moving = (curLeft != 0 || curRight != 0);
  cmdSrc = moving ? src : SRC_NONE;
  lastCmdMs = millis();
}

void motionWatchdog() {
  if (cmdSrc == SRC_NONE || cmdSrc == SRC_REFLEX) return;
  uint32_t timeout = cmdSrc == SRC_UDP ? UDP_CMD_TIMEOUT_MS : WEB_CMD_TIMEOUT_MS;
  if (millis() - lastCmdMs > timeout) {
    Serial.printf("[motion] WATCHDOG STOP: %s commands stopped arriving\n", srcName(cmdSrc));
    stopAll();
    watchdogTrips++;
  }
}

IPAddress motionPeer; uint16_t motionPeerPort = 0; uint32_t motionPeerSeen = 0;

void handleMotionUdp() {
  int sz = udpMotion.parsePacket();
  while (sz > 0) {
    uint8_t buf[32] = {0};
    int n = udpMotion.read(buf, sizeof(buf));
    motionPeer = udpMotion.remoteIP(); motionPeerPort = udpMotion.remotePort(); motionPeerSeen = millis();
    if (n >= 12 && memcmp(buf, "TMC1", 4) == 0) {
      float fwd, turn;
      memcpy(&fwd, buf + 4, 4); memcpy(&turn, buf + 8, 4);
      if (isnan(fwd) || isnan(turn)) { stopAll(); }
      else {
        fwd = constrain(fwd, -1.0f, 1.0f); turn = constrain(turn, -1.0f, 1.0f);
        float l = fwd - turn, r = fwd + turn;
        float m = max(fabsf(l), fabsf(r));
        if (m > 1.0f) { l /= m; r /= m; }
        commandMotors(l, r, SRC_UDP);
        udpCommands++;
      }
    } else if (n >= 4 && memcmp(buf, "TMS1", 4) == 0) {
      stopAll();
    }
    sz = udpMotion.parsePacket();
  }
}

String blockSummary() {
  String s;
  for (int d = 0; d < 4; d++) if (blockDir[d]) { if (s.length()) s += ","; s += dirName((Dir)d); }
  return s.length() ? s : "none";
}

void sendMotionStatus() {
  if (motionPeerPort == 0 || millis() - motionPeerSeen > 3000) return;
  char body[260];
  int n = snprintf(body, sizeof(body),
    "TMH1{\"left\":%.2f,\"right\":%.2f,\"src\":\"%s\",\"wd_trips\":%lu,\"speed\":%d,\"trim\":%d,\"cmds\":%lu,\"block\":\"%s\",\"blocked_cmds\":%lu}",
    curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips, hw.speed, hw.trim, (unsigned long)udpCommands,
    blockSummary().c_str(), (unsigned long)blockedCommands);
  udpMotion.beginPacket(motionPeer, motionPeerPort);
  udpMotion.write((uint8_t *)body, n);
  udpMotion.endPacket();
}

// ================= LIDAR (unchanged bridge) =================
HardwareSerial Lidar(2);
static const uint32_t LIDAR_BAUD = 460800;
static const int MAX_PTS = 1400, PTS_PER_CHUNK = 250, MAX_SUBS = 3;
static const uint32_t SUB_TIMEOUT_MS = 5000;
struct Pt { uint16_t a, d; uint8_t q; };
Pt revBuf[MAX_PTS]; int revN = 0; uint32_t revStartMs = 0;
uint32_t revs = 0, badSync = 0, lastByteMs = 0, lastRevStart = 0, packetsSent = 0, sendErrors = 0;
float revHz = 0; int lastRevPts = 0;
uint8_t node[5]; int ni = 0;
struct Sub { IPAddress ip; uint16_t port; uint32_t seen; bool used; };
Sub subs[MAX_SUBS];

void lidarCmd(uint8_t c) { uint8_t b[2] = {0xA5, c}; Lidar.write(b, 2); Lidar.flush(); }
void lidarDrain(uint32_t ms) { uint32_t t = millis(); while (millis() - t < ms) { while (Lidar.available()) Lidar.read(); delay(1); } }

bool lidarDescriptor(uint32_t timeoutMs, uint32_t &len, uint8_t &type) {
  uint8_t d[7]; int n = 0; uint32_t t = millis();
  while (millis() - t < timeoutMs) {
    if (!Lidar.available()) { delay(1); continue; }
    uint8_t b = Lidar.read();
    if (n == 0 && b != 0xA5) continue;
    if (n == 1 && b != 0x5A) { n = (b == 0xA5) ? 1 : 0; continue; }
    d[n++] = b;
    if (n == 7) {
      uint32_t v = d[2] | (d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
      len = v & 0x3FFFFFFF; type = d[6]; return true;
    }
  }
  return false;
}

bool lidarStartScan() {
  lidarCmd(0x25); delay(20); lidarDrain(100);
  lidarCmd(0x20);
  uint32_t len; uint8_t type;
  bool ok = lidarDescriptor(2000, len, type) && len == 5 && type == 0x81;
  Serial.printf("[lidar] scan start %s\n", ok ? "OK" : "FAILED");
  ni = 0; revN = 0;
  return ok;
}

int activeSubs() { int n = 0; for (auto &s : subs) if (s.used) n++; return n; }

void lidarSendToSubs(const uint8_t *buf, size_t len) {
  for (auto &s : subs) {
    if (!s.used) continue;
    udpLidar.beginPacket(s.ip, s.port);
    udpLidar.write(buf, len);
    if (udpLidar.endPacket()) packetsSent++; else sendErrors++;
  }
}

void lidarSendRev() {
  if (activeSubs() == 0 || revN == 0) return;
  static uint8_t pkt[16 + PTS_PER_CHUNK * 5];
  uint8_t chunks = (revN + PTS_PER_CHUNK - 1) / PTS_PER_CHUNK;
  for (uint8_t c = 0; c < chunks; c++) {
    int start = c * PTS_PER_CHUNK;
    uint16_t n = min(PTS_PER_CHUNK, revN - start);
    size_t o = 0;
    memcpy(pkt, "TLS1", 4); o = 4;
    memcpy(pkt + o, &revs, 4); o += 4;
    memcpy(pkt + o, &revStartMs, 4); o += 4;
    pkt[o++] = c; pkt[o++] = chunks;
    memcpy(pkt + o, &n, 2); o += 2;
    for (int i = 0; i < n; i++) {
      const Pt &p = revBuf[start + i];
      memcpy(pkt + o, &p.a, 2); o += 2;
      memcpy(pkt + o, &p.d, 2); o += 2;
      pkt[o++] = p.q;
    }
    lidarSendToSubs(pkt, o);
  }
}

void lidarFinishRev() {
  uint32_t now = millis();
  if (lastRevStart) { float hz = 1000.0f / (float)(now - lastRevStart); revHz = revHz == 0 ? hz : revHz * 0.8f + hz * 0.2f; }
  lastRevStart = now;
  revs++; lastRevPts = revN;
  lidarSendRev();
  revN = 0; revStartMs = now;
}

void lidarByte(uint8_t b) {
  node[ni++] = b;
  if (ni == 1) { if ((b & 1) == ((b >> 1) & 1)) { ni = 0; badSync++; } return; }
  if (ni == 2) { if (!(b & 1)) { ni = 0; badSync++; } return; }
  if (ni < 5) return;
  ni = 0;
  if ((node[0] & 1) && revN > 0) lidarFinishRev();
  uint16_t dq2 = node[3] | (node[4] << 8);
  if (dq2 == 0 || revN >= MAX_PTS) return;
  revBuf[revN].a = (node[1] >> 1) | (node[2] << 7);
  revBuf[revN].d = dq2;
  revBuf[revN].q = node[0] >> 2;
  revN++;
}

void handleLidarUdp() {
  int sz = udpLidar.parsePacket();
  while (sz > 0) {
    char msg[16] = {0};
    udpLidar.read(msg, min(sz, 15));
    if (strncmp(msg, "TLSYN", 5) == 0 && sz >= 9) {
      uint32_t seq; memcpy(&seq, msg + 5, 4);
      int64_t us = esp_timer_get_time();
      uint8_t r[17];
      memcpy(r, "TLSY1", 5); memcpy(r + 5, &seq, 4); memcpy(r + 9, &us, 8);
      udpLidar.beginPacket(udpLidar.remoteIP(), udpLidar.remotePort());
      udpLidar.write(r, sizeof(r));
      udpLidar.endPacket();
    } else if (strncmp(msg, "TLSUB", 5) == 0) {
      IPAddress ip = udpLidar.remoteIP(); uint16_t port = udpLidar.remotePort();
      Sub *slot = nullptr;
      for (auto &s : subs) if (s.used && s.ip == ip && s.port == port) slot = &s;
      if (!slot) for (auto &s : subs) if (!s.used) { slot = &s; break; }
      if (slot) { slot->ip = ip; slot->port = port; slot->seen = millis(); slot->used = true; }
    }
    sz = udpLidar.parsePacket();
  }
  for (auto &s : subs) if (s.used && millis() - s.seen > SUB_TIMEOUT_MS) s.used = false;
}

void sendLidarStatus() {
  char body[256];
  int n = snprintf(body, sizeof(body),
    "TLH1{\"uptime_ms\":%lu,\"revs\":%lu,\"hz\":%.2f,\"pts\":%d,\"sync_errs\":%lu,\"rssi\":%d,\"subs\":%d,\"sent\":%lu,\"send_errs\":%lu}",
    (unsigned long)millis(), (unsigned long)revs, revHz, lastRevPts, (unsigned long)badSync, apMode ? 0 : WiFi.RSSI(),
    activeSubs(), (unsigned long)packetsSent, (unsigned long)sendErrors);
  lidarSendToSubs((uint8_t *)body, n);
}

// ================= SENSOR DRIVERS =================
HardwareSerial Tof(1);
Sensor *tofSensor = nullptr; uint8_t tofBuf[16]; int tofN = 0;

void tofByte(uint8_t b) {
  if (tofN == 0 && b != 0x57) return;
  if (tofN == 1 && b != 0x00) { tofN = (b == 0x57) ? 1 : 0; return; }
  tofBuf[tofN++] = b;
  if (tofN < 16) return;
  tofN = 0;
  uint8_t sum = 0;
  for (int i = 0; i < 15; i++) sum += tofBuf[i];
  if (sum != tofBuf[15] || !tofSensor) return;
  int32_t dis = tofBuf[9] | (tofBuf[10] << 8) | (tofBuf[11] << 16);
  if (dis & 0x800000) dis |= 0xFF000000;
  uint8_t status = tofBuf[12];
  uint16_t strength = tofBuf[13] | (tofBuf[14] << 8);
  tofSensor->lastMs = millis();
  tofSensor->ok = (status == 0 && strength > 0 && dis > 0);
  tofSensor->value = tofSensor->ok ? dis : -1;
}

void IRAM_ATTR usIsr(void *arg) {
  Sensor *s = (Sensor *)arg;
  if (digitalRead(s->pinB)) s->echoStart = micros();
  else { s->echoUs = micros() - s->echoStart; s->echoDone = true; }
}

void usTick(Sensor &s) {
  uint32_t now = millis();
  if (s.echoDone) {
    s.echoDone = false;
    uint32_t us = s.echoUs;
    s.value = (us > 100 && us < 30000) ? (int)(us * 0.1715f) : -1;
    s.ok = s.value > 0;
    s.lastMs = now;
  }
  if (now - s.lastTrigMs >= 60) {
    s.lastTrigMs = now;
    if (now - s.lastMs > 200) { s.value = -1; s.ok = false; }
    digitalWrite(s.pinA, LOW); delayMicroseconds(2);
    digitalWrite(s.pinA, HIGH); delayMicroseconds(10);
    digitalWrite(s.pinA, LOW);
  }
}

void setupSensors() {
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    if (!s.enabled) continue;
    switch (s.type) {
      case ST_BUMPER: if (s.pinA >= 0) pinMode(s.pinA, INPUT_PULLUP); break;
      case ST_TOF:
        if (!tofSensor && s.pinA >= 0 && s.pinB >= 0) { tofSensor = &s; Tof.setRxBufferSize(1024); Tof.begin(921600, SERIAL_8N1, s.pinA, s.pinB); }
        break;
      case ST_ULTRASONIC:
        if (s.pinA >= 0 && s.pinB >= 0) {
          pinMode(s.pinA, OUTPUT); digitalWrite(s.pinA, LOW);
          pinMode(s.pinB, INPUT);
          attachInterruptArg(digitalPinToInterrupt(s.pinB), usIsr, &s, CHANGE);
        }
        break;
      default: break;
    }
  }
}

void readSensors() {
  uint32_t now = millis();
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    if (!s.enabled) continue;
    switch (s.type) {
      case ST_BUMPER:
        if (s.pinA >= 0) { s.value = digitalRead(s.pinA) == HIGH ? 1 : 0; s.ok = true; s.lastMs = now; } // NC wiring: HIGH = pressed
        break;
      case ST_TOF:
        if (&s == tofSensor) { while (Tof.available()) tofByte(Tof.read()); if (now - s.lastMs > 300) { s.ok = false; s.value = -1; } }
        break;
      case ST_ULTRASONIC: if (s.pinA >= 0) usTick(s); break;
      default: break;
    }
  }
}

// ================= REFLEX =================
bool wasBlocked[4] = {false, false, false, false};

void reflexUpdate() {
  uint32_t now = millis();
  bool nb[4] = {false, false, false, false};
  char nr[4][24] = {"", "", "", ""};
  bool newBump = false; int bumpDir = DIR_FRONT, bumpBackoffMs = 150;
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    if (!s.enabled) continue;
    Dir d = dirOf(s.yawDeg);
    bool hit = false;
    if (s.role == ROLE_BUMP && s.type == ST_BUMPER) hit = s.ok && s.value == 1;
    else if (s.role == ROLE_CLIFF && s.floorMm > 0) hit = !s.ok || s.value > s.floorMm * 3 / 2;
    else if (s.role == ROLE_OBSTACLE) hit = s.ok && s.value > 0 && s.value < s.stopMm;
    if (hit) {
      nb[d] = true;
      if (!nr[d][0]) snprintf(nr[d], sizeof(nr[d]), "%s", s.role == ROLE_BUMP ? "bumper" : s.role == ROLE_CLIFF ? "cliff" : typeName(s.type));
      if (s.role == ROLE_BUMP && !wasBlocked[d]) { newBump = true; bumpDir = d; bumpBackoffMs = s.backoffMs; }
    }
  }
  if (newBump && (bumpDir == DIR_FRONT || bumpDir == DIR_BACK)) {
    reflexEvents++;
    stopAll();
    Serial.printf("[reflex] bumper hit at the %s: %s\n", dirName((Dir)bumpDir), bumpBackoffMs > 0 ? "short back-off" : "stop");
    if (bumpBackoffMs > 0) {               // a short, firm nudge away from the hit
      float v = bumpDir == DIR_FRONT ? -0.85f : 0.85f;
      applyMotors(v, v);
      cmdSrc = SRC_REFLEX;
      backoffUntilMs = now + bumpBackoffMs;
    }
  }
  if (backoffUntilMs && now >= backoffUntilMs) { backoffUntilMs = 0; stopAll(); }
  for (int d = 0; d < 4; d++) {
    if (nb[d] && !wasBlocked[d] && !(newBump && d == bumpDir)) {
      reflexEvents++;
      Serial.printf("[reflex] %s blocked: %s\n", dirName((Dir)d), nr[d]);
      if (curLeft != 0 || curRight != 0) { // moving into it right now?
        float fwd = curLeft + curRight;
        if ((d == DIR_FRONT && fwd > 0.05f) || (d == DIR_BACK && fwd < -0.05f)) stopAll();
      }
    }
    blockDir[d] = nb[d]; strlcpy(blockReason[d], nr[d], sizeof(blockReason[d])); wasBlocked[d] = nb[d];
  }
}

// ---- sensor feed + capabilities (UDP 5603) ----
Sub sensorSubs[MAX_SUBS];
// The brain announces itself ("TBRN1:<port>") so this page can link to its full controls.
IPAddress brainIp; uint16_t brainPort = 0; uint32_t brainSeenMs = 0;
bool brainAlive() { return brainPort != 0 && millis() - brainSeenMs < 10000; }
String brainUrl() { return "http://" + brainIp.toString() + ":" + String(brainPort) + "/"; }

String sensorsJson() {
  JsonDocument doc;
  doc["t"] = millis();
  JsonArray arr = doc["sensors"].to<JsonArray>();
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    if (!s.enabled || s.type == ST_LIDAR || s.type == ST_CAMERA) continue;
    JsonObject o = arr.add<JsonObject>();
    o["id"] = s.id; o["v"] = s.value; o["ok"] = s.ok;
  }
  JsonObject blk = doc["block"].to<JsonObject>();
  for (int d = 0; d < 4; d++) if (blockDir[d]) blk[dirName((Dir)d)] = blockReason[d];
  doc["reflexEvents"] = reflexEvents;
  String out; serializeJson(doc, out); return out;
}

void handleSensorUdp() {
  int sz = udpSensor.parsePacket();
  while (sz > 0) {
    char msg[16] = {0};
    udpSensor.read(msg, min(sz, 15));
    if (strncmp(msg, "TBRN1:", 6) == 0) {
      int port = atoi(msg + 6);
      if (port > 0 && port < 65536) { brainIp = udpSensor.remoteIP(); brainPort = port; brainSeenMs = millis(); }
    } else if (strncmp(msg, "TSSUB", 5) == 0) {
      IPAddress ip = udpSensor.remoteIP(); uint16_t port = udpSensor.remotePort();
      Sub *slot = nullptr; bool fresh = false;
      for (auto &s : sensorSubs) if (s.used && s.ip == ip && s.port == port) slot = &s;
      if (!slot) for (auto &s : sensorSubs) if (!s.used) { slot = &s; fresh = true; break; }
      if (slot) {
        slot->ip = ip; slot->port = port; slot->seen = millis(); slot->used = true;
        if (fresh) {
          String body = "TCAP1" + hardwareJson(false);
          udpSensor.beginPacket(ip, port); udpSensor.write((const uint8_t *)body.c_str(), body.length()); udpSensor.endPacket();
        }
      }
    }
    sz = udpSensor.parsePacket();
  }
  for (auto &s : sensorSubs) if (s.used && millis() - s.seen > SUB_TIMEOUT_MS) s.used = false;
}

void sendSensorFeed() {
  bool any = false; for (auto &s : sensorSubs) if (s.used) any = true;
  if (!any) return;
  String body = "TSN1" + sensorsJson();
  for (auto &s : sensorSubs) {
    if (!s.used) continue;
    udpSensor.beginPacket(s.ip, s.port); udpSensor.write((const uint8_t *)body.c_str(), body.length()); udpSensor.endPacket();
  }
}

// ================= WEB =================
void handleRoot() { server.send(200, "text/html", MAIN_page); }

void handleMove() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing direction parameter"); return; }
  String d = server.arg("value");
  if (d == "forward")       { commandMotors(1, 1, SRC_WEB);   server.send(200, "text/plain", "Moving Forward"); }
  else if (d == "backward") { commandMotors(-1, -1, SRC_WEB); server.send(200, "text/plain", "Moving Backward"); }
  else if (d == "left")     { commandMotors(1, -1, SRC_WEB);  server.send(200, "text/plain", "Turning Left"); }
  else if (d == "right")    { commandMotors(-1, 1, SRC_WEB);  server.send(200, "text/plain", "Turning Right"); }
  else if (d == "stop")     { stopAll();                      server.send(200, "text/plain", "Stopped"); }
  else server.send(400, "text/plain", "Invalid direction");
}

void handleJoystick() {
  if (!server.hasArg("left") || !server.hasArg("right")) { server.send(400, "text/plain", "Missing joystick parameters"); return; }
  float l = server.arg("left").toFloat(), r = server.arg("right").toFloat();
  commandMotors(l, r, SRC_WEB);
  server.send(200, "text/plain", "Joystick: L=" + String(l) + " R=" + String(r));
}

void handleSpeed() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing speed parameter"); return; }
  switch (server.arg("value").toInt()) {
    case 1: hw.speed = 160; break; case 2: hw.speed = 220; break; case 3: hw.speed = 255; break;
    default: server.send(400, "text/plain", "Invalid speed level"); return;
  }
  prefs.begin("tankbot", false); prefs.putInt("speed", hw.speed); prefs.end();
  server.send(200, "text/plain", "Speed: " + String(hw.speed));
}

void handleTrim() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing trim parameter"); return; }
  hw.trim = constrain(server.arg("value").toInt(), -20, 20);
  prefs.begin("tankbot", false); prefs.putInt("trim", hw.trim); prefs.end();
  server.send(200, "text/plain", "Trim: " + String(hw.trim));
}
void handleGetTrim() { server.send(200, "text/plain", String(hw.trim)); }

void handleHardware() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    if (!loadSensorsJson(body)) { server.send(400, "application/json", "{\"error\":\"invalid hardware JSON\"}"); return; }
    saveHardware();
    stopAll();
    server.send(200, "application/json", "{\"ok\":true,\"restarting\":true}");
    delay(300);
    ESP.restart();
    return;
  }
  server.send(200, "application/json", hardwareJson(true));
}
void handleSensors() { server.send(200, "application/json", sensorsJson()); }

/// Where the brain's full controls are (empty object if no brain has been seen in 10 s).
void handleBrainApi() {
  server.send(200, "application/json", brainAlive() ? "{\"url\":\"" + brainUrl() + "\"}" : "{}");
}

/// tankbot.local/brain: the one address to bookmark; it always leads to the brain, wherever it is.
void handleBrainRedirect() {
  if (brainAlive()) {
    server.sendHeader("Location", brainUrl());
    server.send(302, "text/plain", "Brain at " + brainUrl());
    return;
  }
  server.send(200, "text/html",
    "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>No brain</title>"
    "<style>body{font-family:-apple-system,sans-serif;background:#0f1417;color:#e6eef0;padding:24px;max-width:520px;margin:auto;line-height:1.5}"
    "a{color:#64ffda}</style></head><body><h2>No brain found</h2>"
    "<p>The full controls (maps, tap-to-go, setup of the robot's sensors) run on a <b>brain</b>: the TankBot app on a phone or tablet on this Wi-Fi.</p>"
    "<p>Open the app, choose <b>Mounted brain</b> or <b>Brain in hand</b>, and this page will send you there automatically.</p>"
    "<p><a href='/'>Back to simple driving</a></p><script>setTimeout(()=>location.reload(),4000)</script></body></html>");
}

/// Cliff sensor pointed at the floor: remember today's floor reading as normal. /tof/calibrate?id=tof
void handleTofCalibrate() {
  String id = server.hasArg("id") ? server.arg("id") : "";
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    if ((id.length() == 0 && s.role == ROLE_CLIFF) || id == s.id) {
      if (!s.ok || s.value <= 0) { server.send(400, "text/plain", "No valid reading on " + String(s.id)); return; }
      s.floorMm = s.value; saveHardware();
      server.send(200, "text/plain", "Floor reading for " + String(s.id) + " set to " + String(s.floorMm) + " mm");
      return;
    }
  }
  server.send(404, "text/plain", "No cliff sensor with that id");
}

// Fallback page: works with no brain. The Bot page in the controller is the real editor.
String setupPage() {
  String h = "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>TankBot setup</title>"
    "<style>body{font-family:sans-serif;background:#101416;color:#e6eef0;padding:16px;max-width:720px;margin:auto}"
    "textarea{width:100%;height:280px;background:#1b2227;color:#e6eef0;border:1px solid #3a4a55;border-radius:6px;font-family:monospace;font-size:12px}"
    "button{background:#23303a;color:#e6eef0;border:1px solid #3a4a55;border-radius:8px;padding:8px 14px;font-size:14px}"
    "h3{color:#64ffda}td{padding:2px 10px 2px 0}</style></head><body><h2>" + String(hw.name) + " setup (firmware " + FW_VERSION + ")</h2>"
    "<p>With a brain connected, use the controller's <b>Bot</b> page instead. This page is the no-brain fallback. <a href='/' style='color:#64ffda'>Drive page</a></p>"
    "<h3>Attached sensors (live)</h3><table>";
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    h += "<tr><td>" + String(s.name) + "</td><td>" + typeName(s.type) + " / " + roleName(s.role) + " / " + dirName(dirOf(s.yawDeg)) + "</td><td>slot " + s.slot +
         "</td><td>" + (s.enabled ? (s.type == ST_BUMPER ? (s.value == 1 ? "PRESSED" : "ok") : (s.type == ST_LIDAR ? "streaming" : (s.ok ? String(s.value) + " mm" : "-"))) : "disabled") + "</td></tr>";
  }
  h += "</table><p>Blocked directions: " + blockSummary() + ". Reflex events: " + String(reflexEvents) + ".</p>";
  h += "<h3>Hardware description (JSON)</h3><form method='POST' action='/setup'><textarea name='hw'>" + hardwareJson(false) + "</textarea>"
       "<p><button type=submit>Save and restart</button> <a href='/tof/calibrate' style='margin-left:12px;color:#64ffda'>Calibrate cliff sensor floor now</a></p></form>"
       "<p style='color:#9fb3bb;font-size:13px'>Slots on the standard board: BUMP1 (13), BUMP2 (23), TOF (32/33), US1 (14/34), US2 (2/35), I2C (21/22), LIDAR, CUSTOM (set pinA/pinB), NONE. "
       "Roles: obstacle, cliff, bump, orientation, mapping. yawDeg: 0 = forward, 90 = left, 180 = back, -90 = right.</p></body></html>";
  return h;
}

void handleSetup() {
  if (server.method() == HTTP_POST && server.hasArg("hw")) {
    if (!loadSensorsJson(server.arg("hw"))) { server.send(400, "text/plain", "Invalid JSON"); return; }
    saveHardware(); stopAll();
    server.send(200, "text/html", "<html><body style='font-family:sans-serif;background:#101416;color:#e6eef0;padding:16px'>Saved. Restarting... <a href='/setup' style='color:#64ffda'>back</a></body></html>");
    delay(300); ESP.restart(); return;
  }
  server.send(200, "text/html", setupPage());
}

// ================= NETWORK / OTA =================
void setupNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[wifi] joining home network");
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    apMode = true; apSinceMs = millis();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.printf("[wifi] home network unavailable -> access point '%s'\n", AP_SSID);
  }
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("tanklidar", "udp", LIDAR_PORT);
    MDNS.addService("tankmotion", "udp", MOTION_PORT);
    MDNS.addService("tanksensors", "udp", SENSOR_PORT);
  }
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/speed", handleSpeed);
  server.on("/trim", handleTrim);
  server.on("/getTrim", handleGetTrim);
  server.on("/joystick", handleJoystick);
  server.on("/setup", handleSetup);
  server.on("/api/hardware", handleHardware);
  server.on("/api/capabilities", handleHardware);
  server.on("/api/sensors", handleSensors);
  server.on("/api/brain", handleBrainApi);
  server.on("/brain", handleBrainRedirect);
  server.on("/tof/calibrate", handleTofCalibrate);
  server.onNotFound(handleRoot);
  server.begin();
  udpLidar.begin(LIDAR_PORT);
  udpMotion.begin(MOTION_PORT);
  udpSensor.begin(SENSOR_PORT);
}

void setupOta() {
  ArduinoOTA.setHostname(HOSTNAME);
#ifdef OTA_PASS
  ArduinoOTA.setPassword(OTA_PASS);
#endif
  ArduinoOTA.onStart([]() { stopAll(); if (hw.lidar) Lidar.end(); Serial.println("[ota] update starting"); });
  ArduinoOTA.onEnd([]() { Serial.println("[ota] update done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[ota] error %u\n", e); });
  ArduinoOTA.begin();
}

void wifiRecovery() {
  if (!apMode) return;
  if (WiFi.softAPgetStationNum() > 0) { apSinceMs = millis(); return; }
  if (millis() - apSinceMs > 60000) { Serial.println("[wifi] AP idle for 60 s: restarting to retry home Wi-Fi"); delay(100); ESP.restart(); }
}

// ================= main =================
void setup() {
  loadHardware();
  setupMotors();
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== %s firmware v%s (%s) ===\n", hw.name, FW_VERSION, hw.drive);
  for (int i = 0; i < nSensors; i++) {
    Sensor &s = sensors[i];
    Serial.printf("[hw] %s: %s %s slot %s pins %d/%d yaw %.0f %s%s\n", s.id, typeName(s.type), roleName(s.role), s.slot, s.pinA, s.pinB, s.yawDeg,
                  s.floorTilt ? "floor-tilt " : "", s.enabled ? "" : "(disabled)");
  }
  setupSensors();
  setupNetwork();
  setupOta();
  if (hw.lidar) {
    Lidar.setRxBufferSize(8192);
    Lidar.begin(LIDAR_BAUD, SERIAL_8N1, hw.lidarRx, hw.lidarTx);
    delay(50);
    lidarStartScan();
  }
  lastByteMs = millis(); revStartMs = millis();
}

void loop() {
  if (hw.lidar) {
    while (Lidar.available()) { lidarByte(Lidar.read()); lastByteMs = millis(); }
    if (millis() - lastByteMs > 8000) { Serial.println("[lidar] no data for 8 s, restarting scan"); lidarStartScan(); lastByteMs = millis(); }
  }
  readSensors();
  reflexUpdate();
  ArduinoOTA.handle();
  handleMotionUdp();
  motionWatchdog();
  if (apMode) dnsServer.processNextRequest();
  server.handleClient();
  motionWatchdog();
  handleLidarUdp();
  handleSensorUdp();
  wifiRecovery();

  static uint32_t lastLidarStatus = 0, lastMotionStatus = 0, lastSensorFeed = 0, lastLog = 0;
  uint32_t now = millis();
  if (now - lastLidarStatus >= 1000) { lastLidarStatus = now; if (hw.lidar) sendLidarStatus(); }
  if (now - lastMotionStatus >= 250) { lastMotionStatus = now; sendMotionStatus(); }
  if (now - lastSensorFeed >= 50) { lastSensorFeed = now; sendSensorFeed(); }
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.printf("[stat] lidar %.1f Hz %d pts | motors L%.2f R%.2f src %s wd %lu | block %s | %s %s\n",
      revHz, lastRevPts, curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips, blockSummary().c_str(),
      apMode ? "AP" : "wifi", apMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
  }
  delay(1);
}
