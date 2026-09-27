// TankBot firmware v2: modular and configurable.
//
// Modules: CONFIG (flash-stored setup, /setup page), MOTION (+watchdog), LIDAR bridge,
// SENSORS (bumpers, TOFSense UART, ultrasonic), REFLEX (on-board safety), NETWORK/OTA, WEB.
//
// Everything about the robot's wiring lives in the config, defaulting to the standard wiring
// (docs/wiring.md). The robot works on its own from its web page; a brain adds mapping etc.
//
// UDP ports: 5601 lidar (docs/protocol.md), 5602 motion, 5603 sensors:
//   client sends "TSSUB" -> gets "TCAP1"+json once, then "TSN1"+json at 20 Hz while subscribed.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <math.h>
#include "esp_timer.h"
#include "secrets.h"
#include "web_ui.h"

static const char *FW_VERSION = "2.0";

// ================= CONFIG =================
struct Config {
  char name[24] = "TankBot";
  char drive[12] = "tank";              // tank | wheelchair | mecanum
  // pins (standard wiring)
  int in1 = 16, in2 = 17, in3 = 18, in4 = 19, ena = 25, enb = 26;
  int lidarRx = 4, lidarTx = 27;
  int tofRx = 32, tofTx = 33;
  int usTrig = 14, usEcho = 34;
  int bumpL = 13, bumpR = 23;
  // attached sensors
  bool lidar = true, tof = false, us = false, hasBumpL = false, hasBumpR = false;
  // reflex settings
  int usStopMm = 150;                   // ultrasonic: no forward motion closer than this
  int floorMm = 0;                      // TOF pointed at the floor: calibrated reading (0 = not calibrated)
  int speed = 220;                      // max PWM (speed level)
  int trim = 0;
} cfg;

Preferences prefs;

void loadConfig() {
  prefs.begin("tankbot", true);
  String n = prefs.getString("name", cfg.name); n.toCharArray(cfg.name, sizeof(cfg.name));
  String d = prefs.getString("drive", cfg.drive); d.toCharArray(cfg.drive, sizeof(cfg.drive));
  cfg.in1 = prefs.getInt("in1", cfg.in1); cfg.in2 = prefs.getInt("in2", cfg.in2);
  cfg.in3 = prefs.getInt("in3", cfg.in3); cfg.in4 = prefs.getInt("in4", cfg.in4);
  cfg.ena = prefs.getInt("ena", cfg.ena); cfg.enb = prefs.getInt("enb", cfg.enb);
  cfg.lidarRx = prefs.getInt("lidarRx", cfg.lidarRx); cfg.lidarTx = prefs.getInt("lidarTx", cfg.lidarTx);
  cfg.tofRx = prefs.getInt("tofRx", cfg.tofRx); cfg.tofTx = prefs.getInt("tofTx", cfg.tofTx);
  cfg.usTrig = prefs.getInt("usTrig", cfg.usTrig); cfg.usEcho = prefs.getInt("usEcho", cfg.usEcho);
  cfg.bumpL = prefs.getInt("bumpL", cfg.bumpL); cfg.bumpR = prefs.getInt("bumpR", cfg.bumpR);
  cfg.lidar = prefs.getBool("sLidar", cfg.lidar); cfg.tof = prefs.getBool("sTof", cfg.tof);
  cfg.us = prefs.getBool("sUs", cfg.us);
  cfg.hasBumpL = prefs.getBool("sBumpL", cfg.hasBumpL); cfg.hasBumpR = prefs.getBool("sBumpR", cfg.hasBumpR);
  cfg.usStopMm = prefs.getInt("usStop", cfg.usStopMm); cfg.floorMm = prefs.getInt("floorMm", cfg.floorMm);
  cfg.speed = prefs.getInt("speed", cfg.speed); cfg.trim = prefs.getInt("trim", cfg.trim);
  prefs.end();
}

void saveConfig() {
  prefs.begin("tankbot", false);
  prefs.putString("name", cfg.name); prefs.putString("drive", cfg.drive);
  prefs.putInt("in1", cfg.in1); prefs.putInt("in2", cfg.in2); prefs.putInt("in3", cfg.in3); prefs.putInt("in4", cfg.in4);
  prefs.putInt("ena", cfg.ena); prefs.putInt("enb", cfg.enb);
  prefs.putInt("lidarRx", cfg.lidarRx); prefs.putInt("lidarTx", cfg.lidarTx);
  prefs.putInt("tofRx", cfg.tofRx); prefs.putInt("tofTx", cfg.tofTx);
  prefs.putInt("usTrig", cfg.usTrig); prefs.putInt("usEcho", cfg.usEcho);
  prefs.putInt("bumpL", cfg.bumpL); prefs.putInt("bumpR", cfg.bumpR);
  prefs.putBool("sLidar", cfg.lidar); prefs.putBool("sTof", cfg.tof); prefs.putBool("sUs", cfg.us);
  prefs.putBool("sBumpL", cfg.hasBumpL); prefs.putBool("sBumpR", cfg.hasBumpR);
  prefs.putInt("usStop", cfg.usStopMm); prefs.putInt("floorMm", cfg.floorMm);
  prefs.putInt("speed", cfg.speed); prefs.putInt("trim", cfg.trim);
  prefs.end();
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

// ================= REFLEX state (filled by SENSORS, used by MOTION) =================
enum Block { BLOCK_NONE, BLOCK_BUMPER, BLOCK_CLIFF, BLOCK_ULTRASONIC };
Block blockForward = BLOCK_NONE;
uint32_t backoffUntilMs = 0;          // bumper reflex: reversing until this time
uint32_t reflexEvents = 0;

// ================= MOTION =================
enum CmdSource { SRC_NONE, SRC_WEB, SRC_UDP, SRC_REFLEX };
float curLeft = 0, curRight = 0;
CmdSource cmdSrc = SRC_NONE;
uint32_t lastCmdMs = 0, watchdogTrips = 0, udpCommands = 0, blockedCommands = 0;

const char *srcName(CmdSource s) { return s == SRC_WEB ? "web" : s == SRC_UDP ? "udp" : s == SRC_REFLEX ? "reflex" : "none"; }

void stopMotorsRaw() {
  digitalWrite(cfg.in1, LOW); digitalWrite(cfg.in2, LOW); digitalWrite(cfg.in3, LOW); digitalWrite(cfg.in4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0); ledcWrite(PWM_CHANNEL_B, 0);
  curLeft = curRight = 0;
}

void setupMotors() {
  pinMode(cfg.in1, OUTPUT); pinMode(cfg.in2, OUTPUT); pinMode(cfg.in3, OUTPUT); pinMode(cfg.in4, OUTPUT);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(cfg.ena, PWM_CHANNEL_A);
  ledcAttachPin(cfg.enb, PWM_CHANNEL_B);
  stopMotorsRaw();
}

void applyMotors(float left, float right) {
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  if (fabsf(left) < 0.05f && fabsf(right) < 0.05f) { stopMotorsRaw(); return; }
  int leftSpeed = abs((int)(left * cfg.speed));
  int rightSpeed = abs((int)(right * cfg.speed));
  if (cfg.trim < 0) leftSpeed = constrain(leftSpeed + (int)(cfg.trim * fabsf(left)), 0, 255);
  else if (cfg.trim > 0) rightSpeed = constrain(rightSpeed - (int)(cfg.trim * fabsf(right)), 0, 255);
  if (left >= 0) { digitalWrite(cfg.in1, HIGH); digitalWrite(cfg.in2, LOW); }
  else           { digitalWrite(cfg.in1, LOW);  digitalWrite(cfg.in2, HIGH); }
  if (right >= 0) { digitalWrite(cfg.in3, LOW);  digitalWrite(cfg.in4, HIGH); }
  else            { digitalWrite(cfg.in3, HIGH); digitalWrite(cfg.in4, LOW); }
  ledcWrite(PWM_CHANNEL_A, leftSpeed);
  ledcWrite(PWM_CHANNEL_B, rightSpeed);
  curLeft = left; curRight = right;
}

void stopAll() { stopMotorsRaw(); cmdSrc = SRC_NONE; }

/// Every command goes through here; reflexes veto forward motion.
void commandMotors(float left, float right, CmdSource src) {
  if (isnan(left) || isnan(right)) { stopAll(); return; }
  if (src != SRC_REFLEX && millis() < backoffUntilMs) return;      // bumper back-off in progress
  if (src != SRC_REFLEX && blockForward != BLOCK_NONE && (left + right) > 0.05f) {
    // net-forward motion is vetoed; turning in place and reversing are still allowed
    blockedCommands++;
    if (curLeft + curRight > 0.05f) stopAll();
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

// ---- UDP motion API (port 5602) ----
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

const char *blockName() {
  return blockForward == BLOCK_BUMPER ? "bumper" : blockForward == BLOCK_CLIFF ? "cliff" : blockForward == BLOCK_ULTRASONIC ? "ultrasonic" : "none";
}

void sendMotionStatus() {
  if (motionPeerPort == 0 || millis() - motionPeerSeen > 3000) return;
  char body[240];
  int n = snprintf(body, sizeof(body),
    "TMH1{\"left\":%.2f,\"right\":%.2f,\"src\":\"%s\",\"wd_trips\":%lu,\"speed\":%d,\"trim\":%d,\"cmds\":%lu,\"block\":\"%s\",\"blocked_cmds\":%lu}",
    curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips, cfg.speed, cfg.trim, (unsigned long)udpCommands,
    blockName(), (unsigned long)blockedCommands);
  udpMotion.beginPacket(motionPeer, motionPeerPort);
  udpMotion.write((uint8_t *)body, n);
  udpMotion.endPacket();
}

// ================= LIDAR =================
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
      if (!slot) for (auto &s : subs) if (!s.used) { slot = &s; Serial.printf("[lidar] new subscriber %s:%u\n", ip.toString().c_str(), port); break; }
      if (slot) { slot->ip = ip; slot->port = port; slot->seen = millis(); slot->used = true; }
    }
    sz = udpLidar.parsePacket();
  }
  for (auto &s : subs) if (s.used && millis() - s.seen > SUB_TIMEOUT_MS) {
    Serial.printf("[lidar] subscriber %s timed out\n", s.ip.toString().c_str()); s.used = false;
  }
}

void sendLidarStatus() {
  char body[256];
  int n = snprintf(body, sizeof(body),
    "TLH1{\"uptime_ms\":%lu,\"revs\":%lu,\"hz\":%.2f,\"pts\":%d,\"sync_errs\":%lu,\"rssi\":%d,\"subs\":%d,\"sent\":%lu,\"send_errs\":%lu}",
    (unsigned long)millis(), (unsigned long)revs, revHz, lastRevPts, (unsigned long)badSync, apMode ? 0 : WiFi.RSSI(),
    activeSubs(), (unsigned long)packetsSent, (unsigned long)sendErrors);
  lidarSendToSubs((uint8_t *)body, n);
}

// ================= SENSORS =================
// Bumpers: switch wired COM + NC to ground, internal pull-up. Closed (LOW) = untouched.
// Open (HIGH) = pressed, and a broken wire also reads pressed: fail-safe.
bool bumpLPressed = false, bumpRPressed = false;

// TOFSense-F2 Mini on UART1, NLink frame 0 (16 bytes) streamed at 50 Hz.
HardwareSerial Tof(1);
int tofMm = -1; bool tofValid = false; uint32_t tofLastMs = 0, tofFrames = 0;
uint8_t tofBuf[16]; int tofN = 0;

void tofByte(uint8_t b) {
  if (tofN == 0 && b != 0x57) return;
  if (tofN == 1 && b != 0x00) { tofN = (b == 0x57) ? 1 : 0; return; }
  tofBuf[tofN++] = b;
  if (tofN < 16) return;
  tofN = 0;
  uint8_t sum = 0;
  for (int i = 0; i < 15; i++) sum += tofBuf[i];
  if (sum != tofBuf[15]) return;
  int32_t dis = tofBuf[9] | (tofBuf[10] << 8) | (tofBuf[11] << 16);
  if (dis & 0x800000) dis |= 0xFF000000;      // signed 24-bit
  uint8_t status = tofBuf[12];
  uint16_t strength = tofBuf[13] | (tofBuf[14] << 8);
  tofFrames++;
  tofLastMs = millis();
  tofValid = (status == 0 && strength > 0 && dis > 0);
  tofMm = tofValid ? dis : -1;
}

// HC-SR04(P): trigger every 60 ms, time the echo with a pin interrupt (nothing blocks).
volatile uint32_t usEchoStart = 0, usEchoUs = 0; volatile bool usEchoDone = false;
int usMm = -1; uint32_t usLastTrigMs = 0, usLastMs = 0;

void IRAM_ATTR usEchoIsr() {
  if (digitalRead(cfg.usEcho)) usEchoStart = micros();
  else { usEchoUs = micros() - usEchoStart; usEchoDone = true; }
}

void usTick() {
  uint32_t now = millis();
  if (usEchoDone) {
    usEchoDone = false;
    uint32_t us = usEchoUs;
    usMm = (us > 100 && us < 30000) ? (int)(us * 0.1715f) : -1;   // 343 m/s, there and back
    usLastMs = now;
  }
  if (now - usLastTrigMs >= 60) {
    usLastTrigMs = now;
    if (now - usLastMs > 200) usMm = -1;   // no echo lately
    digitalWrite(cfg.usTrig, LOW); delayMicroseconds(2);
    digitalWrite(cfg.usTrig, HIGH); delayMicroseconds(10);
    digitalWrite(cfg.usTrig, LOW);
  }
}

void setupSensors() {
  if (cfg.hasBumpL) pinMode(cfg.bumpL, INPUT_PULLUP);
  if (cfg.hasBumpR) pinMode(cfg.bumpR, INPUT_PULLUP);
  if (cfg.tof) { Tof.setRxBufferSize(1024); Tof.begin(921600, SERIAL_8N1, cfg.tofRx, cfg.tofTx); }
  if (cfg.us) {
    pinMode(cfg.usTrig, OUTPUT); digitalWrite(cfg.usTrig, LOW);
    pinMode(cfg.usEcho, INPUT);
    attachInterrupt(digitalPinToInterrupt(cfg.usEcho), usEchoIsr, CHANGE);
  }
}

void readSensors() {
  if (cfg.hasBumpL) bumpLPressed = digitalRead(cfg.bumpL) == HIGH;
  if (cfg.hasBumpR) bumpRPressed = digitalRead(cfg.bumpR) == HIGH;
  if (cfg.tof) {
    while (Tof.available()) tofByte(Tof.read());
    if (millis() - tofLastMs > 300) { tofValid = false; tofMm = -1; }
  }
  if (cfg.us) usTick();
}

// ================= REFLEX =================
// Decides blockForward from the sensors and runs the bumper back-off. Runs with no brain attached.
Block lastBlock = BLOCK_NONE;

void reflexUpdate() {
  uint32_t now = millis();
  Block b = BLOCK_NONE;
  bool bump = (cfg.hasBumpL && bumpLPressed) || (cfg.hasBumpR && bumpRPressed);
  if (bump) {
    b = BLOCK_BUMPER;
    if (lastBlock != BLOCK_BUMPER) {              // fresh hit: stop, then back off briefly
      reflexEvents++;
      Serial.println("[reflex] bumper hit: backing off");
      applyMotors(-0.9f, -0.9f);
      cmdSrc = SRC_REFLEX;
      backoffUntilMs = now + 350;
    }
  } else if (cfg.tof && cfg.floorMm > 0 && (!tofValid || tofMm > cfg.floorMm * 3 / 2)) {
    b = BLOCK_CLIFF;                              // floor is not where it should be: a drop
  } else if (cfg.us && usMm > 0 && usMm < cfg.usStopMm) {
    b = BLOCK_ULTRASONIC;
  }
  if (backoffUntilMs && now >= backoffUntilMs) { backoffUntilMs = 0; stopAll(); }
  if (b != BLOCK_NONE && b != lastBlock && b != BLOCK_BUMPER) {
    reflexEvents++;
    Serial.printf("[reflex] forward blocked: %s\n", b == BLOCK_CLIFF ? "cliff" : "ultrasonic");
    if (curLeft + curRight > 0.05f) stopAll();
  }
  blockForward = b;
  lastBlock = b;
}

// ---- sensor feed + capabilities (UDP 5603) ----
Sub sensorSubs[MAX_SUBS];

void capabilitiesJson(char *out, size_t n) {
  snprintf(out, n,
    "{\"name\":\"%s\",\"fw\":\"%s\",\"drive\":\"%s\",\"sensors\":{\"lidar\":%s,\"tof\":%s,\"ultrasonic\":%s,\"bumperL\":%s,\"bumperR\":%s},"
    "\"reflex\":{\"usStopMm\":%d,\"floorMm\":%d},\"pins\":{\"in1\":%d,\"in2\":%d,\"in3\":%d,\"in4\":%d,\"ena\":%d,\"enb\":%d,"
    "\"lidarRx\":%d,\"lidarTx\":%d,\"tofRx\":%d,\"tofTx\":%d,\"usTrig\":%d,\"usEcho\":%d,\"bumpL\":%d,\"bumpR\":%d}}",
    cfg.name, FW_VERSION, cfg.drive, cfg.lidar ? "true" : "false", cfg.tof ? "true" : "false", cfg.us ? "true" : "false",
    cfg.hasBumpL ? "true" : "false", cfg.hasBumpR ? "true" : "false", cfg.usStopMm, cfg.floorMm,
    cfg.in1, cfg.in2, cfg.in3, cfg.in4, cfg.ena, cfg.enb, cfg.lidarRx, cfg.lidarTx, cfg.tofRx, cfg.tofTx,
    cfg.usTrig, cfg.usEcho, cfg.bumpL, cfg.bumpR);
}

void sensorsJson(char *out, size_t n) {
  snprintf(out, n,
    "{\"t\":%lu,\"bumpL\":%d,\"bumpR\":%d,\"tofMm\":%d,\"tofOk\":%s,\"usMm\":%d,\"block\":\"%s\",\"floorMm\":%d,\"reflexEvents\":%lu}",
    (unsigned long)millis(), cfg.hasBumpL ? (bumpLPressed ? 1 : 0) : -1, cfg.hasBumpR ? (bumpRPressed ? 1 : 0) : -1,
    tofMm, tofValid ? "true" : "false", usMm, blockName(), cfg.floorMm, (unsigned long)reflexEvents);
}

void handleSensorUdp() {
  int sz = udpSensor.parsePacket();
  while (sz > 0) {
    char msg[16] = {0};
    udpSensor.read(msg, min(sz, 15));
    if (strncmp(msg, "TSSUB", 5) == 0) {
      IPAddress ip = udpSensor.remoteIP(); uint16_t port = udpSensor.remotePort();
      Sub *slot = nullptr; bool fresh = false;
      for (auto &s : sensorSubs) if (s.used && s.ip == ip && s.port == port) slot = &s;
      if (!slot) for (auto &s : sensorSubs) if (!s.used) { slot = &s; fresh = true; break; }
      if (slot) {
        slot->ip = ip; slot->port = port; slot->seen = millis(); slot->used = true;
        if (fresh) {                                    // announce what this robot has
          char body[600]; memcpy(body, "TCAP1", 5); capabilitiesJson(body + 5, sizeof(body) - 5);
          udpSensor.beginPacket(ip, port); udpSensor.write((uint8_t *)body, strlen(body)); udpSensor.endPacket();
        }
      }
    }
    sz = udpSensor.parsePacket();
  }
  for (auto &s : sensorSubs) if (s.used && millis() - s.seen > SUB_TIMEOUT_MS) s.used = false;
}

void sendSensorFeed() {
  char body[300]; memcpy(body, "TSN1", 4); sensorsJson(body + 4, sizeof(body) - 4);
  size_t len = strlen(body);
  for (auto &s : sensorSubs) {
    if (!s.used) continue;
    udpSensor.beginPacket(s.ip, s.port); udpSensor.write((uint8_t *)body, len); udpSensor.endPacket();
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
    case 1: cfg.speed = 160; break;
    case 2: cfg.speed = 220; break;
    case 3: cfg.speed = 255; break;
    default: server.send(400, "text/plain", "Invalid speed level"); return;
  }
  saveConfig();
  server.send(200, "text/plain", "Speed: " + String(cfg.speed));
}

void handleTrim() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing trim parameter"); return; }
  cfg.trim = constrain(server.arg("value").toInt(), -20, 20);
  saveConfig();
  server.send(200, "text/plain", "Trim: " + String(cfg.trim));
}

void handleGetTrim() { server.send(200, "text/plain", String(cfg.trim)); }

void handleCaps() { char b[600]; capabilitiesJson(b, sizeof(b)); server.send(200, "application/json", b); }
void handleSensors() { char b[300]; sensorsJson(b, sizeof(b)); server.send(200, "application/json", b); }

/// TOF pointed at the floor: remember today's floor reading as "normal".
void handleTofCalibrate() {
  if (!cfg.tof || !tofValid) { server.send(400, "text/plain", "No valid TOF reading"); return; }
  cfg.floorMm = tofMm; saveConfig();
  server.send(200, "text/plain", "Floor distance set to " + String(cfg.floorMm) + " mm");
}

// Setup page: plain form, works from any browser, no brain needed.
String setupPage() {
  String h = "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>TankBot setup</title>"
    "<style>body{font-family:sans-serif;background:#101416;color:#e6eef0;padding:16px;max-width:560px;margin:auto}"
    "label{display:block;margin:8px 0}input[type=number]{width:70px}input,select{background:#1b2227;color:#e6eef0;border:1px solid #3a4a55;border-radius:6px;padding:4px}"
    "h3{margin:18px 0 6px;color:#64ffda}button{background:#23303a;color:#e6eef0;border:1px solid #3a4a55;border-radius:8px;padding:8px 14px;font-size:14px}"
    ".g{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}</style></head><body>"
    "<h2>TankBot setup</h2><p>Firmware " + String(FW_VERSION) + ". Saving restarts the robot. <a href='/' style='color:#64ffda'>Drive page</a></p>"
    "<form method='POST' action='/setup'>";
  h += "<label>Name <input name=name value='" + String(cfg.name) + "'></label>";
  h += "<label>Drive type <select name=drive>";
  for (const char *d : {"tank", "wheelchair", "mecanum"}) h += String("<option") + (strcmp(cfg.drive, d) == 0 ? " selected" : "") + ">" + d + "</option>";
  h += "</select></label>";
  h += "<h3>Attached sensors</h3>";
  auto cb = [&](const char *n, const char *label, bool v) { h += String("<label><input type=checkbox name=") + n + (v ? " checked" : "") + "> " + label + "</label>"; };
  cb("lidar", "RPLidar", cfg.lidar); cb("tof", "TOFSense ToF (pointed at the floor: cliff sensor)", cfg.tof);
  cb("us", "Ultrasonic HC-SR04(P) (low obstacles ahead)", cfg.us); cb("bumpL", "Left bumper", cfg.hasBumpL); cb("bumpR", "Right bumper", cfg.hasBumpR);
  h += "<h3>Reflexes</h3><label>Ultrasonic stop distance <input type=number name=usStop value=" + String(cfg.usStopMm) + "> mm</label>";
  h += "<label>ToF floor reading <input type=number name=floorMm value=" + String(cfg.floorMm) + "> mm (0 = not calibrated; current ToF: " + String(tofMm) + " mm)</label>";
  h += "<h3>Pins (standard wiring by default)</h3><div class=g>";
  auto pin = [&](const char *n, const char *label, int v) { h += String("<label>") + label + " <input type=number name=" + n + " value=" + v + "></label>"; };
  pin("in1", "IN1", cfg.in1); pin("in2", "IN2", cfg.in2); pin("in3", "IN3", cfg.in3); pin("in4", "IN4", cfg.in4);
  pin("ena", "ENA", cfg.ena); pin("enb", "ENB", cfg.enb); pin("lidarRx", "Lidar RX", cfg.lidarRx); pin("lidarTx", "Lidar TX", cfg.lidarTx);
  pin("tofRx", "ToF RX", cfg.tofRx); pin("tofTx", "ToF TX", cfg.tofTx); pin("usTrig", "US TRIG", cfg.usTrig); pin("usEcho", "US ECHO", cfg.usEcho);
  pin("bumpL", "Bumper L", cfg.bumpL); pin("bumpR", "Bumper R", cfg.bumpR);
  h += "</div><p><button type=submit>Save and restart</button> <a href='/tof/calibrate' style='margin-left:12px;color:#64ffda'>Calibrate ToF floor now</a></p></form>";
  h += "<p style='color:#9fb3bb;font-size:13px'>Live: bumper L " + String(cfg.hasBumpL ? (bumpLPressed ? "PRESSED" : "ok") : "-") +
       ", bumper R " + String(cfg.hasBumpR ? (bumpRPressed ? "PRESSED" : "ok") : "-") + ", ToF " + String(tofMm) + " mm, ultrasonic " + String(usMm) +
       " mm, forward block: " + blockName() + ". <a href='/api/sensors' style='color:#64ffda'>JSON</a></p></body></html>";
  return h;
}

int argPin(const char *n, int cur) { return server.hasArg(n) ? constrain(server.arg(n).toInt(), 0, 39) : cur; }

void handleSetup() {
  if (server.method() == HTTP_POST) {
    if (server.hasArg("name")) server.arg("name").substring(0, 23).toCharArray(cfg.name, sizeof(cfg.name));
    if (server.hasArg("drive")) server.arg("drive").substring(0, 11).toCharArray(cfg.drive, sizeof(cfg.drive));
    cfg.lidar = server.hasArg("lidar"); cfg.tof = server.hasArg("tof"); cfg.us = server.hasArg("us");
    cfg.hasBumpL = server.hasArg("bumpL"); cfg.hasBumpR = server.hasArg("bumpR");
    if (server.hasArg("usStop")) cfg.usStopMm = constrain(server.arg("usStop").toInt(), 30, 2000);
    if (server.hasArg("floorMm")) cfg.floorMm = constrain(server.arg("floorMm").toInt(), 0, 5000);
    cfg.in1 = argPin("in1", cfg.in1); cfg.in2 = argPin("in2", cfg.in2); cfg.in3 = argPin("in3", cfg.in3); cfg.in4 = argPin("in4", cfg.in4);
    cfg.ena = argPin("ena", cfg.ena); cfg.enb = argPin("enb", cfg.enb);
    cfg.lidarRx = argPin("lidarRx", cfg.lidarRx); cfg.lidarTx = argPin("lidarTx", cfg.lidarTx);
    cfg.tofRx = argPin("tofRx", cfg.tofRx); cfg.tofTx = argPin("tofTx", cfg.tofTx);
    cfg.usTrig = argPin("usTrig", cfg.usTrig); cfg.usEcho = argPin("usEcho", cfg.usEcho);
    cfg.bumpL = argPin("bumpL", cfg.bumpL); cfg.bumpR = argPin("bumpR", cfg.bumpR);
    saveConfig();
    stopAll();
    server.send(200, "text/html", "<html><body style='font-family:sans-serif;background:#101416;color:#e6eef0;padding:16px'>Saved. Restarting... <a href='/setup' style='color:#64ffda'>back to setup</a> in a few seconds.</body></html>");
    delay(300);
    ESP.restart();
    return;
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
    Serial.printf("[wifi] home network unavailable -> access point '%s' at %s (will retry home Wi-Fi when idle)\n", AP_SSID, WiFi.softAPIP().toString().c_str());
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
  server.on("/api/capabilities", handleCaps);
  server.on("/api/sensors", handleSensors);
  server.on("/tof/calibrate", handleTofCalibrate);
  server.onNotFound(handleRoot);
  server.begin();
  udpLidar.begin(LIDAR_PORT);
  udpMotion.begin(MOTION_PORT);
  udpSensor.begin(SENSOR_PORT);
  Serial.printf("[net] web :80 (/setup), lidar UDP :%u, motion UDP :%u, sensors UDP :%u\n", LIDAR_PORT, MOTION_PORT, SENSOR_PORT);
}

void setupOta() {
  ArduinoOTA.setHostname(HOSTNAME);
#ifdef OTA_PASS
  ArduinoOTA.setPassword(OTA_PASS);
#endif
  ArduinoOTA.onStart([]() { stopAll(); if (cfg.lidar) Lidar.end(); Serial.println("[ota] update starting, motors stopped"); });
  ArduinoOTA.onEnd([]() { Serial.println("[ota] update done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[ota] error %u\n", e); });
  ArduinoOTA.begin();
}

/// In fallback AP mode with nobody connected for a minute: restart to retry the home network.
void wifiRecovery() {
  if (!apMode) return;
  if (WiFi.softAPgetStationNum() > 0) { apSinceMs = millis(); return; }
  if (millis() - apSinceMs > 60000) { Serial.println("[wifi] AP idle for 60 s: restarting to retry home Wi-Fi"); delay(100); ESP.restart(); }
}

// ================= main =================
void setup() {
  loadConfig();
  setupMotors();                      // motors stopped before anything else
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== %s firmware v%s (%s) ===\n", cfg.name, FW_VERSION, cfg.drive);
  Serial.printf("[cfg] speed %d trim %d | lidar %d tof %d us %d bumpers %d/%d | usStop %d floor %d\n",
                cfg.speed, cfg.trim, cfg.lidar, cfg.tof, cfg.us, cfg.hasBumpL, cfg.hasBumpR, cfg.usStopMm, cfg.floorMm);
  setupSensors();
  setupNetwork();
  setupOta();
  if (cfg.lidar) {
    Lidar.setRxBufferSize(8192);
    Lidar.begin(LIDAR_BAUD, SERIAL_8N1, cfg.lidarRx, cfg.lidarTx);
    delay(50);
    lidarStartScan();
  }
  lastByteMs = millis(); revStartMs = millis();
}

void loop() {
  if (cfg.lidar) {
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
  if (now - lastLidarStatus >= 1000) { lastLidarStatus = now; if (cfg.lidar) sendLidarStatus(); }
  if (now - lastMotionStatus >= 250) { lastMotionStatus = now; sendMotionStatus(); }
  if (now - lastSensorFeed >= 50) { lastSensorFeed = now; sendSensorFeed(); }
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.printf("[stat] lidar %.1f Hz %d pts subs %d | motors L%.2f R%.2f src %s wd %lu | block %s | tof %d us %d bump %d%d | %s %s\n",
      revHz, lastRevPts, activeSubs(), curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips, blockName(),
      tofMm, usMm, bumpLPressed, bumpRPressed, apMode ? "AP" : "wifi",
      apMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
  }
  delay(1);
}
