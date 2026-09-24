// TankBot combined firmware: motion + lidar bridge + web control page.
// See docs/protocol.md. Three separate parts: MOTION, LIDAR, NETWORK.
//
// Safety: every motion command source is covered by a watchdog.
//   UDP (app) commands must repeat within 300 ms, web page commands within 500 ms,
//   otherwise the motors stop on their own.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <math.h>
#include "secrets.h"
#include "web_ui.h"

// ================= configuration =================
#define IN1 16  // Left motor direction
#define IN2 17
#define IN3 18  // Right motor direction
#define IN4 19
#define ENA 25  // Left motor PWM
#define ENB 26  // Right motor PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define SPEED_SLOW 160
#define SPEED_MEDIUM 220
#define SPEED_FAST 255

static const int LIDAR_RX = 4, LIDAR_TX = 27;      // lidar TX -> GPIO4, lidar RX -> GPIO27
static const uint32_t LIDAR_BAUD = 460800;
static const uint16_t LIDAR_PORT = 5601, MOTION_PORT = 5602;
static const char *HOSTNAME = "tankbot";          // http://tankbot.local
static const char *AP_SSID = "TankBot";           // fallback access point (original behaviour)
static const char *AP_PASS = "tankbot2025";
static const uint32_t UDP_CMD_TIMEOUT_MS = 300, WEB_CMD_TIMEOUT_MS = 500;

WebServer server(80);
DNSServer dnsServer;
Preferences preferences;
WiFiUDP udpLidar, udpMotion;
bool apMode = false;

// ================= MOTION =================
enum CmdSource { SRC_NONE, SRC_WEB, SRC_UDP };
int currentSpeed = SPEED_MEDIUM;
int motorTrim = 0;
float curLeft = 0, curRight = 0;
CmdSource cmdSrc = SRC_NONE;
uint32_t lastCmdMs = 0, watchdogTrips = 0, udpCommands = 0;

const char *srcName(CmdSource s) { return s == SRC_WEB ? "web" : s == SRC_UDP ? "udp" : "none"; }

void stopMotorsRaw() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0); ledcWrite(PWM_CHANNEL_B, 0);
  curLeft = curRight = 0;
}

void setupMotors() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  stopMotorsRaw();
}

// left/right in -1..1. Same mapping as the original joystick handler (speed level + trim).
void applyMotors(float left, float right) {
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  if (fabsf(left) < 0.05f && fabsf(right) < 0.05f) { stopMotorsRaw(); return; }
  int leftSpeed = abs((int)(left * currentSpeed));
  int rightSpeed = abs((int)(right * currentSpeed));
  if (motorTrim < 0) leftSpeed = constrain(leftSpeed + (int)(motorTrim * fabsf(left)), 0, 255);
  else if (motorTrim > 0) rightSpeed = constrain(rightSpeed - (int)(motorTrim * fabsf(right)), 0, 255);
  if (left >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  if (right >= 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else            { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  ledcWrite(PWM_CHANNEL_A, leftSpeed);
  ledcWrite(PWM_CHANNEL_B, rightSpeed);
  curLeft = left; curRight = right;
}

void stopAll() { stopMotorsRaw(); cmdSrc = SRC_NONE; }

void commandMotors(float left, float right, CmdSource src) {
  if (isnan(left) || isnan(right)) { stopAll(); return; }
  applyMotors(left, right);
  bool moving = (curLeft != 0 || curRight != 0);
  cmdSrc = moving ? src : SRC_NONE;
  lastCmdMs = millis();
}

void motionWatchdog() {
  if (cmdSrc == SRC_NONE) return;
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
        float l = fwd - turn, r = fwd + turn;          // same mixing as the web joystick
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

void sendMotionStatus() {
  if (motionPeerPort == 0 || millis() - motionPeerSeen > 3000) return;
  char body[200];
  int n = snprintf(body, sizeof(body),
    "TMH1{\"left\":%.2f,\"right\":%.2f,\"src\":\"%s\",\"wd_trips\":%lu,\"speed\":%d,\"trim\":%d,\"cmds\":%lu}",
    curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips, currentSpeed, motorTrim, (unsigned long)udpCommands);
  udpMotion.beginPacket(motionPeer, motionPeerPort);
  udpMotion.write((uint8_t *)body, n);
  udpMotion.endPacket();
}

// ---- web handlers (original page + routes) ----
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
    case 1: currentSpeed = SPEED_SLOW;   server.send(200, "text/plain", "Speed: Slow"); break;
    case 2: currentSpeed = SPEED_MEDIUM; server.send(200, "text/plain", "Speed: Medium"); break;
    case 3: currentSpeed = SPEED_FAST;   server.send(200, "text/plain", "Speed: Fast"); break;
    default: server.send(400, "text/plain", "Invalid speed level");
  }
}

void handleTrim() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing trim parameter"); return; }
  motorTrim = constrain(server.arg("value").toInt(), -20, 20);
  preferences.begin("tankbot", false); preferences.putInt("trim", motorTrim); preferences.end();
  server.send(200, "text/plain", "Trim: " + String(motorTrim));
}

void handleGetTrim() { server.send(200, "text/plain", String(motorTrim)); }

// ================= LIDAR =================
HardwareSerial Lidar(2);
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
    if (strncmp(msg, "TLSUB", 5) == 0) {
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

// ================= NETWORK =================
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
    apMode = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.printf("[wifi] home network unavailable -> access point '%s' at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("tanklidar", "udp", LIDAR_PORT);
    MDNS.addService("tankmotion", "udp", MOTION_PORT);
    Serial.printf("[mdns] %s.local advertised (http, tanklidar, tankmotion)\n", HOSTNAME);
  }
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/speed", handleSpeed);
  server.on("/trim", handleTrim);
  server.on("/getTrim", handleGetTrim);
  server.on("/joystick", handleJoystick);
  server.onNotFound(handleRoot);   // also serves captive-portal checks in AP mode
  server.begin();
  udpLidar.begin(LIDAR_PORT);
  udpMotion.begin(MOTION_PORT);
  Serial.printf("[net] web on :80, lidar UDP :%u, motion UDP :%u\n", LIDAR_PORT, MOTION_PORT);
}

// ================= main =================
void setup() {
  setupMotors();                      // motors stopped before anything else
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TankBot combined firmware ===");
  preferences.begin("tankbot", true);
  motorTrim = preferences.getInt("trim", 0);
  preferences.end();
  Serial.printf("[motion] trim %d, speed %d, watchdog udp %lu ms / web %lu ms\n", motorTrim, currentSpeed,
                (unsigned long)UDP_CMD_TIMEOUT_MS, (unsigned long)WEB_CMD_TIMEOUT_MS);
  setupNetwork();
  Lidar.setRxBufferSize(8192);
  Lidar.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  delay(50);
  lidarStartScan();
  lastByteMs = millis(); revStartMs = millis();
}

void loop() {
  // lidar bytes first: they arrive continuously
  while (Lidar.available()) { lidarByte(Lidar.read()); lastByteMs = millis(); }
  if (millis() - lastByteMs > 8000) { Serial.println("[lidar] no data for 8 s, restarting scan"); lidarStartScan(); lastByteMs = millis(); }

  handleMotionUdp();
  motionWatchdog();
  if (apMode) dnsServer.processNextRequest();
  server.handleClient();
  motionWatchdog();
  handleLidarUdp();

  static uint32_t lastLidarStatus = 0, lastMotionStatus = 0, lastLog = 0;
  uint32_t now = millis();
  if (now - lastLidarStatus >= 1000) { lastLidarStatus = now; sendLidarStatus(); }
  if (now - lastMotionStatus >= 250) { lastMotionStatus = now; sendMotionStatus(); }
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.printf("[stat] lidar %.1f Hz %d pts subs %d | motors L%.2f R%.2f src %s wd_trips %lu | %s %s\n",
      revHz, lastRevPts, activeSubs(), curLeft, curRight, srcName(cmdSrc), (unsigned long)watchdogTrips,
      apMode ? "AP" : "wifi", apMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
  }
  delay(1);
}
