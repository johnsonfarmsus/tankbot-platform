// TankBot lidar bridge v1
// - Motors held OFF
// - RPLidar C1 on UART2: RX=GPIO4 (lidar TX), TX=GPIO27 (lidar RX), 460800 baud
// - Joins Wi-Fi (credentials in secrets.h), advertises mDNS tanklidar.local, _tanklidar._udp
// - Clients send "TLSUB" to UDP 5601 at least every 5 s; each full rotation is streamed back to them
//
// Scan packet (little-endian): 'TLS1' | u32 rev | u32 t_ms | u8 chunk | u8 chunks | u16 n | n x {u16 angle_q6, u16 dist_q2, u8 quality}
//   angle deg = angle_q6/64, distance mm = dist_q2/4. Only valid (non-zero) points are sent.
// Status packet: 'TLH1' followed by a JSON text body, once per second.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <math.h>
#include "secrets.h"

static const int MOTOR_PINS[] = {16, 17, 18, 19, 25, 26};
static const int LIDAR_RX = 4, LIDAR_TX = 27;
static const uint32_t LIDAR_BAUD = 460800;
static const uint16_t UDP_PORT = 5601;
static const int MAX_PTS = 1400;
static const int PTS_PER_CHUNK = 250;
static const int MAX_SUBS = 3;
static const uint32_t SUB_TIMEOUT_MS = 5000;

HardwareSerial Lidar(2);
WiFiUDP udp;

struct Pt { uint16_t a, d; uint8_t q; };
Pt revBuf[MAX_PTS]; int revN = 0; uint32_t revStartMs = 0;
uint32_t revs = 0, badSync = 0, lastByteMs = 0, lastRevStart = 0, packetsSent = 0, sendErrors = 0;
float revHz = 0; int lastRevPts = 0;
uint8_t node[5]; int ni = 0;

struct Sub { IPAddress ip; uint16_t port; uint32_t seen; bool used; };
Sub subs[MAX_SUBS];

void motorsOff() { for (int p : MOTOR_PINS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); } }
void sendCmd(uint8_t c) { uint8_t b[2] = {0xA5, c}; Lidar.write(b, 2); Lidar.flush(); }
void drain(uint32_t ms) { uint32_t t = millis(); while (millis() - t < ms) { while (Lidar.available()) Lidar.read(); delay(1); } }

bool readDescriptor(uint32_t timeoutMs, uint32_t &len, uint8_t &type) {
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

bool startScan() {
  sendCmd(0x25); delay(20); drain(100);
  sendCmd(0x20);
  uint32_t len; uint8_t type;
  bool ok = readDescriptor(2000, len, type) && len == 5 && type == 0x81;
  Serial.printf("[lidar] scan start %s\n", ok ? "OK" : "FAILED");
  ni = 0; revN = 0;
  return ok;
}

int activeSubs() { int n = 0; for (auto &s : subs) if (s.used) n++; return n; }

void sendToSubs(const uint8_t *buf, size_t len) {
  for (auto &s : subs) {
    if (!s.used) continue;
    udp.beginPacket(s.ip, s.port);
    udp.write(buf, len);
    if (udp.endPacket()) packetsSent++; else sendErrors++;
  }
}

void sendRev() {
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
    sendToSubs(pkt, o);
  }
}

void finishRev() {
  uint32_t now = millis();
  if (lastRevStart) { float hz = 1000.0f / (float)(now - lastRevStart); revHz = revHz == 0 ? hz : revHz * 0.8f + hz * 0.2f; }
  lastRevStart = now;
  revs++; lastRevPts = revN;
  sendRev();
  revN = 0; revStartMs = now;
}

void handleByte(uint8_t b) {
  node[ni++] = b;
  if (ni == 1) { if ((b & 1) == ((b >> 1) & 1)) { ni = 0; badSync++; } return; }
  if (ni == 2) { if (!(b & 1)) { ni = 0; badSync++; } return; }
  if (ni < 5) return;
  ni = 0;
  bool start = node[0] & 1;
  if (start && revN > 0) finishRev();
  uint16_t dq2 = node[3] | (node[4] << 8);
  if (dq2 == 0 || revN >= MAX_PTS) return;
  revBuf[revN].a = (node[1] >> 1) | (node[2] << 7);
  revBuf[revN].d = dq2;
  revBuf[revN].q = node[0] >> 2;
  revN++;
}

void handleUdp() {
  int sz = udp.parsePacket();
  while (sz > 0) {
    char msg[16] = {0};
    udp.read(msg, min(sz, 15));
    if (strncmp(msg, "TLSUB", 5) == 0) {
      IPAddress ip = udp.remoteIP(); uint16_t port = udp.remotePort();
      Sub *slot = nullptr;
      for (auto &s : subs) if (s.used && s.ip == ip && s.port == port) slot = &s;
      if (!slot) for (auto &s : subs) if (!s.used) { slot = &s; Serial.printf("[udp] new subscriber %s:%u\n", ip.toString().c_str(), port); break; }
      if (slot) { slot->ip = ip; slot->port = port; slot->seen = millis(); slot->used = true; }
    }
    sz = udp.parsePacket();
  }
  for (auto &s : subs) if (s.used && millis() - s.seen > SUB_TIMEOUT_MS) {
    Serial.printf("[udp] subscriber %s timed out\n", s.ip.toString().c_str()); s.used = false;
  }
}

void sendStatus() {
  char body[256];
  int n = snprintf(body, sizeof(body),
    "TLH1{\"uptime_ms\":%lu,\"revs\":%lu,\"hz\":%.2f,\"pts\":%d,\"sync_errs\":%lu,\"rssi\":%d,\"subs\":%d,\"sent\":%lu,\"send_errs\":%lu}",
    (unsigned long)millis(), (unsigned long)revs, revHz, lastRevPts, (unsigned long)badSync, WiFi.RSSI(), activeSubs(),
    (unsigned long)packetsSent, (unsigned long)sendErrors);
  sendToSubs((uint8_t *)body, n);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("tanklidar");
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[wifi] connecting");
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin("tanklidar")) { MDNS.addService("tanklidar", "udp", UDP_PORT); Serial.println("[mdns] tanklidar.local advertised"); }
    udp.begin(UDP_PORT);
    Serial.printf("[udp] listening on port %u\n", UDP_PORT);
  } else {
    Serial.printf("[wifi] FAILED to connect (status %d). Check secrets.h and that the network is 2.4 GHz.\n", WiFi.status());
  }
}

void setup() {
  motorsOff();
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TankBot lidar bridge v1 ===");
  connectWifi();
  Lidar.setRxBufferSize(8192);
  Lidar.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  delay(50);
  startScan();
  lastByteMs = millis(); revStartMs = millis();
}

void loop() {
  motorsOff();
  while (Lidar.available()) { handleByte(Lidar.read()); lastByteMs = millis(); }
  if (millis() - lastByteMs > 8000) { Serial.println("[lidar] no data for 8 s, restarting scan"); startScan(); lastByteMs = millis(); }
  if (WiFi.status() == WL_CONNECTED) {
    handleUdp();
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus >= 1000) { lastStatus = millis(); sendStatus(); }
  } else {
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > 10000) { lastRetry = millis(); Serial.println("[wifi] reconnecting..."); WiFi.reconnect(); }
  }
  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 2000) {
    lastLog = millis();
    Serial.printf("[stat] %.1f Hz | %d pts | sync errs %lu | wifi %s %s | subs %d | sent %lu | send errs %lu\n",
      revHz, lastRevPts, (unsigned long)badSync, WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-", activeSubs(),
      (unsigned long)packetsSent, (unsigned long)sendErrors);
  }
  delay(1);
}
