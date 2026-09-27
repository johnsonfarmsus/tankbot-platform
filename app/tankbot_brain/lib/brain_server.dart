// The brain's control server: serves the remote-control page and a WebSocket.
// Remote -> brain: {"type":"drive","f":..,"t":..} at ~20 Hz, {"type":"stop"},
//                  {"type":"set", maxSpeed/obstacleStop/mapping}, {"type":"clearMap"}
// Brain -> remote: {"type":"telem",...} ~5 Hz, {"type":"map","png":base64,...} ~1 Hz
// Safety: if drive messages stop for 300 ms (or the socket closes), onRemoteSilent() fires.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'pose_client.dart' show appClockMs;

class BrainServer {
  BrainServer({this.port = 8080, required this.onMessage, required this.onRemoteSilent, this.onConnect});

  final int port;
  final void Function(Map<String, dynamic> msg) onMessage;
  final void Function() onRemoteSilent;
  final void Function()? onConnect;

  HttpServer? _server;
  final Set<WebSocket> _clients = {};
  double _lastDriveMs = 0;
  bool _remoteDriving = false;
  Timer? _watchdog;
  String? url;
  String? error;

  int get clientCount => _clients.length;

  Future<void> start() async {
    try {
      _server = await HttpServer.bind(InternetAddress.anyIPv4, port, shared: true);
    } catch (e) {
      error = 'server failed: $e';
      return;
    }
    _server!.listen(_handle, onError: (_) {});
    _watchdog = Timer.periodic(const Duration(milliseconds: 50), (_) {
      if (_remoteDriving && appClockMs() - _lastDriveMs > 300) {
        _remoteDriving = false;
        onRemoteSilent();
      }
    });
    url = await _findUrl();
  }

  Future<String?> _findUrl() async {
    try {
      for (final ni in await NetworkInterface.list(type: InternetAddressType.IPv4)) {
        if (!(ni.name.startsWith('en') || ni.name.startsWith('wlan') || ni.name.startsWith('bridge'))) continue;
        for (final a in ni.addresses) {
          if (!a.isLoopback) return 'http://${a.address}:$port';
        }
      }
    } catch (_) {}
    return null;
  }

  Future<void> _handle(HttpRequest req) async {
    if (req.uri.path == '/ws' && WebSocketTransformer.isUpgradeRequest(req)) {
      final ws = await WebSocketTransformer.upgrade(req);
      _clients.add(ws);
      onConnect?.call();
      ws.listen((data) {
        if (data is! String) return;
        Map<String, dynamic> m;
        try {
          m = jsonDecode(data) as Map<String, dynamic>;
        } catch (_) {
          return;
        }
        if (m['type'] == 'drive') {
          _lastDriveMs = appClockMs();
          _remoteDriving = true;
        } else if (m['type'] == 'stop') {
          _remoteDriving = false;
        }
        onMessage(m);
      }, onDone: () => _drop(ws), onError: (_) => _drop(ws), cancelOnError: true);
      return;
    }
    if (req.uri.path == '/' || req.uri.path == '/index.html') {
      req.response.headers.contentType = ContentType.html;
      req.response.headers.set('Cache-Control', 'no-store');
      req.response.write(remotePageHtml);
    } else {
      req.response.statusCode = HttpStatus.notFound;
    }
    await req.response.close();
  }

  void _drop(WebSocket ws) {
    _clients.remove(ws);
    if (_remoteDriving) {
      _remoteDriving = false;
      onRemoteSilent();
    }
  }

  void broadcast(Map<String, dynamic> msg) {
    if (_clients.isEmpty) return;
    final s = jsonEncode(msg);
    for (final c in _clients.toList()) {
      try {
        c.add(s);
      } catch (_) {
        _clients.remove(c);
      }
    }
  }

  Future<void> stop() async {
    _watchdog?.cancel();
    for (final c in _clients.toList()) {
      await c.close();
    }
    _clients.clear();
    await _server?.close(force: true);
  }
}

const String remotePageHtml = r'''<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>TankBot Remote</title>
<style>
:root{color-scheme:dark}
html,body{margin:0;height:100%}
body{background:#101416;color:#e6eef0;font-family:-apple-system,system-ui,sans-serif;user-select:none;-webkit-user-select:none;
 touch-action:none;display:flex;flex-direction:column;height:100vh;height:100dvh}
header{padding:8px 12px;font-size:13px;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
#conn{font-weight:600}
#stat{color:#9fb3bb}
#view{flex:1;min-height:200px;position:relative}
canvas#map{width:100%;height:100%;display:block}
#banner{display:none;background:#c62828;text-align:center;padding:6px;font-weight:600}
#bottom{display:flex;gap:14px;padding:10px 12px;align-items:center}
#stick{width:170px;height:170px;flex:none;touch-action:none}
.ctl{font-size:13px;flex:1;display:flex;flex-direction:column;gap:6px;min-width:0}
.row{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
button{background:#23303a;color:#e6eef0;border:1px solid #3a4a55;border-radius:8px;padding:6px 10px;font-size:13px}
input[type=range]{width:100%}
</style></head><body>
<header><span id="conn">Connecting...</span><span id="stat"></span>
<span style="margin-left:auto;display:flex;gap:8px"><button id="botbtn">Bot</button><button id="mapsbtn">Maps</button><button id="setbtn">Settings</button><button id="mode">Radar view</button></span></header>
<div id="settings" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,0.6);z-index:10;align-items:center;justify-content:center">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:min(420px,90vw);display:flex;flex-direction:column;gap:12px;font-size:14px">
  <div style="font-weight:600;font-size:16px">Settings</div>
  <label>Steering trim: <span id="trimv">-</span>
   <input id="trim" type="range" min="-20" max="20" step="1" value="0"></label>
  <div style="display:flex;justify-content:space-between;color:#9fb3bb;font-size:12px"><span>&larr; steer left</span><span>steer right &rarr;</span></div>
  <div style="color:#9fb3bb;font-size:12px">If it drifts right when driving straight, move the slider toward left (and the other way round). Saved on the robot. You can keep driving with the arrow keys while this is open.</div>
  <label style="display:flex;align-items:center;gap:8px"><input id="obs" type="checkbox"> Obstacle stop (recommended: on)</label>
  <label>Obstacle stop distance: <span id="sdv">-</span>
   <input id="sd" type="range" min="150" max="1000" step="25" value="300"></label>
  <div style="font-weight:600;margin-top:4px">This brain</div>
  <div id="capsInfo" style="color:#9fb3bb;font-size:12px;line-height:1.5;white-space:pre-line"></div>
  <button id="setclose">Done</button>
 </div>
</div>
<div id="bot" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,0.6);z-index:10;align-items:center;justify-content:center">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:min(760px,94vw);max-height:90vh;overflow:auto;display:flex;flex-direction:column;gap:10px;font-size:13px">
  <div style="font-weight:600;font-size:16px">Bot</div>
  <div class="row">
   <label>Name <input id="botName" style="width:120px"></label>
   <label>Drive <select id="botDrive"><option value="tank">Tank</option><option value="wheelchair">Wheelchair</option><option value="mecanum">Mecanum</option></select></label>
   <label>Platform width <input id="botW" type="number" min="50" max="3000" style="width:64px"> mm</label>
   <label>length <input id="botL" type="number" min="50" max="3000" style="width:64px"> mm</label>
  </div>
  <div class="row" style="align-items:flex-start">
   <div><div style="color:#9fb3bb">Top view (front is up) - drag sensors</div><canvas id="botTop" width="300" height="300" style="background:#101416;border:1px solid #2c3a44;border-radius:8px;touch-action:none"></canvas></div>
   <div><div style="color:#9fb3bb">Side view from the left - drag up/down for height</div><canvas id="botSide" width="300" height="220" style="background:#101416;border:1px solid #2c3a44;border-radius:8px;touch-action:none"></canvas></div>
  </div>
  <div style="font-weight:600">Sensors</div>
  <div id="botSensors" style="display:flex;flex-direction:column;gap:4px"></div>
  <div class="row"><select id="botAddType"><option value="lidar">Lidar</option><option value="camera">Phone camera</option><option value="bumper">Bumper</option><option value="tof">ToF distance</option><option value="imu">IMU</option><option value="depth">Depth camera</option></select><button id="botAdd">Add sensor</button></div>
  <div style="color:#9fb3bb;font-size:12px">Positions are from the front and left edges of the platform; heights are above the floor. The phone camera is where the robot's tracked position sits; the lidar offset and the planning footprint are worked out from these.</div>
  <div class="row"><button id="botSave">Save to robot</button><button id="botCancel">Cancel</button><span id="botInfo" style="color:#9fb3bb"></span></div>
 </div>
</div>
<div id="maps" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,0.6);z-index:10;align-items:center;justify-content:center">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:min(520px,92vw);max-height:85vh;overflow:auto;display:flex;flex-direction:column;gap:10px;font-size:14px">
  <div style="font-weight:600;font-size:16px">Maps</div>
  <div id="mapActive" style="color:#9fb3bb"></div>
  <div id="mapQuality" style="color:#9fb3bb;font-size:12px"></div>
  <div class="row"><button id="relocBtn">Find me again</button><button id="homeBtn">I'm at home</button></div>
  <div class="row"><input id="mapName" style="flex:1;min-width:0;background:#101416;color:#e6eef0;border:1px solid #3a4a55;border-radius:8px;padding:6px" placeholder="Map name"><button id="mapSave">Save</button></div>
  <div class="row"><button id="mapNew">New map here</button></div>
  <div style="font-weight:600;margin-top:6px">Saved maps</div>
  <div id="mapList" style="display:flex;flex-direction:column;gap:6px"></div>
  <div style="color:#9fb3bb;font-size:12px">Load switches to a saved map; the robot then finds itself on it with the lidar. If it can't, drive a little, or put it on the home spot (white circle, facing along its line) and press I'm at home. Maps autosave every 20 s, and the last map loads automatically.</div>
  <button id="mapsClose">Done</button>
 </div>
</div>
<div id="view"><canvas id="map"></canvas>
<div id="edittools" style="display:none;position:absolute;top:8px;left:8px;right:8px;background:rgba(16,20,22,0.92);border:1px solid #3a4a55;border-radius:10px;padding:8px;gap:6px;flex-wrap:wrap;align-items:center;font-size:13px">
 <button data-tool="pan">Pan</button><button data-tool="erase">Eraser</button><button data-tool="nogo">No-go line</button><button data-tool="unnogo">Remove no-go</button>
 <label>Eraser <select id="eraseR"><option value="0.1">10 cm</option><option value="0.2" selected>20 cm</option><option value="0.4">40 cm</option></select></label>
 <button id="undoEdit">Undo</button><button id="recenter">Recenter</button><button id="editDone">Done</button>
 <span id="edithint" style="color:#9fb3bb;width:100%"></span>
</div></div>
<div id="banner">OBSTACLE AHEAD - forward blocked</div>
<div id="mountbar" style="display:none;text-align:center;padding:6px;font-weight:600"></div>
<div id="navbar" style="display:none;background:#0d47a1;padding:6px;font-weight:600;align-items:center;gap:8px;justify-content:center;flex-wrap:wrap"><span id="navtext"></span><button id="navStop">Stop</button><button id="navDismiss">OK</button></div>
<div id="locbar" style="display:none;background:#8d6e00;padding:6px;font-weight:600;align-items:center;gap:8px;justify-content:center;flex-wrap:wrap"><span id="loctext"></span><button id="locRetry">Try again</button><button id="locHome">I'm at home</button></div>
<div id="bottom"><canvas id="stick" width="340" height="340"></canvas>
<div class="ctl">
<div id="motors">Motors: -</div>
<label>Max speed <span id="msv"></span><input id="ms" type="range" min="0.2" max="1" step="0.1" value="0.6"></label>
<label>View range <span id="rgv"></span><input id="rg" type="range" min="1" max="12" step="0.5" value="4"></label>
<div class="row"><button id="mapping">Pause mapping</button><button id="clear">New map here</button><button id="editbtn">Edit map</button><button id="gobtn" style="border-color:#448aff">Go to...</button></div>
<div style="color:#9fb3bb;font-size:12px">Keyboard: arrow keys or W A S D to drive, Space to stop</div>
</div></div>
<script>
const $ = id => document.getElementById(id);
let ws = null, telem = null, mapImg = null, mapMeta = null, mode = "map", sendTimer = null;

function connect() {
  ws = new WebSocket("ws://" + location.host + "/ws");
  ws.onopen = () => { $("conn").textContent = "Connected"; $("conn").style.color = "#64ffda"; };
  ws.onclose = () => {
    $("conn").textContent = "Disconnected - retrying"; $("conn").style.color = "#ff5252";
    telem = null; stopDrive(); setTimeout(connect, 1000);
  };
  ws.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.type === "telem") { telem = m; updateUi(); if (mapsOpen) renderActive(); }
    else if (m.type === "maps") { mapsData = m; renderMaps(); }
    else if (m.type === "bot") { botProfile = m.profile; if (botOpen && !botDraft) openBot(); }
    else if (m.type === "map") {
      const img = new Image();
      img.onload = () => { mapImg = img; mapMeta = m; };
      img.src = "data:image/png;base64," + m.png;
    }
  };
}
function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }

// ---- joystick: sends 20x per second while held; the brain stops the robot if these stop ----
const stick = $("stick"), sctx = stick.getContext("2d");
let knob = null, activePointer = null;
const keys = new Set();
const KEYMAP = {ArrowUp: "u", KeyW: "u", ArrowDown: "d", KeyS: "d", ArrowLeft: "l", KeyA: "l", ArrowRight: "r", KeyD: "r"};
function keyVector() {
  const f = (keys.has("u") ? 1 : 0) - (keys.has("d") ? 1 : 0);
  let t = (keys.has("r") ? 1 : 0) - (keys.has("l") ? 1 : 0);
  if (f !== 0) t *= 0.5;   // arc while driving; full spin when turning alone
  return {f, t};
}
function currentVector() {
  if (knob) return {f: dz(-knob.y), t: dz(knob.x)};
  if (keys.size) return keyVector();
  return null;
}
function drawStick() {
  const w = stick.width, r = w / 2;
  sctx.clearRect(0, 0, w, w);
  sctx.beginPath(); sctx.arc(r, r, r - 4, 0, Math.PI * 2);
  sctx.fillStyle = "rgba(255,255,255,0.06)"; sctx.fill();
  sctx.lineWidth = 4; sctx.strokeStyle = (knob || keys.size) ? "#64ffda" : "#3a4a55"; sctx.stroke();
  const kv = keys.size ? keyVector() : null;
  const active = knob || kv;
  const k = knob || (kv ? {x: kv.t, y: -kv.f} : {x: 0, y: 0});
  sctx.beginPath(); sctx.arc(r + k.x * (r - 60), r + k.y * (r - 60), 52, 0, Math.PI * 2);
  sctx.fillStyle = active ? "#64ffda" : "#55636b"; sctx.fill();
}
function stickPos(e) {
  const b = stick.getBoundingClientRect();
  let x = (e.clientX - b.left) / b.width * 2 - 1, y = (e.clientY - b.top) / b.height * 2 - 1;
  const d = Math.hypot(x, y); if (d > 1) { x /= d; y /= d; }
  return {x, y};
}
function dz(v) { return Math.abs(v) < 0.08 ? 0 : v; }
function startDrive() {
  if (sendTimer) return;
  sendTimer = setInterval(() => { const v = currentVector(); if (v) send({type: "drive", f: v.f, t: v.t}); }, 50);
}
function stopDrive() { if (sendTimer) { clearInterval(sendTimer); sendTimer = null; } send({type: "stop"}); }
function release() { activePointer = null; knob = null; keys.clear(); stopDrive(); drawStick(); }
stick.addEventListener("pointerdown", e => {
  activePointer = e.pointerId; stick.setPointerCapture(e.pointerId);
  knob = stickPos(e); startDrive(); drawStick();
});
stick.addEventListener("pointermove", e => { if (e.pointerId === activePointer) { knob = stickPos(e); drawStick(); } });
stick.addEventListener("pointerup", e => { if (e.pointerId === activePointer) release(); });
stick.addEventListener("pointercancel", e => { if (e.pointerId === activePointer) release(); });
document.addEventListener("visibilitychange", () => { if (document.hidden) release(); });
const typing = e => e.target && ((e.target.tagName === "INPUT" && (e.target.type === "text" || e.target.type === "number")) || e.target.tagName === "SELECT");
document.addEventListener("keydown", e => {
  if (typing(e)) return;
  if (e.code === "Space") { e.preventDefault(); release(); return; }
  const k = KEYMAP[e.code]; if (!k) return;
  e.preventDefault();
  if (e.repeat || keys.has(k)) return;
  keys.add(k); startDrive();
  const v = currentVector(); if (v) send({type: "drive", f: v.f, t: v.t});
  drawStick();
});
document.addEventListener("keyup", e => {
  if (typing(e)) return;
  const k = KEYMAP[e.code]; if (!k) return;
  e.preventDefault();
  keys.delete(k);
  if (keys.size === 0 && !knob) stopDrive();
  else { const v = currentVector(); if (v) send({type: "drive", f: v.f, t: v.t}); }
  drawStick();
});
window.addEventListener("blur", () => { if (knob || keys.size) release(); });

// ---- controls ----
$("ms").oninput = e => { $("msv").textContent = Math.round(e.target.value * 100) + "%"; send({type: "set", maxSpeed: parseFloat(e.target.value)}); };
$("rg").oninput = e => { $("rgv").textContent = e.target.value + " m"; };
$("obs").onchange = e => send({type: "set", obstacleStop: e.target.checked});
$("mapping").onclick = () => send({type: "set", mapping: !(telem && telem.settings.mapping)});
$("clear").onclick = () => { if (confirm("Start a new map here? The current map is saved first.")) send({type: "clearMap"}); };
$("mode").onclick = () => { mode = mode === "map" ? "radar" : "map"; $("mode").textContent = mode === "map" ? "Radar view" : "Map view"; };
$("rgv").textContent = $("rg").value + " m";

// ---- settings panel ----
// Robot trim > 0 steers left, < 0 steers right; the slider shows "steer right" as positive.
let settingsOpen = false;
function syncSettings() {
  if (!telem) return;
  const s = telem.settings;
  if (typeof s.trim === "number") { $("trim").value = -s.trim; $("trimv").textContent = -s.trim; }
  if (typeof s.stopDistMm === "number") { $("sd").value = s.stopDistMm; $("sdv").textContent = Math.round(s.stopDistMm / 10) + " cm"; }
}
$("setbtn").onclick = () => { $("settings").style.display = "flex"; settingsOpen = true; syncSettings();
  if (telem) $("capsInfo").textContent = capsText(telem.caps, telem.tracking); };
$("setclose").onclick = () => { $("settings").style.display = "none"; settingsOpen = false; };
$("trim").oninput = e => { $("trimv").textContent = e.target.value; };
$("trim").onchange = e => send({type: "set", trim: -parseInt(e.target.value, 10)});
$("sd").oninput = e => { $("sdv").textContent = Math.round(e.target.value / 10) + " cm"; };
$("sd").onchange = e => send({type: "set", stopDistMm: parseFloat(e.target.value)});

// ---- localisation + capabilities ----
$("locRetry").onclick = () => send({type: "reloc"});
$("locHome").onclick = () => { if (confirm("Is the robot on the map's home spot (white circle), facing along its line?")) send({type: "atHome"}); };
function capsText(c, tr) {
  if (!c) return "";
  const yn = v => v ? "yes" : "no";
  let s = "Phone: " + (c.model || c.platform || "unknown") +
    "\nCamera tracking: " + yn(c.worldTracking) + " | Depth camera: " + yn(c.sceneDepth) +
    " | Surface labels: " + yn(c.meshClassification) + "\nPlace memory: " + yn(c.worldMaps) +
    " | Barometer: " + yn(c.barometer) + " | GPS: " + yn(c.gps) + " | Robot lidar: " + yn(c.robotLidar);
  if (tr) s += "\nNow tracking with: " + tr.source + " | lidar corrections: " + tr.matchHits + " used, " + tr.matchMisses + " skipped";
  return s;
}

// ---- Bot tab: the bot profile editor ----
let botProfile = null, botDraft = null, botOpen = false, botDrag = null;
const SENSOR_LABEL = {lidar: "Lidar", camera: "Phone camera", bumper: "Bumper", tof: "ToF", imu: "IMU", depth: "Depth cam"};
const SENSOR_COLOR = {lidar: "#64ffda", camera: "#ffab40", bumper: "#ff5252", tof: "#448aff", imu: "#ce93d8", depth: "#ffd54f"};
function openBot() {
  if (!botProfile) { send({type: "bot.get"}); return; }
  botDraft = JSON.parse(JSON.stringify(botProfile));
  $("botName").value = botDraft.name; $("botDrive").value = botDraft.drive;
  $("botW").value = botDraft.platform.widthMm; $("botL").value = botDraft.platform.lengthMm;
  renderBotSensors(); drawBot();
}
$("botbtn").onclick = () => { $("bot").style.display = "flex"; botOpen = true; botDraft = null; openBot(); };
$("botCancel").onclick = () => { $("bot").style.display = "none"; botOpen = false; botDraft = null; };
$("botSave").onclick = () => {
  if (!botDraft) return;
  botDraft.name = $("botName").value; botDraft.drive = $("botDrive").value;
  botDraft.platform.widthMm = parseFloat($("botW").value); botDraft.platform.lengthMm = parseFloat($("botL").value);
  send({type: "bot.set", profile: botDraft});
  $("botInfo").textContent = "Saved";
  setTimeout(() => { $("botInfo").textContent = ""; }, 2000);
};
["botW", "botL"].forEach(id => $(id).oninput = () => { if (!botDraft) return;
  botDraft.platform.widthMm = parseFloat($("botW").value) || 100; botDraft.platform.lengthMm = parseFloat($("botL").value) || 100; drawBot(); });
$("botAdd").onclick = () => {
  if (!botDraft) return;
  const type = $("botAddType").value;
  botDraft.sensors.push({id: type + "_" + Date.now(), type, name: SENSOR_LABEL[type],
    fromLeftMm: botDraft.platform.widthMm / 2, fromFrontMm: 20, heightMm: 50, yawDeg: 0});
  renderBotSensors(); drawBot();
};
function numInput(v, min, max, w, onchange) {
  const i = document.createElement("input"); i.type = "number"; i.value = Math.round(v); i.min = min; i.max = max; i.style.width = w + "px";
  i.oninput = () => onchange(parseFloat(i.value) || 0); return i;
}
function renderBotSensors() {
  const box = $("botSensors"); box.textContent = "";
  botDraft.sensors.forEach((sn, idx) => {
    const row = document.createElement("div"); row.className = "row";
    const dot = document.createElement("span"); dot.style.cssText = "display:inline-block;width:10px;height:10px;border-radius:5px;background:" + SENSOR_COLOR[sn.type];
    const nm = document.createElement("input"); nm.value = sn.name; nm.style.width = "110px"; nm.oninput = () => { sn.name = nm.value; drawBot(); };
    const lab = t => { const e = document.createElement("span"); e.textContent = t; e.style.color = "#9fb3bb"; return e; };
    const del = document.createElement("button"); del.textContent = "x"; del.onclick = () => { botDraft.sensors.splice(idx, 1); renderBotSensors(); drawBot(); };
    row.append(dot, lab(SENSOR_LABEL[sn.type]), nm,
      lab("from left"), numInput(sn.fromLeftMm, -500, 3000, 60, v => { sn.fromLeftMm = v; drawBot(); }),
      lab("from front"), numInput(sn.fromFrontMm, -500, 3000, 60, v => { sn.fromFrontMm = v; drawBot(); }),
      lab("height"), numInput(sn.heightMm, 0, 3000, 60, v => { sn.heightMm = v; drawBot(); }),
      lab("yaw"), numInput(sn.yawDeg, -180, 180, 50, v => { sn.yawDeg = v; drawBot(); }), del);
    box.append(row);
  });
}
function botTopScale() { const W = botDraft.platform.widthMm, L = botDraft.platform.lengthMm; const sc = 250 / Math.max(W, L); return {sc, ox: 150 - W * sc / 2, oy: 150 - L * sc / 2}; }
function botSideScale() { const L = botDraft.platform.lengthMm; let H = 100; for (const sn of botDraft.sensors) H = Math.max(H, sn.heightMm + 40); const sc = Math.min(250 / L, 180 / H); return {sc, ox: 150 - L * sc / 2, floorY: 200}; }
function drawBot() {
  if (!botDraft) return;
  const W = botDraft.platform.widthMm, L = botDraft.platform.lengthMm;
  // top view
  const c = $("botTop").getContext("2d"), t = botTopScale();
  c.clearRect(0, 0, 300, 300);
  c.fillStyle = "#2a3238"; c.fillRect(t.ox, t.oy, W * t.sc, L * t.sc);
  c.strokeStyle = "#64ffda"; c.lineWidth = 2; c.strokeRect(t.ox, t.oy, W * t.sc, L * t.sc);
  c.fillStyle = "#64ffda"; c.beginPath(); c.moveTo(150, t.oy - 12); c.lineTo(143, t.oy - 3); c.lineTo(157, t.oy - 3); c.closePath(); c.fill();
  c.fillStyle = "#9fb3bb"; c.font = "11px sans-serif"; c.fillText("front", 160, t.oy - 4);
  for (const sn of botDraft.sensors) {
    const x = t.ox + sn.fromLeftMm * t.sc, y = t.oy + sn.fromFrontMm * t.sc;
    c.fillStyle = SENSOR_COLOR[sn.type]; c.beginPath(); c.arc(x, y, 9, 0, Math.PI * 2); c.fill();
    const a = -sn.yawDeg * Math.PI / 180 - Math.PI / 2;
    c.strokeStyle = SENSOR_COLOR[sn.type]; c.beginPath(); c.moveTo(x, y); c.lineTo(x + Math.cos(a) * 16, y + Math.sin(a) * 16); c.stroke();
    c.fillStyle = "#e6eef0"; c.fillText(sn.name, x + 12, y + 4);
  }
  // side view from the left: front to the right
  const s = $("botSide").getContext("2d"), v = botSideScale();
  s.clearRect(0, 0, 300, 220);
  s.strokeStyle = "#556"; s.beginPath(); s.moveTo(0, v.floorY); s.lineTo(300, v.floorY); s.stroke();
  s.fillStyle = "#9fb3bb"; s.font = "11px sans-serif"; s.fillText("floor", 4, v.floorY - 4); s.fillText("front", 260, v.floorY + 14);
  const platY = v.floorY - 60 * v.sc;
  s.fillStyle = "#2a3238"; s.fillRect(v.ox, platY - 4, L * v.sc, 4);
  for (const sn of botDraft.sensors) {
    const x = v.ox + (L - sn.fromFrontMm) * v.sc, y = v.floorY - sn.heightMm * v.sc;
    s.strokeStyle = "#3a4a55"; s.beginPath(); s.moveTo(x, platY); s.lineTo(x, y); s.stroke();
    s.fillStyle = SENSOR_COLOR[sn.type]; s.beginPath(); s.arc(x, y, 8, 0, Math.PI * 2); s.fill();
    s.fillStyle = "#e6eef0"; s.fillText(sn.name + " " + Math.round(sn.heightMm) + " mm", x + 11, y + 4);
  }
}
function botHit(canvas, e, mapper) {
  const b = canvas.getBoundingClientRect(), px = (e.clientX - b.left) * canvas.width / b.width, py = (e.clientY - b.top) * canvas.height / b.height;
  let best = null, bd = 16;
  for (const sn of botDraft.sensors) { const q = mapper(sn); const d = Math.hypot(q[0] - px, q[1] - py); if (d < bd) { bd = d; best = sn; } }
  return {sn: best, px, py};
}
$("botTop").addEventListener("pointerdown", e => { if (!botDraft) return; const t = botTopScale(); const h = botHit($("botTop"), e, sn => [t.ox + sn.fromLeftMm * t.sc, t.oy + sn.fromFrontMm * t.sc]); if (h.sn) { botDrag = {sn: h.sn, view: "top"}; $("botTop").setPointerCapture(e.pointerId); } });
$("botTop").addEventListener("pointermove", e => { if (!botDrag || botDrag.view !== "top") return; const t = botTopScale(); const b = $("botTop").getBoundingClientRect();
  const px = (e.clientX - b.left) * 300 / b.width, py = (e.clientY - b.top) * 300 / b.height;
  botDrag.sn.fromLeftMm = Math.round(Math.max(-50, Math.min(botDraft.platform.widthMm + 50, (px - t.ox) / t.sc)));
  botDrag.sn.fromFrontMm = Math.round(Math.max(-50, Math.min(botDraft.platform.lengthMm + 50, (py - t.oy) / t.sc)));
  renderBotSensors(); drawBot(); });
$("botSide").addEventListener("pointerdown", e => { if (!botDraft) return; const v = botSideScale(); const h = botHit($("botSide"), e, sn => [v.ox + (botDraft.platform.lengthMm - sn.fromFrontMm) * v.sc, v.floorY - sn.heightMm * v.sc]); if (h.sn) { botDrag = {sn: h.sn, view: "side"}; $("botSide").setPointerCapture(e.pointerId); } });
$("botSide").addEventListener("pointermove", e => { if (!botDrag || botDrag.view !== "side") return; const v = botSideScale(); const b = $("botSide").getBoundingClientRect();
  const py = (e.clientY - b.top) * 220 / b.height;
  botDrag.sn.heightMm = Math.round(Math.max(0, (v.floorY - py) / v.sc));
  renderBotSensors(); drawBot(); });
const botEndDrag = () => { botDrag = null; };
["botTop", "botSide"].forEach(id => { $(id).addEventListener("pointerup", botEndDrag); $(id).addEventListener("pointercancel", botEndDrag); });

// ---- maps panel ----
let mapsOpen = false, mapsData = null;
function fmtDate(ms) {
  const d = new Date(ms);
  return d.toLocaleDateString() + " " + d.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
}
function renderActive() {
  const a = telem && telem.mapInfo; if (!a) return;
  $("mapActive").textContent = "Current: " + a.name + " - " + a.keyframes + " keyframes - " +
    (a.loading ? "loading..." : (a.unsaved ? "unsaved changes" : "saved"));
  const q = telem.quality;
  if (q) $("mapQuality").textContent = "Map quality: " + q.matchHits + " lidar corrections, " + q.matchMisses +
    " rejected, " + q.skippedTurning + " scans skipped while spinning, " + q.loopClosures + " loop closures" +
    (q.loopClosures ? " (last one fixed " + q.lastLoopCm.toFixed(0) + " cm / " + q.lastLoopDeg.toFixed(1) + " deg)" : "") +
    (q.rebuilding ? " - redrawing map..." : "");
}
function renderMaps() {
  if (!mapsData) return;
  const activeId = (telem && telem.mapInfo && telem.mapInfo.id) || (mapsData.active && mapsData.active.id);
  const list = $("mapList"); list.textContent = "";
  if (!mapsData.list.length) { list.textContent = "No saved maps yet."; return; }
  for (const mp of mapsData.list) {
    const row = document.createElement("div"); row.className = "row";
    row.style.borderTop = "1px solid #2c3a44"; row.style.paddingTop = "6px";
    const info = document.createElement("div"); info.style.flex = "1"; info.style.minWidth = "0";
    const nm = document.createElement("div"); nm.style.fontWeight = "600";
    nm.textContent = mp.name + (mp.id === activeId ? "  (current)" : "");
    const sub = document.createElement("div"); sub.style.color = "#9fb3bb"; sub.style.fontSize = "12px";
    sub.textContent = mp.keyframes + " keyframes" + (mp.sizeM ? " - " + mp.sizeM : "") + " - saved " + fmtDate(mp.updated);
    info.append(nm, sub); row.append(info);
    if (mp.id !== activeId) {
      const lb = document.createElement("button"); lb.textContent = "Load";
      lb.onclick = () => {
        if (confirm("Switch to \"" + mp.name + "\"? The current map is saved first.")) {
          send({type: "maps.load", id: mp.id});
        }
      };
      const db = document.createElement("button"); db.textContent = "Delete";
      db.onclick = () => { if (confirm("Delete \"" + mp.name + "\"? This cannot be undone.")) send({type: "maps.delete", id: mp.id}); };
      row.append(lb, db);
    }
    list.append(row);
  }
}
$("mapsbtn").onclick = () => {
  $("maps").style.display = "flex"; mapsOpen = true;
  if (telem && telem.mapInfo) $("mapName").value = telem.mapInfo.name;
  renderActive(); send({type: "maps.list"});
};
$("mapsClose").onclick = () => { $("maps").style.display = "none"; mapsOpen = false; };
$("relocBtn").onclick = () => send({type: "reloc"});
$("homeBtn").onclick = () => { if (confirm("Is the robot on the home spot (white circle), facing along its line?")) send({type: "atHome"}); };
$("mapSave").onclick = () => send({type: "maps.save", name: $("mapName").value});
$("mapNew").onclick = () => { if (confirm("Start a new map here? The current map is saved first.")) send({type: "clearMap"}); };
$("msv").textContent = Math.round($("ms").value * 100) + "%";

let settingsInit = false;
function updateUi() {
  const t = telem, s = t.settings, st = t.stats, m = t.motion;
  if (!settingsInit) { $("ms").value = s.maxSpeed; $("msv").textContent = Math.round(s.maxSpeed * 100) + "%"; settingsInit = true; }
  $("obs").checked = s.obstacleStop;
  $("mapping").textContent = s.mapping ? "Pause mapping" : "Resume mapping";
  $("banner").style.display = t.blocked ? "block" : "none";
  const mb = $("mountbar"), mt = t.mount;
  if (mt && mt.robotMode && mt.note) {
    mb.style.display = "block"; mb.textContent = mt.note;
    mb.style.background = mt.state === "mounted" ? "#00695c" : "#8d6e00";
  } else mb.style.display = "none";
  $("stat").textContent = st.scanRate.toFixed(1) + " scans/s | " + (t.tracking ? t.tracking.source : "AR " + st.ar) + " | remotes " + st.remotes;
  const nv = t.nav, nb = $("navbar");
  const navActive = nv && (nv.state === "driving" || nv.state === "blocked");
  if (goMode) { nb.style.display = "flex"; $("navtext").textContent = "Tap a spot on the map to drive there"; $("navStop").textContent = "Cancel"; $("navStop").style.display = ""; $("navDismiss").style.display = "none"; }
  else if (navActive) { nb.style.display = "flex"; $("navtext").textContent = nv.note || nv.state; $("navStop").textContent = "Stop"; $("navStop").style.display = ""; $("navDismiss").style.display = "none"; }
  else if (nv && nv.note && nv.note !== navDismissed) { nb.style.display = "flex"; $("navtext").textContent = nv.note; $("navStop").style.display = "none"; $("navDismiss").style.display = ""; }
  else nb.style.display = "none";
  const lb = $("locbar");
  if (t.loc && t.loc.state !== "tracking") { lb.style.display = "flex"; $("loctext").textContent = t.loc.note; }
  else lb.style.display = "none";
  if (settingsOpen) $("capsInfo").textContent = capsText(t.caps, t.tracking);
  $("motors").textContent = m ? "Motors L " + m.left.toFixed(2) + "  R " + m.right.toFixed(2) + " (" + m.src + ")" : "Motors: -";
}

// ---- drawing ----
const cv = $("map"), ctx = cv.getContext("2d");
function drawArrow(x, y, h, good) {
  const dx = Math.cos(h), dy = -Math.sin(h), sx = -dy, sy = dx;
  ctx.beginPath(); ctx.moveTo(x + dx * 14, y + dy * 14);
  ctx.lineTo(x - dx * 9 + sx * 9, y - dy * 9 + sy * 9);
  ctx.lineTo(x - dx * 9 - sx * 9, y - dy * 9 - sy * 9); ctx.closePath();
  ctx.fillStyle = good ? "#ffab40" : "#888"; ctx.fill();
}
function frame() {
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#15191c"; ctx.fillRect(0, 0, W, H);
  const t = telem;
  if (t) {
    const range = parseFloat($("rg").value), R = Math.min(W, H) / 2 - 10, ppm = R / range, cx = W / 2, cy = H / 2;
    const p = t.pose || {x: 0, y: 0, h: Math.PI / 2, good: false};
    const off = t.lidarOffset || {fwd: 0, left: 0};
    if (mode === "map") {
      const vx = p.x + panX, vy = p.y + panY;
      view = {cx, cy, ppm, vx, vy};
      const toS = (x, y) => [cx + (x - vx) * ppm, cy - (y - vy) * ppm];
      if (mapImg && mapMeta) {
        const tl = toS(mapMeta.left, mapMeta.top);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(mapImg, tl[0], tl[1], mapImg.width * mapMeta.res * ppm, mapImg.height * mapMeta.res * ppm);
      }
      ctx.strokeStyle = "rgba(255,255,255,0.07)"; ctx.lineWidth = 1; ctx.beginPath();
      for (let gx = Math.floor(vx - range * 2); gx <= vx + range * 2; gx++) { const a = toS(gx, 0)[0]; ctx.moveTo(a, 0); ctx.lineTo(a, H); }
      for (let gy = Math.floor(vy - range * 2); gy <= vy + range * 2; gy++) { const b = toS(0, gy)[1]; ctx.moveTo(0, b); ctx.lineTo(W, b); }
      ctx.stroke();
      // home marker: where this map started, and the direction the robot faced
      const hm = toS(0, 0);
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(hm[0], hm[1], 7, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(hm[0], hm[1]); ctx.lineTo(hm[0], hm[1] - 16); ctx.stroke();
      if (t.scan && p.good) {
        const ch = Math.cos(p.h), sh = Math.sin(p.h);
        const ox = p.x + ch * off.fwd - sh * off.left, oy = p.y + sh * off.fwd + ch * off.left;
        ctx.fillStyle = "rgba(255,213,79,0.9)";
        for (const s of t.scan) {
          const ar = s[0] * Math.PI / 180, f = s[1] * Math.cos(ar), lf = -s[1] * Math.sin(ar);
          const q = toS(ox + ch * f - sh * lf, oy + sh * f + ch * lf);
          ctx.fillRect(q[0] - 1.5, q[1] - 1.5, 3, 3);
        }
      }
      if (t.nogo) {
        ctx.strokeStyle = "#ff5252"; ctx.lineWidth = 3;
        for (const l of t.nogo) {
          const a = toS(l[0], l[1]), b = toS(l[2], l[3]);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        }
      }
      if (editMode && hover) {
        if (tool === "erase") {
          const hs = toS(hover.x, hover.y);
          ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1;
          ctx.beginPath(); ctx.arc(hs[0], hs[1], parseFloat($("eraseR").value) * ppm, 0, Math.PI * 2); ctx.stroke();
        }
        if (tool === "nogo" && nogoStart) {
          const a = toS(nogoStart.x, nogoStart.y), b = toS(hover.x, hover.y);
          ctx.strokeStyle = "#ff5252"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.setLineDash([]);
        }
      }
      if (t.nav && t.nav.path && t.nav.path.length > 1) {
        ctx.strokeStyle = "#448aff"; ctx.lineWidth = 3; ctx.beginPath();
        t.nav.path.forEach((q, i) => { const s2 = toS(q[0], q[1]); if (i) ctx.lineTo(s2[0], s2[1]); else ctx.moveTo(s2[0], s2[1]); });
        ctx.stroke();
      }
      if (t.nav && t.nav.goal && (t.nav.state === "driving" || t.nav.state === "blocked")) {
        const gs2 = toS(t.nav.goal[0], t.nav.goal[1]);
        ctx.strokeStyle = "#448aff"; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(gs2[0], gs2[1], 9, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath(); ctx.arc(gs2[0], gs2[1], 3, 0, Math.PI * 2); ctx.fillStyle = "#448aff"; ctx.fill();
      }
      const rs = toS(p.x, p.y);
      drawArrow(rs[0], rs[1], p.h, p.good);
    } else {
      ctx.strokeStyle = "rgba(255,255,255,0.15)";
      for (let r = 1; r <= range; r++) { ctx.beginPath(); ctx.arc(cx, cy, r * ppm, 0, Math.PI * 2); ctx.stroke(); }
      ctx.fillStyle = "#64ffda";
      if (t.scan) for (const s of t.scan) {
        if (s[1] > range) continue;
        const ar = s[0] * Math.PI / 180;
        ctx.fillRect(cx + s[1] * ppm * Math.sin(ar) - 1.5, cy - s[1] * ppm * Math.cos(ar) - 1.5, 3, 3);
      }
      drawArrow(cx, cy, Math.PI / 2, true);
    }
  }
  requestAnimationFrame(frame);
}
// ---- map editing ----
let editMode = false, tool = "pan", panX = 0, panY = 0, nogoStart = null, hover = null;
let editDrag = null, lastErase = null, strokeId = 0, view = null;
const HINTS = {
  pan: "Drag the map to look around.",
  erase: "Drag over ghost walls or junk to wipe them back to open floor.",
  nogo: "Click two points to draw a line the robot must never cross.",
  unnogo: "Click a red no-go line to remove it.",
};
function setTool(t) {
  tool = t; nogoStart = null;
  document.querySelectorAll("#edittools [data-tool]").forEach(b => b.style.outline = b.dataset.tool === t ? "2px solid #64ffda" : "none");
  $("edithint").textContent = HINTS[t];
}
document.querySelectorAll("#edittools [data-tool]").forEach(b => b.onclick = () => setTool(b.dataset.tool));
$("editbtn").onclick = () => {
  editMode = true;
  if (mode !== "map") { mode = "map"; $("mode").textContent = "Radar view"; }
  $("edittools").style.display = "flex"; setTool("pan");
};
$("editDone").onclick = () => { editMode = false; panX = panY = 0; nogoStart = null; hover = null; $("edittools").style.display = "none"; };
$("recenter").onclick = () => { panX = panY = 0; };
$("undoEdit").onclick = () => send({type: "map.undo"});
function toWorld(e) {
  if (!view) return null;
  const b = cv.getBoundingClientRect(), sx = e.clientX - b.left, sy = e.clientY - b.top;
  return {x: view.vx + (sx - view.cx) / view.ppm, y: view.vy - (sy - view.cy) / view.ppm};
}
function eraseAt(w) {
  const r = parseFloat($("eraseR").value);
  if (lastErase && Math.hypot(w.x - lastErase.x, w.y - lastErase.y) < r * 0.5) return;
  lastErase = w;
  send({type: "map.erase", x: w.x, y: w.y, r: r, stroke: strokeId});
}
function segDist(px, py, l) {
  const x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3], dx = x2 - x1, dy = y2 - y1;
  const L = dx * dx + dy * dy || 1e-9;
  let t = ((px - x1) * dx + (py - y1) * dy) / L; t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}
// ---- tap-to-go ----
let goMode = false, navDismissed = null;
$("gobtn").onclick = () => {
  if (editMode) $("editDone").onclick();
  if (mode !== "map") { mode = "map"; $("mode").textContent = "Radar view"; }
  goMode = true;
};
$("navStop").onclick = () => { if (goMode) goMode = false; else send({type: "nav.cancel"}); };
$("navDismiss").onclick = () => { navDismissed = telem && telem.nav ? telem.nav.note : null; };
cv.addEventListener("pointerdown", e => {
  if (!goMode || mode !== "map") return;
  const w = toWorld(e); if (!w) return;
  goMode = false; navDismissed = null;
  send({type: "nav.goto", x: w.x, y: w.y});
});
cv.addEventListener("pointerdown", e => {
  if (!editMode || mode !== "map") return;
  const w = toWorld(e); if (!w) return;
  cv.setPointerCapture(e.pointerId);
  if (tool === "pan") editDrag = {x: e.clientX, y: e.clientY};
  else if (tool === "erase") { editDrag = true; lastErase = null; strokeId = Date.now(); eraseAt(w); }
  else if (tool === "nogo") {
    if (!nogoStart) nogoStart = w;
    else { send({type: "map.nogo", x1: nogoStart.x, y1: nogoStart.y, x2: w.x, y2: w.y}); nogoStart = null; }
  } else if (tool === "unnogo" && telem && telem.nogo) {
    let best = null, bd = 0.3;
    for (const l of telem.nogo) { const d = segDist(w.x, w.y, l); if (d < bd) { bd = d; best = l; } }
    if (best) send({type: "map.nogoDelete", id: best[4]});
  }
});
cv.addEventListener("pointermove", e => {
  if (!editMode) return;
  hover = toWorld(e);
  if (tool === "pan" && editDrag && view) {
    panX -= (e.clientX - editDrag.x) / view.ppm; panY += (e.clientY - editDrag.y) / view.ppm;
    editDrag = {x: e.clientX, y: e.clientY};
  } else if (tool === "erase" && editDrag && hover) eraseAt(hover);
});
const endEditDrag = () => { editDrag = null; };
cv.addEventListener("pointerup", endEditDrag);
cv.addEventListener("pointercancel", endEditDrag);
cv.addEventListener("pointerleave", () => { hover = null; });

drawStick(); connect(); requestAnimationFrame(frame);
</script></body></html>
''';
