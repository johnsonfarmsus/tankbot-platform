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
<nav style="margin-left:auto;display:flex;gap:6px;flex-wrap:wrap"><button class="tab" data-page="drive">Drive</button><button class="tab" data-page="maps">Maps</button><button class="tab" data-page="bot">Bot</button><button class="tab" data-page="settings">Settings</button><button id="mode">Radar view</button></nav></header>
<div id="settings" style="display:none;flex:1;min-height:0;overflow:auto;padding:12px">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:100%;max-width:640px;margin:0 auto;box-sizing:border-box;display:flex;flex-direction:column;gap:12px;font-size:14px">
  <div style="font-weight:600;font-size:16px">Settings</div>
  <label>Steering trim: <span id="trimv">-</span>
   <input id="trim" type="range" min="-20" max="20" step="1" value="0"></label>
  <div style="display:flex;justify-content:space-between;color:#9fb3bb;font-size:12px"><span>&larr; steer left</span><span>steer right &rarr;</span></div>
  <div style="color:#9fb3bb;font-size:12px">If it drifts right when driving straight, move the slider toward left (and the other way round). Saved on the robot. You can keep driving with the arrow keys while this is open.</div>
  <label>Minimum power to move: <span id="minpv">-</span>
   <input id="minp" type="range" min="30" max="100" step="5"></label>
  <div style="color:#9fb3bb;font-size:12px">The lowest power at which this robot actually moves. Autonomous driving never sends less than this.</div>
  <label>Cruise power: <span id="cruisev">-</span>
   <input id="cruise" type="range" min="30" max="100" step="5"></label>
  <div style="color:#9fb3bb;font-size:12px">The power it drives best at. Autonomous driving uses this for straight runs; the manual max-speed slider defaults to it.</div>
  <label style="display:flex;align-items:center;gap:8px"><input id="obs" type="checkbox"> Obstacle stop (recommended: on)</label>
  <div class="row">
   <label>Obstacle stop distance <input id="sd" type="number" min="100" max="2000" step="10" style="width:70px"> mm</label>
   <label>Obstacle pass distance <input id="pd" type="number" min="0" max="1000" step="10" style="width:70px"> mm</label>
  </div>
  <div style="color:#9fb3bb;font-size:12px">Stop: never drive forward with something closer than this ahead. Pass: routes keep at least this much clearance around the robot's body (from the Bot page dimensions).</div>
  <div style="font-weight:600;margin-top:4px">Robot</div>
  <div id="robotInfo" style="color:#9fb3bb;font-size:12px;line-height:1.5;white-space:pre-line"></div>
  <div style="font-weight:600;margin-top:4px">This brain</div>
  <div id="capsInfo" style="color:#9fb3bb;font-size:12px;line-height:1.5;white-space:pre-line"></div>
  <button id="setclose">Back to Drive</button>
 </div>
</div>
<div id="bot" style="display:none;flex:1;min-height:0;overflow:auto;padding:12px">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:100%;max-width:1000px;margin:0 auto;box-sizing:border-box;display:flex;flex-direction:column;gap:10px;font-size:13px">
  <div style="font-weight:600;font-size:16px">Bot</div>
  <div class="row">
   <label>Name <input id="botName" style="width:120px"></label>
   <label>Drive <select id="botDrive"><option value="tank">Tank</option><option value="wheelchair">Wheelchair</option><option value="mecanum">Mecanum</option></select></label>
   <label>Platform width <input id="botW" type="number" min="50" max="3000" style="width:64px"> mm</label>
   <label>length <input id="botL" type="number" min="50" max="3000" style="width:64px"> mm</label>
  </div>

  <div class="row" style="align-items:flex-start">
   <div><div style="color:#9fb3bb">Top view (front is up) - drag sensors</div><canvas id="botTop" width="460" height="460" style="background:#101416;border:1px solid #2c3a44;border-radius:8px;touch-action:none"></canvas></div>
   <div><div style="color:#9fb3bb">Side view from the left - drag up/down for height</div><canvas id="botSide" width="460" height="320" style="background:#101416;border:1px solid #2c3a44;border-radius:8px;touch-action:none"></canvas></div>
  </div>
  <div style="font-weight:600">Sensors</div>
  <div id="botSensors" style="display:flex;flex-direction:column;gap:4px"></div>
  <div class="row"><select id="botAddType"><option value="lidar">Lidar</option><option value="camera">Phone camera</option><option value="bumper">Bumper</option><option value="tof">ToF distance</option><option value="ultrasonic">Ultrasonic</option><option value="imu">IMU</option><option value="depth">Depth camera</option></select><button id="botAdd">Add sensor</button></div>
  <div style="color:#9fb3bb;font-size:12px">Positions are from the front and left edges of the platform; heights are above the floor. The phone camera is where the robot's tracked position sits; the lidar offset and the planning footprint are worked out from these.</div>
  <div class="row"><button id="botSave">Save to robot</button><button id="botCancel">Back to Drive (discards unsaved changes)</button><span id="botInfo" style="color:#9fb3bb"></span></div>
 </div>
</div>
<div id="maps" style="display:none;flex:1;min-height:0;overflow:auto;padding:12px">
 <div style="background:#1b2227;border:1px solid #3a4a55;border-radius:12px;padding:16px;width:100%;max-width:760px;margin:0 auto;box-sizing:border-box;display:flex;flex-direction:column;gap:10px;font-size:14px">
  <div style="font-weight:600;font-size:16px">Maps</div>
  <div id="mapActive" style="color:#9fb3bb"></div>
  <div id="mapQuality" style="color:#9fb3bb;font-size:12px"></div>
  <div class="row"><button id="relocBtn">Find me again</button><button id="homeBtn">I'm at home</button></div>
  <div class="row"><input id="mapName" style="flex:1;min-width:0;background:#101416;color:#e6eef0;border:1px solid #3a4a55;border-radius:8px;padding:6px" placeholder="Map name"><button id="mapSave">Save</button></div>
  <div class="row"><button id="mapNew">New map here</button></div>
  <div style="font-weight:600;margin-top:6px">Saved maps</div>
  <div id="mapList" style="display:flex;flex-direction:column;gap:6px"></div>
  <div style="color:#9fb3bb;font-size:12px">Load switches to a saved map; the robot then finds itself on it with the lidar. If it can't, drive a little, or put it on the home spot (white circle, facing along its line) and press I'm at home. Maps autosave every 20 s, and the last map loads automatically.</div>
  <button id="mapsClose">Back to Drive</button>
 </div>
</div>
<div id="drive" style="display:flex;flex-direction:column;flex:1;min-height:0">
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
</div></div></div>
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
  if (typeof s.stopDistMm === "number" && document.activeElement !== $("sd")) $("sd").value = Math.round(s.stopDistMm);
  if (typeof s.passDistMm === "number" && document.activeElement !== $("pd")) $("pd").value = Math.round(s.passDistMm);
  if (typeof s.minPower === "number") { $("minp").value = Math.round(s.minPower * 100); $("minpv").textContent = Math.round(s.minPower * 100) + "%"; }
  if (typeof s.cruisePower === "number") { $("cruise").value = Math.round(s.cruisePower * 100); $("cruisev").textContent = Math.round(s.cruisePower * 100) + "%"; }
}
$("minp").oninput = e => { $("minpv").textContent = e.target.value + "%"; };
$("minp").onchange = e => send({type: "set", minPower: parseFloat(e.target.value) / 100});
$("cruise").oninput = e => { $("cruisev").textContent = e.target.value + "%"; };
$("cruise").onchange = e => send({type: "set", cruisePower: parseFloat(e.target.value) / 100});

$("setclose").onclick = () => showPage("drive");
$("trim").oninput = e => { $("trimv").textContent = e.target.value; };
$("trim").onchange = e => send({type: "set", trim: -parseInt(e.target.value, 10)});
$("sd").onchange = e => send({type: "set", stopDistMm: parseFloat(e.target.value)});
$("pd").onchange = e => send({type: "set", passDistMm: parseFloat(e.target.value)});
function robotText(r) {
  if (!r || !r.caps) return "Robot: no capability report yet (is the robot on firmware v2?)";
  const c = r.caps, sn = c.sensors || {}, on = [];
  for (const k of ["lidar", "tof", "ultrasonic", "bumperL", "bumperR"]) if (sn[k]) on.push(k);
  let tier = "Drive";
  if (sn.bumperL || sn.bumperR || sn.tof || sn.ultrasonic) tier = "Reflexes";
  if (sn.lidar) tier = "Mapping";
  let t = "Robot: " + c.name + " (firmware " + c.fw + ", " + c.drive + ")\nAttached: " + (on.length ? on.join(", ") : "none") + "\nTier: " + tier;
  const l = r.live;
  if (l) t += "\nLive: bumpers " + (l.bumpL < 0 ? "-" : l.bumpL ? "HIT" : "ok") + " / " + (l.bumpR < 0 ? "-" : l.bumpR ? "HIT" : "ok") +
    ", ToF " + (l.tofMm < 0 ? "-" : l.tofMm + " mm") + ", ultrasonic " + (l.usMm < 0 ? "-" : l.usMm + " mm") +
    ", forward block: " + l.block + ", reflex events " + l.reflexEvents;
  else t += "\nLive readings: not arriving";
  return t;
}

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
const SENSOR_LABEL = {lidar: "Lidar", camera: "Phone camera", bumper: "Bumper", tof: "ToF", ultrasonic: "Ultrasonic", imu: "IMU", depth: "Depth cam"};
const SENSOR_COLOR = {lidar: "#64ffda", camera: "#ffab40", bumper: "#ff5252", tof: "#448aff", ultrasonic: "#80cbc4", imu: "#ce93d8", depth: "#ffd54f"};
function openBot() {
  if (!botProfile) { send({type: "bot.get"}); return; }
  botDraft = JSON.parse(JSON.stringify(botProfile));
  $("botName").value = botDraft.name; $("botDrive").value = botDraft.drive;
  $("botW").value = botDraft.platform.widthMm; $("botL").value = botDraft.platform.lengthMm;
  renderBotSensors(); drawBot();
}

$("botCancel").onclick = () => showPage("drive");
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
function botTopScale() {
  const cv2 = $("botTop"), W = botDraft.platform.widthMm, L = botDraft.platform.lengthMm;
  const sc = (cv2.width - 70) / Math.max(W, L);
  return {sc, ox: cv2.width / 2 - W * sc / 2, oy: cv2.height / 2 - L * sc / 2, cw: cv2.width, ch: cv2.height};
}
function botSideScale() {
  const cv2 = $("botSide"), L = botDraft.platform.lengthMm;
  let H = 100; for (const sn of botDraft.sensors) H = Math.max(H, sn.heightMm + 40);
  const sc = Math.min((cv2.width - 70) / L, (cv2.height - 50) / H);
  return {sc, ox: cv2.width / 2 - L * sc / 2, floorY: cv2.height - 24, cw: cv2.width, ch: cv2.height};
}
function drawBot() {
  if (!botDraft) return;
  const W = botDraft.platform.widthMm, L = botDraft.platform.lengthMm;
  const c = $("botTop").getContext("2d"), t = botTopScale();
  c.clearRect(0, 0, t.cw, t.ch);
  c.fillStyle = "#2a3238"; c.fillRect(t.ox, t.oy, W * t.sc, L * t.sc);
  c.strokeStyle = "#64ffda"; c.lineWidth = 2; c.strokeRect(t.ox, t.oy, W * t.sc, L * t.sc);
  c.fillStyle = "#64ffda"; c.beginPath(); c.moveTo(t.cw / 2, t.oy - 14); c.lineTo(t.cw / 2 - 8, t.oy - 3); c.lineTo(t.cw / 2 + 8, t.oy - 3); c.closePath(); c.fill();
  c.fillStyle = "#9fb3bb"; c.font = "12px sans-serif"; c.fillText("front", t.cw / 2 + 12, t.oy - 5);
  c.fillText(W + " x " + L + " mm", t.ox, t.oy + L * t.sc + 16);
  for (const sn of botDraft.sensors) {
    const x = t.ox + sn.fromLeftMm * t.sc, y = t.oy + sn.fromFrontMm * t.sc;
    c.fillStyle = SENSOR_COLOR[sn.type]; c.beginPath(); c.arc(x, y, 10, 0, Math.PI * 2); c.fill();
    const a2 = -sn.yawDeg * Math.PI / 180 - Math.PI / 2;
    c.strokeStyle = SENSOR_COLOR[sn.type]; c.lineWidth = 2; c.beginPath(); c.moveTo(x, y); c.lineTo(x + Math.cos(a2) * 20, y + Math.sin(a2) * 20); c.stroke();
    c.fillStyle = "#e6eef0"; c.fillText(sn.name, x + 13, y + 4);
  }
  const sctx = $("botSide").getContext("2d"), v = botSideScale();
  sctx.clearRect(0, 0, v.cw, v.ch);
  sctx.strokeStyle = "#556"; sctx.beginPath(); sctx.moveTo(0, v.floorY); sctx.lineTo(v.cw, v.floorY); sctx.stroke();
  sctx.fillStyle = "#9fb3bb"; sctx.font = "12px sans-serif"; sctx.fillText("floor", 6, v.floorY - 6); sctx.fillText("front", v.cw - 44, v.floorY + 16);
  const platY = v.floorY - 60 * v.sc;
  sctx.fillStyle = "#2a3238"; sctx.fillRect(v.ox, platY - 5, L * v.sc, 5);
  for (const sn of botDraft.sensors) {
    const x = v.ox + (L - sn.fromFrontMm) * v.sc, y = v.floorY - sn.heightMm * v.sc;
    sctx.strokeStyle = "#3a4a55"; sctx.beginPath(); sctx.moveTo(x, platY); sctx.lineTo(x, y); sctx.stroke();
    sctx.fillStyle = SENSOR_COLOR[sn.type]; sctx.beginPath(); sctx.arc(x, y, 9, 0, Math.PI * 2); sctx.fill();
    sctx.fillStyle = "#e6eef0"; sctx.fillText(sn.name + " " + Math.round(sn.heightMm) + " mm", x + 12, y + 4);
  }
}
function canvasPt(canvas, e) {
  const b = canvas.getBoundingClientRect();
  return [(e.clientX - b.left) * canvas.width / b.width, (e.clientY - b.top) * canvas.height / b.height];
}
function botHit(canvas, e, mapper) {
  const [px, py] = canvasPt(canvas, e);
  let best = null, bd = 18;
  for (const sn of botDraft.sensors) { const q = mapper(sn); const d = Math.hypot(q[0] - px, q[1] - py); if (d < bd) { bd = d; best = sn; } }
  return best;
}
$("botTop").addEventListener("pointerdown", e => {
  if (!botDraft) return;
  const t = botTopScale();
  const sn = botHit($("botTop"), e, sn => [t.ox + sn.fromLeftMm * t.sc, t.oy + sn.fromFrontMm * t.sc]);
  if (sn) { botDrag = {sn, view: "top"}; $("botTop").setPointerCapture(e.pointerId); }
});
$("botTop").addEventListener("pointermove", e => {
  if (!botDrag || botDrag.view !== "top") return;
  const t = botTopScale(), [px, py] = canvasPt($("botTop"), e);
  botDrag.sn.fromLeftMm = Math.round(Math.max(-50, Math.min(botDraft.platform.widthMm + 50, (px - t.ox) / t.sc)));
  botDrag.sn.fromFrontMm = Math.round(Math.max(-50, Math.min(botDraft.platform.lengthMm + 50, (py - t.oy) / t.sc)));
  renderBotSensors(); drawBot();
});
$("botSide").addEventListener("pointerdown", e => {
  if (!botDraft) return;
  const v = botSideScale();
  const sn = botHit($("botSide"), e, sn => [v.ox + (botDraft.platform.lengthMm - sn.fromFrontMm) * v.sc, v.floorY - sn.heightMm * v.sc]);
  if (sn) { botDrag = {sn, view: "side"}; $("botSide").setPointerCapture(e.pointerId); }
});
$("botSide").addEventListener("pointermove", e => {
  if (!botDrag || botDrag.view !== "side") return;
  const v = botSideScale(), [, py] = canvasPt($("botSide"), e);
  botDrag.sn.heightMm = Math.round(Math.max(0, (v.floorY - py) / v.sc));
  renderBotSensors(); drawBot();
});
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

$("mapsClose").onclick = () => showPage("drive");
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
  if (settingsOpen) { $("capsInfo").textContent = capsText(t.caps, t.tracking); $("robotInfo").textContent = robotText(t.robot); }
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

// ---- pages ----
const PAGES = ["drive", "maps", "bot", "settings"];
let page = "drive";
function showPage(p) {
  if (page === "drive" && p !== "drive") { release(); goMode = false; if (editMode) $("editDone").onclick(); } // never drive from another page
  page = p;
  for (const n of PAGES) $(n).style.display = n === p ? (n === "drive" ? "flex" : "block") : "none";
  document.querySelectorAll(".tab").forEach(b => b.style.outline = b.dataset.page === p ? "2px solid #64ffda" : "none");
  $("mode").style.display = p === "drive" ? "" : "none";
  mapsOpen = p === "maps"; settingsOpen = p === "settings"; botOpen = p === "bot";
  if (p === "maps") { if (telem && telem.mapInfo) $("mapName").value = telem.mapInfo.name; renderActive(); send({type: "maps.list"}); }
  if (p === "settings") { syncSettings(); if (telem) $("capsInfo").textContent = capsText(telem.caps, telem.tracking); }
  if (p === "bot") { botDraft = null; openBot(); } else botDraft = null;
}
document.querySelectorAll(".tab").forEach(b => b.onclick = () => showPage(b.dataset.page));
showPage("drive");
drawStick(); connect(); requestAnimationFrame(frame);
</script></body></html>
''';
