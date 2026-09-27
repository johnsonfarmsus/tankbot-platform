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
  BrainServer({this.port = 8080, required this.onMessage, required this.onRemoteSilent});

  final int port;
  final void Function(Map<String, dynamic> msg) onMessage;
  final void Function() onRemoteSilent;

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
<span style="margin-left:auto"><button id="mode">Radar view</button></span></header>
<div id="view"><canvas id="map"></canvas></div>
<div id="banner">OBSTACLE AHEAD - forward blocked</div>
<div id="mountbar" style="display:none;text-align:center;padding:6px;font-weight:600"></div>
<div id="bottom"><canvas id="stick" width="340" height="340"></canvas>
<div class="ctl">
<div id="motors">Motors: -</div>
<label>Max speed <span id="msv"></span><input id="ms" type="range" min="0.2" max="1" step="0.1" value="0.6"></label>
<label>View range <span id="rgv"></span><input id="rg" type="range" min="1" max="12" step="0.5" value="4"></label>
<div class="row"><label><input id="obs" type="checkbox"> Obstacle stop</label></div>
<div class="row"><button id="mapping">Pause mapping</button><button id="clear">New map here</button></div>
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
    if (m.type === "telem") { telem = m; updateUi(); }
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
function drawStick() {
  const w = stick.width, r = w / 2;
  sctx.clearRect(0, 0, w, w);
  sctx.beginPath(); sctx.arc(r, r, r - 4, 0, Math.PI * 2);
  sctx.fillStyle = "rgba(255,255,255,0.06)"; sctx.fill();
  sctx.lineWidth = 4; sctx.strokeStyle = knob ? "#64ffda" : "#3a4a55"; sctx.stroke();
  const k = knob || {x: 0, y: 0};
  sctx.beginPath(); sctx.arc(r + k.x * (r - 60), r + k.y * (r - 60), 52, 0, Math.PI * 2);
  sctx.fillStyle = knob ? "#64ffda" : "#55636b"; sctx.fill();
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
  sendTimer = setInterval(() => { if (knob) send({type: "drive", f: dz(-knob.y), t: dz(knob.x)}); }, 50);
}
function stopDrive() { if (sendTimer) { clearInterval(sendTimer); sendTimer = null; } send({type: "stop"}); }
function release() { activePointer = null; knob = null; stopDrive(); drawStick(); }
stick.addEventListener("pointerdown", e => {
  activePointer = e.pointerId; stick.setPointerCapture(e.pointerId);
  knob = stickPos(e); startDrive(); drawStick();
});
stick.addEventListener("pointermove", e => { if (e.pointerId === activePointer) { knob = stickPos(e); drawStick(); } });
stick.addEventListener("pointerup", e => { if (e.pointerId === activePointer) release(); });
stick.addEventListener("pointercancel", e => { if (e.pointerId === activePointer) release(); });
document.addEventListener("visibilitychange", () => { if (document.hidden) release(); });
window.addEventListener("blur", () => { if (knob) release(); });

// ---- controls ----
$("ms").oninput = e => { $("msv").textContent = Math.round(e.target.value * 100) + "%"; send({type: "set", maxSpeed: parseFloat(e.target.value)}); };
$("rg").oninput = e => { $("rgv").textContent = e.target.value + " m"; };
$("obs").onchange = e => send({type: "set", obstacleStop: e.target.checked});
$("mapping").onclick = () => send({type: "set", mapping: !(telem && telem.settings.mapping)});
$("clear").onclick = () => { if (confirm("Start a new map with the robot's current spot as the origin?")) send({type: "clearMap"}); };
$("mode").onclick = () => { mode = mode === "map" ? "radar" : "map"; $("mode").textContent = mode === "map" ? "Radar view" : "Map view"; };
$("rgv").textContent = $("rg").value + " m";
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
  $("stat").textContent = st.scanRate.toFixed(1) + " scans/s | AR " + st.ar + " | mapped " + st.mapped + " | remotes " + st.remotes;
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
      const toS = (x, y) => [cx + (x - p.x) * ppm, cy - (y - p.y) * ppm];
      if (mapImg && mapMeta) {
        const tl = toS(mapMeta.left, mapMeta.top);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(mapImg, tl[0], tl[1], mapImg.width * mapMeta.res * ppm, mapImg.height * mapMeta.res * ppm);
      }
      ctx.strokeStyle = "rgba(255,255,255,0.07)"; ctx.lineWidth = 1; ctx.beginPath();
      for (let gx = Math.floor(p.x - range * 2); gx <= p.x + range * 2; gx++) { const a = toS(gx, 0)[0]; ctx.moveTo(a, 0); ctx.lineTo(a, H); }
      for (let gy = Math.floor(p.y - range * 2); gy <= p.y + range * 2; gy++) { const b = toS(0, gy)[1]; ctx.moveTo(0, b); ctx.lineTo(W, b); }
      ctx.stroke();
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
      drawArrow(cx, cy, p.h, p.good);
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
drawStick(); connect(); requestAnimationFrame(frame);
</script></body></html>
''';
