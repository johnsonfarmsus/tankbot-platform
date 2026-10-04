// The brain's control server: serves the remote-control page and a WebSocket.
// Remote -> brain: {"type":"drive","f":..,"t":..} at ~20 Hz, {"type":"stop"},
//                  {"type":"set", maxSpeed/obstacleStop/mapping}, {"type":"clearMap"}
// Brain -> remote: {"type":"telem",...} ~5 Hz, {"type":"map","png":base64,...} ~1 Hz
// Safety: if drive messages stop for 300 ms (or the socket closes), onRemoteSilent() fires.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:flutter/services.dart' show rootBundle;
import 'pose_client.dart' show appClockMs;

class BrainServer {
  BrainServer({this.port = 8080, required this.onMessage, required this.onRemoteSilent, this.onConnect});

  final int port;
  final void Function(Map<String, dynamic> msg) onMessage;
  final void Function() onRemoteSilent;
  final void Function()? onConnect;

  HttpServer? _server;
  String? _pageHtml; // the controller page (assets/controller.html)
  final Set<WebSocket> _clients = {};
  double _lastDriveMs = 0;
  bool _remoteDriving = false;
  Timer? _watchdog;
  String? url;
  String? error;

  int get clientCount => _clients.length;

  Future<void> start() async {
    try {
      _pageHtml = await rootBundle.loadString('assets/controller.html');
    } catch (_) {}
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
      req.response.write(_pageHtml ?? remotePageHtml);
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

/// Shown only if the controller page asset could not be loaded.
const String remotePageHtml = '<!doctype html><html><body style="font-family:sans-serif;background:#0d1215;color:#e8eef0;padding:24px">'
    '<h2>Controller page missing</h2><p>The app was built without assets/controller.html.</p></body></html>';
