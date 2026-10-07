// The robot's storage for its brains (firmware v3.1): robot-level settings and a compact copy of the
// most recent map. Plain HTTP to the ESP32; every call times out quickly and fails soft (null / false).
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

class RobotStore {
  static const _timeout = Duration(seconds: 6);

  static Future<HttpClientResponse?> _send(String method, String ip, String path, {List<int>? body, String type = 'text/plain'}) async {
    final c = HttpClient()..connectionTimeout = const Duration(seconds: 4);
    try {
      final req = await c.openUrl(method, Uri.parse('http://$ip$path')).timeout(_timeout);
      if (body != null) {
        req.headers.contentType = ContentType.parse(type);
        req.contentLength = body.length;
        req.add(body);
      }
      return await req.close().timeout(_timeout);
    } catch (_) {
      return null;
    } finally {
      c.close();
    }
  }

  static Future<Uint8List?> _bytes(HttpClientResponse? r) async {
    if (r == null || r.statusCode != 200) return null;
    try {
      final b = BytesBuilder(copy: false);
      await for (final chunk in r.timeout(const Duration(seconds: 20))) {
        b.add(chunk);
      }
      return b.toBytes();
    } catch (_) {
      return null;
    }
  }

  static Future<Map<String, dynamic>?> _json(String ip, String path) async {
    final b = await _bytes(await _send('GET', ip, path));
    if (b == null) return null;
    try {
      final v = jsonDecode(utf8.decode(b));
      return v is Map<String, dynamic> ? v : null;
    } catch (_) {
      return null;
    }
  }

  static Future<bool> _ok(String ip, String path, List<int> body, {String type = 'text/plain'}) async {
    final r = await _send('POST', ip, path, body: body, type: type);
    if (r == null) return false;
    await r.drain<void>().catchError((_) {});
    return r.statusCode == 200;
  }

  /// The brain the robot heard from most recently: {"url": ...} or {}.
  static Future<Map<String, dynamic>?> brain(String ip) => _json(ip, '/api/brain');

  /// Robot-level settings ({} when none have been stored yet).
  static Future<Map<String, dynamic>?> settings(String ip) => _json(ip, '/api/settings');
  static Future<bool> putSettings(String ip, Map<String, dynamic> s) =>
      _ok(ip, '/api/settings', utf8.encode(jsonEncode(s)), type: 'application/json');

  /// {id, name, updated, size, keyframes} of the stored map, or {} when there is none.
  static Future<Map<String, dynamic>?> mapInfo(String ip) => _json(ip, '/api/map/info');
  static Future<Uint8List?> downloadMap(String ip) async => _bytes(await _send('GET', ip, '/api/map'));

  /// Chunked upload; the robot only replaces its stored map once every byte has arrived.
  static Future<bool> uploadMap(String ip, Uint8List bytes, Map<String, dynamic> info) async {
    if (!await _ok(ip, '/api/map/begin?size=${bytes.length}', const [])) return false;
    for (var off = 0; off < bytes.length; off += 6000) {
      final end = off + 6000 < bytes.length ? off + 6000 : bytes.length;
      if (!await _ok(ip, '/api/map/chunk', ascii.encode(base64Encode(bytes.sublist(off, end))))) return false;
    }
    return _ok(ip, '/api/map/end', utf8.encode(jsonEncode(info)), type: 'application/json');
  }
}
