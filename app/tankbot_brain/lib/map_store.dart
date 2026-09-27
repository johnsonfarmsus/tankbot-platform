// Saved maps: each map is a list of keyframes (robot pose + full lidar scan).
// The occupancy grid is rebuilt from keyframes, so maps can later be corrected
// (scan matching, loop closure) and redrawn.
//
// On disk (app Documents/maps/<id>/):
//   meta.json      {id, name, created, updated, keyframes, sizeM, lidarFwdM, lidarLeftM, version}
//   keyframes.bin  'TKF1' u32 count, then per keyframe:
//                  f64 unix-ms, f32 x, f32 y, f32 heading, u16 n, n x (u16 angle_q6, u16 dist_q2)
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/services.dart';
import 'lidar_client.dart';

class Keyframe {
  final double t; // unix ms
  double x, y, h; // map frame (metres, radians); corrected by loop closing
  final Uint16List raw; // interleaved angle_q6, dist_q2

  Keyframe(this.t, this.x, this.y, this.h, this.raw);

  factory Keyframe.fromScan(double x, double y, double h, List<LidarPoint> pts) {
    final raw = Uint16List(pts.length * 2);
    for (var i = 0; i < pts.length; i++) {
      raw[i * 2] = (pts[i].angleDeg * 64).round().clamp(0, 65535);
      raw[i * 2 + 1] = (pts[i].distMm * 4).round().clamp(0, 65535);
    }
    return Keyframe(DateTime.now().millisecondsSinceEpoch.toDouble(), x, y, h, raw);
  }

  List<LidarPoint> points() =>
      [for (var i = 0; i + 1 < raw.length; i += 2) LidarPoint(raw[i] / 64.0, raw[i + 1] / 4.0, 0)];
}

class MapSession {
  MapSession(this.id, this.name, this.created, {this.lidarFwdM = 0, this.lidarLeftM = 0});

  final String id;
  String name;
  final DateTime created;
  DateTime updated = DateTime.now();
  final List<Keyframe> keyframes = [];
  double lidarFwdM, lidarLeftM;
  int savedCount = 0;
  DateTime? savedAt;
  bool renamed = false;
  bool edited = false; // keyframes corrected or map edited since the last save
  /// Map edits: {'type': 'erase', x, y, r} and {'type': 'nogo', x1, y1, x2, y2}, each with id + stroke.
  final List<Map<String, dynamic>> edits = [];
  int nextEditId = 1;

  bool get unsaved => keyframes.length != savedCount || renamed || edited;

  static MapSession fresh({double lidarFwdM = 0, double lidarLeftM = 0}) {
    final now = DateTime.now();
    String two(int v) => v.toString().padLeft(2, '0');
    return MapSession('m${now.millisecondsSinceEpoch}',
        'Map ${now.year}-${two(now.month)}-${two(now.day)} ${two(now.hour)}:${two(now.minute)}', now,
        lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
  }

  /// Rough extent of the robot's travel (metres), for the map list.
  String sizeText() {
    if (keyframes.isEmpty) return '';
    var x0 = keyframes.first.x, x1 = x0, y0 = keyframes.first.y, y1 = y0;
    for (final k in keyframes) {
      x0 = math.min(x0, k.x);
      x1 = math.max(x1, k.x);
      y0 = math.min(y0, k.y);
      y1 = math.max(y1, k.y);
    }
    return 'travelled area ${(x1 - x0).toStringAsFixed(1)} x ${(y1 - y0).toStringAsFixed(1)} m';
  }

  Map<String, dynamic> meta() => {
        'id': id,
        'name': name,
        'created': created.millisecondsSinceEpoch,
        'updated': updated.millisecondsSinceEpoch,
        'keyframes': keyframes.length,
        'sizeM': sizeText(),
        'lidarFwdM': lidarFwdM,
        'lidarLeftM': lidarLeftM,
        'version': 1,
      };
}

class MapStore {
  static const _native = MethodChannel('tankbot/arkit');
  Directory? _root;

  Future<Directory?> _dir() async {
    if (_root != null) return _root;
    try {
      final p = await _native.invokeMethod<String>('documentsDir');
      if (p == null) return null;
      final d = Directory('$p/maps');
      await d.create(recursive: true);
      _root = d;
    } catch (_) {
      return null;
    }
    return _root;
  }

  Future<List<Map<String, dynamic>>> list() async {
    final root = await _dir();
    if (root == null) return [];
    final out = <Map<String, dynamic>>[];
    await for (final e in root.list()) {
      if (e is! Directory) continue;
      try {
        final meta = jsonDecode(await File('${e.path}/meta.json').readAsString()) as Map<String, dynamic>;
        out.add(meta);
      } catch (_) {}
    }
    out.sort((a, b) => ((b['updated'] ?? 0) as num).compareTo((a['updated'] ?? 0) as num));
    return out;
  }

  Future<bool> save(MapSession m) async {
    final root = await _dir();
    if (root == null) return false;
    final d = Directory('${root.path}/${m.id}');
    await d.create(recursive: true);
    final count = m.keyframes.length;
    final bytes = _encode(m.keyframes.sublist(0, count));
    // write to temp files then rename, so a crash mid-save never corrupts a map
    final kTmp = File('${d.path}/keyframes.bin.tmp');
    await kTmp.writeAsBytes(bytes, flush: true);
    await kTmp.rename('${d.path}/keyframes.bin');
    m.updated = DateTime.now();
    final mTmp = File('${d.path}/meta.json.tmp');
    await mTmp.writeAsString(jsonEncode(m.meta()), flush: true);
    await mTmp.rename('${d.path}/meta.json');
    final eTmp = File('${d.path}/edits.json.tmp');
    await eTmp.writeAsString(jsonEncode({'next': m.nextEditId, 'edits': m.edits}), flush: true);
    await eTmp.rename('${d.path}/edits.json');
    m.savedCount = count;
    m.renamed = false;
    m.edited = false;
    m.savedAt = DateTime.now();
    return true;
  }

  Future<MapSession?> load(String id) async {
    final root = await _dir();
    if (root == null) return null;
    try {
      final meta = jsonDecode(await File('${root.path}/$id/meta.json').readAsString()) as Map<String, dynamic>;
      final m = MapSession(
        meta['id'] as String,
        meta['name'] as String,
        DateTime.fromMillisecondsSinceEpoch((meta['created'] as num).toInt()),
        lidarFwdM: (meta['lidarFwdM'] as num?)?.toDouble() ?? 0,
        lidarLeftM: (meta['lidarLeftM'] as num?)?.toDouble() ?? 0,
      );
      m.keyframes.addAll(_decode(await File('${root.path}/$id/keyframes.bin').readAsBytes()));
      final ef = File('${root.path}/$id/edits.json');
      if (await ef.exists()) {
        final e = jsonDecode(await ef.readAsString()) as Map<String, dynamic>;
        m.nextEditId = (e['next'] as num?)?.toInt() ?? 1;
        for (final x in (e['edits'] as List? ?? [])) {
          m.edits.add(Map<String, dynamic>.from(x as Map));
        }
      }
      m.savedCount = m.keyframes.length;
      m.savedAt = DateTime.now();
      return m;
    } catch (_) {
      return null;
    }
  }

  Future<void> setLast(String id) async {
    final root = await _dir();
    if (root == null) return;
    try {
      await File('${root.path}/last.json').writeAsString(jsonEncode({'id': id}));
    } catch (_) {}
  }

  Future<String?> getLast() async {
    final root = await _dir();
    if (root == null) return null;
    try {
      final m = jsonDecode(await File('${root.path}/last.json').readAsString()) as Map<String, dynamic>;
      final id = m['id'] as String?;
      if (id == null || !await File('${root.path}/$id/meta.json').exists()) return null;
      return id;
    } catch (_) {
      return null;
    }
  }

  Future<void> delete(String id) async {
    final root = await _dir();
    if (root == null || id.contains('/') || id.contains('..')) return;
    final d = Directory('${root.path}/$id');
    if (await d.exists()) await d.delete(recursive: true);
  }

  static Uint8List _encode(List<Keyframe> kfs) {
    var size = 8;
    for (final k in kfs) {
      size += 8 + 12 + 2 + k.raw.length * 2;
    }
    final bd = ByteData(size);
    var o = 0;
    for (final c in 'TKF1'.codeUnits) {
      bd.setUint8(o++, c);
    }
    bd.setUint32(o, kfs.length, Endian.little);
    o += 4;
    for (final k in kfs) {
      bd.setFloat64(o, k.t, Endian.little);
      o += 8;
      bd.setFloat32(o, k.x, Endian.little);
      o += 4;
      bd.setFloat32(o, k.y, Endian.little);
      o += 4;
      bd.setFloat32(o, k.h, Endian.little);
      o += 4;
      bd.setUint16(o, k.raw.length ~/ 2, Endian.little);
      o += 2;
      for (final v in k.raw) {
        bd.setUint16(o, v, Endian.little);
        o += 2;
      }
    }
    return bd.buffer.asUint8List();
  }

  static List<Keyframe> _decode(Uint8List data) {
    final bd = ByteData.sublistView(data);
    if (data.length < 8 || String.fromCharCodes(data.sublist(0, 4)) != 'TKF1') return [];
    final count = bd.getUint32(4, Endian.little);
    var o = 8;
    final out = <Keyframe>[];
    for (var i = 0; i < count && o + 22 <= data.length; i++) {
      final t = bd.getFloat64(o, Endian.little);
      final x = bd.getFloat32(o + 8, Endian.little);
      final y = bd.getFloat32(o + 12, Endian.little);
      final h = bd.getFloat32(o + 16, Endian.little);
      final n = bd.getUint16(o + 20, Endian.little);
      o += 22;
      if (o + n * 4 > data.length) break;
      final raw = Uint16List(n * 2);
      for (var j = 0; j < n * 2; j++) {
        raw[j] = bd.getUint16(o, Endian.little);
        o += 2;
      }
      out.add(Keyframe(t, x, y, h, raw));
    }
    return out;
  }
}
