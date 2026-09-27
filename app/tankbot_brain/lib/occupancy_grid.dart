// Top-down occupancy grid built from lidar scans placed at the robot's pose.
// Each cell holds log-odds: negative = free, positive = occupied, ~0 = unknown.
import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'lidar_client.dart';
import 'pose_client.dart';

class OccupancyGrid {
  OccupancyGrid({this.resolution = 0.05, this.size = 800});

  final double resolution; // metres per cell (5 cm)
  final int size; // cells per side (800 x 5 cm = 40 m)
  late final Float32List _lo = Float32List(size * size);

  static const double _free = -0.4, _hit = 0.85, _min = -4.0, _max = 4.0;
  static const double maxRangeM = 8.0, minRangeM = 0.15;

  // Region that has been touched, in cells (for fast rendering).
  int minCx = 1 << 30, maxCx = -1, minCy = 1 << 30, maxCy = -1;
  bool dirty = false;
  int scansIntegrated = 0;

  int _c(double m) => (m / resolution).floor() + size ~/ 2;
  double cellToWorld(int c) => (c - size ~/ 2) * resolution;

  // ---- likelihood field: how close each cell is to the nearest wall (for scan matching) ----
  static const double fieldSigmaM = 0.06, fieldMaxM = 0.3;
  Float32List? _field;
  int _fx0 = 0, _fy0 = 0, _fw = 0, _fh = 0, _fieldScans = -1;

  /// Rebuild the field if the map changed (cheap: two passes over the mapped area).
  void ensureField({bool force = false}) {
    if (maxCx < 0) return;
    if (!force && _field != null && _fieldScans == scansIntegrated) return;
    final m = (fieldMaxM / resolution).ceil() + 1;
    final x0 = math.max(0, minCx - m), x1 = math.min(size - 1, maxCx + m);
    final y0 = math.max(0, minCy - m), y1 = math.min(size - 1, maxCy + m);
    final w = x1 - x0 + 1, h = y1 - y0 + 1;
    const inf = 1e9;
    final d = Float32List(w * h);
    for (var y = 0; y < h; y++) {
      final row = (y0 + y) * size;
      for (var x = 0; x < w; x++) {
        d[y * w + x] = _lo[row + x0 + x] > 0.5 ? 0 : inf;
      }
    }
    const a = 1.0, b = 1.41421356;
    // chamfer distance transform: forward pass then backward pass
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        final i = y * w + x;
        var v = d[i];
        if (x > 0 && d[i - 1] + a < v) v = d[i - 1] + a;
        if (y > 0) {
          if (d[i - w] + a < v) v = d[i - w] + a;
          if (x > 0 && d[i - w - 1] + b < v) v = d[i - w - 1] + b;
          if (x < w - 1 && d[i - w + 1] + b < v) v = d[i - w + 1] + b;
        }
        d[i] = v;
      }
    }
    for (var y = h - 1; y >= 0; y--) {
      for (var x = w - 1; x >= 0; x--) {
        final i = y * w + x;
        var v = d[i];
        if (x < w - 1 && d[i + 1] + a < v) v = d[i + 1] + a;
        if (y < h - 1) {
          if (d[i + w] + a < v) v = d[i + w] + a;
          if (x < w - 1 && d[i + w + 1] + b < v) v = d[i + w + 1] + b;
          if (x > 0 && d[i + w - 1] + b < v) v = d[i + w - 1] + b;
        }
        d[i] = v;
      }
    }
    final twoSig2 = 2 * fieldSigmaM * fieldSigmaM;
    final maxCells = fieldMaxM / resolution;
    for (var i = 0; i < d.length; i++) {
      final dc = d[i];
      if (dc > maxCells) {
        d[i] = 0;
      } else {
        final dm = dc * resolution;
        d[i] = math.exp(-dm * dm / twoSig2);
      }
    }
    _field = d;
    _fx0 = x0;
    _fy0 = y0;
    _fw = w;
    _fh = h;
    _fieldScans = scansIntegrated;
  }

  /// 0..1: 1 on a wall, falling off over a few centimetres.
  double likelihood(double wx, double wy) {
    final f = _field;
    if (f == null) return 0;
    final x = (wx / resolution).floor() + size ~/ 2 - _fx0, y = (wy / resolution).floor() + size ~/ 2 - _fy0;
    if (x < 0 || y < 0 || x >= _fw || y >= _fh) return 0;
    return f[y * _fw + x];
  }

  /// Log-odds at a world position (0 = unknown / outside the grid).
  double at(double wx, double wy) {
    final cx = (wx / resolution).floor() + size ~/ 2, cy = (wy / resolution).floor() + size ~/ 2;
    if (cx < 0 || cy < 0 || cx >= size || cy >= size) return 0;
    return _lo[cy * size + cx];
  }

  /// Centres of known-free cells, sampled every `stepM` metres (candidate robot positions).
  List<ui.Offset> freeCellCentres(double stepM) {
    final out = <ui.Offset>[];
    if (maxCx < 0) return out;
    final st = math.max(1, (stepM / resolution).round());
    for (var cy = minCy; cy <= maxCy; cy += st) {
      for (var cx = minCx; cx <= maxCx; cx += st) {
        if (_lo[cy * size + cx] < -0.5) {
          out.add(ui.Offset(cellToWorld(cx) + resolution / 2, cellToWorld(cy) + resolution / 2));
        }
      }
    }
    return out;
  }

  void clear() {
    _lo.fillRange(0, _lo.length, 0);
    minCx = 1 << 30; maxCx = -1; minCy = 1 << 30; maxCy = -1;
    scansIntegrated = 0;
    _field = null;
    _fieldScans = -1;
    dirty = true;
  }

  void _touch(int cx, int cy) {
    if (cx < minCx) minCx = cx;
    if (cx > maxCx) maxCx = cx;
    if (cy < minCy) minCy = cy;
    if (cy > maxCy) maxCy = cy;
  }

  void _add(int cx, int cy, double v) {
    if (cx < 0 || cy < 0 || cx >= size || cy >= size) return;
    final i = cy * size + cx;
    final n = _lo[i] + v;
    _lo[i] = n < _min ? _min : (n > _max ? _max : n);
    _touch(cx, cy);
  }

  /// Adds one scan. Lidar angles are clockwise from the robot's front.
  /// lidarFwdM / lidarLeftM: lidar position relative to the phone (0 until measured).
  void integrate(Pose pose, List<LidarPoint> pts, {double lidarFwdM = 0, double lidarLeftM = 0}) {
    final ch = math.cos(pose.heading), sh = math.sin(pose.heading);
    final ox = pose.x + ch * lidarFwdM - sh * lidarLeftM;
    final oy = pose.y + sh * lidarFwdM + ch * lidarLeftM;
    final x0 = _c(ox), y0 = _c(oy);
    for (final p in pts) {
      final d = p.distMm / 1000.0;
      if (d < minRangeM) continue;
      final hit = d <= maxRangeM;
      final r = hit ? d : maxRangeM;
      final a = p.angleDeg * math.pi / 180.0;
      final fwd = r * math.cos(a), left = -r * math.sin(a); // clockwise angle -> right is negative left
      final wx = ox + ch * fwd - sh * left;
      final wy = oy + sh * fwd + ch * left;
      _ray(x0, y0, _c(wx), _c(wy), hit);
    }
    scansIntegrated++;
    dirty = true;
  }

  // Bresenham line: cells along the beam are free, the end cell is occupied.
  void _ray(int x0, int y0, int x1, int y1, bool hit) {
    var dx = (x1 - x0).abs(), dy = -(y1 - y0).abs();
    final sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    var err = dx + dy, x = x0, y = y0;
    while (true) {
      if (x == x1 && y == y1) break;
      _add(x, y, _free);
      final e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
    if (hit) _add(x1, y1, _hit);
  }

  /// Renders the touched region. Returns the image and its world bounds
  /// (left, top = max y, cell size). Row 0 of the image is the highest y.
  Future<MapImage?> render() async {
    if (maxCx < 0) return null;
    final x0 = math.max(0, minCx - 2), x1 = math.min(size - 1, maxCx + 2);
    final y0 = math.max(0, minCy - 2), y1 = math.min(size - 1, maxCy + 2);
    final w = x1 - x0 + 1, h = y1 - y0 + 1;
    final px = Uint8List(w * h * 4);
    var o = 0;
    for (var cy = y1; cy >= y0; cy--) {
      final row = cy * size;
      for (var cx = x0; cx <= x1; cx++) {
        final v = _lo[row + cx];
        if (v > 0.5) {
          px[o] = 100; px[o + 1] = 255; px[o + 2] = 218; px[o + 3] = 255; // occupied: teal
        } else if (v < -0.5) {
          px[o] = 70; px[o + 1] = 78; px[o + 2] = 84; px[o + 3] = 255; // free: slate
        } else {
          px[o + 3] = 0; // unknown: transparent
        }
        o += 4;
      }
    }
    final c = Completer<ui.Image>();
    ui.decodeImageFromPixels(px, w, h, ui.PixelFormat.rgba8888, c.complete);
    final img = await c.future;
    dirty = false;
    return MapImage(img, cellToWorld(x0), cellToWorld(y1 + 1), resolution);
  }
}

class MapImage {
  final ui.Image image;
  final double leftM, topM, resolution;
  MapImage(this.image, this.leftM, this.topM, this.resolution);
}
