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

  void clear() {
    _lo.fillRange(0, _lo.length, 0);
    minCx = 1 << 30; maxCx = -1; minCy = 1 << 30; maxCy = -1;
    scansIntegrated = 0;
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
