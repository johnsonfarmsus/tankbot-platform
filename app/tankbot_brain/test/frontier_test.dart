import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/frontier.dart';

/// A drawn floor plan: 5 cm cells, everything unexplored until painted.
class Plan {
  Plan(this.wM, this.hM) : w = (wM / res).round(), h = (hM / res).round() {
    lo = Float32List(w * h);
  }
  static const res = 0.05;
  final double wM, hM;
  final int w, h;
  late Float32List lo;
  final rnd = math.Random(3);

  int c(double m) => (m / res).floor();
  double at(int cx, int cy) => (cx < 0 || cy < 0 || cx >= w || cy >= h) ? 0 : lo[cy * w + cx];
  void set(int cx, int cy, double v) {
    if (cx >= 0 && cy >= 0 && cx < w && cy < h) lo[cy * w + cx] = v;
  }

  /// open floor inside a rectangle (metres)
  void floor(double x0, double y0, double x1, double y1) {
    for (var cy = c(y0); cy < c(y1); cy++) {
      for (var cx = c(x0); cx < c(x1); cx++) {
        set(cx, cy, -3);
      }
    }
  }

  /// a jagged lidar wall from (x0,y0) to (x1,y1), with gaps [from, to] in metres along it
  void wall(double x0, double y0, double x1, double y1, {List<(double, double)> gaps = const []}) {
    final len = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    for (var d = 0.0; d <= len; d += res / 2) {
      if (gaps.any((g) => d >= g.$1 && d <= g.$2)) continue;
      final f = d / len, jag = (rnd.nextInt(3) - 1) * res; // +-1 cell of jaggedness
      final x = x0 + (x1 - x0) * f + (y1 == y0 ? 0 : jag), y = y0 + (y1 - y0) * f + (x1 == x0 ? 0 : jag);
      if (rnd.nextDouble() < 0.12) continue; // missing wall pixels, like a real lidar wall
      set(c(x), c(y), 3);
    }
  }

  FrontierPick? find(double rx, double ry, {List<(double, double)> skip = const []}) => FrontierFinder.next(
      lo: at, x0: 0, y0: 0, x1: w - 1, y1: h - 1, res: res, cellToWorld: (i) => i * res,
      robotX: rx, robotY: ry, robotCx: c(rx), robotCy: c(ry), skip: skip);
}

void main() {
  test('a fully mapped room with jagged walls and small gaps is complete', () {
    final p = Plan(8, 7);
    p.floor(1.05, 1.05, 5.95, 4.95);
    p.wall(1, 1, 6, 1, gaps: [(1.0, 1.4), (3.0, 3.3)]); // 40 and 30 cm gaps
    p.wall(6, 1, 6, 5, gaps: [(2.0, 2.45)]);
    p.wall(6, 5, 1, 5);
    p.wall(1, 5, 1, 1, gaps: [(0.5, 0.9)]);
    final r = p.find(3, 3);
    // ignore: avoid_print
    print('closed room: $r');
    expect(r, isNull);
  });

  test('a doorway into unexplored space is found', () {
    final p = Plan(10, 7);
    p.floor(1.05, 1.05, 5.95, 4.95);
    p.floor(5.9, 2.55, 6.6, 3.4); // the lidar saw a little way through the doorway
    p.wall(1, 1, 6, 1);
    p.wall(6, 1, 6, 5, gaps: [(1.55, 2.45)]); // 90 cm doorway at y = 2.55..3.45
    p.wall(6, 5, 1, 5);
    p.wall(1, 5, 1, 1, gaps: [(1.0, 1.35)]); // plus a 35 cm crack that must be ignored
    final r = p.find(2, 2);
    // ignore: avoid_print
    print('doorway: $r');
    expect(r, isNotNull);
    expect(r!.x, greaterThan(5.7));
    expect(r.y, inInclusiveRange(2.5, 3.5));
  });

  test('it finishes its own room before going through a door', () {
    // a 7 x 4 m room; only the left part is explored. A doorway near the robot leads out.
    final p = Plan(12, 8);
    p.floor(1.05, 1.05, 4.0, 4.95); // explored part of the room
    p.wall(1, 1, 4.0, 1); // walls seen so far
    p.wall(1, 5, 4.0, 5);
    p.wall(1, 1, 1, 5, gaps: [(1.0, 1.9)]); // 90 cm doorway in the left wall
    p.floor(0.3, 2.05, 1.1, 2.85); // a peek through the doorway
    final r = p.find(1.8, 2.4); // the robot stands right by the door
    // ignore: avoid_print
    print('half-explored room: $r');
    expect(r, isNotNull);
    expect(r!.inRoom, isTrue);
    expect(r.x, greaterThan(3.5)); // the unexplored rest of the room, not the door
  });

  test('space seen through a gap too narrow for the robot is not a target', () {
    final p = Plan(10, 7);
    p.floor(1.05, 1.05, 5.95, 4.95);
    p.wall(1, 1, 6, 1);
    p.wall(6, 1, 6, 5, gaps: [(1.7, 2.05)]); // 35 cm gap (under furniture)...
    p.floor(6.0, 2.7, 8.5, 3.05); // ...and the lidar saw a long strip beyond it
    p.wall(6, 5, 1, 5);
    p.wall(1, 5, 1, 1);
    final r = p.find(3, 3);
    // ignore: avoid_print
    print('seen through a narrow gap: $r');
    expect(r, isNull);
  });
}
