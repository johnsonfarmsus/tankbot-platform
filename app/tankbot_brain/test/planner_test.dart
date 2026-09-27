import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/lidar_client.dart';
import 'package:tankbot_brain/occupancy_grid.dart';
import 'package:tankbot_brain/planner.dart';
import 'package:tankbot_brain/pose_client.dart';

final walls = <List<double>>[
  [-3, -2, 3, -2], [3, -2, 3, 2], [3, 2, -3, 2], [-3, 2, -3, -2],
  [1.0, 0.5, 2.2, 0.5], [2.2, 0.5, 2.2, 1.2], [2.2, 1.2, 1.0, 1.2], [1.0, 1.2, 1.0, 0.5], // cabinet (closed box)
  [-1.8, -1.2, -1.5, -1.2], [-1.5, -1.2, -1.5, -0.9], [-1.5, -0.9, -1.8, -0.9], [-1.8, -0.9, -1.8, -1.2], // pillar
];

double? rayHit(double ox, double oy, double dx, double dy) {
  double? best;
  for (final w in walls) {
    final x1 = w[0], y1 = w[1], x2 = w[2], y2 = w[3];
    final ex = x2 - x1, ey = y2 - y1;
    final den = dx * ey - dy * ex;
    if (den.abs() < 1e-9) continue;
    final t = ((x1 - ox) * ey - (y1 - oy) * ex) / den;
    final u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && (best == null || t < best)) best = t;
  }
  return best;
}

List<LidarPoint> simScan(double x, double y, double h) {
  final pts = <LidarPoint>[];
  for (var a = 0.0; a < 360; a += 0.8) {
    final phi = h - a * math.pi / 180;
    final d = rayHit(x, y, math.cos(phi), math.sin(phi));
    if (d == null || d > 8) continue;
    pts.add(LidarPoint(a, d * 1000, 40));
  }
  return pts;
}

double wallClearance(double x, double y) {
  var best = double.infinity;
  for (final w in walls) {
    final ax = w[0], ay = w[1], bx = w[2], by = w[3];
    final dx = bx - ax, dy = by - ay;
    var t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy);
    t = t.clamp(0.0, 1.0);
    best = math.min(best, math.sqrt(math.pow(x - (ax + t * dx), 2) + math.pow(y - (ay + t * dy), 2)));
  }
  return best;
}

void main() {
  final grid = OccupancyGrid();
  for (var x = -2.4; x <= 2.5; x += 0.8) {
    for (var y = -1.6; y <= 1.7; y += 0.8) {
      if (wallClearance(x, y) < 0.25 || (x > 0.9 && x < 2.3 && y > 0.4 && y < 1.3)) continue;
      grid.integrate(Pose(0, x, y, 0, true), simScan(x, y, 0));
    }
  }

  test('plans around the cabinet with full clearance', () {
    final r = Planner.plan(grid, [], [], -2.3, -1.5, 2.6, 1.6);
    expect(r.error, isNull);
    var minClear = double.infinity, len = 0.0;
    for (var i = 1; i < r.path.length; i++) {
      final a = r.path[i - 1], b = r.path[i];
      len += (b - a).distance;
      for (var k = 0; k <= 20; k++) {
        final q = Offset.lerp(a, b, k / 20)!;
        if ((q - r.path.first).distance < Planner.startEscapeM) continue; // start may be tight
        minClear = math.min(minClear, wallClearance(q.dx, q.dy));
      }
    }
    // ignore: avoid_print
    print('route: ${r.path.length} waypoints, ${len.toStringAsFixed(2)} m, closest wall ${(minClear * 100).toStringAsFixed(0)} cm');
    expect(minClear, greaterThan(Planner.robotRadiusM - 0.05));
  });

  test('refuses a goal inside the cabinet', () {
    final r = Planner.plan(grid, [], [], -2.3, -1.5, 1.6, 0.85);
    expect(r.error, isNotNull);
  });

  test('a no-go line across the room blocks the trip', () {
    final r = Planner.plan(grid, [[0.0, -2.1, 0.0, 2.1]], [], -2.3, -1.5, 2.6, -1.5);
    expect(r.error, isNotNull);
  });

  test('live obstacles force a detour', () {
    final plain = Planner.plan(grid, [], [], -2.3, -1.5, 2.6, -1.5);
    final blocker = [for (var y = -2.0; y <= -0.6; y += 0.03) Offset(0.3, y)];
    final detour = Planner.plan(grid, [], blocker, -2.3, -1.5, 2.6, -1.5);
    expect(detour.error, isNull);
    double len(List<Offset> p) { var l = 0.0; for (var i = 1; i < p.length; i++) { l += (p[i] - p[i - 1]).distance; } return l; }
    // ignore: avoid_print
    print('straight ${len(plain.path).toStringAsFixed(2)} m vs detour ${len(detour.path).toStringAsFixed(2)} m');
    expect(len(detour.path), greaterThan(len(plain.path) + 0.3));
  });
}
