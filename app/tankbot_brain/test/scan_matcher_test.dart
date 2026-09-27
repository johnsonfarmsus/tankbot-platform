import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/lidar_client.dart';
import 'package:tankbot_brain/occupancy_grid.dart';
import 'package:tankbot_brain/pose_client.dart';
import 'package:tankbot_brain/scan_matcher.dart';

// A 6 x 4 m room with an L-shaped cabinet and a pillar, so it has no symmetry.
final walls = <List<double>>[
  [-3, -2, 3, -2], [3, -2, 3, 2], [3, 2, -3, 2], [-3, 2, -3, -2],
  [1.0, 0.5, 2.2, 0.5], [2.2, 0.5, 2.2, 1.2], // L cabinet
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

/// Simulated lidar scan from (x, y, h): angles clockwise from the robot's front.
List<LidarPoint> simScan(double x, double y, double h, math.Random rnd) {
  final pts = <LidarPoint>[];
  for (var a = 0.0; a < 360; a += 0.8) {
    final phi = h - a * math.pi / 180; // clockwise lidar angle -> world direction
    final d = rayHit(x, y, math.cos(phi), math.sin(phi));
    if (d == null || d > 8) continue;
    pts.add(LidarPoint(a, (d + (rnd.nextDouble() - 0.5) * 0.02) * 1000, 40)); // ~1 cm noise
  }
  return pts;
}

void main() {
  final rnd = math.Random(7);
  final grid = OccupancyGrid();
  // build the map from a few viewpoints
  for (final p in [[0.0, 0.0, 1.57], [-1.5, 1.0, 0.3], [1.5, -1.0, 2.5], [0.5, 1.2, -1.0], [-2.2, -1.5, 0.8]]) {
    for (var i = 0; i < 3; i++) {
      grid.integrate(Pose(0, p[0], p[1], p[2], true), simScan(p[0], p[1], p[2], rnd));
    }
  }
  final matcher = ScanMatcher(grid);

  test('local match corrects a drifted pose', () {
    const tx = 0.8, ty = -0.6, th = 0.9;
    final pts = ScanMatcher.robotFrame(simScan(tx, ty, th, rnd), stride: 2);
    final r = matcher.local(pts, tx + 0.05, ty - 0.04, th + 0.03);
    // ignore: avoid_print
    print('local: ${r.x.toStringAsFixed(3)}, ${r.y.toStringAsFixed(3)}, ${r.h.toStringAsFixed(3)} hit ${r.hitRatio.toStringAsFixed(2)}');
    expect((r.x - tx).abs(), lessThan(0.03));
    expect((r.y - ty).abs(), lessThan(0.03));
    expect((r.h - th).abs(), lessThan(0.02));
    expect(r.hitRatio, greaterThan(0.5));
  });

  test('global relocalisation finds a lost robot', () async {
    const tx = -0.9, ty = 0.4, th = -2.2;
    final pts = ScanMatcher.robotFrame(simScan(tx, ty, th, rnd));
    final sw = Stopwatch()..start();
    final res = await matcher.global(pts);
    final b = res.first;
    var dh = b.h - th;
    while (dh > math.pi) { dh -= 2 * math.pi; }
    while (dh < -math.pi) { dh += 2 * math.pi; }
    // ignore: avoid_print
    print('global: ${b.x.toStringAsFixed(3)}, ${b.y.toStringAsFixed(3)}, h err ${dh.toStringAsFixed(3)} hit ${b.hitRatio.toStringAsFixed(2)} '
        '(${res.length} candidates, ${sw.elapsedMilliseconds} ms); runner-up score ratio '
        '${res.length > 1 ? (res[1].score / b.score).toStringAsFixed(2) : "-"}');
    expect((b.x - tx).abs(), lessThan(0.05));
    expect((b.y - ty).abs(), lessThan(0.05));
    expect(dh.abs(), lessThan(0.03));
  });

  test('pose correction maps raw camera poses onto the map', () {
    final c = PoseCorrection();
    final raw = Pose(0, 1.0, 2.0, 0.5, true);
    c.setSoThat(raw, -0.3, 0.7, 2.0);
    final out = c.apply(raw);
    expect((out.x + 0.3).abs(), lessThan(1e-9));
    expect((out.y - 0.7).abs(), lessThan(1e-9));
    expect((out.heading - 2.0).abs(), lessThan(1e-9));
  });
}
