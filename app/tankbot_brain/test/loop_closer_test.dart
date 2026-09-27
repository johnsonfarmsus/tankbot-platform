import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/lidar_client.dart';
import 'package:tankbot_brain/loop_closer.dart';
import 'package:tankbot_brain/map_store.dart';

// Same 6 x 4 m room as the scan matcher test.
final walls = <List<double>>[
  [-3, -2, 3, -2], [3, -2, 3, 2], [3, 2, -3, 2], [-3, 2, -3, -2],
  [1.0, 0.5, 2.2, 0.5], [2.2, 0.5, 2.2, 1.2],
  [-1.8, -1.2, -1.5, -1.2], [-1.5, -1.2, -1.5, -0.9], [-1.5, -0.9, -1.8, -0.9], [-1.8, -0.9, -1.8, -1.2],
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

List<LidarPoint> simScan(double x, double y, double h, math.Random rnd) {
  final pts = <LidarPoint>[];
  for (var a = 0.0; a < 360; a += 0.8) {
    final phi = h - a * math.pi / 180;
    final d = rayHit(x, y, math.cos(phi), math.sin(phi));
    if (d == null || d > 8) continue;
    pts.add(LidarPoint(a, (d + (rnd.nextDouble() - 0.5) * 0.02) * 1000, 40));
  }
  return pts;
}

double angErr(double a, double b) {
  var d = a - b;
  while (d > math.pi) { d -= 2 * math.pi; }
  while (d < -math.pi) { d += 2 * math.pi; }
  return d.abs();
}

void main() {
  test('loop closing pulls a drifted loop back into place', () async {
    final rnd = math.Random(3);
    // true path: a loop around the room, keyframes every 15 cm, turning on the spot at corners
    final corners = [[-2.4, -1.6], [2.5, -1.6], [2.5, 1.6], [-2.4, 1.6], [-2.4, -1.3]];
    final truth = <List<double>>[];
    for (var c = 0; c < corners.length - 1; c++) {
      final ax = corners[c][0], ay = corners[c][1], bx = corners[c + 1][0], by = corners[c + 1][1];
      final h = math.atan2(by - ay, bx - ax);
      if (truth.isNotEmpty) { // turn in place in 5 degree steps
        final h0 = truth.last[2];
        var dh = h - h0;
        while (dh > math.pi) { dh -= 2 * math.pi; }
        while (dh < -math.pi) { dh += 2 * math.pi; }
        final steps = (dh.abs() / 0.087).ceil();
        for (var i = 1; i < steps; i++) { truth.add([ax, ay, h0 + dh * i / steps]); }
      }
      final len = math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
      final n = (len / 0.15).ceil();
      for (var i = 0; i <= n; i++) { truth.add([ax + (bx - ax) * i / n, ay + (by - ay) * i / n, h]); }
    }
    // drifted estimate: heading error grows to ~7 degrees, positions dead-reckoned with it
    final kfs = <Keyframe>[];
    var ex = truth[0][0], ey = truth[0][1];
    for (var k = 0; k < truth.length; k++) {
      final drift = 0.12 * k / truth.length;
      if (k > 0) {
        final dx = truth[k][0] - truth[k - 1][0], dy = truth[k][1] - truth[k - 1][1];
        ex += math.cos(drift) * dx - math.sin(drift) * dy;
        ey += math.sin(drift) * dx + math.cos(drift) * dy;
      }
      final t = truth[k];
      kfs.add(Keyframe.fromScan(ex, ey, t[2] + drift, simScan(t[0], t[1], t[2], rnd)));
    }
    final last = truth.last;
    final before = math.sqrt(math.pow(kfs.last.x - last[0], 2) + math.pow(kfs.last.y - last[1], 2));
    final beforeH = angErr(kfs.last.h, last[2]);

    final lc = await LoopCloser.check(kfs, 0, 0);
    expect(lc, isNotNull, reason: 'the loop should be detected');
    LoopCloser.apply(kfs, lc!);
    final after = math.sqrt(math.pow(kfs.last.x - last[0], 2) + math.pow(kfs.last.y - last[1], 2));
    final afterH = angErr(kfs.last.h, last[2]);
    // worst error anywhere on the path, before vs after
    // ignore: avoid_print
    print('${kfs.length} keyframes; end of loop off by ${(before * 100).toStringAsFixed(1)} cm / '
        '${(beforeH * 180 / math.pi).toStringAsFixed(1)} deg before, ${(after * 100).toStringAsFixed(1)} cm / '
        '${(afterH * 180 / math.pi).toStringAsFixed(1)} deg after (loop fix ${lc.corrCm.toStringAsFixed(0)} cm, '
        '${lc.corrDeg.toStringAsFixed(1)} deg, ${(lc.hitRatio * 100).round()}% match)');
    expect(after, lessThan(before * 0.35));
    expect(afterH, lessThan(0.035));
  });
}
