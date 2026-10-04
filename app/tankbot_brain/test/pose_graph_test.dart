import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/pose_graph.dart';

/// True path from a list of (distance m, turn deg) legs, a pose every 0.15 m.
List<(double, double, double)> truePath(List<(double, double)> legs) {
  final out = <(double, double, double)>[(0, 0, 0)];
  var x = 0.0, y = 0.0, h = 0.0;
  for (final (dist, turn) in legs) {
    h += turn * math.pi / 180;
    out.add((x, y, h));
    final n = (dist / 0.15).round();
    for (var k = 0; k < n; k++) {
      x += 0.15 * math.cos(h);
      y += 0.15 * math.sin(h);
      out.add((x, y, h));
    }
  }
  return out;
}

/// Odometry edges with a heading drift of `driftDegPerM`, and dead-reckoned initial poses.
PoseGraph drifted(List<(double, double, double)> truth, double driftDegPerM) {
  final xs = <double>[truth[0].$1], ys = <double>[truth[0].$2], hs = <double>[truth[0].$3];
  final edges = <PGEdge>[];
  for (var i = 1; i < truth.length; i++) {
    final a = truth[i - 1], b = truth[i];
    var (dx, dy, dth) = relativePose(a.$1, a.$2, a.$3, b.$1, b.$2, b.$3);
    dth += driftDegPerM * math.pi / 180 * math.sqrt(dx * dx + dy * dy);
    edges.add(PGEdge(i - 1, i, dx, dy, dth));
    final c = math.cos(hs.last), s = math.sin(hs.last);
    xs.add(xs.last + c * dx - s * dy);
    ys.add(ys.last + s * dx + c * dy);
    hs.add(wrapAngle(hs.last + dth));
  }
  final g = PoseGraph(xs, ys, hs);
  g.edges.addAll(edges);
  return g;
}

double endError(PoseGraph g, List<(double, double, double)> t) =>
    math.sqrt(math.pow(g.x.last - t.last.$1, 2) + math.pow(g.y.last - t.last.$2, 2));

double maxError(PoseGraph g, List<(double, double, double)> t) {
  var m = 0.0;
  for (var i = 0; i < t.length; i++) {
    m = math.max(m, math.sqrt(math.pow(g.x[i] - t[i].$1, 2) + math.pow(g.y[i] - t[i].$2, 2)));
  }
  return m;
}

void main() {
  test('wall alignment straightens a one-way trip through a square house', () {
    final t = truePath([(5, 0), (6, 90), (4, 90), (3, -90), (5, 0)]);
    final g = drifted(t, 0.8); // 0.8 deg of heading drift per metre: a visibly curved map
    final before = maxError(g, t);
    // what the brain does: each keyframe's scan sees the house's walls; compare with the reference
    // direction from the first keyframes, and pull the heading by the folded difference
    const ref = 0.0; // house walls along the map axes (taken from the first keyframes)
    // the brain straightens every few metres, so drift never builds up far; here: grow the trip in
    // 4 m pieces, re-reading the walls and straightening after each
    final full = g.x.length;
    for (var upto = 27; ; upto = math.min(full, upto + 27)) {
      g.headingPriors.clear();
      for (var i = 0; i < upto; i++) {
        final wallWorldEst = ref + (g.h[i] - t[i].$3); // the scan's walls, placed with the current heading
        final d = WallDirection.wrap90(wallWorldEst - ref);
        if (d.abs() < 10 * math.pi / 180) g.headingPriors.add(PGHeadingPrior(i, g.h[i] - d, 0.05));
      }
      g.optimize();
      if (upto == full) break;
    }
    final after = maxError(g, t);
    // ignore: avoid_print
    print('one-way 23 m trip, 0.8 deg/m drift: worst position error ${(before * 100).toStringAsFixed(0)} cm -> ${(after * 100).toStringAsFixed(0)} cm');
    expect(after, lessThan(before * 0.2));
  });

  test('a loop link pulls a drifted loop closed', () {
    final t = truePath([(4, 0), (4, 90), (4, 90), (4, 90)]);
    final g = drifted(t, 1.0);
    final before = endError(g, t);
    final a = t.first, b = t.last;
    final (dx, dy, dth) = relativePose(a.$1, a.$2, a.$3, b.$1, b.$2, b.$3);
    g.edges.add(PGEdge(0, t.length - 1, dx, dy, dth, sigmaT: 0.02, sigmaR: 0.005, robust: true, kind: 'loop'));
    g.optimize();
    final after = endError(g, t);
    // ignore: avoid_print
    print('16 m loop, 1 deg/m drift: end error ${(before * 100).toStringAsFixed(0)} cm -> ${(after * 100).toStringAsFixed(1)} cm, worst anywhere ${(maxError(g, t) * 100).toStringAsFixed(0)} cm');
    expect(after, lessThan(0.05));
  });

  /// Worst error after the best rigid fit onto the truth: is the map the right SHAPE?
  double shapeError(List<double> xs, List<double> ys, List<(double, double, double)> t) {
    var mx = 0.0, my = 0.0, tx = 0.0, ty = 0.0;
    for (var i = 0; i < t.length; i++) { mx += xs[i]; my += ys[i]; tx += t[i].$1; ty += t[i].$2; }
    final n = t.length.toDouble();
    mx /= n; my /= n; tx /= n; ty /= n;
    var sxx = 0.0, sxy = 0.0;
    for (var i = 0; i < t.length; i++) {
      final ax = xs[i] - mx, ay = ys[i] - my, bx = t[i].$1 - tx, by = t[i].$2 - ty;
      sxx += ax * bx + ay * by; sxy += ax * by - ay * bx;
    }
    final a = math.atan2(sxy, sxx), c = math.cos(a), s = math.sin(a);
    var worst = 0.0;
    for (var i = 0; i < t.length; i++) {
      final px = c * (xs[i] - mx) - s * (ys[i] - my) + tx, py = s * (xs[i] - mx) + c * (ys[i] - my) + ty;
      worst = math.max(worst, math.sqrt(math.pow(px - t[i].$1, 2) + math.pow(py - t[i].$2, 2)));
    }
    return worst;
  }

  /// Worst real-world (east/north) position error, using the solved map-to-world alignment.
  double worldError(PoseGraph g, List<(double, double, double)> t) {
    const rot = 0.5;
    final c = math.cos(g.gphi), s = math.sin(g.gphi), cr = math.cos(rot), sr = math.sin(rot);
    var worst = 0.0;
    for (var i = 0; i < t.length; i++) {
      final pe = c * g.x[i] - s * g.y[i] + g.gx, pn = s * g.x[i] + c * g.y[i] + g.gy;
      final te = cr * t[i].$1 - sr * t[i].$2 + 100, tn = sr * t[i].$1 + cr * t[i].$2 - 40;
      worst = math.max(worst, math.sqrt(math.pow(pe - te, 2) + math.pow(pn - tn, 2)));
    }
    return worst;
  }

  (double, double, PoseGraph) gpsRun(double driftDegPerM, int seed) {
    final t = truePath([(80, 0), (60, 60), (60, -45)]); // 200 m
    final g = drifted(t, driftDegPerM);
    final before = maxError(g, t);
    final rnd = math.Random(seed);
    const rot = 0.5; // map frame is rotated 28.6 deg from east/north
    for (var i = 0; i < t.length; i += 60) {
      // a fix every ~9 m, +-3 m noise
      final ex = math.cos(rot) * t[i].$1 - math.sin(rot) * t[i].$2 + 100 + (rnd.nextDouble() - 0.5) * 6;
      final ny = math.sin(rot) * t[i].$1 + math.cos(rot) * t[i].$2 - 40 + (rnd.nextDouble() - 0.5) * 6;
      g.gps.add(PGGps(i, ex, ny, 3));
    }
    final shapeBefore = shapeError(g.x, g.y, t);
    g.initGpsAlignment();
    g.optimize(maxIterations: 100);
    // ignore: avoid_print
    print('  shape error ${shapeBefore.toStringAsFixed(1)} m -> ${shapeError(g.x, g.y, t).toStringAsFixed(1)} m, '
        'real-world position error after: ${worldError(g, t).toStringAsFixed(1)} m');
    return (before, maxError(g, t), g);
  }

  test('good GPS anchors a long outdoor drive (realistic drift)', () {
    final (before, after, g) = gpsRun(0.1, 5);
    // ignore: avoid_print
    print('200 m drive, 0.1 deg/m drift, fixes every 9 m at +-3 m: worst error ${before.toStringAsFixed(1)} m -> ${after.toStringAsFixed(1)} m; '
        'north ${(g.gphi * 180 / math.pi).toStringAsFixed(1)} deg (true 28.6)');
    final t = truePath([(80, 0), (60, 60), (60, -45)]);
    expect(shapeError(g.x, g.y, t), lessThanOrEqualTo(6.0));
    expect(worldError(g, t), lessThanOrEqualTo(6.0));
    expect((g.gphi - 0.5).abs(), lessThan(5 * math.pi / 180));
  });

  test('GPS with extreme drift (report only)', () {
    final (before, after, g) = gpsRun(0.3, 5);
    // ignore: avoid_print
    print('REPORT 200 m drive, 0.3 deg/m drift (60 deg total): worst error ${before.toStringAsFixed(1)} m -> ${after.toStringAsFixed(1)} m; '
        'north ${(g.gphi * 180 / math.pi).toStringAsFixed(1)} deg (true 28.6)');
  });

  test('wall direction of a rotated room', () {
    // 4 x 3 m room rotated 17 deg, robot in the middle facing along the map x axis
    const rot = 17 * math.pi / 180;
    final walls = [[-2.0, -1.5, 2.0, -1.5], [2.0, -1.5, 2.0, 1.5], [2.0, 1.5, -2.0, 1.5], [-2.0, 1.5, -2.0, -1.5]]
        .map((w) {
      final c = math.cos(rot), s = math.sin(rot);
      return [c * w[0] - s * w[1], s * w[0] + c * w[1], c * w[2] - s * w[3], s * w[2] + c * w[3]];
    }).toList();
    final pts = <(double, double)>[];
    final rnd = math.Random(1);
    for (var a = 0.0; a < 360; a += 0.8) {
      final phi = -a * math.pi / 180;
      final dx = math.cos(phi), dy = math.sin(phi);
      double? best;
      for (final w in walls) {
        final ex = w[2] - w[0], ey = w[3] - w[1];
        final den = dx * ey - dy * ex;
        if (den.abs() < 1e-9) continue;
        final tt = (w[0] * ey - w[1] * ex) / den;
        final u = (w[0] * dy - w[1] * dx) / den;
        if (tt > 0 && u >= 0 && u <= 1 && (best == null || tt < best)) best = tt;
      }
      if (best == null) continue;
      final d = best + (rnd.nextDouble() - 0.5) * 0.006;
      pts.add((d * dx, d * dy));
    }
    final (theta, strength) = WallDirection.ofScan(pts);
    // ignore: avoid_print
    print('room rotated 17 deg: detected ${(theta * 180 / math.pi).toStringAsFixed(1)} deg, strength ${strength.toStringAsFixed(2)}');
    expect(WallDirection.wrap90(theta - rot).abs(), lessThan(1.5 * math.pi / 180));
    expect(strength, greaterThan(0.7));
  });
}
