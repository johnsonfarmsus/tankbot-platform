// Loop closing: when the robot comes back to a place it mapped earlier in the drive,
// match its scan against that older part of the map. If the path has drifted, spread the
// correction back along the keyframes driven since then, so the whole loop straightens out.
import 'dart:async';
import 'dart:math' as math;
import 'map_store.dart';
import 'occupancy_grid.dart';
import 'pose_client.dart';
import 'scan_matcher.dart';

class LoopClosure {
  /// Keyframes from startIndex onward get the correction, growing from 0 to 1 along the path.
  final int startIndex, endIndex;
  /// Full correction: rotate about (pivotX, pivotY) by dTheta, then translate by (tx, ty).
  final double pivotX, pivotY, dTheta, tx, ty;
  final double corrCm, corrDeg, hitRatio;
  const LoopClosure(this.startIndex, this.endIndex, this.pivotX, this.pivotY, this.dTheta, this.tx, this.ty,
      this.corrCm, this.corrDeg, this.hitRatio);
}

double _sq(double v) => v * v;

double _ad(double a, double b) {
  var d = a - b;
  while (d > math.pi) {
    d -= 2 * math.pi;
  }
  while (d < -math.pi) {
    d += 2 * math.pi;
  }
  return d;
}

class LoopCloser {
  /// Only compare against map made at least this much driving ago.
  static const double minLoopPathM = 4.0;
  /// The robot must be this close to an old keyframe to try.
  static const double searchRadiusM = 2.5;
  /// Old keyframes within this distance build the reference map.
  static const double contextRadiusM = 4.0;
  /// Reference map: keyframes within this much driving of the first visit.
  static const double refPathM = 3.0;

  static List<double> _pathLengths(List<Keyframe> kfs, int n) {
    final cum = List<double>.filled(n, 0);
    for (var i = 1; i < n; i++) {
      cum[i] = cum[i - 1] + math.sqrt(_sq(kfs[i].x - kfs[i - 1].x) + _sq(kfs[i].y - kfs[i - 1].y));
    }
    return cum;
  }

  static Future<LoopClosure?> check(List<Keyframe> kfs, double fwdM, double leftM) async {
    final n = kfs.length;
    if (n < 30) return null;
    final cum = _pathLengths(kfs, n);
    final cur = kfs[n - 1];
    // Anchor on the FIRST visit to this place: the earliest keyframe close to where we are
    // now. Early keyframes carry the least drift, so they make the best reference.
    var anchor = -1;
    for (var i = 0; i < n; i++) {
      if (cum[n - 1] - cum[i] < minLoopPathM) break; // the rest are recent
      final d = math.sqrt(_sq(kfs[i].x - cur.x) + _sq(kfs[i].y - cur.y));
      if (d < searchRadiusM) {
        anchor = i;
        break;
      }
    }
    if (anchor < 0) return null;
    // reference: the stretch of path around that first visit (+-3 m of driving), all of it old
    final old = <int>[
      for (var i = 0; i < n; i++)
        if ((cum[i] - cum[anchor]).abs() <= refPathM && cum[n - 1] - cum[i] >= minLoopPathM) i
    ];
    if (old.length < 5) return null;
    final nearest = anchor;

    // reference map from the old keyframes only
    final g = OccupancyGrid();
    for (var k = 0; k < old.length; k++) {
      final kf = kfs[old[k]];
      g.integrate(Pose(0, kf.x, kf.y, kf.h, true), kf.points(), lidarFwdM: fwdM, lidarLeftM: leftM);
      if (k % 20 == 19) await Future<void>.delayed(Duration.zero);
    }
    final m = ScanMatcher(g);
    final pts = ScanMatcher.robotFrame(cur.points(), fwdM: fwdM, leftM: leftM, stride: 2);
    if (pts.length < 60) return null;
    var r = m.local(pts, cur.x, cur.y, cur.h, lin: 0.5, linStep: 0.05, ang: 0.26, angStep: 0.035);
    if (r.atEdge) return null; // drift bigger than we are willing to trust
    await Future<void>.delayed(Duration.zero);
    r = m.local(pts, r.x, r.y, r.h, lin: 0.05, linStep: 0.01, ang: 0.035, angStep: 0.007);
    if (r.hitRatio < 0.55) return null;

    final dth = _ad(r.h, cur.h);
    final dcm = math.sqrt(_sq(r.x - cur.x) + _sq(r.y - cur.y)) * 100;
    if (dcm < 3 && dth.abs() < 0.017) return null; // already consistent (under 3 cm and 1 degree)

    // Anchor on the old keyframe we are closing onto; everything after it gets corrected.
    final start = nearest + 1;
    final p = kfs[start - 1];
    final c = math.cos(dth), s = math.sin(dth);
    final rx = p.x + c * (cur.x - p.x) - s * (cur.y - p.y);
    final ry = p.y + s * (cur.x - p.x) + c * (cur.y - p.y);
    return LoopClosure(start, n - 1, p.x, p.y, dth, r.x - rx, r.y - ry, dcm, dth * 180 / math.pi, r.hitRatio);
  }

  /// Transform a pose by a fraction w (0..1) of the correction.
  static (double, double, double) transform(LoopClosure lc, double x, double y, double h, double w) {
    final a = lc.dTheta * w, c = math.cos(a), s = math.sin(a);
    final nx = lc.pivotX + c * (x - lc.pivotX) - s * (y - lc.pivotY) + lc.tx * w;
    final ny = lc.pivotY + s * (x - lc.pivotX) + c * (y - lc.pivotY) + lc.ty * w;
    return (nx, ny, h + a);
  }

  /// Apply the correction to the keyframes, spread along the path (full correction from
  /// endIndex onward, including keyframes added while the check was running).
  static void apply(List<Keyframe> kfs, LoopClosure lc) {
    final n = kfs.length;
    final cum = _pathLengths(kfs, n);
    final base = cum[lc.startIndex - 1];
    final span = math.max(1e-6, cum[lc.endIndex] - base);
    for (var k = lc.startIndex; k < n; k++) {
      final w = k >= lc.endIndex ? 1.0 : ((cum[k] - base) / span).clamp(0.0, 1.0);
      final t = transform(lc, kfs[k].x, kfs[k].y, kfs[k].h, w);
      kfs[k].x = t.$1;
      kfs[k].y = t.$2;
      kfs[k].h = t.$3;
    }
  }
}
