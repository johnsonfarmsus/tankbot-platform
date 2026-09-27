// Lidar scan matching against the occupancy grid.
//  - local(): small search around a guess (every scan): corrects camera drift,
//             or tracks the robot on its own when no camera tracking is available.
//  - global(): search the whole map (relocalisation): "where am I on this saved map?"
// Works with any phone: it only needs the robot's lidar and the map.
import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' show Offset;
import 'lidar_client.dart';
import 'occupancy_grid.dart';
import 'pose_client.dart';

class MatchResult {
  final double x, y, h, score, hitRatio;
  final bool atEdge;
  const MatchResult(this.x, this.y, this.h, this.score, this.hitRatio, this.atEdge);
}

/// Rigid correction applied to camera-tracking poses: corrected = R(dh) * raw + (dx, dy).
class PoseCorrection {
  double dx = 0, dy = 0, dh = 0;

  void reset() {
    dx = 0;
    dy = 0;
    dh = 0;
  }

  Pose apply(Pose p) {
    final c = math.cos(dh), s = math.sin(dh);
    return Pose(p.t, c * p.x - s * p.y + dx, s * p.x + c * p.y + dy, p.heading + dh, p.good, p.fy);
  }

  /// Choose the correction so that apply(raw) == (x, y, h).
  void setSoThat(Pose raw, double x, double y, double h) {
    dh = h - raw.heading;
    final c = math.cos(dh), s = math.sin(dh);
    dx = x - (c * raw.x - s * raw.y);
    dy = y - (s * raw.x + c * raw.y);
  }
}

class ScanMatcher {
  ScanMatcher(this.grid);
  final OccupancyGrid grid;

  /// Lidar points in the robot frame: dx = forward (m), dy = left (m).
  static List<Offset> robotFrame(List<LidarPoint> pts,
      {double fwdM = 0, double leftM = 0, int stride = 1, double minR = 0.2, double maxR = 6.0}) {
    final out = <Offset>[];
    for (var i = 0; i < pts.length; i += stride) {
      final d = pts[i].distMm / 1000.0;
      if (d < minR || d > maxR) continue;
      final a = pts[i].angleDeg * math.pi / 180.0;
      out.add(Offset(fwdM + d * math.cos(a), leftM - d * math.sin(a)));
    }
    return out;
  }

  static List<Offset> _rotate(List<Offset> pts, double h) {
    final c = math.cos(h), s = math.sin(h);
    return [for (final p in pts) Offset(c * p.dx - s * p.dy, s * p.dx + c * p.dy)];
  }

  /// Sum of "closeness to a wall" over the points. Smooth, so the search slides scans onto walls.
  double _score(List<Offset> rot, double x, double y) {
    var s = 0.0;
    for (final p in rot) {
      s += grid.likelihood(x + p.dx, y + p.dy);
    }
    return s;
  }

  /// For whole-map search: also count against points that land where the map is
  /// confidently empty (open floor), which separates look-alike places.
  double _scoreGlobal(List<Offset> rot, double x, double y) {
    var s = 0.0;
    for (final p in rot) {
      final wx = x + p.dx, wy = y + p.dy;
      final l = grid.likelihood(wx, wy);
      if (l > 0.05) {
        s += l;
      } else if (grid.at(wx, wy) < -1.5) {
        s -= 0.5;
      }
    }
    return s;
  }

  /// Points within ~7 cm of a mapped wall.
  int _hits(List<Offset> rot, double x, double y) {
    var n = 0;
    for (final p in rot) {
      if (grid.likelihood(x + p.dx, y + p.dy) > 0.5) n++;
    }
    return n;
  }

  MatchResult local(List<Offset> pts, double x0, double y0, double h0,
      {double lin = 0.08, double linStep = 0.02, double ang = 0.05, double angStep = 0.01}) {
    grid.ensureField();
    final nl = (lin / linStep).round(), na = (ang / angStep).round();
    var best = double.negativeInfinity, bx = x0, by = y0, bh = h0;
    var bestRot = <Offset>[];
    var bi = 0, bj = 0, bk = 0;
    for (var k = -na; k <= na; k++) {
      final h = h0 + k * angStep;
      final rot = _rotate(pts, h);
      for (var i = -nl; i <= nl; i++) {
        for (var j = -nl; j <= nl; j++) {
          final x = x0 + i * linStep, y = y0 + j * linStep;
          // tiny preference for small corrections breaks ties toward the guess
          final sc = _score(rot, x, y) - 0.0005 * (i * i + j * j + k * k);
          if (sc > best) {
            best = sc;
            bx = x;
            by = y;
            bh = h;
            bestRot = rot;
            bi = i;
            bj = j;
            bk = k;
          }
        }
      }
    }
    final hits = pts.isEmpty ? 0.0 : _hits(bestRot, bx, by) / pts.length;
    final edge = (nl > 0 && (bi.abs() == nl || bj.abs() == nl)) || (na > 0 && bk.abs() == na);
    return MatchResult(bx, by, bh, best, hits, edge);
  }

  /// Wide but fast: coarse search (4 cm / 2 deg steps) then fine (1 cm / 0.4 deg).
  MatchResult localCoarseFine(List<Offset> pts, double x0, double y0, double h0,
      {double lin = 0.12, double ang = 0.17}) {
    final c = local(pts, x0, y0, h0, lin: lin, linStep: 0.04, ang: ang, angStep: 0.035);
    final f = local(pts, c.x, c.y, c.h, lin: 0.04, linStep: 0.01, ang: 0.035, angStep: 0.007);
    return MatchResult(f.x, f.y, f.h, f.score, f.hitRatio, c.atEdge);
  }

  /// Whole-map search. Returns refined candidates, best first.
  Future<List<MatchResult>> global(List<Offset> pts, {double step = 0.15, int headings = 120, int keep = 12}) async {
    grid.ensureField(force: true);
    final coarse = [for (var i = 0; i < pts.length; i += 4) pts[i]];
    final cands = grid.freeCellCentres(step);
    final rots = [for (var k = 0; k < headings; k++) _rotate(coarse, k * 2 * math.pi / headings)];
    final top = <MatchResult>[];
    var n = 0;
    for (final c in cands) {
      for (var k = 0; k < headings; k++) {
        final sc = _scoreGlobal(rots[k], c.dx, c.dy);
        if (top.length < keep || sc > top.last.score) {
          _insertTop(top, MatchResult(c.dx, c.dy, k * 2 * math.pi / headings, sc, 0, false), keep);
        }
      }
      if (++n % 120 == 0) await Future<void>.delayed(Duration.zero); // keep the app responsive
    }
    final refined = <MatchResult>[];
    for (final t in top) {
      var r = local(pts, t.x, t.y, t.h, lin: 0.2, linStep: 0.05, ang: 0.09, angStep: 0.03);
      r = local(pts, r.x, r.y, r.h, lin: 0.04, linStep: 0.01, ang: 0.02, angStep: 0.005);
      refined.add(MatchResult(r.x, r.y, r.h, _scoreGlobal(_rotate(pts, r.h), r.x, r.y), r.hitRatio, r.atEdge));
      await Future<void>.delayed(Duration.zero);
    }
    refined.sort((a, b) => b.score.compareTo(a.score));
    return refined;
  }

  static double _ad(double a, double b) {
    var d = a - b;
    while (d > math.pi) {
      d -= 2 * math.pi;
    }
    while (d < -math.pi) {
      d += 2 * math.pi;
    }
    return d;
  }

  /// Keep the best `keep` candidates, merging ones that are basically the same pose.
  static void _insertTop(List<MatchResult> top, MatchResult r, int keep) {
    for (var i = 0; i < top.length; i++) {
      final t = top[i];
      if ((t.x - r.x).abs() < 0.6 && (t.y - r.y).abs() < 0.6 && _ad(t.h, r.h).abs() < 0.45) {
        if (r.score > t.score) {
          top[i] = r;
          top.sort((a, b) => b.score.compareTo(a.score));
        }
        return;
      }
    }
    top.add(r);
    top.sort((a, b) => b.score.compareTo(a.score));
    if (top.length > keep) top.removeLast();
  }
}
