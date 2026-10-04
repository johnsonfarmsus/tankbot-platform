// Pose-graph map optimisation.
//
// Nodes: keyframe poses (x, y, heading). Factors:
//   - relative links between keyframes (driving odometry, loop closures; loops use a robust kernel)
//   - heading priors (wall alignment: houses are built square)
//   - GPS positions (only good fixes; they also solve a map->east/north alignment, 3 extra unknowns)
// Gauss-Newton; each step solves the sparse normal equations with block-preconditioned conjugate
// gradients. Node 0 is held fixed (it defines the map frame).
import 'dart:math' as math;
import 'dart:typed_data';

class PGEdge {
  final int i, j;
  final double dx, dy, dth; // measured pose of j in i's frame
  final double sigmaT, sigmaR;
  final bool robust;
  final String kind; // 'odo' | 'loop'
  /// Default uncertainty of one driving step between keyframes (~15 cm apart): tracking (camera +
  /// lidar) is good to about 1 cm and 0.17 degrees per step. Too loose and noisy GPS could bend the map.
  const PGEdge(this.i, this.j, this.dx, this.dy, this.dth, {this.sigmaT = 0.01, this.sigmaR = 0.003, this.robust = false, this.kind = 'odo'});

  Map<String, dynamic> toJson() => {'i': i, 'j': j, 'dx': dx, 'dy': dy, 'dth': dth, 'st': sigmaT, 'sr': sigmaR, 'robust': robust, 'kind': kind};
  static PGEdge? fromJson(dynamic m) {
    if (m is! Map) return null;
    double n(dynamic v) => (v as num).toDouble();
    try {
      return PGEdge((m['i'] as num).toInt(), (m['j'] as num).toInt(), n(m['dx']), n(m['dy']), n(m['dth']),
          sigmaT: n(m['st']), sigmaR: n(m['sr']), robust: m['robust'] == true, kind: (m['kind'] as String?) ?? 'odo');
    } catch (_) {
      return null;
    }
  }
}

class PGHeadingPrior {
  final int i;
  final double target, sigma;
  const PGHeadingPrior(this.i, this.target, this.sigma);
}

class PGGps {
  final int i;
  final double east, north, sigma; // metres in a local east/north frame
  const PGGps(this.i, this.east, this.north, this.sigma);
}

double wrapAngle(double a) {
  while (a > math.pi) {
    a -= 2 * math.pi;
  }
  while (a < -math.pi) {
    a += 2 * math.pi;
  }
  return a;
}

/// Pose of j relative to i (the measurement an edge stores).
(double, double, double) relativePose(double xi, double yi, double hi, double xj, double yj, double hj) {
  final c = math.cos(hi), s = math.sin(hi);
  final tx = xj - xi, ty = yj - yi;
  return (c * tx + s * ty, -s * tx + c * ty, wrapAngle(hj - hi));
}

class PoseGraph {
  PoseGraph(this.x, this.y, this.h);
  final List<double> x, y, h;
  final List<PGEdge> edges = [];
  final List<PGHeadingPrior> headingPriors = [];
  final List<PGGps> gps = [];
  // map -> east/north alignment (solved when GPS factors are present)
  double gx = 0, gy = 0, gphi = 0;

  int get n => x.length;

  /// Best rigid fit of map positions onto GPS east/north, as the starting alignment.
  /// Uses the earliest fixes (until they span 20 m): early in a drive the map has not drifted yet.
  void initGpsAlignment() {
    if (gps.length < 2) return;
    final sorted = [...gps]..sort((a, b) => a.i.compareTo(b.i));
    final use = <PGGps>[];
    for (final g in sorted) {
      use.add(g);
      final f = use.first;
      final span = math.sqrt(math.pow(g.east - f.east, 2) + math.pow(g.north - f.north, 2));
      if (use.length >= 3 && span >= 20) break;
    }
    var mx = 0.0, my = 0.0, me = 0.0, mn = 0.0;
    for (final g in use) {
      mx += x[g.i];
      my += y[g.i];
      me += g.east;
      mn += g.north;
    }
    final k = use.length.toDouble();
    mx /= k;
    my /= k;
    me /= k;
    mn /= k;
    var sxx = 0.0, sxy = 0.0;
    for (final g in use) {
      final ax = x[g.i] - mx, ay = y[g.i] - my, bx = g.east - me, by = g.north - mn;
      sxx += ax * bx + ay * by;
      sxy += ax * by - ay * bx;
    }
    gphi = math.atan2(sxy, sxx);
    final c = math.cos(gphi), s = math.sin(gphi);
    gx = me - (c * mx - s * my);
    gy = mn - (s * mx + c * my);
  }

  /// Levenberg-Marquardt (damped Gauss-Newton: cautious steps, only improvements are kept).
  /// Returns (iterations, final cost).
  (int, double) optimize({int maxIterations = 30, double huber = 3.0}) {
    final useGps = gps.length >= 3;
    final m = n + (useGps ? 1 : 0); // block count (alignment = extra block)
    var cost = 0.0;
    var it = 0;
    var lambda = 1e-4;
    double? accepted;
    for (; it < maxIterations; it++) {
      final hd = List.generate(m, (_) => Float64List(9));
      final off = <int, Float64List>{}; // key a*m+b, a<b: block H_ab
      final b = Float64List(3 * m);
      cost = 0;

      void addOff(int a, int c, List<double> blk) {
        // blk is H_ac (3x3 row-major); store upper triangle
        if (a < c) {
          final o = off.putIfAbsent(a * m + c, () => Float64List(9));
          for (var q = 0; q < 9; q++) {
            o[q] += blk[q];
          }
        } else {
          final o = off.putIfAbsent(c * m + a, () => Float64List(9));
          for (var r = 0; r < 3; r++) {
            for (var q = 0; q < 3; q++) {
              o[q * 3 + r] += blk[r * 3 + q];
            }
          }
        }
      }

      for (final e in edges) {
        final i = e.i, j = e.j;
        final c = math.cos(h[i]), s = math.sin(h[i]);
        final tx = x[j] - x[i], ty = y[j] - y[i];
        final r0 = c * tx + s * ty - e.dx;
        final r1 = -s * tx + c * ty - e.dy;
        final r2 = wrapAngle(h[j] - h[i] - e.dth);
        final wt = 1 / (e.sigmaT * e.sigmaT), wr = 1 / (e.sigmaR * e.sigmaR);
        var rho = 1.0;
        final chi = math.sqrt(wt * (r0 * r0 + r1 * r1) + wr * r2 * r2);
        if (e.robust && chi > huber) rho = huber / chi;
        cost += rho * chi * chi;
        // A = d r / d xi, B = d r / d xj (rows: r0, r1, r2; cols: x, y, h)
        final a = [-c, -s, -s * tx + c * ty, s, -c, -c * tx - s * ty, 0.0, 0.0, -1.0];
        final bb = [c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0];
        final w = [wt * rho, wt * rho, wr * rho];
        final res = [r0, r1, r2];
        final hii = Float64List(9), hjj = Float64List(9), hij = Float64List(9);
        for (var p = 0; p < 3; p++) {
          for (var q = 0; q < 3; q++) {
            var sii = 0.0, sjj = 0.0, sij = 0.0;
            for (var k = 0; k < 3; k++) {
              sii += a[k * 3 + p] * w[k] * a[k * 3 + q];
              sjj += bb[k * 3 + p] * w[k] * bb[k * 3 + q];
              sij += a[k * 3 + p] * w[k] * bb[k * 3 + q];
            }
            hii[p * 3 + q] = sii;
            hjj[p * 3 + q] = sjj;
            hij[p * 3 + q] = sij;
          }
          var gi = 0.0, gj = 0.0;
          for (var k = 0; k < 3; k++) {
            gi += a[k * 3 + p] * w[k] * res[k];
            gj += bb[k * 3 + p] * w[k] * res[k];
          }
          b[i * 3 + p] += gi;
          b[j * 3 + p] += gj;
        }
        for (var q = 0; q < 9; q++) {
          hd[i][q] += hii[q];
          hd[j][q] += hjj[q];
        }
        addOff(i, j, hij);
      }

      for (final pr in headingPriors) {
        final r = wrapAngle(h[pr.i] - pr.target);
        final w = 1 / (pr.sigma * pr.sigma);
        cost += w * r * r;
        hd[pr.i][8] += w;
        b[pr.i * 3 + 2] += w * r;
      }

      if (useGps) {
        final gb = n; // alignment block
        final c = math.cos(gphi), s = math.sin(gphi);
        for (final g in gps) {
          final i = g.i;
          final pe = c * x[i] - s * y[i] + gx, pn = s * x[i] + c * y[i] + gy;
          final res = [pe - g.east, pn - g.north];
          final w = 1 / (g.sigma * g.sigma);
          cost += w * (res[0] * res[0] + res[1] * res[1]);
          // d/d node (x, y, h): [[c, -s, 0], [s, c, 0]]; d/d align (gx, gy, gphi): [[1, 0, -s x - c y], [0, 1, c x - s y]]
          final ji = [c, -s, 0.0, s, c, 0.0];
          final jg = [1.0, 0.0, -s * x[i] - c * y[i], 0.0, 1.0, c * x[i] - s * y[i]];
          final hii = Float64List(9), hgg = Float64List(9), hig = Float64List(9);
          for (var p = 0; p < 3; p++) {
            for (var q = 0; q < 3; q++) {
              hii[p * 3 + q] = w * (ji[p] * ji[q] + ji[3 + p] * ji[3 + q]);
              hgg[p * 3 + q] = w * (jg[p] * jg[q] + jg[3 + p] * jg[3 + q]);
              hig[p * 3 + q] = w * (ji[p] * jg[q] + ji[3 + p] * jg[3 + q]);
            }
            b[i * 3 + p] += w * (ji[p] * res[0] + ji[3 + p] * res[1]);
            b[gb * 3 + p] += w * (jg[p] * res[0] + jg[3 + p] * res[1]);
          }
          for (var q = 0; q < 9; q++) {
            hd[i][q] += hii[q];
            hd[gb][q] += hgg[q];
          }
          addOff(i, gb, hig);
        }
      }

      // hold node 0 fixed: take it out of the system (identity row, no coupling) - a huge pinning
      // weight would wreck the conditioning and stall the solver
      for (var q = 0; q < 9; q++) {
        hd[0][q] = q % 4 == 0 ? 1.0 : 0.0;
      }
      b[0] = 0;
      b[1] = 0;
      b[2] = 0;
      off.removeWhere((key, _) => key ~/ m == 0);
      for (var bi = 1; bi < m; bi++) {
        for (var d = 0; d < 3; d++) {
          hd[bi][d * 4] += 1e-6; // tiny damping keeps blocks invertible
        }
      }

      accepted ??= this.cost(huber: huber); // same measure as the acceptance test
      // damping: H + lambda * diag(H)
      for (var bi = 1; bi < m; bi++) {
        for (var d = 0; d < 3; d++) {
          hd[bi][d * 4] *= 1 + lambda;
        }
      }
      final saved = (List<double>.from(x), List<double>.from(y), List<double>.from(h), gx, gy, gphi);
      final dx = _skylineSolve(hd, off, b, m) ?? _pcg(hd, off, b, m);
      var maxStep = 0.0;
      for (var k = 0; k < n; k++) {
        x[k] -= dx[k * 3];
        y[k] -= dx[k * 3 + 1];
        h[k] = wrapAngle(h[k] - dx[k * 3 + 2]);
        maxStep = math.max(maxStep, math.max(dx[k * 3].abs(), math.max(dx[k * 3 + 1].abs(), dx[k * 3 + 2].abs())));
      }
      if (useGps) {
        gx -= dx[n * 3];
        gy -= dx[n * 3 + 1];
        gphi = wrapAngle(gphi - dx[n * 3 + 2]);
      }
      final newCost = this.cost(huber: huber);
      if (newCost <= accepted) {
        accepted = newCost;
        lambda = math.max(lambda / 3, 1e-7);
        if (maxStep < 1e-5) {
          it++;
          break;
        }
      } else {
        // worse: undo the step and take a more cautious one next time
        for (var k = 0; k < n; k++) {
          x[k] = saved.$1[k];
          y[k] = saved.$2[k];
          h[k] = saved.$3[k];
        }
        gx = saved.$4;
        gy = saved.$5;
        gphi = saved.$6;
        lambda *= 8;
        if (lambda > 1e8) break;
      }
    }
    return (it, accepted ?? cost);
  }

  /// Total weighted error of the current poses.
  double cost({double huber = 3.0}) {
    var total = 0.0;
    for (final e in edges) {
      final c = math.cos(h[e.i]), s = math.sin(h[e.i]);
      final tx = x[e.j] - x[e.i], ty = y[e.j] - y[e.i];
      final r0 = c * tx + s * ty - e.dx, r1 = -s * tx + c * ty - e.dy, r2 = wrapAngle(h[e.j] - h[e.i] - e.dth);
      final chi2 = (r0 * r0 + r1 * r1) / (e.sigmaT * e.sigmaT) + r2 * r2 / (e.sigmaR * e.sigmaR);
      final chi = math.sqrt(chi2);
      total += e.robust && chi > huber ? 2 * huber * chi - huber * huber : chi2;
    }
    for (final pr in headingPriors) {
      final r = wrapAngle(h[pr.i] - pr.target);
      total += r * r / (pr.sigma * pr.sigma);
    }
    if (gps.length >= 3) {
      final c = math.cos(gphi), s = math.sin(gphi);
      for (final g in gps) {
        final pe = c * x[g.i] - s * y[g.i] + gx - g.east, pn = s * x[g.i] + c * y[g.i] + gy - g.north;
        total += (pe * pe + pn * pn) / (g.sigma * g.sigma);
      }
    }
    return total;
  }

  /// Exact solve of H d = b by skyline (envelope) Cholesky. A pose graph is a long chain plus a few
  /// cross-links, so each row of H is nonzero only from its first link to the diagonal; Cholesky
  /// never fills outside that envelope, which keeps this fast and exact. Returns null if H is not
  /// positive definite (the caller then falls back to conjugate gradients).
  static Float64List? _skylineSolve(List<Float64List> hd, Map<int, Float64List> off, Float64List b, int m) {
    final nn = 3 * m;
    final first = List<int>.generate(nn, (r) => (r ~/ 3) * 3);
    off.forEach((key, _) {
      final a = key ~/ m, c = key % m; // a < c: rows of block c reach back to block a
      for (var p = 0; p < 3; p++) {
        if (3 * a < first[3 * c + p]) first[3 * c + p] = 3 * a;
      }
    });
    final start = Int32List(nn + 1);
    for (var r = 0; r < nn; r++) {
      start[r + 1] = start[r] + (r - first[r] + 1);
    }
    final l = Float64List(start[nn]); // row r holds columns first[r]..r
    double get(int r, int c) => l[start[r] + c - first[r]];
    void put(int r, int c, double v) => l[start[r] + c - first[r]] = v;
    for (var bi = 0; bi < m; bi++) {
      final d = hd[bi];
      for (var p = 0; p < 3; p++) {
        for (var q = 0; q <= p; q++) {
          put(3 * bi + p, 3 * bi + q, d[p * 3 + q]);
        }
      }
    }
    off.forEach((key, blk) {
      final a = key ~/ m, c = key % m;
      for (var p = 0; p < 3; p++) {
        for (var q = 0; q < 3; q++) {
          put(3 * c + p, 3 * a + q, blk[q * 3 + p]); // lower part: H_ca = H_ac transposed
        }
      }
    });
    // Cholesky in place: L L^T
    for (var i = 0; i < nn; i++) {
      final fi = first[i];
      for (var j = fi; j <= i; j++) {
        var sum = get(i, j);
        final k0 = math.max(fi, first[j]);
        final oi = start[i] - fi, oj = start[j] - first[j];
        for (var k = k0; k < j; k++) {
          sum -= l[oi + k] * l[oj + k];
        }
        if (j < i) {
          put(i, j, sum / get(j, j));
        } else {
          if (sum <= 1e-18) return null;
          put(i, i, math.sqrt(sum));
        }
      }
    }
    // forward: L z = b
    final z = Float64List(nn);
    for (var i = 0; i < nn; i++) {
      var sum = b[i];
      final oi = start[i] - first[i];
      for (var k = first[i]; k < i; k++) {
        sum -= l[oi + k] * z[k];
      }
      z[i] = sum / get(i, i);
    }
    // backward: L^T x = z
    final xs = Float64List.fromList(z);
    for (var i = nn - 1; i >= 0; i--) {
      xs[i] /= get(i, i);
      final xi = xs[i], oi = start[i] - first[i];
      for (var k = first[i]; k < i; k++) {
        xs[k] -= l[oi + k] * xi;
      }
    }
    return xs;
  }

  /// Solve H d = b (H block-sparse symmetric) with block-Jacobi preconditioned conjugate gradients.
  static Float64List _pcg(List<Float64List> hd, Map<int, Float64List> off, Float64List b, int m) {
    final inv = [for (final blk in hd) _inv3(blk)];
    final xs = Float64List(3 * m);
    final r = Float64List.fromList(b);
    Float64List prec(Float64List v) {
      final out = Float64List(3 * m);
      for (var i = 0; i < m; i++) {
        final iv = inv[i];
        for (var p = 0; p < 3; p++) {
          out[i * 3 + p] = iv[p * 3] * v[i * 3] + iv[p * 3 + 1] * v[i * 3 + 1] + iv[p * 3 + 2] * v[i * 3 + 2];
        }
      }
      return out;
    }

    Float64List mul(Float64List v) {
      final out = Float64List(3 * m);
      for (var i = 0; i < m; i++) {
        final d = hd[i];
        for (var p = 0; p < 3; p++) {
          out[i * 3 + p] += d[p * 3] * v[i * 3] + d[p * 3 + 1] * v[i * 3 + 1] + d[p * 3 + 2] * v[i * 3 + 2];
        }
      }
      off.forEach((key, blk) {
        final i = key ~/ m, j = key % m;
        for (var p = 0; p < 3; p++) {
          out[i * 3 + p] += blk[p * 3] * v[j * 3] + blk[p * 3 + 1] * v[j * 3 + 1] + blk[p * 3 + 2] * v[j * 3 + 2];
          out[j * 3 + p] += blk[p] * v[i * 3] + blk[3 + p] * v[i * 3 + 1] + blk[6 + p] * v[i * 3 + 2];
        }
      });
      return out;
    }

    double dot(Float64List a, Float64List c) {
      var s = 0.0;
      for (var i = 0; i < a.length; i++) {
        s += a[i] * c[i];
      }
      return s;
    }

    var z = prec(r);
    var pdir = Float64List.fromList(z);
    var rz = dot(r, z);
    final r0 = math.sqrt(dot(r, r));
    if (r0 < 1e-14) return xs;
    for (var k = 0; k < 3 * m + 50 && k < 2000; k++) {
      final ap = mul(pdir);
      final alpha = rz / dot(pdir, ap);
      for (var i = 0; i < xs.length; i++) {
        xs[i] += alpha * pdir[i];
        r[i] -= alpha * ap[i];
      }
      if (math.sqrt(dot(r, r)) < 1e-10 * r0) break;
      z = prec(r);
      final rzNew = dot(r, z);
      final beta = rzNew / rz;
      rz = rzNew;
      for (var i = 0; i < pdir.length; i++) {
        pdir[i] = z[i] + beta * pdir[i];
      }
    }
    return xs;
  }

  static Float64List _inv3(Float64List a) {
    final det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) + a[2] * (a[3] * a[7] - a[4] * a[6]);
    final out = Float64List(9);
    if (det.abs() < 1e-30) {
      for (var d = 0; d < 3; d++) {
        out[d * 4] = a[d * 4] == 0 ? 0 : 1 / a[d * 4];
      }
      return out;
    }
    final id = 1 / det;
    out[0] = (a[4] * a[8] - a[5] * a[7]) * id;
    out[1] = (a[2] * a[7] - a[1] * a[8]) * id;
    out[2] = (a[1] * a[5] - a[2] * a[4]) * id;
    out[3] = (a[5] * a[6] - a[3] * a[8]) * id;
    out[4] = (a[0] * a[8] - a[2] * a[6]) * id;
    out[5] = (a[2] * a[3] - a[0] * a[5]) * id;
    out[6] = (a[3] * a[7] - a[4] * a[6]) * id;
    out[7] = (a[1] * a[6] - a[0] * a[7]) * id;
    out[8] = (a[0] * a[4] - a[1] * a[3]) * id;
    return out;
  }
}

/// Dominant wall direction in a lidar scan (robot frame), modulo 90 degrees, with a strength 0..1.
/// Uses short straight runs of consecutive points; angles are folded with the 4x trick so walls at
/// right angles reinforce each other.
class WallDirection {
  /// pts: robot-frame points in scan order (x forward, y left).
  static (double, double) ofScan(List<(double, double)> pts) {
    var c4 = 0.0, s4 = 0.0, wsum = 0.0;
    for (var i = 0; i + 4 < pts.length; i++) {
      final a = pts[i], m = pts[i + 2], b = pts[i + 4];
      final dx = b.$1 - a.$1, dy = b.$2 - a.$2;
      final len = math.sqrt(dx * dx + dy * dy);
      if (len < 0.04 || len > 0.4) continue; // not neighbours on one surface
      // the middle point must sit on the line: a straight piece of wall
      final dev = ((m.$1 - a.$1) * dy - (m.$2 - a.$2) * dx).abs() / len;
      if (dev > 0.012) continue;
      final phi = math.atan2(dy, dx);
      c4 += len * math.cos(4 * phi);
      s4 += len * math.sin(4 * phi);
      wsum += len;
    }
    if (wsum < 0.5) return (0, 0); // too little straight wall in view
    return (math.atan2(s4, c4) / 4, math.sqrt(c4 * c4 + s4 * s4) / wsum);
  }

  /// Angle difference folded into (-45, 45] degrees.
  static double wrap90(double a) {
    const q = math.pi / 2;
    var d = a % q;
    if (d > q / 2) d -= q;
    if (d <= -q / 2) d += q;
    return d;
  }
}
