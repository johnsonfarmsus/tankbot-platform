// Route planning on the occupancy grid.
//  - obstacles: mapped walls, no-go lines, and live lidar points
//  - every obstacle is padded by the robot's radius; routes prefer extra clearance
//  - only floor the map KNOWS is open is allowed (unknown = off-limits)
//  - A* search, then line-of-sight smoothing into a few waypoints
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' show Offset;
import 'occupancy_grid.dart';

class PlanResult {
  final List<Offset> path;
  final String? error;
  const PlanResult(this.path, [this.error]);
}

class _Heap {
  final List<double> k = [];
  final List<int> v = [];
  bool get isEmpty => v.isEmpty;

  void push(double key, int val) {
    k.add(key);
    v.add(val);
    var i = k.length - 1;
    while (i > 0) {
      final p = (i - 1) >> 1;
      if (k[p] <= k[i]) break;
      final tk = k[p], tv = v[p];
      k[p] = k[i];
      v[p] = v[i];
      k[i] = tk;
      v[i] = tv;
      i = p;
    }
  }

  int pop() {
    final top = v[0];
    final lk = k.removeLast(), lv = v.removeLast();
    if (v.isNotEmpty) {
      k[0] = lk;
      v[0] = lv;
      var i = 0;
      while (true) {
        final l = 2 * i + 1, r = l + 1;
        var m = i;
        if (l < k.length && k[l] < k[m]) m = l;
        if (r < k.length && k[r] < k[m]) m = r;
        if (m == i) break;
        final tk = k[m], tv = v[m];
        k[m] = k[i];
        v[m] = v[i];
        k[i] = tk;
        v[i] = tv;
        i = m;
      }
    }
    return top;
  }
}

class Planner {
  // TankBot footprint treated as a 200 x 200 mm square: half-diagonal 14 cm + 5 cm safety margin
  static const double robotRadiusM = 0.19;
  static const double comfortM = 0.45; // prefer at least this much clearance when there is room
  static const double startEscapeM = 0.30; // allow leaving a tight spot near the start

  static PlanResult plan(OccupancyGrid g, List<List<double>> nogo, List<Offset> live,
      double sx, double sy, double gx, double gy, {double robotRadius = robotRadiusM}) {
    if (g.maxCx < 0) return const PlanResult([], 'The map is empty');
    final res = g.resolution;
    const margin = 12;
    final x0 = math.max(0, g.minCx - margin), y0 = math.max(0, g.minCy - margin);
    final x1 = math.min(g.size - 1, g.maxCx + margin), y1 = math.min(g.size - 1, g.maxCy + margin);
    final w = x1 - x0 + 1, h = y1 - y0 + 1, n = w * h;
    int cx(double m) => g.toCell(m) - x0;
    int cy(double m) => g.toCell(m) - y0;
    double wx(int x) => g.cellToWorld(x + x0) + res / 2;
    double wy(int y) => g.cellToWorld(y + y0) + res / 2;
    bool inside(int x, int y) => x >= 0 && y >= 0 && x < w && y < h;

    final lo = Float32List(n);
    final obs = Uint8List(n);
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        final v = g.cellLo(x + x0, y + y0);
        lo[y * w + x] = v;
        if (v > 0.5) obs[y * w + x] = 1;
      }
    }
    void mark(double mx, double my) {
      final x = cx(mx), y = cy(my);
      if (inside(x, y)) obs[y * w + x] = 1;
    }
    for (final l in nogo) {
      final len = math.sqrt((l[2] - l[0]) * (l[2] - l[0]) + (l[3] - l[1]) * (l[3] - l[1]));
      final steps = math.max(1, (len / (res * 0.5)).ceil());
      for (var k = 0; k <= steps; k++) {
        mark(l[0] + (l[2] - l[0]) * k / steps, l[1] + (l[3] - l[1]) * k / steps);
      }
    }
    for (final o in live) {
      mark(o.dx, o.dy);
    }

    // distance (in cells) to the nearest obstacle: chamfer transform
    final d = Float32List(n);
    for (var i = 0; i < n; i++) {
      d[i] = obs[i] == 1 ? 0 : 1e9;
    }
    const a = 1.0, b = 1.41421356;
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
    final rC = robotRadius / res, comfortC = math.max(comfortM, robotRadius + 0.2) / res, escC = startEscapeM / res;

    final sX = cx(sx), sY = cy(sy);
    if (!inside(sX, sY)) return const PlanResult([], "I'm not on the map");
    bool nearStart(int x, int y) => (x - sX) * (x - sX) + (y - sY) * (y - sY) <= escC * escC;
    bool free(int x, int y) {
      if (!inside(x, y)) return false;
      final i = y * w + x;
      if (obs[i] == 1) return false;
      if (nearStart(x, y)) return true;
      return lo[i] < -0.5 && d[i] >= rC;
    }

    // goal: must be open floor with room for the robot; snap to the nearest such cell within 30 cm
    var gX = cx(gx), gY = cy(gy);
    if (!free(gX, gY)) {
      final r = (0.3 / res).ceil();
      var best = -1, bestD = 1e9;
      for (var dy = -r; dy <= r; dy++) {
        for (var dx = -r; dx <= r; dx++) {
          final dd = (dx * dx + dy * dy).toDouble();
          if (dd <= r * r && dd < bestD && free(gX + dx, gY + dy)) {
            bestD = dd;
            best = (gY + dy) * w + (gX + dx);
          }
        }
      }
      if (best < 0) return const PlanResult([], "That spot isn't open floor the robot fits in (too close to a wall, or not mapped yet)");
      gX = best % w;
      gY = best ~/ w;
    }

    // A*
    final gs = Float32List(n)..fillRange(0, n, double.infinity);
    final parent = Int32List(n)..fillRange(0, n, -1);
    final closed = Uint8List(n);
    final heap = _Heap();
    final start = sY * w + sX, goal = gY * w + gX;
    double hEst(int i) {
      final dx = (i % w - gX).toDouble(), dy = (i ~/ w - gY).toDouble();
      return math.sqrt(dx * dx + dy * dy);
    }
    gs[start] = 0;
    heap.push(hEst(start), start);
    const dxs = [1, -1, 0, 0, 1, 1, -1, -1], dys = [0, 0, 1, -1, 1, -1, 1, -1];
    const lens = [1.0, 1.0, 1.0, 1.0, 1.41421356, 1.41421356, 1.41421356, 1.41421356];
    var found = false;
    while (!heap.isEmpty) {
      final cur = heap.pop();
      if (closed[cur] == 1) continue;
      closed[cur] = 1;
      if (cur == goal) {
        found = true;
        break;
      }
      final x = cur % w, y = cur ~/ w;
      for (var k = 0; k < 8; k++) {
        final nx = x + dxs[k], ny = y + dys[k];
        if (!free(nx, ny)) continue;
        final ni = ny * w + nx;
        if (closed[ni] == 1) continue;
        final dist = d[ni];
        var prox = 0.0;
        if (dist < comfortC) prox = ((comfortC - dist) / math.max(1e-6, comfortC - rC)).clamp(0.0, 1.0);
        final cost = gs[cur] + lens[k] * (1 + 3 * prox * prox);
        if (cost < gs[ni]) {
          gs[ni] = cost;
          parent[ni] = cur;
          heap.push(cost + hEst(ni), ni);
        }
      }
    }
    if (!found) return const PlanResult([], "I can't find a way there (walls, no-go lines or unmapped floor in the way)");

    final cells = <int>[];
    for (var c = goal; c != -1; c = parent[c]) {
      cells.add(c);
    }
    final path = cells.reversed.toList();

    // line of sight between cells: every cell on the straight line must be free
    bool los(int a, int b) {
      var x = a % w, y = a ~/ w;
      final xe = b % w, ye = b ~/ w;
      final dx = (xe - x).abs(), dy = -(ye - y).abs();
      final sxx = x < xe ? 1 : -1, syy = y < ye ? 1 : -1;
      var err = dx + dy;
      while (true) {
        if (!free(x, y)) return false;
        if (x == xe && y == ye) return true;
        final e2 = 2 * err;
        if (e2 >= dy) {
          err += dy;
          x += sxx;
        }
        if (e2 <= dx) {
          err += dx;
          y += syy;
        }
      }
    }
    final out = <Offset>[Offset(sx, sy)];
    var i = 0;
    while (i < path.length - 1) {
      var j = math.min(path.length - 1, i + 80); // look up to ~4 m ahead
      while (j > i + 1 && !los(path[i], path[j])) {
        j--;
      }
      out.add(Offset(wx(path[j] % w), wy(path[j] ~/ w)));
      i = j;
    }
    return PlanResult(out);
  }
}
