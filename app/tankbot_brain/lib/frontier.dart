// Frontier search for autonomous exploration.
//
// A frontier is open floor next to unexplored space. Raw frontiers are useless indoors: a lidar wall
// is a jagged, broken line of cells, and every little gap "touches" the unexplored space behind it.
// So:
//   - walls close small gaps: unexplored space only counts if it is at least closeGapM/2 from any
//     wall cell (gaps narrower than closeGapM are sealed; doorways stay open)
//   - an opening must lead somewhere: enough open unexplored space within 1 m of the target
//   - rooms: closing gaps narrower than doorM (doorways) splits the floor into rooms; frontiers in
//     the robot's own room are strongly preferred, so it finishes a room before moving on
//   - the goal is the cell of the opening with the most clearance from walls
import 'dart:math' as math;
import 'dart:typed_data';

class FrontierPick {
  final double x, y;
  final int size; // cells in the opening
  final int gain; // open unexplored cells within 1 m
  final bool inRoom;
  const FrontierPick(this.x, this.y, this.size, this.gain, this.inRoom);
  @override
  String toString() => 'Frontier(${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)}, size $size, gain $gain, ${inRoom ? 'this room' : 'elsewhere'})';
}

class FrontierStats {
  int openings = 0, tooSmall = 0, leadsNowhere = 0, unreachable = 0, tried = 0, tooClose = 0;
  int drivableCells = 0;
  @override
  String toString() {
    final why = [
      if (tooSmall > 0) '$tooSmall too small',
      if (leadsNowhere > 0) '$leadsNowhere lead nowhere',
      if (unreachable > 0) '$unreachable not reachable',
      if (tried > 0) '$tried already tried',
      if (tooClose > 0) '$tooClose right here',
    ];
    return '$openings opening${openings == 1 ? '' : 's'}${why.isEmpty ? '' : ': ${why.join(', ')}'}';
  }
}

class FrontierFinder {
  /// Why the last search picked what it did (or nothing).
  static FrontierStats lastStats = FrontierStats();

  /// lo(cx, cy): occupancy log-odds of a cell (> 0.5 wall, < -0.5 open floor, else unexplored).
  /// [x0..x1, y0..y1]: cell bounds of the map. cellToWorld(c): world coordinate of a cell index.
  static FrontierPick? next({
    required double Function(int cx, int cy) lo,
    required int x0,
    required int y0,
    required int x1,
    required int y1,
    required double res,
    required double Function(int c) cellToWorld,
    required double robotX,
    required double robotY,
    required int robotCx,
    required int robotCy,
    List<(double, double)> skip = const [],
    double closeGapM = 0.6,
    double doorM = 1.0,
    double minGainM2 = 0.1,
    double robotRadiusM = 0.22,
  }) {
    const pad = 3;
    final bx0 = x0 - pad, by0 = y0 - pad;
    final w = x1 - x0 + 1 + 2 * pad, h = y1 - y0 + 1 + 2 * pad;
    final n = w * h;
    final kind = Uint8List(n); // 0 unexplored, 1 open floor, 2 wall
    for (var j = 0; j < h; j++) {
      for (var i = 0; i < w; i++) {
        final cx = bx0 + i, cy = by0 + j;
        if (cx < x0 || cx > x1 || cy < y0 || cy > y1) continue;
        final v = lo(cx, cy);
        kind[j * w + i] = v > 0.5 ? 2 : (v < -0.5 ? 1 : 0);
      }
    }
    // distance (cells, 8-neighbour) to the nearest wall, capped
    final closeR = (closeGapM / 2 / res).ceil();
    final doorR = (doorM / 2 / res).ceil();
    final cap = math.max(closeR, doorR) + 1;
    final dist = Int16List(n)..fillRange(0, n, cap);
    final queue = Int32List(n);
    var qh = 0, qt = 0;
    for (var k = 0; k < n; k++) {
      if (kind[k] == 2) {
        dist[k] = 0;
        queue[qt++] = k;
      }
    }
    while (qh < qt) {
      final k = queue[qh++];
      final d = dist[k] + 1;
      if (d >= cap) continue;
      final i = k % w, j = k ~/ w;
      for (var dj = -1; dj <= 1; dj++) {
        for (var di = -1; di <= 1; di++) {
          final ni = i + di, nj = j + dj;
          if (ni < 0 || nj < 0 || ni >= w || nj >= h) continue;
          final m = nj * w + ni;
          if (dist[m] > d) {
            dist[m] = d;
            queue[qt++] = m;
          }
        }
      }
    }
    bool openUnknown(int k) => kind[k] == 0 && dist[k] >= closeR;

    // open floor reachable from the robot through gaps at least 2*r wide
    Uint8List region(int r) {
      final out = Uint8List(n);
      var start = -1, bestD = 1 << 30;
      final ri = robotCx - bx0, rj = robotCy - by0, search = (1.0 / res).ceil();
      for (var dj = -search; dj <= search; dj++) {
        for (var di = -search; di <= search; di++) {
          final i = ri + di, j = rj + dj;
          if (i < 0 || j < 0 || i >= w || j >= h) continue;
          final k = j * w + i;
          if (kind[k] == 1 && dist[k] >= r && di * di + dj * dj < bestD) {
            bestD = di * di + dj * dj;
            start = k;
          }
        }
      }
      if (start < 0) return out;
      var qh = 0, qt = 0;
      queue[qt++] = start;
      out[start] = 1;
      while (qh < qt) {
        final k = queue[qh++];
        final i = k % w, j = k ~/ w;
        for (final (di, dj) in const [(1, 0), (-1, 0), (0, 1), (0, -1)]) {
          final ni = i + di, nj = j + dj;
          if (ni < 0 || nj < 0 || ni >= w || nj >= h) continue;
          final m = nj * w + ni;
          if (out[m] == 0 && kind[m] == 1 && dist[m] >= r) {
            out[m] = 1;
            queue[qt++] = m;
          }
        }
      }
      return out;
    }

    final room = region(doorR); // doorways closed: the robot's own room
    final drivable = region(math.max(1, (robotRadiusM / res).ceil())); // where the robot itself fits
    final stats = FrontierStats();
    lastStats = stats;
    for (var k = 0; k < n; k++) {
      stats.drivableCells += drivable[k];
    }

    // frontier cells: open floor next to open unexplored space
    final front = Uint8List(n);
    for (var j = 1; j < h - 1; j++) {
      for (var i = 1; i < w - 1; i++) {
        final k = j * w + i;
        if (kind[k] != 1) continue;
        if (openUnknown(k + 1) || openUnknown(k - 1) || openUnknown(k + w) || openUnknown(k - w)) front[k] = 1;
      }
    }

    final seen = Uint8List(n);
    final gainR = (1.0 / res).ceil();
    final minGain = (minGainM2 / (res * res)).ceil();
    final roomR = (0.25 / res).ceil();
    final driveR = (0.3 / res).ceil();
    bool near(Uint8List reg, int pi, int pj, int r) {
      for (var dj = -r; dj <= r; dj++) {
        for (var di = -r; di <= r; di++) {
          final i = pi + di, j = pj + dj;
          if (i >= 0 && j >= 0 && i < w && j < h && reg[j * w + i] == 1) return true;
        }
      }
      return false;
    }
    FrontierPick? best;
    var bestScore = double.infinity;
    for (var s = 0; s < n; s++) {
      if (front[s] == 0 || seen[s] == 1) continue;
      final cells = <int>[];
      final stack = <int>[s];
      seen[s] = 1;
      while (stack.isNotEmpty) {
        final k = stack.removeLast();
        cells.add(k);
        final i = k % w, j = k ~/ w;
        for (var dj = -1; dj <= 1; dj++) {
          for (var di = -1; di <= 1; di++) {
            final ni = i + di, nj = j + dj;
            if (ni < 0 || nj < 0 || ni >= w || nj >= h) continue;
            final m = nj * w + ni;
            if (front[m] == 1 && seen[m] == 0) {
              seen[m] = 1;
              stack.add(m);
            }
          }
        }
      }
      stats.openings++;
      if (cells.length < 6) {
        stats.tooSmall++; // too small to be an opening
        continue;
      }
      // goal: the cell with the most clearance from walls
      var pick = cells.first;
      for (final k in cells) {
        if (dist[k] > dist[pick]) pick = k;
      }
      final pi = pick % w, pj = pick ~/ w;
      if (!near(drivable, pi, pj, driveR)) {
        stats.unreachable++; // seen through a gap the robot can't fit through
        continue;
      }
      // it must lead somewhere: open unexplored space within 1 m
      var gain = 0;
      for (var dj = -gainR; dj <= gainR; dj++) {
        for (var di = -gainR; di <= gainR; di++) {
          final i = pi + di, j = pj + dj;
          if (i < 0 || j < 0 || i >= w || j >= h || di * di + dj * dj > gainR * gainR) continue;
          if (openUnknown(j * w + i)) gain++;
        }
      }
      if (gain < minGain) {
        stats.leadsNowhere++;
        continue;
      }
      final inRoom = near(room, pi, pj, roomR);
      final x = cellToWorld(bx0 + pi) + res / 2, y = cellToWorld(by0 + pj) + res / 2;
      if (skip.any((q) => math.sqrt((q.$1 - x) * (q.$1 - x) + (q.$2 - y) * (q.$2 - y)) < 0.6)) {
        stats.tried++;
        continue;
      }
      final d = math.sqrt((x - robotX) * (x - robotX) + (y - robotY) * (y - robotY));
      if (d < 0.4) {
        stats.tooClose++; // already there
        continue;
      }
      final score = d - 0.01 * cells.length - (inRoom ? 3.0 : 0.0);
      if (score < bestScore) {
        bestScore = score;
        best = FrontierPick(x, y, cells.length, gain, inRoom);
      }
    }
    return best;
  }
}
