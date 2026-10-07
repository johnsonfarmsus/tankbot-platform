import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/occupancy_grid.dart';
import 'package:tankbot_brain/compact_map.dart';
import 'package:tankbot_brain/map_store.dart';

/// A house-sized grid: rooms of open floor with walls, like a real lidar map.
OccupancyGrid house() {
  final g = OccupancyGrid();
  final rnd = math.Random(4);
  for (final room in [
    [0.0, 0.0, 5.0, 4.0],
    [5.2, 0.0, 9.0, 6.0],
    [0.0, 4.2, 5.0, 9.0],
    [-4.0, -3.0, 0.0, 2.0],
  ]) {
    for (var x = room[0]; x <= room[2]; x += 0.05) {
      for (var y = room[1]; y <= room[3]; y += 0.05) {
        g.eraseCircle(x, y, 0.03); // open floor
      }
    }
    for (var d = 0.0; d <= 1.0; d += 0.005) {
      // jagged walls
      final j = (rnd.nextDouble() - 0.5) * 0.04;
      g.markCircle(room[0] + (room[2] - room[0]) * d, room[1] + j, 0.03);
      g.markCircle(room[0] + (room[2] - room[0]) * d, room[3] + j, 0.03);
      g.markCircle(room[0] + j, room[1] + (room[3] - room[1]) * d, 0.03);
      g.markCircle(room[2] + j, room[1] + (room[3] - room[1]) * d, 0.03);
    }
  }
  return g;
}

void main() {
  test('compact map round trip: every cell, the edits and the identity survive, and it is small', () {
    final g = house();
    final edits = [
      {'type': 'nogo', 'id': 1, 'stroke': 's1', 'x1': 1.0, 'y1': 1.0, 'x2': 2.0, 'y2': 1.0},
      {'type': 'erase', 'id': 2, 'stroke': 's2', 'x': 3.0, 'y': 3.0, 'r': 0.2},
    ];
    final bytes = CompactMap.encode(g, {'id': 'm123', 'name': 'House', 'edits': edits, 'nextEditId': 3, 'lastPose': [1.0, 2.0, 0.5]});
    final cm = CompactMap.decode(bytes)!;
    final (x0, y0, w, h, cells) = g.exportKinds();
    // ignore: avoid_print
    print('house ${(w * 0.05).toStringAsFixed(1)} x ${(h * 0.05).toStringAsFixed(1)} m: ${w * h} cells -> ${bytes.length} bytes compact');
    expect(cm.id, 'm123');
    expect(cm.name, 'House');
    expect(cm.cells, cells);
    expect((cm.x0, cm.y0, cm.w, cm.h), (x0, y0, w, h));
    expect(bytes.length, lessThan(60 * 1024));

    final m = MapSession.fromCompact(cm, bytes);
    expect(m.edits.length, 2);
    expect(m.edits.first['type'], 'nogo');
    expect(m.nextEditId, 3);
    expect(m.lastPose, [1.0, 2.0, 0.5]);
  });

  test('a new grid built from the compact map matches the original, and counts as an established map', () {
    final g = house();
    final cm = CompactMap.decode(CompactMap.encode(g, {'id': 'm1', 'name': 'House'}))!;
    final g2 = OccupancyGrid();
    cm.applyTo(g2);
    expect(g2.exportKinds().$5, g.exportKinds().$5);
    expect(g2.scansIntegrated, greaterThanOrEqualTo(15)); // matching and relocalisation will use it
    // walls and open floor read the same through the normal accessors
    expect(g2.at(2.5, 0.0) > 0.5, g.at(2.5, 0.0) > 0.5);
    expect(g2.at(2.5, 2.0) < -0.5, isTrue);
  });

  test('garbage is rejected, not half-loaded', () {
    expect(CompactMap.decode(CompactMap.encode(house(), {'id': 'x'}).sublist(0, 50)), isNull);
  });
}
