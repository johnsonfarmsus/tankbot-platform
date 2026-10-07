// Compact map: what the robot stores for its brains (and what a new brain starts from).
//
// 'TCM1' | u32 header length | header JSON (utf-8) | zlib(cells)
// cells: one byte per grid cell in the map's bounding box: 0 unexplored, 1 open floor, 2 wall.
// The header carries the map's identity, its edits (no-go lines, erasures, marks), the lidar offset and
// the last pose. Raw keyframes are not included: they are only needed to re-straighten old areas, and
// a brain builds fresh ones as it drives.
import 'dart:convert';
import 'dart:io' show ZLibCodec;
import 'dart:typed_data';

import 'occupancy_grid.dart';

class CompactMap {
  CompactMap(this.header, this.cells);
  final Map<String, dynamic> header;
  final Uint8List cells;

  String get id => header['id'] as String;
  String get name => (header['name'] as String?) ?? 'Robot map';
  int get w => (header['w'] as num).toInt();
  int get h => (header['h'] as num).toInt();
  int get x0 => (header['x0'] as num).toInt(); // cell index relative to the grid centre
  int get y0 => (header['y0'] as num).toInt();
  double get res => (header['res'] as num).toDouble();

  /// Pack the grid's current state with the map's identity and edits.
  static Uint8List encode(OccupancyGrid g, Map<String, dynamic> header) {
    final (x0, y0, w, h, cells) = g.exportKinds();
    final hdr = Map<String, dynamic>.from(header)
      ..['v'] = 1
      ..['res'] = g.resolution
      ..['x0'] = x0
      ..['y0'] = y0
      ..['w'] = w
      ..['h'] = h;
    final hb = utf8.encode(jsonEncode(hdr));
    final z = ZLibCodec(level: 9).encode(cells);
    final out = BytesBuilder()
      ..add(ascii.encode('TCM1'))
      ..add((ByteData(4)..setUint32(0, hb.length, Endian.little)).buffer.asUint8List())
      ..add(hb)
      ..add(z);
    return out.toBytes();
  }

  static CompactMap? decode(Uint8List b) {
    try {
      if (b.length < 8 || ascii.decode(b.sublist(0, 4)) != 'TCM1') return null;
      final hl = ByteData.sublistView(b, 4, 8).getUint32(0, Endian.little);
      final header = jsonDecode(utf8.decode(b.sublist(8, 8 + hl))) as Map<String, dynamic>;
      final cells = Uint8List.fromList(ZLibCodec().decode(b.sublist(8 + hl)));
      final m = CompactMap(header, cells);
      if (cells.length != m.w * m.h) return null;
      return m;
    } catch (_) {
      return null;
    }
  }

  /// Lay this map into a grid as its base layer (before any keyframes or edits).
  void applyTo(OccupancyGrid g) => g.loadKinds(x0, y0, w, h, cells, res);
}
