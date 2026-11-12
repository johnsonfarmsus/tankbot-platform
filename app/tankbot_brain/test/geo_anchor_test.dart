import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/geo_anchor.dart';

/// A made-up place (not anyone's address): the true anchor the fit should recover.
const lat0 = 45.0, lon0 = -120.0;
const truePhi = 0.6; // the map's x axis points 34 degrees north of east
const trueE0 = 12.0, trueN0 = -7.0;

(double, double) truth(double x, double y) {
  final c = math.cos(truePhi), s = math.sin(truePhi);
  final e = c * x - s * y + trueE0, n = s * x + c * y + trueN0;
  return (lat0 + n / 111320.0, lon0 + e / (111320.0 * math.cos(lat0 * math.pi / 180)));
}

/// A drive out of a front door and down a sidewalk: GPS starts once outdoors, fixes every ~2 m,
/// each off by phone-grade noise (and errors that drift slowly, like real GPS).
List<GeoSample> drive(double length, {int seed = 1, double acc = 4}) {
  final r = math.Random(seed);
  final out = <GeoSample>[];
  var be = 0.0, bn = 0.0; // slowly wandering bias
  for (var d = 0.0; d <= length; d += 2) {
    final x = 3 + d, y = 2 + 0.03 * d; // straight along the sidewalk
    be = 0.9 * be + 0.1 * (r.nextDouble() - 0.5) * 2 * acc;
    bn = 0.9 * bn + 0.1 * (r.nextDouble() - 0.5) * 2 * acc;
    final (la, lo) = truth(x, y);
    final mLon = 111320.0 * math.cos(lat0 * math.pi / 180);
    out.add(GeoSample(x, y, la + (bn + (r.nextDouble() - 0.5) * acc) / 111320.0, lo + (be + (r.nextDouble() - 0.5) * acc) / mLon, acc));
  }
  return out;
}

double errM(GeoAnchor a, double x, double y) {
  final (la, lo) = a.toLatLon(x, y);
  final (tl, tn) = truth(x, y);
  final mLon = 111320.0 * math.cos(lat0 * math.pi / 180);
  return math.sqrt(math.pow((la - tl) * 111320.0, 2) + math.pow((lo - tn) * mLon, 2));
}

void main() {
  test('too little GPS (few fixes or under 15 m) gives no anchor', () {
    expect(GeoAnchor.fit(drive(6)), isNull);
    expect(GeoAnchor.fit(drive(40).take(3).toList()), isNull);
  });

  test('a short sidewalk drive anchors the map to a few metres, and says so honestly', () {
    final a = GeoAnchor.fit(drive(30))!;
    final e = errM(a, 15, 2.5);
    // ignore: avoid_print
    print('30 m drive: error ${e.toStringAsFixed(1)} m, claimed ${a.sigmaM.toStringAsFixed(1)} m, rotation ${(a.sigmaRot * 180 / math.pi).toStringAsFixed(1)} deg');
    expect(e, lessThan(4));
    expect(a.sigmaM, greaterThan(0.5)); // doesn't claim more than phone GPS can give
  });

  test('a long drive (around a block) pins the rotation down; far points stay accurate', () {
    final a = GeoAnchor.fit(drive(150, seed: 3))!;
    final rotErrDeg = ((a.phi - truePhi) * 180 / math.pi).abs();
    final far = errM(a, 150, 6.5);
    // ignore: avoid_print
    print('150 m drive: rotation off ${rotErrDeg.toStringAsFixed(1)} deg (claimed ${(a.sigmaRot * 180 / math.pi).toStringAsFixed(1)}), far end off ${far.toStringAsFixed(1)} m');
    expect(rotErrDeg, lessThan(3));
    expect(far, lessThan(5));
  });

  test('map -> lat/long -> map round trip is exact', () {
    final a = GeoAnchor.fit(drive(60))!;
    for (final (x, y) in [(0.0, 0.0), (25.0, -3.0), (-10.0, 40.0)]) {
      final (la, lo) = a.toLatLon(x, y);
      final (x2, y2) = a.toMap(la, lo);
      expect((x2 - x).abs() + (y2 - y).abs(), lessThan(1e-6));
    }
  });

  test('saved and loaded anchors are identical', () {
    final a = GeoAnchor.fit(drive(60))!;
    final b = GeoAnchor.fromJson(a.toJson())!;
    expect(b.toLatLon(10, 5), a.toLatLon(10, 5));
  });

  test('coordinates are read the ways people paste them', () {
    expect(parseLatLon('45.12345, -120.5'), (45.12345, -120.5));
    expect(parseLatLon('45.12345 -120.5'), (45.12345, -120.5));
    expect(parseLatLon('45.1°N 120.5°W'), (45.1, -120.5));
    expect(parseLatLon('nonsense'), isNull);
    expect(parseLatLon('95, 10'), isNull);
  });
}
