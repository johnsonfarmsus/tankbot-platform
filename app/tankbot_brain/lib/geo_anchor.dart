// Where a map is in the world. A map keeps its own metres (x, y); the anchor turns them into
// latitude / longitude and back: local east / north (metres from lat0, lon0) = rotate(map) + offset.
// The scale is 1 (the lidar measures true metres).
//
// The anchor is fitted automatically from GPS fixes paired with where the robot was on the map at
// the time (only good fixes, only while tracking is good). It sharpens as the fixes spread out: the
// position comes from their average, the rotation from how far apart they are.
import 'dart:math' as math;

class GeoSample {
  GeoSample(this.x, this.y, this.lat, this.lon, this.acc);
  final double x, y, lat, lon, acc; // map metres; degrees; GPS horizontal accuracy in metres

  Map<String, dynamic> toJson() => {'x': x, 'y': y, 'lat': lat, 'lon': lon, 'acc': acc};
  static GeoSample? fromJson(dynamic j) {
    if (j is! Map) return null;
    double? n(String k) => (j[k] as num?)?.toDouble();
    final x = n('x'), y = n('y'), lat = n('lat'), lon = n('lon'), acc = n('acc');
    if (x == null || y == null || lat == null || lon == null || acc == null) return null;
    return GeoSample(x, y, lat, lon, acc);
  }
}

class GeoAnchor {
  GeoAnchor({
    required this.lat0,
    required this.lon0,
    required this.phi,
    required this.e0,
    required this.n0,
    required this.cx,
    required this.cy,
    required this.sigmaM,
    required this.sigmaRot,
    required this.samples,
    required this.spanM,
    required this.updatedMs,
  });

  final double lat0, lon0; // local east / north origin
  final double phi; // rotation from map axes to east / north (radians, counter-clockwise)
  final double e0, n0; // offset: map origin in east / north metres
  final double cx, cy; // where the fixes are centred, in map metres (the anchor is best there)
  final double sigmaM; // position uncertainty at the centre (1 sigma, metres)
  final double sigmaRot; // rotation uncertainty (radians)
  final int samples;
  final double spanM; // how far apart the fixes are
  final int updatedMs;

  static const _mLat = 111320.0;
  double get _mLon => _mLat * math.cos(lat0 * math.pi / 180);

  /// Map metres -> (latitude, longitude).
  (double, double) toLatLon(double x, double y) {
    final c = math.cos(phi), s = math.sin(phi);
    final e = c * x - s * y + e0, n = s * x + c * y + n0;
    return (lat0 + n / _mLat, lon0 + e / _mLon);
  }

  /// (latitude, longitude) -> map metres.
  (double, double) toMap(double lat, double lon) {
    final e = (lon - lon0) * _mLon - e0, n = (lat - lat0) * _mLat - n0;
    final c = math.cos(phi), s = math.sin(phi);
    return (c * e + s * n, -s * e + c * n);
  }

  /// Uncertainty of a converted point, metres: the centre's, plus the rotation's effect with distance.
  double sigmaAt(double x, double y) => sigmaM + math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) * sigmaRot;

  /// Compass bearing (degrees clockwise from north) of a map heading (radians, counter-clockwise from +x).
  double bearingDeg(double heading) {
    final b = 90 - (heading + phi) * 180 / math.pi;
    return ((b % 360) + 360) % 360;
  }

  Map<String, dynamic> toJson() => {
        'lat0': lat0, 'lon0': lon0, 'phi': phi, 'e0': e0, 'n0': n0, 'cx': cx, 'cy': cy,
        'sigmaM': sigmaM, 'sigmaRot': sigmaRot, 'samples': samples, 'spanM': spanM, 'updatedMs': updatedMs,
      };

  static GeoAnchor? fromJson(dynamic j) {
    if (j is! Map) return null;
    double n(String k) => (j[k] as num?)?.toDouble() ?? double.nan;
    final a = GeoAnchor(
      lat0: n('lat0'), lon0: n('lon0'), phi: n('phi'), e0: n('e0'), n0: n('n0'), cx: n('cx'), cy: n('cy'),
      sigmaM: n('sigmaM'), sigmaRot: n('sigmaRot'),
      samples: (j['samples'] as num?)?.toInt() ?? 0, spanM: n('spanM'), updatedMs: (j['updatedMs'] as num?)?.toInt() ?? 0,
    );
    return [a.lat0, a.lon0, a.phi, a.e0, a.n0, a.sigmaM, a.sigmaRot].any((v) => v.isNaN) ? null : a;
  }

  /// Fit from GPS samples. Needs at least [minSamples] fixes spread over [minSpanM] metres, else null.
  static GeoAnchor? fit(List<GeoSample> s, {int nowMs = 0, int minSamples = 5, double minSpanM = 15}) {
    if (s.length < minSamples) return null;
    final lat0 = s.map((g) => g.lat).reduce((a, b) => a + b) / s.length;
    final lon0 = s.map((g) => g.lon).reduce((a, b) => a + b) / s.length;
    final mLon = _mLat * math.cos(lat0 * math.pi / 180);
    final e = [for (final g in s) (g.lon - lon0) * mLon];
    final n = [for (final g in s) (g.lat - lat0) * _mLat];
    final w = [for (final g in s) 1 / (g.acc.clamp(1.0, 50.0) * g.acc.clamp(1.0, 50.0))];
    final sw = w.reduce((a, b) => a + b);
    var mx = 0.0, my = 0.0, me = 0.0, mn = 0.0;
    for (var i = 0; i < s.length; i++) {
      mx += w[i] * s[i].x;
      my += w[i] * s[i].y;
      me += w[i] * e[i];
      mn += w[i] * n[i];
    }
    mx /= sw;
    my /= sw;
    me /= sw;
    mn /= sw;
    // spread of the fixes on the map (sets how well the rotation is known)
    var span = 0.0, spread2 = 0.0, sxx = 0.0, sxy = 0.0;
    for (var i = 0; i < s.length; i++) {
      final ax = s[i].x - mx, ay = s[i].y - my, bx = e[i] - me, by = n[i] - mn;
      sxx += w[i] * (ax * bx + ay * by);
      sxy += w[i] * (ax * by - ay * bx);
      spread2 += w[i] * (ax * ax + ay * ay);
      for (var k = i + 1; k < s.length; k += math.max(1, s.length ~/ 40)) {
        span = math.max(span, math.sqrt(math.pow(s[i].x - s[k].x, 2) + math.pow(s[i].y - s[k].y, 2)));
      }
    }
    if (span < minSpanM) return null;
    final phi = math.atan2(sxy, sxx);
    final c = math.cos(phi), sn = math.sin(phi);
    final e0 = me - (c * mx - sn * my), n0 = mn - (sn * mx + c * my);
    // residuals: how well the fixes agree with one rigid placement
    var res2 = 0.0;
    for (var i = 0; i < s.length; i++) {
      final pe = c * s[i].x - sn * s[i].y + e0 - e[i], pn = sn * s[i].x + c * s[i].y + n0 - n[i];
      res2 += w[i] * (pe * pe + pn * pn);
    }
    final rms = math.sqrt(res2 / sw);
    final meanAcc = s.map((g) => g.acc).reduce((a, b) => a + b) / s.length;
    final perFix = math.max(rms, meanAcc * 0.7);
    // GPS errors are correlated over time and short distances: count roughly one independent fix per 5 m
    final nEff = math.max(1.0, math.min(s.length.toDouble(), span / 5 + 1));
    final sigmaM = perFix / math.sqrt(nEff);
    final spreadRms = math.sqrt(spread2 / sw);
    final sigmaRot = spreadRms > 0.5 ? perFix / (spreadRms * math.sqrt(nEff)) : math.pi;
    return GeoAnchor(
      lat0: lat0, lon0: lon0, phi: phi, e0: e0, n0: n0, cx: mx, cy: my,
      sigmaM: sigmaM, sigmaRot: sigmaRot, samples: s.length, spanM: span, updatedMs: nowMs,
    );
  }
}

/// "47.48120, -118.25510", "47.4812 -118.2551", "47.4812°N 118.2551°W" -> (lat, lon), else null.
(double, double)? parseLatLon(String text) {
  final t = text.trim().toUpperCase();
  final nums = RegExp(r'-?\d+(?:\.\d+)?').allMatches(t).map((m) => double.parse(m.group(0)!)).toList();
  if (nums.length != 2) return null;
  var lat = nums[0], lon = nums[1];
  if (t.contains('S') && lat > 0) lat = -lat;
  if (t.contains('W') && lon > 0) lon = -lon;
  if (lat.abs() > 90 || lon.abs() > 180) return null;
  return (lat, lon);
}
