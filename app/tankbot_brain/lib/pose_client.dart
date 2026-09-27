// Phone pose from ARKit (iOS). Android/ARCore to come.
// All times are in app-clock milliseconds (see appClockMs).
import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/services.dart';

/// One monotonic clock for the whole app, so lidar scans and poses can be matched.
final Stopwatch _appClock = Stopwatch()..start();
double appClockMs() => _appClock.elapsedMicroseconds / 1000.0;

/// 2D pose on the floor. Map frame: x = ARKit x, y = -ARKit z (so +y is the
/// direction the phone faced when tracking started). heading is radians,
/// counter-clockwise from +x.
class Pose {
  final double t, x, y, heading;
  final bool good;
  const Pose(this.t, this.x, this.y, this.heading, this.good);
}

class PoseClient {
  static const _events = EventChannel('tankbot/arkit_pose');
  static const _methods = MethodChannel('tankbot/arkit');

  final List<Pose> history = [];
  final _ctrl = StreamController<Pose>.broadcast();
  StreamSubscription? _sub;
  double? _offsetMs; // app clock minus iOS uptime clock
  String state = 'off';
  Stream<Pose> get poses => _ctrl.stream;
  Pose? get latest => history.isEmpty ? null : history.last;

  Future<bool> start() async {
    try {
      final ok = await _methods.invokeMethod<bool>('supported') ?? false;
      if (!ok) {
        state = 'ARKit not supported on this phone';
        return false;
      }
    } on MissingPluginException {
      state = 'Position tracking not available on this platform yet';
      return false;
    }
    state = 'starting';
    _sub = _events.receiveBroadcastStream().listen(_onEvent, onError: (e) => state = 'error: $e');
    return true;
  }

  Future<void> reset() async {
    history.clear();
    try {
      await _methods.invokeMethod('reset');
    } catch (_) {}
  }

  void _onEvent(dynamic e) {
    final m = Map<String, dynamic>.from(e as Map);
    final now = appClockMs();
    final sentMs = (m['sent'] as num) * 1000.0;
    // Delivery latency is always >= 0, so the smallest (now - sent) is the best offset estimate.
    // Let it creep up slowly so a one-off glitch cannot stick forever.
    final off = now - sentMs;
    _offsetMs = _offsetMs == null ? off : math.min(off, _offsetMs! + 0.05);
    final t = (m['t'] as num) * 1000.0 + _offsetMs!;
    final x = (m['x'] as num).toDouble();
    final z = (m['z'] as num).toDouble();
    final fx = (m['fx'] as num).toDouble();
    final fz = (m['fz'] as num).toDouble();
    state = m['state'] as String;
    final flat = fx * fx + fz * fz; // camera must look roughly horizontal for a heading
    final p = Pose(t, x, -z, math.atan2(-fz, fx), state == 'normal' && flat > 0.25);
    history.add(p);
    while (history.isNotEmpty && t - history.first.t > 5000) {
      history.removeAt(0);
    }
    _ctrl.add(p);
  }

  /// Pose at app-clock time t, interpolated. Null if t is outside the recent history.
  Pose? at(double t) {
    if (history.length < 2) return null;
    if (t < history.first.t || t > history.last.t) return null;
    var lo = 0, hi = history.length - 1;
    while (hi - lo > 1) {
      final mid = (lo + hi) >> 1;
      if (history[mid].t <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    final a = history[lo], b = history[hi];
    final span = b.t - a.t;
    final f = span <= 0 ? 0.0 : (t - a.t) / span;
    var dh = b.heading - a.heading;
    while (dh > math.pi) {
      dh -= 2 * math.pi;
    }
    while (dh < -math.pi) {
      dh += 2 * math.pi;
    }
    return Pose(t, a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.heading + dh * f, a.good && b.good);
  }

  void dispose() {
    _sub?.cancel();
    _ctrl.close();
  }
}
