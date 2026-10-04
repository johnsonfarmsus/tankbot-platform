// Guardian: one decision, "is forward motion clear, and if not, why?", using the robot's real
// geometry from the bot profile. Manual obstacle stop, tap-to-go and the banner all ask this.
import 'dart:math' as math;
import 'dart:ui' show Offset;
import 'bot_profile.dart';
import 'lidar_client.dart';

class GuardVerdict {
  final bool forwardClear;
  final String reason; // '' when clear
  final double? frontMm; // nearest thing ahead of the body's front edge, within its width
  const GuardVerdict(this.forwardClear, this.reason, this.frontMm);
}

class Guardian {
  /// Points closer to the lidar than this are the robot itself or noise.
  static const double selfMaskM = 0.12;
  /// Extra width checked on each side of the body.
  static const double sideMarginM = 0.03;

  static GuardVerdict evaluate({
    required BotProfile profile,
    required LidarScan? scan,
    required bool scanStale,
    required double stopDistMm,
    String reflexBlock = 'none',
    bool lidarExpected = true,
    List<Offset> depthObstacles = const [], // platform frame (fwd, left)
    List<Offset> dropOffs = const [],
    double? depthStopMm,
  }) {
    final dStop = depthStopMm ?? stopDistMm;
    if (reflexBlock != 'none') return GuardVerdict(false, 'robot reflex: $reflexBlock', null);
    final frontEdge0 = profile.lengthMm / 2000.0, halfWidth0 = profile.widthMm / 2000.0 + sideMarginM;
    double? depthNearest;
    for (final o in depthObstacles) {
      if (o.dy.abs() > halfWidth0) continue;
      final ahead = o.dx - frontEdge0;
      if (ahead > 0 && (depthNearest == null || ahead < depthNearest)) depthNearest = ahead;
    }
    double? dropNearest;
    for (final o in dropOffs) {
      if (o.dy.abs() > halfWidth0 + 0.05) continue;
      final ahead = o.dx - frontEdge0;
      if (ahead > 0 && (dropNearest == null || ahead < dropNearest)) dropNearest = ahead;
    }
    if (dropNearest != null && dropNearest * 1000 < math.max(stopDistMm, 400)) {
      return GuardVerdict(false, 'drop-off ahead (${(dropNearest * 1000).round()} mm)', dropNearest * 1000);
    }
    if (depthNearest != null && depthNearest * 1000 < dStop) {
      return GuardVerdict(false, 'depth camera: ${(depthNearest * 1000).round()} mm ahead', depthNearest * 1000);
    }
    if (!lidarExpected) return const GuardVerdict(true, '', null);
    if (scan == null || scanStale) return const GuardVerdict(false, 'no fresh lidar data', null);

    // lidar position in the platform frame (x forward, y left, origin at platform centre)
    final lidar = profile.byType('lidar');
    final lx = lidar == null ? 0.0 : profile.fwdM(lidar);
    final ly = lidar == null ? 0.0 : profile.leftM(lidar);
    final frontEdge = profile.lengthMm / 2000.0;
    final halfWidth = profile.widthMm / 2000.0 + sideMarginM;

    double? nearest;
    for (final p in scan.points) {
      final d = p.distMm / 1000.0;
      if (d < selfMaskM) continue;
      final a = p.angleDeg * math.pi / 180.0;
      final fwd = lx + d * math.cos(a), left = ly - d * math.sin(a);
      if (left.abs() > halfWidth) continue;
      final ahead = fwd - frontEdge; // distance in front of the body
      if (ahead <= 0) continue; // beside or behind the front edge
      if (nearest == null || ahead < nearest) nearest = ahead;
    }
    final mm = nearest == null ? null : nearest * 1000;
    if (mm != null && mm < stopDistMm) return GuardVerdict(false, 'lidar: ${mm.round()} mm ahead', mm);
    return GuardVerdict(true, '', mm);
  }
}

/// Capability tier from what the robot announces plus what this device can do.
class Tier {
  static (String, String) compute({
    required Map<String, dynamic>? robotCaps,
    required Map<String, dynamic> phoneCaps,
    required bool mounted,
    required bool motionConnected,
  }) {
    final raw = robotCaps?['sensors'];
    bool any(bool Function(Map e) f) => raw is List && raw.any((e) => e is Map && e['enabled'] != false && f(e));
    final legacy = raw is Map ? raw : const {};
    final hasReflex = any((e) => ['bump', 'cliff', 'obstacle'].contains(e['role']) && e['type'] != 'lidar') ||
        legacy['bumperL'] == true || legacy['bumperR'] == true || legacy['tof'] == true || legacy['ultrasonic'] == true;
    final hasLidar = any((e) => e['type'] == 'lidar') || legacy['lidar'] == true;
    final hasDepth = mounted && phoneCaps['sceneDepth'] == true;
    if (!motionConnected) return ('No robot', 'Connect to the robot (tankbot.local) to drive.');
    if (hasLidar && hasDepth) return ('3D awareness', 'Everything is unlocked on this hardware.');
    if (hasLidar) {
      return ('Mapping', mounted
          ? 'A phone with a depth camera (LiDAR iPhone, ARCore depth) unlocks low-obstacle and drop-off detection.'
          : 'Mount the phone on the robot to add camera tracking and depth sensing.');
    }
    if (hasReflex) return ('Reflexes + brain', 'Add a lidar to unlock mapping, remembering places and tap-to-go.');
    return ('Drive + brain', 'Add bumpers, a ToF or an ultrasonic sensor for on-board reflexes; add a lidar for mapping.');
  }
}
