// Guardian: one decision, "is forward motion clear, and if not, why?", using the robot's real
// geometry from the bot profile. Manual obstacle stop, tap-to-go and the banner all ask this.
import 'dart:math' as math;
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
  }) {
    if (reflexBlock != 'none') return GuardVerdict(false, 'robot reflex: $reflexBlock', null);
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
    final sn = (robotCaps?['sensors'] as Map?) ?? {};
    final hasReflex = sn['bumperL'] == true || sn['bumperR'] == true || sn['tof'] == true || sn['ultrasonic'] == true;
    final hasLidar = sn['lidar'] == true;
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
