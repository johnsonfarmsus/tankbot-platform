// Turns the phone's depth-camera point cloud into 2D obstacles and drop-offs in the platform
// frame (x forward, y left, origin at the platform centre), using the profile's camera position.
import 'dart:typed_data';
import 'dart:ui' show Offset;
import 'bot_profile.dart';

class DepthObstacles {
  double floorToleranceM = 0.06; // above this counts as an obstacle (from the profile)
  static const double dropM = 0.08; // floor lower than this = drop-off
  static const double maxFwdM = 2.0, maxSideM = 1.0, cliffFwdM = 1.2;
  static const double cellM = 0.05;
  static const int minHitsPerCell = 2; // noise suppression

  List<Offset> obstacles = []; // platform frame, metres
  List<Offset> cliffs = [];
  int frames = 0;
  double lastMs = -1e9;
  double robotHeightM = 0.3;

  /// pts: interleaved fwd, left, up (metres) relative to the camera, level frame.
  void update(Float32List pts, BotProfile profile, double nowMs) {
    final cam = profile.byType('camera');
    final camH = cam == null ? 0.2 : profile.absHeightM(cam);
    final camFwd = cam == null ? 0.0 : profile.fwdM(cam);
    final camLeft = cam == null ? 0.0 : profile.leftM(cam);
    robotHeightM = profile.robotHeightM;
    floorToleranceM = profile.depthMinHeightMm / 1000.0;
    final floorUp = -camH;
    final obs = <int, int>{}, cliff = <int, int>{};
    for (var i = 0; i + 2 < pts.length; i += 3) {
      final fwd = pts[i], left = pts[i + 1], up = pts[i + 2];
      if (fwd < 0.1 || fwd > maxFwdM || left.abs() > maxSideM) continue;
      final pf = camFwd + fwd, pl = camLeft + left;
      final key = ((pf / cellM).round() + 2000) * 4000 + ((pl / cellM).round() + 2000);
      if (up > floorUp + floorToleranceM && up < floorUp + robotHeightM) {
        obs[key] = (obs[key] ?? 0) + 1;
      } else if (up < floorUp - dropM && fwd < cliffFwdM) {
        cliff[key] = (cliff[key] ?? 0) + 1;
      }
    }
    Offset unkey(int k) => Offset(((k ~/ 4000) - 2000) * cellM, ((k % 4000) - 2000) * cellM);
    obstacles = [for (final e in obs.entries) if (e.value >= minHitsPerCell) unkey(e.key)];
    cliffs = [for (final e in cliff.entries) if (e.value >= minHitsPerCell) unkey(e.key)];
    frames++;
    lastMs = nowMs;
  }

  bool fresh(double nowMs) => nowMs - lastMs < 800;
}
