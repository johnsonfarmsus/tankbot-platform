import 'dart:typed_data';
import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/bot_profile.dart';
import 'package:tankbot_brain/depth_obstacles.dart';
import 'package:tankbot_brain/guardian.dart';

void main() {
  final profile = BotProfile.tankbotDefault(); // camera 200 mm high, 38-40 mm from the front
  final camH = 0.2;

  Float32List cloud(List<List<double>> pts) => Float32List.fromList([for (final p in pts) ...p]);

  test('floor points are ignored, a low box ahead becomes an obstacle', () {
    final d = DepthObstacles();
    final pts = <List<double>>[];
    for (var f = 0.2; f < 1.5; f += 0.05) {
      for (var l = -0.3; l <= 0.3; l += 0.05) {
        pts.add([f, l, -camH + 0.01]); // floor, with a little noise
      }
    }
    for (var i = 0; i < 6; i++) {
      pts.add([0.3, 0.0, -camH + 0.12]); // a 12 cm box, 0.3 m ahead of the camera
    }
    d.update(cloud(pts), profile, 1000);
    expect(d.obstacles.length, 1);
    expect(d.cliffs, isEmpty);
    // platform frame: camera is 45 mm ahead of the platform centre
    expect(d.obstacles.first.dx, closeTo(0.345, 0.03));
    final v = Guardian.evaluate(profile: profile, scan: null, scanStale: true, stopDistMm: 300, lidarExpected: false,
        depthObstacles: d.obstacles, dropOffs: d.cliffs);
    expect(v.forwardClear, isFalse);
    expect(v.reason, contains('depth camera'));
  });

  test('a stair edge ahead is a drop-off and blocks forward', () {
    final d = DepthObstacles();
    final pts = <List<double>>[];
    for (var f = 0.2; f < 0.35; f += 0.05) {
      pts.add([f, 0, -camH]); // floor up to 0.35 m
    }
    for (var i = 0; i < 4; i++) {
      pts.add([0.4, 0.0, -camH - 0.25]); // then the floor is 25 cm lower
    }
    d.update(cloud(pts), profile, 1000);
    expect(d.cliffs.length, 1);
    final v = Guardian.evaluate(profile: profile, scan: null, scanStale: true, stopDistMm: 300, lidarExpected: false,
        depthObstacles: d.obstacles, dropOffs: d.cliffs);
    expect(v.forwardClear, isFalse);
    expect(v.reason, contains('drop-off'));
  });

  test('a table top above the robot is not an obstacle', () {
    final d = DepthObstacles();
    final pts = [for (var i = 0; i < 6; i++) [0.5, 0.0, -camH + 0.7]]; // 70 cm off the floor
    d.update(cloud(pts), profile, 1000);
    expect(d.obstacles, isEmpty);
  });
}
