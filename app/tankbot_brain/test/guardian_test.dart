import 'package:flutter_test/flutter_test.dart';
import 'package:tankbot_brain/bot_profile.dart';
import 'package:tankbot_brain/guardian.dart';
import 'package:tankbot_brain/lidar_client.dart';

void main() {
  final profile = BotProfile.tankbotDefault(); // lidar 40 mm from the front, platform 170 mm long
  LidarScan scanOf(List<LidarPoint> pts) => LidarScan(1, 0, pts, DateTime.now(), 0);

  test('something straight ahead within stop distance blocks forward', () {
    // lidar is 40 mm behind the front edge; a point 0.30 m from the lidar is 0.26 m ahead of the body
    final v = Guardian.evaluate(profile: profile, scan: scanOf([const LidarPoint(0, 300, 40)]), scanStale: false, stopDistMm: 300);
    expect(v.forwardClear, isFalse);
    expect(v.frontMm!, closeTo(260, 3));
  });

  test('something beside the robot does not block forward', () {
    // point 0.4 m out at 90 deg (robot\'s right): well outside the body's width
    final v = Guardian.evaluate(profile: profile, scan: scanOf([const LidarPoint(90, 400, 40)]), scanStale: false, stopDistMm: 300);
    expect(v.forwardClear, isTrue);
    expect(v.frontMm, isNull);
  });

  test('a wall far ahead is clear, and its distance is reported', () {
    final v = Guardian.evaluate(profile: profile, scan: scanOf([const LidarPoint(0, 1500, 40)]), scanStale: false, stopDistMm: 300);
    expect(v.forwardClear, isTrue);
    expect(v.frontMm!, closeTo(1460, 3));
  });

  test('robot reflexes always win', () {
    final v = Guardian.evaluate(profile: profile, scan: scanOf([]), scanStale: false, stopDistMm: 300, reflexBlock: 'bumper');
    expect(v.forwardClear, isFalse);
    expect(v.reason, contains('bumper'));
  });

  test('stale lidar blocks forward', () {
    final v = Guardian.evaluate(profile: profile, scan: scanOf([]), scanStale: true, stopDistMm: 300);
    expect(v.forwardClear, isFalse);
  });
}
