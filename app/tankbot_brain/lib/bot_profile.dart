// Bot profile: one robot's size, drive type and sensor layout. See docs/bot-profile.md.
// Everything that used to be hard-coded (lidar offset, footprint) is derived from this.
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:flutter/services.dart';

class BotSensor {
  String id, type, name;
  /// Position from the platform's left and front edges (negative fromFrontMm = ahead of the platform).
  /// heightMm is relative to the platform top (negative = below it).
  double fromLeftMm, fromFrontMm, heightMm, yawDeg;
  /// Bumper bars: width of the bar (mm), centred on fromLeftMm.
  double widthMm;
  BotSensor(this.id, this.type, this.name,
      {required this.fromLeftMm, required this.fromFrontMm, this.heightMm = 0, this.yawDeg = 0, this.widthMm = 0});

  static const types = ['lidar', 'camera', 'bumper', 'tof', 'ultrasonic', 'imu', 'depth'];

  Map<String, dynamic> toJson() => {
        'id': id,
        'type': type,
        'name': name,
        'fromLeftMm': fromLeftMm,
        'fromFrontMm': fromFrontMm,
        'heightMm': heightMm,
        'yawDeg': yawDeg,
        if (widthMm > 0) 'widthMm': widthMm,
      };

  static BotSensor? fromJson(dynamic j) {
    if (j is! Map) return null;
    final type = j['type'];
    if (type is! String || !types.contains(type)) return null;
    double num_(dynamic v, double d) => v is num && v.isFinite ? v.toDouble() : d;
    final id = (j['id'] is String && (j['id'] as String).isNotEmpty) ? j['id'] as String : 's${DateTime.now().microsecondsSinceEpoch}';
    return BotSensor(id, type, (j['name'] as String?) ?? type,
        fromLeftMm: num_(j['fromLeftMm'], 0).clamp(-500, 3000).toDouble(),
        fromFrontMm: num_(j['fromFrontMm'], 0).clamp(-500, 3000).toDouble(),
        heightMm: num_(j['heightMm'], 0).clamp(-1000, 3000).toDouble(),
        yawDeg: num_(j['yawDeg'], 0).clamp(-180, 180).toDouble(),
        widthMm: num_(j['widthMm'], 0).clamp(0, 3000).toDouble());
  }
}

class BotProfile {
  String name;
  String drive; // tank | wheelchair | mecanum
  double widthMm, lengthMm; // platform: left-right, front-back
  double platformHeightMm; // platform top above the floor; sensor heights are relative to it
  double trackMm, axleFromFrontMm; // rotation centre for tank / wheelchair
  /// Fraction of full power below which this robot does not move, and the power it drives best at.
  double minPower, cruisePower;
  /// Obstacle handling (mm): never drive forward closer than stopDistMm to something ahead;
  /// plan routes that keep at least passDistMm of clearance around the body.
  double stopDistMm, passDistMm;
  /// Depth camera: block forward closer than depthStopMm; ignore things lower than depthMinHeightMm.
  double depthStopMm, depthMinHeightMm;
  final List<BotSensor> sensors;

  BotProfile({
    required this.name,
    required this.drive,
    required this.widthMm,
    required this.lengthMm,
    this.platformHeightMm = 60,
    required this.trackMm,
    required this.axleFromFrontMm,
    required this.sensors,
    this.minPower = 0.8,
    this.cruisePower = 0.9,
    this.stopDistMm = 300,
    this.passDistMm = 100,
    this.depthStopMm = 250,
    this.depthMinHeightMm = 60,
  });

  static const drives = ['tank', 'wheelchair', 'mecanum'];

  /// The TankBot as built (TP101 chassis, tower with phone under the lidar). Adjust in the Bot tab.
  static BotProfile tankbotDefault() => BotProfile(
        name: 'TankBot',
        drive: 'tank',
        widthMm: 185,
        lengthMm: 170,
        trackMm: 160,
        axleFromFrontMm: 85,
        sensors: [
          BotSensor('lidar', 'lidar', 'RPLidar C1', fromLeftMm: 92, fromFrontMm: 40, heightMm: 240),
          BotSensor('cam', 'camera', 'Phone camera', fromLeftMm: 108, fromFrontMm: 40, heightMm: 140),
        ],
      );

  // ---- robot frame (x forward, y left, metres, origin at platform centre) ----
  double fwdM(BotSensor s) => (lengthMm / 2 - s.fromFrontMm) / 1000.0;
  double leftM(BotSensor s) => (widthMm / 2 - s.fromLeftMm) / 1000.0;

  BotSensor? byType(String type) {
    for (final s in sensors) {
      if (s.type == type) return s;
    }
    return null;
  }

  /// Where the tracked pose sits on the robot: the phone camera if present, else the platform centre.
  (double, double) get trackedPoint {
    final c = byType('camera');
    return c == null ? (0.0, 0.0) : (fwdM(c), leftM(c));
  }

  /// Lidar position relative to the tracked point (metres, forward / left).
  double get lidarFwdM {
    final l = byType('lidar');
    if (l == null) return 0;
    return fwdM(l) - trackedPoint.$1;
  }

  double get lidarLeftM {
    final l = byType('lidar');
    if (l == null) return 0;
    return leftM(l) - trackedPoint.$2;
  }

  /// Height above the floor (metres).
  double absHeightM(BotSensor s) => (platformHeightMm + s.heightMm) / 1000.0;

  /// Tallest point of the robot (metres above the floor), with a little margin.
  double get robotHeightM {
    var top = platformHeightMm / 1000.0;
    for (final s in sensors) {
      final h = absHeightM(s);
      if (h > top) top = h;
    }
    return top + 0.05;
  }

  BotSensor? byId(String id) {
    for (final s in sensors) {
      if (s.id == id) return s;
    }
    return null;
  }

  /// A sensor's position relative to the tracked point (metres, forward / left).
  (double, double) sensorOffset(BotSensor s) {
    final (tx, ty) = trackedPoint;
    return (fwdM(s) - tx, leftM(s) - ty);
  }

  /// Farthest platform corner from the tracked point: the circle the body sweeps when turning.
  double get bodyRadiusM {
    final (tx, ty) = trackedPoint;
    final hl = lengthMm / 2000.0, hw = widthMm / 2000.0;
    var r = 0.0;
    for (final cx in [hl, -hl]) {
      for (final cy in [hw, -hw]) {
        r = math.max(r, math.sqrt((cx - tx) * (cx - tx) + (cy - ty) * (cy - ty)));
      }
    }
    return r;
  }

  /// Planning clearance: body radius plus the pass distance.
  double get inflationRadiusM => bodyRadiusM + passDistMm / 1000.0;

  Map<String, dynamic> toJson() => {
        'version': 2,
        'name': name,
        'drive': drive,
        'platform': {'widthMm': widthMm, 'lengthMm': lengthMm, 'heightMm': platformHeightMm},
        'wheels': {'trackMm': trackMm, 'axleFromFrontMm': axleFromFrontMm},
        'power': {'min': minPower, 'cruise': cruisePower},
        'obstacles': {'stopMm': stopDistMm, 'passMm': passDistMm, 'depthStopMm': depthStopMm, 'depthMinHeightMm': depthMinHeightMm},
        'sensors': [for (final s in sensors) s.toJson()],
      };

  static BotProfile? fromJson(dynamic j) {
    if (j is! Map) return null;
    double num_(dynamic v, double d) => v is num && v.isFinite ? v.toDouble() : d;
    final pf = j['platform'], wh = j['wheels'];
    final drive = j['drive'];
    final sensors = <BotSensor>[];
    if (j['sensors'] is List) {
      for (final x in j['sensors'] as List) {
        final s = BotSensor.fromJson(x);
        if (s != null && sensors.length < 32) sensors.add(s);
      }
    }
    final pw = j['power'];
    final minP = num_(pw is Map ? pw['min'] : null, 0.8).clamp(0.3, 1.0).toDouble();
    final cruise = num_(pw is Map ? pw['cruise'] : null, 0.9).clamp(minP, 1.0).toDouble();
    final ob = j['obstacles'];
    final stopMm = num_(ob is Map ? ob['stopMm'] : null, 300).clamp(100, 2000).toDouble();
    final passMm = num_(ob is Map ? ob['passMm'] : null, 100).clamp(0, 1000).toDouble();
    final dStop = num_(ob is Map ? ob['depthStopMm'] : null, 250).clamp(50, 2000).toDouble();
    final dMinH = num_(ob is Map ? ob['depthMinHeightMm'] : null, 60).clamp(10, 300).toDouble();
    final width = num_(pf is Map ? pf['widthMm'] : null, 185).clamp(50, 3000).toDouble();
    final length = num_(pf is Map ? pf['lengthMm'] : null, 170).clamp(50, 3000).toDouble();
    final platH = num_(pf is Map ? pf['heightMm'] : null, 60).clamp(0, 2000).toDouble();
    // version 1 profiles stored heights above the floor: make them platform-relative
    final version = (j['version'] as num?)?.toInt() ?? 1;
    if (version < 2) {
      for (final sn in sensors) {
        sn.heightMm -= platH;
      }
    }
    return BotProfile(
      name: (j['name'] is String && (j['name'] as String).trim().isNotEmpty) ? (j['name'] as String).trim() : 'Bot',
      drive: drives.contains(drive) ? drive as String : 'tank',
      widthMm: width,
      lengthMm: length,
      platformHeightMm: platH,
      trackMm: num_(wh is Map ? wh['trackMm'] : null, width * 0.85).clamp(20, 3000).toDouble(),
      axleFromFrontMm: num_(wh is Map ? wh['axleFromFrontMm'] : null, length / 2).clamp(-1000, 3000).toDouble(),
      sensors: sensors,
      minPower: minP,
      cruisePower: cruise,
      stopDistMm: stopMm,
      passDistMm: passMm,
      depthStopMm: dStop,
      depthMinHeightMm: dMinH,
    );
  }
}

class BotProfileStore {
  static const _native = MethodChannel('tankbot/arkit');

  static Future<File?> _file(String robot) async {
    try {
      final p = await _native.invokeMethod<String>('documentsDir');
      if (p == null) return null;
      final d = Directory('$p/robots/$robot');
      await d.create(recursive: true);
      return File('${d.path}/bot_profile.json');
    } catch (_) {
      return null;
    }
  }

  static Future<BotProfile?> load(String robot) async {
    final f = await _file(robot);
    if (f == null || !await f.exists()) return null;
    try {
      return BotProfile.fromJson(jsonDecode(await f.readAsString()));
    } catch (_) {
      return null;
    }
  }

  static Future<void> save(BotProfile p, String robot) async {
    final f = await _file(robot);
    if (f == null) return;
    try {
      await f.writeAsString(jsonEncode(p.toJson()), flush: true);
    } catch (_) {}
  }
}
