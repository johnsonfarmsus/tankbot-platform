import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'lidar_client.dart';
import 'motion_client.dart';
import 'joystick.dart';
import 'pose_client.dart';
import 'occupancy_grid.dart';
import 'brain_server.dart';
import 'map_store.dart';
import 'scan_matcher.dart';
import 'loop_closer.dart';
import 'planner.dart';
import 'bot_profile.dart';
import 'sensor_client.dart';
import 'app_settings.dart';
import 'role_screens.dart';
import 'guardian.dart';
import 'depth_obstacles.dart';
import 'sensor_log.dart';
import 'pose_graph.dart';

void main() => runApp(const TankBotApp());

class TankBotApp extends StatelessWidget {
  const TankBotApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'TankBot Brain',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.teal, brightness: Brightness.dark),
        useMaterial3: true,
      ),
      home: const RoleGate(),
    );
  }
}

/// Loads the saved role and shows the chooser, the controller, or the brain.
class RoleGate extends StatefulWidget {
  const RoleGate({super.key});
  @override
  State<RoleGate> createState() => _RoleGateState();
}

class _RoleGateState extends State<RoleGate> {
  AppSettings? settings;

  @override
  void initState() {
    super.initState();
    AppSettings.load().then((s) => setState(() => settings = s));
  }

  void _choose(AppRole r) {
    settings!.role = r;
    settings!.save();
    setState(() {});
  }

  void _changeRole() {
    settings!.role = null;
    settings!.save();
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    final s = settings;
    if (s == null) return const Scaffold(body: Center(child: CircularProgressIndicator()));
    switch (s.role) {
      case null:
        return RoleChooser(onChosen: _choose);
      case AppRole.controller:
        return ControllerScreen(settings: s, onChangeRole: _changeRole);
      case AppRole.mounted:
      case AppRole.brain:
        return LidarScreen(key: ValueKey(s.role), role: s.role!, settings: s, onChangeRole: _changeRole);
    }
  }
}

enum ViewMode { radar, map }

class LidarScreen extends StatefulWidget {
  const LidarScreen({super.key, this.role = AppRole.mounted, this.settings, this.onChangeRole});
  final AppRole role;
  final AppSettings? settings;
  final VoidCallback? onChangeRole;
  @override
  State<LidarScreen> createState() => _LidarScreenState();
}

class _LidarScreenState extends State<LidarScreen> with WidgetsBindingObserver {
  static const _native = MethodChannel('tankbot/arkit');

  final client = LidarClient();
  final motion = MotionClient();
  final sensors = SensorClient();
  final depth = DepthObstacles();

  // ---------- sensor log (GPS / compass evaluation) ----------
  final SensorLog sensorLog = SensorLog();
  StreamSubscription? _locSub;
  Timer? _logTimer;
  int gpsFixes = 0, headingReads = 0;
  Map<String, dynamic>? lastGps, lastHeading;
  double lastGpsMs = -1e9;
  String locAuth = 'not asked yet';

  Future<void> _logStart() async {
    if (sensorLog.recording) return;
    final name = await sensorLog.start();
    if (name == null) {
      _flash('Could not start the sensor log');
      return;
    }
    gpsFixes = 0;
    headingReads = 0;
    _logTimer = Timer.periodic(const Duration(milliseconds: 200), (_) {
      final p = robotPose;
      sensorLog.write('pose', [p?.x, p?.y, p?.heading, locState, _mode]);
    });
    _flash('Recording sensor log $name');
  }

  Future<void> _logStop() async {
    _logTimer?.cancel();
    _logTimer = null;
    final n = sensorLog.name, lines = sensorLog.lines;
    await sensorLog.stop();
    _flash('Saved $n ($lines lines)');
  }

  void _onLocation(dynamic e) {
    if (e is! Map) return;
    final m = Map<String, dynamic>.from(e);
    final p = robotPose;
    switch (m['type']) {
      case 'gps':
        gpsFixes++;
        lastGps = m;
        lastGpsMs = appClockMs();
        sensorLog.write('gps', [m['t'], m['lat'], m['lon'], m['hAcc'], m['alt'], m['vAcc'], m['speed'], m['course'], p?.x, p?.y, p?.heading]);
      case 'heading':
        headingReads++;
        lastHeading = m;
        sensorLog.write('heading', [m['t'], m['mag'], m['true'], m['acc'], m['x'], m['y'], m['z'], p?.x, p?.y, p?.heading]);
      case 'auth':
        final st = (m['status'] as num?)?.toInt() ?? -1;
        locAuth = switch (st) { 0 => 'not asked yet', 1 => 'restricted', 2 => 'denied', 3 || 4 => 'allowed', _ => 'unknown' } +
            (m['precise'] == false ? ' (approximate only)' : '');
        sensorLog.write('auth', [st, m['precise']]);
      case 'error':
        sensorLog.write('error', [m['msg']]);
    }
  }
  final poses = PoseClient();
  final grid = OccupancyGrid();
  final store = MapStore();
  late MapSession active;
  double _lastAutosaveMs = 0;
  List<Map<String, dynamic>> savedMaps = [];
  bool _loadingMap = false;

  // Tracking: camera (ARKit/ARCore) corrected by lidar scan matching, or lidar alone.
  late final ScanMatcher matcher = ScanMatcher(grid);
  final PoseCorrection _corr = PoseCorrection();
  String _mode = 'ar'; // 'ar' = camera tracking + lidar, 'lidar' = lidar only
  double _arBadSinceMs = -1;
  Pose? _lidarPose; // pose while tracking with the lidar alone
  Pose? _lastRobotPose;
  String locState = 'tracking'; // tracking | localizing | lost
  String locNote = '';
  String flashMsg = '';
  double flashMs = -1e9;
  void _flash(String m) {
    flashMsg = m;
    flashMs = appClockMs();
  }
  bool _relocRunning = false;
  int _relocAttempts = 0;
  int matchHits = 0, matchMisses = 0;
  double lastCorrCm = 0;
  Map<String, dynamic> caps = {};

  // Map quality: turn handling and loop closing
  static const double _maxMapTurnRate = 0.35; // rad/s (~20 deg/s): faster than this, scans smear
  double _lastPoseMs = 0;
  int skippedTurning = 0, loopClosures = 0;
  double lastLoopCm = 0, lastLoopDeg = 0;
  bool _rebuilding = false, _loopChecking = false;

  // Tap-to-go navigation
  String navState = 'idle'; // idle | driving | blocked | arrived | failed
  String navNote = '';
  Offset? navGoal;
  List<Offset> navPath = [];
  int _navIdx = 1, _navFails = 0;
  double _navLastPlanMs = 0;
  bool _navRotating = false;
  double _navTurnStartMs = 0;
  Timer? _navTimer;
  double _cmdF = 0, _cmdT = 0, _navLastBlockReplanMs = 0;
  int navReplans = 0, navFrontBlocks = 0;
  bool get navActive => navState == 'driving' || navState == 'blocked';
  int _kfSinceLoopCheck = 0, _kfSinceClosure = 999;
  late final BrainServer server;
  final List<StreamSubscription> _subs = [];
  LidarScan? scan;
  Map<String, dynamic>? robotStatus;
  Map<String, dynamic>? motionStatus;
  String link = 'Starting...';
  double rangeMm = 6000;
  double maxSpeed = 0.6;
  bool obstacleStop = true; // on by default; can be turned off in the controller's Settings
  double get stopDistMm => profile.stopDistMm;
  double _wantF = 0, _wantT = 0;
  bool _blocked = false;
  final List<DateTime> _recent = [];
  Timer? _tick, _telemTimer;

  // Mapping
  ViewMode view = ViewMode.radar;
  bool mapping = true;
  bool robotMode = false;

  // Mount detection (robot mode): only map once the phone sits still in its cradle,
  // and pause if the phone moves relative to the robot (lidar scene unchanged).
  String mountState = 'off'; // off (not robot mode) | mounting | mounted
  String mountNote = '';
  double _robotModeSinceMs = 0, _lastDriveMs = -1e9;
  final List<_ScanSig> _sigs = [];
  int disturbances = 0;
  final List<LidarScan> _pending = [];
  final List<Offset> trail = [];
  MapImage? mapImage;
  bool _rendering = false;
  double _lastMapSentMs = -1e9;
  bool _mapChangedSinceSend = false;
  int skippedNoPose = 0;

  // Bot profile (size, drive type, sensor positions): the single source of truth. See docs/bot-profile.md.
  BotProfile profile = BotProfile.tankbotDefault();
  double get lidarFwdM => profile.lidarFwdM;
  double get lidarLeftM => profile.lidarLeftM;

  @override
  void initState() {
    super.initState();
    active = MapSession.fresh(lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
    server = BrainServer(onMessage: _onRemote, onRemoteSilent: _onRelease, onConnect: _onRemoteConnect);
    _subs.add(client.scans.listen((s) {
      final now = DateTime.now();
      _recent.add(now);
      _recent.removeWhere((t) => now.difference(t) > const Duration(seconds: 2));
      scan = s;
      _checkDisturbance(s);
      if (mapping && _mapAllowed) _pending.add(s);
      _processPending();
      if (motion.driving) _applyDrive();
      if (!robotMode) setState(() {});
    }));
    _subs.add(client.status.listen((s) => robotStatus = s));
    _subs.add(client.linkState.listen((s) => setState(() => link = s)));
    _subs.add(motion.status.listen((m) => motionStatus = m));
    _subs.add(poses.poses.listen((p) {
      final rp = robotPose;
      if (rp != null && rp.good && locState == 'tracking' &&
          (trail.isEmpty || (Offset(rp.x, rp.y) - trail.last).distance > 0.05)) {
        trail.add(Offset(rp.x, rp.y));
        if (trail.length > 5000) trail.removeAt(0);
      }
      _checkMounted();
      _processPending();
    }));
    _tick = Timer.periodic(const Duration(milliseconds: 400), (_) async {
      final wantImage = view == ViewMode.map || server.clientCount > 0;
      if (grid.dirty && !_rendering && wantImage) {
        _rendering = true;
        final img = await grid.render();
        _rendering = false;
        if (img != null) {
          mapImage = img;
          _mapChangedSinceSend = true;
        }
      }
      await _autosaveIfDue();
      await _sendMapIfDue();
      if (mounted) setState(() {});
    });
    _telemTimer = Timer.periodic(const Duration(milliseconds: 200), (_) => _sendTelemetry());
    WidgetsBinding.instance.addObserver(this);
    _subs.add(sensors.readings.listen(_onRobotSensors));
    _locSub = const EventChannel('tankbot/location').receiveBroadcastStream().listen(_onLocation, onError: (_) {});
    _subs.add(poses.depth.listen((pts) {
      depth.update(pts, profile, appClockMs());
      _rememberDropOffs();
    }));
    client.start();
    motion.start();
    sensors.start();
    if (widget.role == AppRole.mounted) {
      poses.start();
      WidgetsBinding.instance.addPostFrameCallback((_) => _enterRobotMode()); // this phone rides on the robot
    } else {
      _mode = 'lidar';
      poses.state = 'off (brain in hand)';
    }
    _loadCaps();
    () async {
      await MapStore.migrateOldLayout();
      store.robot = widget.settings?.lastRobot ?? 'TankBot';
      final p = await BotProfileStore.load(store.robot);
      if (p != null && mounted) {
        setState(() {
          profile = p;
          maxSpeed = p.cruisePower;
        });
      }
      _profileReady = true; // only now may the robot's table be merged or the profile saved
      await server.start();
      await _refreshMapList();
      await _loadLastMap(); // remember the house across restarts
      if (mounted) setState(() {});
    }();
    _keepAwake(true); // the brain must never auto-lock: iOS would suspend it
  }

  @override
  void dispose() {
    for (final s in _subs) {
      s.cancel();
    }
    _tick?.cancel();
    _telemTimer?.cancel();
    _navTimer?.cancel();
    _locSub?.cancel();
    final lp = _lastRobotPose;
    if (lp != null && locState == 'tracking') {
      active.lastPose = [lp.x, lp.y, lp.heading];
      active.edited = true;
      _saveActive(); // fire and forget: the app keeps running, the write completes
    }
    WidgetsBinding.instance.removeObserver(this);
    server.stop();
    motion.dispose();
    sensors.dispose();
    client.dispose();
    poses.dispose();
    super.dispose();
  }

  Future<void> _keepAwake(bool on) async {
    try {
      await _native.invokeMethod('keepAwake', on);
    } catch (_) {}
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state != AppLifecycleState.resumed) {
      if (navActive) _navCancel('Stopped: app left the foreground');
      motion.release();
      _saveActive(); // never lose the map to backgrounding or an app update
    }
  }

  // ---------- remote control ----------
  void _onRemote(Map<String, dynamic> m) {
    switch (m['type']) {
      case 'drive':
        if (navActive) _navCancel('Stopped: manual control');
        final f = (m['f'] as num?)?.toDouble() ?? 0;
        final t = (m['t'] as num?)?.toDouble() ?? 0;
        _onStick(f.clamp(-1.0, 1.0), t.clamp(-1.0, 1.0));
        break;
      case 'stop':
        if (navActive) _navCancel('Stopped');
        _onRelease();
        break;
      case 'nav.goto':
        final gxv = (m['x'] as num?)?.toDouble(), gyv = (m['y'] as num?)?.toDouble();
        if (gxv != null && gyv != null) _navGoto(gxv, gyv);
        break;
      case 'nav.cancel':
        _navCancel('Stopped');
        break;
      case 'bot.get':
        server.broadcast({'type': 'bot', 'profile': profile.toJson()});
        break;
      case 'bot.set':
        final np = BotProfile.fromJson(m['profile']);
        if (np != null) {
          for (final x in np.sensors) {
            if (!x.onPhone) x.onRobot = true; // edited here: belongs to the robot's table
          }
          np.hardwareDirty = true; // cleared once the robot confirms (or already matches)
          profile = np;
          maxSpeed = np.cruisePower;
          BotProfileStore.save(np, store.robot);
          server.broadcast({'type': 'bot', 'profile': profile.toJson()});
          _pushHardwareIfChanged();
        }
        break;
      case 'log.start':
        _logStart();
        break;
      case 'log.stop':
        _logStop();
        break;
      case 'bot.calibrate':
        if (m['id'] is String) _calibrateFloor(m['id'] as String);
        break;
      case 'set':
        if (m['maxSpeed'] is num) maxSpeed = (m['maxSpeed'] as num).toDouble().clamp(0.2, 1.0);
        if (m['obstacleStop'] is bool) obstacleStop = m['obstacleStop'] as bool;
        if (m['mapping'] is bool) mapping = m['mapping'] as bool;
        if (m['stopDistMm'] is num || m['passDistMm'] is num) {
          if (m['stopDistMm'] is num) profile.stopDistMm = (m['stopDistMm'] as num).toDouble().clamp(100.0, 2000.0);
          if (m['passDistMm'] is num) profile.passDistMm = (m['passDistMm'] as num).toDouble().clamp(0.0, 1000.0);
          BotProfileStore.save(profile, store.robot);
        }
        if (m['wallAlign'] is bool || m['gpsMaxAccM'] is num) {
          if (m['wallAlign'] is bool) profile.wallAlign = m['wallAlign'] as bool;
          if (m['gpsMaxAccM'] is num) profile.gpsMaxAccM = (m['gpsMaxAccM'] as num).toDouble().clamp(1.0, 50.0);
          BotProfileStore.save(profile, store.robot);
        }
        if (m['depthStopMm'] is num || m['depthMinHeightMm'] is num) {
          if (m['depthStopMm'] is num) profile.depthStopMm = (m['depthStopMm'] as num).toDouble().clamp(50.0, 2000.0);
          if (m['depthMinHeightMm'] is num) profile.depthMinHeightMm = (m['depthMinHeightMm'] as num).toDouble().clamp(10.0, 300.0);
          BotProfileStore.save(profile, store.robot);
        }
        if (m['trim'] is num) _setTrim((m['trim'] as num).round().clamp(-20, 20));
        if (m['minPower'] is num || m['cruisePower'] is num) {
          if (m['minPower'] is num) profile.minPower = (m['minPower'] as num).toDouble().clamp(0.3, 1.0);
          if (m['cruisePower'] is num) profile.cruisePower = (m['cruisePower'] as num).toDouble().clamp(0.3, 1.0);
          if (profile.cruisePower < profile.minPower) profile.cruisePower = profile.minPower;
          maxSpeed = profile.cruisePower;
          BotProfileStore.save(profile, store.robot);
        }
        break;
      case 'clearMap':
        _startNewMap();
        break;
      case 'reloc':
        _relocAttempts = 0;
        _relocalize();
        break;
      case 'atHome':
        _atHome();
        break;
      case 'map.erase':
        final ex = (m['x'] as num?)?.toDouble(), ey = (m['y'] as num?)?.toDouble();
        final er = ((m['r'] as num?)?.toDouble() ?? 0.2).clamp(0.05, 1.0);
        if (ex != null && ey != null) {
          // remembered marks (drop-offs, bumps) under the eraser go for good
          active.edits.removeWhere((e) => e['type'] == 'obstacle' &&
              math.sqrt(math.pow((e['x'] as num) - ex, 2) + math.pow((e['y'] as num) - ey, 2)) < er + ((e['r'] as num?) ?? 0.08));
          active.edits.add({'type': 'erase', 'id': active.nextEditId++, 'stroke': m['stroke'], 'x': ex, 'y': ey, 'r': er});
          grid.eraseCircle(ex, ey, er);
          active.edited = true;
        }
        break;
      case 'map.nogo':
        final nv = [m['x1'], m['y1'], m['x2'], m['y2']];
        if (nv.every((e) => e is num)) {
          final nid = active.nextEditId++;
          active.edits.add({
            'type': 'nogo', 'id': nid, 'stroke': 'nogo$nid',
            'x1': (nv[0] as num).toDouble(), 'y1': (nv[1] as num).toDouble(),
            'x2': (nv[2] as num).toDouble(), 'y2': (nv[3] as num).toDouble(),
          });
          active.edited = true;
        }
        break;
      case 'map.nogoDelete':
        final did = m['id'];
        active.edits.removeWhere((e) => e['type'] == 'nogo' && e['id'] == did);
        active.edited = true;
        break;
      case 'map.clearDropoffs':
        final before = active.edits.length;
        active.edits.removeWhere((e) => e['type'] == 'obstacle' && e['kind'] == 'dropoff');
        if (active.edits.length != before) {
          active.edited = true;
          _rebuildGrid();
          _flash('Cleared ${before - active.edits.length} remembered drop-offs');
        }
        break;
      case 'map.undo':
        if (active.edits.isNotEmpty) {
          final stroke = active.edits.last['stroke'];
          if (stroke != null) {
            active.edits.removeWhere((e) => e['stroke'] == stroke);
          } else {
            active.edits.removeLast();
          }
          active.edited = true;
          _rebuildGrid(); // grid edits (erase / obstacle): redraw without it
        }
        break;
      case 'maps.list':
        _refreshMapList();
        break;
      case 'maps.save':
        final n = (m['name'] as String?)?.trim();
        if (n != null && n.isNotEmpty && n != active.name) {
          active.name = n.length > 60 ? n.substring(0, 60) : n;
          active.renamed = true;
        }
        _saveActive();
        break;
      case 'maps.load':
        if (m['id'] is String) _loadMap(m['id'] as String);
        break;
      case 'maps.delete':
        final id = m['id'];
        if (id is String && id != active.id) store.delete(id).then((_) => _refreshMapList());
        break;
    }
    setState(() {});
  }

  void _sendTelemetry() {
    _ensureFullPower();
    if (server.clientCount == 0) return;
    final p = robotPose;
    final sc = scan;
    final m = motionStatus;
    server.broadcast({
      'type': 'telem',
      'pose': p == null ? null : {'x': p.x, 'y': p.y, 'h': p.heading, 'good': p.good},
      'scan': (sc == null || stale)
          ? null
          : [
              for (var i = 0; i < sc.points.length; i += 2)
                [(sc.points[i].angleDeg * 10).round() / 10, (sc.points[i].distMm).round() / 1000]
            ],
      'lidarOffset': {'fwd': lidarFwdM, 'left': lidarLeftM},
      'robotIp': motion.address,
      'bot': {'name': profile.name, 'drive': profile.drive, 'bodyRadiusM': profile.bodyRadiusM, 'sensors': profile.sensors.length},
      'mapInfo': _mapInfo(),
      'loc': {'state': locState, 'note': locNote},
      'flash': appClockMs() - flashMs < 4000 ? flashMsg : null,
      'tracking': {'source': poseSource, 'matchHits': matchHits, 'matchMisses': matchMisses, 'lastCorrCm': lastCorrCm},
      'caps': caps,
      'nav': {
        'state': navState,
        'note': navNote,
        'goal': navGoal == null ? null : [navGoal!.dx, navGoal!.dy],
        'path': [for (final q in navPath) [(q.dx * 100).round() / 100, (q.dy * 100).round() / 100]],
      },
      'robot': {'caps': sensors.caps, 'live': sensors.fresh ? sensors.latest : null, 'storage': store.robot},
      'depth': {
        'frames': depth.frames,
        'fresh': depth.fresh(appClockMs()),
        'obstacles': [for (final q in _depthWorldSplit(false)) [(q.dx * 100).round() / 100, (q.dy * 100).round() / 100]],
        'dropoffs': [for (final q in _depthWorldSplit(true)) [(q.dx * 100).round() / 100, (q.dy * 100).round() / 100]],
      },
      'nogo': [
        for (final e in active.edits)
          if (e['type'] == 'nogo') [e['x1'], e['y1'], e['x2'], e['y2'], e['id']]
      ],
      'log': {
        'recording': sensorLog.recording,
        'name': sensorLog.name,
        'lines': sensorLog.lines,
        'gpsFixes': gpsFixes,
        'headings': headingReads,
        'auth': locAuth,
        'gpsAcc': lastGps?['hAcc'],
        'gpsAgeS': lastGps == null ? null : ((appClockMs() - lastGpsMs) / 1000).round(),
        'heading': lastHeading?['mag'],
        'headingAcc': lastHeading?['acc'],
      },
      'rangers': [for (final q in _rangerPoints()) [(q.dx * 100).round() / 100, (q.dy * 100).round() / 100]],
      'dropoffs': [
        for (final e in active.edits)
          if (e['type'] == 'obstacle' && e['kind'] == 'dropoff') [e['x'], e['y']]
      ],
      'quality': {
        'matchHits': matchHits,
        'matchMisses': matchMisses,
        'skippedTurning': skippedTurning,
        'loopClosures': loopClosures,
        'lastLoopCm': lastLoopCm,
        'lastLoopDeg': lastLoopDeg,
        'rebuilding': _rebuilding,
        'navReplans': navReplans,
        'navFrontBlocks': navFrontBlocks,
        'optimizations': mapOptimizations,
        'lastOptCm': lastOptMoveCm,
        'lastOptDeg': lastOptMoveDeg,
        'wallAligned': wallAligned,
        'gpsUsed': gpsUsed,
        'geoTags': active.geoTags.length,
        'dropoffMarks': active.edits.where((e) => e['type'] == 'obstacle' && e['kind'] == 'dropoff').length,
      },
      'blocked': _blocked,
      'blockReason': blockReason,
      'guard': {'clear': guard.forwardClear, 'reason': guard.reason, 'frontMm': guard.frontMm},
      'tier': (() {
        final (name, next) = Tier.compute(
            robotCaps: sensors.caps, phoneCaps: caps, mounted: widget.role == AppRole.mounted, motionConnected: motion.connected);
        return {'name': name, 'next': next};
      })(),
      'mount': {'state': mountState, 'note': mountNote, 'robotMode': robotMode, 'disturbances': disturbances},
      'settings': {
        'maxSpeed': maxSpeed,
        'obstacleStop': obstacleStop,
        'mapping': mapping,
        'stopDistMm': stopDistMm,
        'passDistMm': profile.passDistMm,
        'depthStopMm': profile.depthStopMm,
        'depthMinHeightMm': profile.depthMinHeightMm,
        'wallAlign': profile.wallAlign,
        'gpsMaxAccM': profile.gpsMaxAccM,
        'trim': motionStatus?['trim'],
        'minPower': profile.minPower,
        'cruisePower': profile.cruisePower,
      },
      'motion': m == null ? null : {'left': m['left'], 'right': m['right'], 'src': m['src']},
      'stats': {
        'scanRate': _recent.length / 2.0,
        'ar': poses.state,
        'mapped': grid.scansIntegrated,
        'remotes': server.clientCount,
      },
    });
  }

  Future<void> _sendMapIfDue() async {
    final img = mapImage;
    if (server.clientCount == 0 || img == null || !_mapChangedSinceSend) return;
    if (appClockMs() - _lastMapSentMs < 1000) return;
    _lastMapSentMs = appClockMs();
    _mapChangedSinceSend = false;
    final bd = await img.image.toByteData(format: ui.ImageByteFormat.png);
    if (bd == null) return;
    server.broadcast({
      'type': 'map',
      'png': base64Encode(bd.buffer.asUint8List()),
      'left': img.leftM,
      'top': img.topM,
      'res': img.resolution,
    });
  }

  /// Ask the ESP32 for its full power range (speed level 3 = PWM 255), so the brain's
  /// percentages mean what they say. Sent once the robot's address is known.
  bool _speedLevelSet = false;
  Future<void> _ensureFullPower() async {
    final ip = motion.address;
    if (ip == null || _speedLevelSet) return;
    _speedLevelSet = true;
    final c = HttpClient()..connectionTimeout = const Duration(seconds: 2);
    try {
      final req = await c.getUrl(Uri.parse('http://$ip/speed?value=3'));
      final res = await req.close();
      await res.drain<void>();
    } catch (_) {
      _speedLevelSet = false;
    } finally {
      c.close();
    }
  }

  /// Steering trim lives on the robot (saved in its flash); the brain just forwards it.
  Future<void> _setTrim(int trim) async {
    final ip = motion.address;
    if (ip == null) return;
    final c = HttpClient()..connectionTimeout = const Duration(seconds: 2);
    try {
      final req = await c.getUrl(Uri.parse('http://$ip/trim?value=$trim'));
      final res = await req.close();
      await res.drain<void>();
    } catch (_) {
    } finally {
      c.close();
    }
  }

  // ---------- saved maps ----------
  static const double _kfMoveM = 0.15, _kfTurnRad = 0.087; // new keyframe every 15 cm or 5 degrees

  void _maybeKeyframe(Pose p, List<LidarPoint> pts) {
    if (_loadingMap) return;
    final k = active.keyframes;
    if (k.isNotEmpty) {
      final last = k.last;
      final moved = math.sqrt(_sq(p.x - last.x) + _sq(p.y - last.y));
      if (moved < _kfMoveM && _angDiff(p.heading, last.h).abs() < _kfTurnRad) return;
    }
    k.add(Keyframe.fromScan(p.x, p.y, p.heading, pts));
    active.updated = DateTime.now();
    if (k.length >= 2) {
      final ka = k[k.length - 2], kb = k.last;
      final (dx, dy, dth) = relativePose(ka.x, ka.y, ka.h, kb.x, kb.y, kb.h);
      active.graphEdges.add(PGEdge(k.length - 2, k.length - 1, dx, dy, dth));
    }
    _tagGeo(k.length - 1);
    _kfSinceOptimize++;
    if (_kfSinceOptimize >= 40 && !_optimizing && !_rebuilding && !_loopChecking) {
      _kfSinceOptimize = 0;
      _optimizeMap('straighten');
    }
    _kfSinceLoopCheck++;
    _kfSinceClosure++;
    if (_kfSinceLoopCheck >= 10 && _kfSinceClosure >= 15 && !_loopChecking && !_rebuilding) {
      _kfSinceLoopCheck = 0;
      _checkLoop();
    }
  }

  /// Back somewhere mapped earlier in the drive? Straighten the loop and redraw.
  Future<void> _checkLoop() async {
    _loopChecking = true;
    LoopClosure? lc;
    try {
      lc = await LoopCloser.check(active.keyframes, lidarFwdM, lidarLeftM);
    } catch (_) {
      lc = null;
    }
    _loopChecking = false;
    if (lc == null || _rebuilding || _loadingMap) return;
    // a loop link: the newest keyframe, as seen from the first visit to this place
    final anchor = lc.startIndex - 1;
    final ka = active.keyframes[anchor];
    final (dx, dy, dth) = relativePose(ka.x, ka.y, ka.h, lc.matchX, lc.matchY, lc.matchH);
    active.graphEdges.add(PGEdge(anchor, lc.endIndex, dx, dy, dth, sigmaT: 0.02, sigmaR: 0.005, robust: true, kind: 'loop'));
    loopClosures++;
    lastLoopCm = lc.corrCm;
    lastLoopDeg = lc.corrDeg;
    _kfSinceClosure = 0;
    await _optimizeMap('loop');
  }

  // ---------- map straightening (pose graph) ----------
  bool _optimizing = false;
  int _kfSinceOptimize = 0, mapOptimizations = 0, wallAligned = 0, gpsUsed = 0;
  double lastOptMoveCm = 0, lastOptMoveDeg = 0;
  final _wallDir = Expando<List<double>>();
  double _lastTaggedGpsMs = -1e9;

  /// A new GPS fix since the last tag: attach it to this keyframe (used later only if good enough).
  void _tagGeo(int kfIndex) {
    final g = lastGps;
    if (g == null || lastGpsMs <= _lastTaggedGpsMs || appClockMs() - lastGpsMs > 1500) return;
    final acc = (g['hAcc'] as num?)?.toDouble() ?? -1;
    if (acc <= 0 || acc > 50) return;
    _lastTaggedGpsMs = lastGpsMs;
    active.geoTags.add({'i': kfIndex, 'lat': g['lat'], 'lon': g['lon'], 'hAcc': acc});
  }

  (double, double) _wallOf(Keyframe k) {
    final c = _wallDir[k];
    if (c != null) return (c[0], c[1]);
    final pts = [
      for (final q in ScanMatcher.robotFrame(k.points(), fwdM: active.lidarFwdM, leftM: active.lidarLeftM)) (q.dx, q.dy)
    ];
    final r = WallDirection.ofScan(pts);
    _wallDir[k] = [r.$1, r.$2];
    return r;
  }

  /// Maps saved before the pose graph existed: create driving links from the keyframes as they are.
  void _ensureOdometryEdges() {
    final have = <int>{for (final e in active.graphEdges) if (e.kind == 'odo' && e.j == e.i + 1) e.i};
    final kfs = active.keyframes;
    for (var i = 0; i + 1 < kfs.length; i++) {
      if (have.contains(i)) continue;
      final ka = kfs[i], kb = kfs[i + 1];
      final (dx, dy, dth) = relativePose(ka.x, ka.y, ka.h, kb.x, kb.y, kb.h);
      active.graphEdges.add(PGEdge(i, i + 1, dx, dy, dth));
    }
  }

  /// Solve the whole map: driving links, loop links, wall alignment and (good) GPS together.
  Future<void> _optimizeMap(String reason) async {
    if (_optimizing || _rebuilding || _loadingMap) return;
    final kfs = active.keyframes;
    final n = kfs.length;
    if (n < 10) return;
    _optimizing = true;
    try {
      _ensureOdometryEdges();
      final g = PoseGraph([for (final k in kfs.take(n)) k.x], [for (final k in kfs.take(n)) k.y], [for (final k in kfs.take(n)) k.h]);
      g.edges.addAll(active.graphEdges.where((e) => e.i < n && e.j < n));
      // wall alignment: compare each scan's wall direction with the house's (from the earliest keyframes)
      wallAligned = 0;
      if (profile.wallAlign) {
        var c4 = 0.0, s4 = 0.0, used = 0;
        for (var i = 0; i < n && used < 30; i++) {
          final (th, st) = _wallOf(kfs[i]);
          if (st < 0.45) continue;
          final w = th + kfs[i].h;
          c4 += st * math.cos(4 * w);
          s4 += st * math.sin(4 * w);
          used++;
        }
        if (used >= 5) {
          final ref = math.atan2(s4, c4) / 4;
          for (var i = 1; i < n; i++) {
            final (th, st) = _wallOf(kfs[i]);
            if (i % 60 == 59) await Future<void>.delayed(Duration.zero);
            if (st < 0.45) continue;
            final d = WallDirection.wrap90(th + kfs[i].h - ref);
            if (d.abs() > 10 * math.pi / 180) continue; // an angled wall, not the house grid
            g.headingPriors.add(PGHeadingPrior(i, kfs[i].h - d, 0.05));
            wallAligned++;
          }
        }
      }
      // GPS: only fixes better than the threshold, and only once they cover enough ground
      gpsUsed = 0;
      final good = [
        for (final t in active.geoTags)
          if ((t['hAcc'] as num) <= profile.gpsMaxAccM && (t['i'] as num) < n) t
      ];
      if (good.length >= 3) {
        final lat0 = (good.first['lat'] as num).toDouble(), lon0 = (good.first['lon'] as num).toDouble();
        final mlon = 111320.0 * math.cos(lat0 * math.pi / 180);
        final fixes = [
          for (final t in good)
            PGGps((t['i'] as num).toInt(), ((t['lon'] as num) - lon0) * mlon, ((t['lat'] as num) - lat0) * 111320.0,
                (t['hAcc'] as num).toDouble().clamp(1.0, 50.0))
        ];
        var span = 0.0;
        for (final f in fixes) {
          span = math.max(span, math.sqrt(math.pow(f.east - fixes.first.east, 2) + math.pow(f.north - fixes.first.north, 2)));
        }
        if (span >= 15) {
          g.gps.addAll(fixes);
          g.initGpsAlignment();
          gpsUsed = fixes.length;
        }
      }
      await Future<void>.delayed(Duration.zero);
      g.optimize();
      var maxMove = 0.0, maxTurn = 0.0;
      for (var i = 0; i < n; i++) {
        maxMove = math.max(maxMove, math.sqrt(math.pow(g.x[i] - kfs[i].x, 2) + math.pow(g.y[i] - kfs[i].y, 2)));
        maxTurn = math.max(maxTurn, wrapAngle(g.h[i] - kfs[i].h).abs());
      }
      mapOptimizations++;
      lastOptMoveCm = maxMove * 100;
      lastOptMoveDeg = maxTurn * 180 / math.pi;
      if (maxMove < 0.03 && maxTurn < 0.01) return; // already straight: nothing to redraw
      final last = kfs[n - 1];
      final ox = last.x, oy = last.y, oh = last.h;
      for (var i = 0; i < n; i++) {
        kfs[i].x = g.x[i];
        kfs[i].y = g.y[i];
        kfs[i].h = g.h[i];
      }
      // keyframes added while solving, and the live pose, move with the newest solved keyframe
      for (var i = n; i < kfs.length; i++) {
        final (nx, ny, nh) = _carry(ox, oy, oh, g.x[n - 1], g.y[n - 1], g.h[n - 1], kfs[i].x, kfs[i].y, kfs[i].h);
        kfs[i].x = nx;
        kfs[i].y = ny;
        kfs[i].h = nh;
      }
      final rp = robotPose;
      if (rp != null) {
        final (px, py, ph) = _carry(ox, oy, oh, g.x[n - 1], g.y[n - 1], g.h[n - 1], rp.x, rp.y, rp.heading);
        final raw = poses.latest;
        if (_mode == 'ar' && raw != null && raw.good) {
          _corr.setSoThat(raw, px, py, ph);
        } else {
          _lidarPose = Pose(appClockMs(), px, py, ph, true);
        }
        _lastRobotPose = Pose(appClockMs(), px, py, ph, true);
      }
      active.edited = true;
      if (reason == 'loop') _flash('Closed a loop - map straightened (moved up to ${lastOptMoveCm.round()} cm)');
      await _rebuildGrid();
    } finally {
      _optimizing = false;
    }
  }

  /// Where a pose ends up when the keyframe it was measured from moves from (o) to (n).
  (double, double, double) _carry(double ox, double oy, double oh, double nx, double ny, double nh, double px, double py, double ph) {
    final (rx, ry, rh) = relativePose(ox, oy, oh, px, py, ph);
    final c = math.cos(nh), s = math.sin(nh);
    return (nx + c * rx - s * ry, ny + s * rx + c * ry, wrapAngle(nh + rh));
  }

  /// Redraw the whole map from the (corrected) keyframes.
  Future<void> _rebuildGrid() async {
    _rebuilding = true;
    grid.clear();
    trail.clear();
    final kfs = List<Keyframe>.from(active.keyframes);
    for (var i = 0; i < kfs.length; i++) {
      final k = kfs[i];
      grid.integrate(Pose(0, k.x, k.y, k.h, true), k.points(), lidarFwdM: active.lidarFwdM, lidarLeftM: active.lidarLeftM);
      if (i % 40 == 39) await Future<void>.delayed(Duration.zero);
    }
    _applyEdits();
    _rebuilding = false;
    _mapChangedSinceSend = true;
  }

  /// Re-apply eraser edits after the grid is rebuilt from keyframes.
  void _applyEdits() {
    for (final e in active.edits) {
      if (e['type'] == 'erase') {
        grid.eraseCircle((e['x'] as num).toDouble(), (e['y'] as num).toDouble(), (e['r'] as num).toDouble());
      } else if (e['type'] == 'obstacle') {
        grid.markCircle((e['x'] as num).toDouble(), (e['y'] as num).toDouble(), (e['r'] as num).toDouble());
      }
    }
  }

  /// A controller just connected: send it the current map right away.
  void _onRemoteConnect() {
    _mapChangedSinceSend = true;
    _lastMapSentMs = -1e9;
    if (grid.scansIntegrated > 0) grid.dirty = true;
    _broadcastMaps();
    server.broadcast({'type': 'bot', 'profile': profile.toJson()});
  }

  Future<void> _saveActive() async {
    if (active.keyframes.isEmpty || !active.unsaved) return;
    await store.save(active);
    await store.setLast(active.id);
    _lastAutosaveMs = appClockMs();
    await _refreshMapList();
  }

  Future<void> _autosaveIfDue() async {
    if (appClockMs() - _lastAutosaveMs < 20000) return;
    _lastAutosaveMs = appClockMs();
    final lp = _lastRobotPose;
    if (lp != null && locState == 'tracking') {
      final old = active.lastPose;
      if (old == null || math.sqrt(_sq(old[0] - lp.x) + _sq(old[1] - lp.y)) > 0.2 || _angDiff(old[2], lp.heading).abs() > 0.2) {
        active.lastPose = [lp.x, lp.y, lp.heading];
        active.edited = true;
      }
    }
    await _saveActive();
  }

  Future<void> _refreshMapList() async {
    savedMaps = await store.list();
    _broadcastMaps();
  }

  void _broadcastMaps() => server.broadcast({'type': 'maps', 'list': savedMaps, 'active': _mapInfo()});

  Map<String, dynamic> _mapInfo() => {
        'id': active.id,
        'name': active.name,
        'keyframes': active.keyframes.length,
        'unsaved': active.unsaved,
        'savedAgoS': active.savedAt == null ? null : DateTime.now().difference(active.savedAt!).inSeconds,
        'loading': _loadingMap,
      };

  Future<void> _loadCaps() async {
    try {
      final r = await _native.invokeMethod('capabilities');
      caps = Map<String, dynamic>.from(r as Map);
    } on MissingPluginException {
      caps = {'platform': 'other'};
    } catch (_) {}
    caps['robotLidar'] = true; // the robot's lidar is always the mapping backbone
  }

  Future<void> _loadLastMap() async {
    final id = await store.getLast();
    if (id != null && id != active.id) await _loadMap(id);
  }

  /// Load a saved map and find the robot on it using the lidar.
  Future<void> _loadMap(String id) async {
    if (_loadingMap || id == active.id) return;
    await _saveActive();
    final m = await store.load(id);
    if (m == null) return;
    _loadingMap = true;
    _broadcastMaps();
    _onRelease();
    grid.clear();
    trail.clear();
    mapImage = null;
    _pending.clear();
    _corr.reset();
    _lidarPose = null;
    _lastRobotPose = null;
    for (var i = 0; i < m.keyframes.length; i++) {
      final k = m.keyframes[i];
      grid.integrate(Pose(0, k.x, k.y, k.h, true), k.points(), lidarFwdM: m.lidarFwdM, lidarLeftM: m.lidarLeftM);
      if (i % 40 == 39) await Future<void>.delayed(Duration.zero); // keep the app responsive
    }
    active = m;
    _applyEdits();
    _loadingMap = false;
    await store.setLast(m.id);
    locState = 'localizing';
    locNote = 'Finding myself on "${m.name}"...';
    _flash('Loaded "${m.name}"');
    _relocAttempts = 0;
    _broadcastMaps();
    // in robot mode, wait until the phone is settled in its cradle
    if (!robotMode || mountState == 'mounted') _relocalize();
    if (mounted) setState(() {});
  }

  void _onMounted() {
    if (active.keyframes.length >= 10) {
      mountState = 'mounted';
      mountNote = 'Mounted - finding myself on "${active.name}"';
      _relocAttempts = 0;
      _relocalize();
    } else {
      _startNewMap(note: 'Mounted - mapping');
    }
  }

  /// Search the whole saved map for where the current lidar view fits.
  Future<void> _relocalize() async {
    if (_relocRunning || _loadingMap) return;
    if (active.keyframes.length < 10) {
      locState = 'tracking';
      locNote = '';
      return;
    }
    _relocRunning = true;
    locState = 'localizing';
    locNote = 'Finding myself on "${active.name}"...';
    _pending.clear();
    if (mounted) setState(() {});
    await Future<void>.delayed(const Duration(milliseconds: 300));
    final s = scan;
    if (s == null || stale) {
      _relocRunning = false;
      _relocFailed('no lidar data');
      return;
    }
    final t = s.appMs;
    final raw = t == null ? null : (poses.at(t) ?? poses.latest);
    final pts = ScanMatcher.robotFrame(s.points, fwdM: lidarFwdM, leftM: lidarLeftM);
    // 1) quick checks: where I last was on this map, and the home spot
    MatchResult? quick;
    String quickNote = '';
    final guesses = <(List<double>, String)>[
      if (active.lastPose != null) (active.lastPose!, 'picked up where I left off'),
      (<double>[0, 0, math.pi / 2], 'at the home spot'),
    ];
    final coarsePts = [for (var i = 0; i < pts.length; i += 2) pts[i]];
    for (final gss in guesses) {
      final g0 = gss.$1;
      var r = matcher.local(coarsePts, g0[0], g0[1], g0[2], lin: 0.5, linStep: 0.05, ang: 0.35, angStep: 0.035);
      if (r.atEdge) continue;
      r = matcher.local(pts, r.x, r.y, r.h, lin: 0.05, linStep: 0.01, ang: 0.035, angStep: 0.007);
      // symmetric rooms: if facing the other way fits almost as well, don't trust the quick answer
      final flipped = matcher.local(coarsePts, r.x, r.y, r.h + math.pi, lin: 0.2, linStep: 0.05, ang: 0.2, angStep: 0.035);
      if (flipped.score > r.score * 0.85) continue;
      if (r.hitRatio >= 0.75 && (quick == null || r.score > quick.score * 1.03)) {
        quick = r;
        quickNote = gss.$2;
      }
      await Future<void>.delayed(Duration.zero);
    }
    if (quick != null) {
      _relocRunning = false;
      _acceptReloc(quick, raw, 'Found myself on "${active.name}" - $quickNote (${(quick.hitRatio * 100).round()}% of the scan fits)');
      return;
    }
    // 2) search the whole map
    final res = await matcher.global(pts);
    _relocRunning = false;
    if (res.isEmpty) {
      _relocFailed('the map has no open space yet');
      return;
    }
    final best = res.first;
    MatchResult? rival;
    for (final r in res.skip(1)) {
      final far = math.sqrt(_sq(r.x - best.x) + _sq(r.y - best.y)) > 0.6 || _angDiff(r.h, best.h).abs() > 0.4;
      if (far) {
        rival = r;
        break;
      }
    }
    final ambiguous = rival != null && rival.score > best.score * 0.95;
    if (best.hitRatio < 0.5 || ambiguous) {
      _relocFailed(ambiguous ? 'two places look alike' : 'no good match');
      return;
    }
    _acceptReloc(best, raw, 'Found myself on "${active.name}" (${(best.hitRatio * 100).round()}% of the scan fits)');
  }

  void _acceptReloc(MatchResult best, Pose? raw, String note) {
    if (_mode == 'ar' && raw != null && raw.good) {
      _corr.setSoThat(raw, best.x, best.y, best.h);
    } else {
      _lidarPose = Pose(appClockMs(), best.x, best.y, best.h, true);
      _mode = 'lidar';
    }
    _lastRobotPose = Pose(appClockMs(), best.x, best.y, best.h, true);
    locState = 'tracking';
    locNote = note;
    _flash(note);
    _relocAttempts = 0;
    trail.clear();
    _sigs.clear();
    if (mounted) setState(() {});
  }

  void _relocFailed(String why) {
    locState = 'lost';
    if (why == 'no lidar data') {
      locNote = 'Waiting for the robot\'s lidar...';
      Future<void>.delayed(const Duration(seconds: 3), () {
        if (locState == 'lost') _relocalize();
      });
      if (mounted) setState(() {});
      return;
    }
    _relocAttempts++;
    locNote = "Can't find myself yet ($why). Drive a little and I'll keep trying, or put me on the home spot and press I'm at home.";
    if (_relocAttempts < 8) {
      Future<void>.delayed(const Duration(seconds: 4), () {
        if (locState == 'lost') _relocalize();
      });
    }
    if (mounted) setState(() {});
  }

  /// Manual fallback: the robot is on the map's home spot, facing the way it faced when the map started.
  void _atHome() {
    const h = math.pi / 2;
    final raw = poses.latest;
    if (_mode == 'ar' && raw != null && raw.good) {
      _corr.setSoThat(raw, 0, 0, h);
    } else {
      _lidarPose = Pose(appClockMs(), 0, 0, h, true);
      _mode = 'lidar';
    }
    _lastRobotPose = Pose(appClockMs(), 0, 0, h, true);
    locState = 'tracking';
    locNote = 'Placed on the home spot';
    _relocAttempts = 0;
    trail.clear();
    if (mounted) setState(() {});
  }

  // ---------- robot sensors (ESP32 feed) ----------

  /// Where a profile sensor is in map coordinates right now.
  Offset? _sensorWorld(BotSensor sn) {
    final p = robotPose;
    if (p == null) return null;
    final (fwd, left) = profile.sensorOffset(sn);
    final c = math.cos(p.heading), sd = math.sin(p.heading);
    return Offset(p.x + c * fwd - sd * left, p.y + sd * fwd + c * left);
  }

  String? _appliedHardware;
  bool _profileReady = false;

  /// The robot's own sensor table is the truth for robot-side hardware; phone sensors stay in the
  /// profile. Placements measured in the profile before sensors lived on the robot carry over once.
  void _mergeRobotHardware() {
    if (!_profileReady) return; // never merge into the built-in default while the saved profile loads
    final caps = sensors.caps;
    final list = caps?['sensors'];
    if (list is! List) return; // older firmware: nothing to merge
    final key = jsonEncode(list);
    if (key == _appliedHardware) return;
    _appliedHardware = key;
    // our own edits haven't reached the robot yet: keep them (the periodic sync retries the push)
    if (profile.hardwareDirty) return;
    final robotSensors = [
      for (final h in list)
        if (h is Map) BotSensor.fromHardware(h)
    ].whereType<BotSensor>().toList();
    final legacy = profile.hardwareMigrated ? <BotSensor>[] : [for (final x in profile.sensors) if (!x.onPhone && !x.onRobot) x];
    profile.hardwareMigrated = true;
    var carried = false;
    for (final r in robotSensors) {
      final i = legacy.indexWhere((l) => l.type == r.type);
      if (i < 0) continue;
      final l = legacy.removeAt(i);
      r.fromLeftMm = l.fromLeftMm;
      r.fromFrontMm = l.fromFrontMm;
      r.heightMm = l.heightMm;
      r.yawDeg = l.yawDeg;
      if (l.widthMm > 0) r.widthMm = l.widthMm;
      carried = true;
    }
    profile.sensors
      ..removeWhere((x) => !x.onPhone)
      ..addAll(robotSensors);
    if (caps!['drive'] is String) profile.drive = caps['drive'] as String;
    if (carried) profile.hardwareDirty = true;
    BotProfileStore.save(profile, store.robot);
    server.broadcast({'type': 'bot', 'profile': profile.toJson()});
    if (carried) _pushHardwareIfChanged(); // write the measured placements back to the robot
  }

  /// HTTP to the robot's ESP32.
  Future<(int, String)> _robotHttp(String method, String path, [String? body]) async {
    final ip = motion.address;
    if (ip == null) return (0, 'robot not connected');
    final c = HttpClient()..connectionTimeout = const Duration(seconds: 3);
    try {
      final uri = Uri.parse('http://$ip$path');
      final req = method == 'POST' ? await c.postUrl(uri) : await c.getUrl(uri);
      if (body != null) {
        // the ESP32's web server can't read chunked bodies: send the length up front
        final bytes = utf8.encode(body);
        req.headers.contentType = ContentType.json;
        req.contentLength = bytes.length;
        req.add(bytes);
      }
      final res = await req.close();
      final text = await res.transform(utf8.decoder).join();
      return (res.statusCode, text);
    } catch (e) {
      return (0, '$e');
    } finally {
      c.close();
    }
  }

  /// Send the robot-side sensors to the ESP32 if they differ from what it has (it restarts to apply).
  Future<void> _pushHardwareIfChanged() async {
    if (!_profileReady) return;
    final caps = sensors.caps;
    if (caps == null || caps['sensors'] is! List) return;
    final mine = [for (final x in profile.sensors) if (!x.onPhone) x.toHardware()];
    final theirs = [
      for (final h in caps['sensors'] as List)
        if (h is Map) BotSensor.fromHardware(h)?.toHardware()
    ].whereType<Map<String, dynamic>>().toList();
    if (jsonEncode(mine) == jsonEncode(theirs)) {
      if (profile.hardwareDirty) {
        profile.hardwareDirty = false; // the robot has it
        BotProfileStore.save(profile, store.robot);
      }
      return;
    }
    final body = jsonEncode({'name': caps['name'], 'drive': profile.drive, 'pins': caps['pins'], 'sensors': mine});
    final (code, text) = await _robotHttp('POST', '/api/hardware', body);
    if (code == 200) {
      profile.hardwareDirty = false;
      BotProfileStore.save(profile, store.robot);
      _appliedHardware = null;
      sensors.caps = null; // a fresh announce arrives after the restart
      _flash('Sensor setup sent to the robot - it restarts to apply (about 10 s)');
    } else {
      _flash('Could not update the robot: ${code == 0 ? text : 'HTTP $code $text'}');
    }
  }

  /// Cliff sensors: remember the current floor reading as normal.
  Future<void> _calibrateFloor(String id) async {
    final (code, text) = await _robotHttp('GET', '/tof/calibrate?id=${Uri.encodeQueryComponent(id)}');
    _flash(code == 200 ? text : 'Calibration failed: $text');
    if (code == 200) {
      final (c2, hwJson) = await _robotHttp('GET', '/api/hardware');
      if (c2 == 200) {
        try {
          sensors.caps = jsonDecode(hwJson) as Map<String, dynamic>;
          _mergeRobotHardware();
        } catch (_) {}
      }
    }
  }

  bool get _robotHasLidar {
    final raw = sensors.caps?['sensors'];
    if (raw is List) return raw.any((e) => e is Map && e['type'] == 'lidar' && e['enabled'] != false);
    if (raw is Map) return raw['lidar'] != false;
    return true;
  }

  bool _switchingRobot = false;

  /// A robot with a different name announced itself: save what we have and open its profile and maps.
  Future<void> _switchRobotIfNeeded() async {
    final name = sensors.caps?['name'];
    if (name is! String || _switchingRobot) return;
    final clean = MapStore.safeName(name);
    if (clean == store.robot) return;
    _switchingRobot = true;
    try {
      await _saveActive();
      BotProfileStore.save(profile, store.robot);
      store.robot = clean;
      widget.settings?.lastRobot = clean;
      widget.settings?.save();
      _appliedHardware = null;
      final p = await BotProfileStore.load(clean);
      profile = p ?? BotProfile.tankbotDefault()
        ..name = name;
      maxSpeed = profile.cruisePower;
      active = MapSession.fresh(lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
      grid.clear();
      trail.clear();
      mapImage = null;
      await _refreshMapList();
      await _loadLastMap();
      server.broadcast({'type': 'bot', 'profile': profile.toJson()});
      locNote = 'Switched to robot "$name"';
    } finally {
      _switchingRobot = false;
    }
    if (mounted) setState(() {});
  }

  final Map<String, int> _prevSensorVals = {};

  double _lastHwCheckMs = 0;

  void _onRobotSensors(Map<String, dynamic> r) {
    _switchRobotIfNeeded();
    _mergeRobotHardware();
    if (appClockMs() - _lastHwCheckMs > 15000) {
      _lastHwCheckMs = appClockMs();
      _pushHardwareIfChanged(); // retries if an earlier update didn't reach the robot
    }
    final list = r['sensors'];
    if (list is! List) return;
    for (final e in list) {
      if (e is! Map || e['id'] is! String) continue;
      final id = e['id'] as String;
      final v = e['ok'] == true ? ((e['v'] as num?)?.toInt() ?? -1) : -1;
      final sn = profile.byId(id);
      if (sn != null && sn.type == 'bumper' && sn.role == 'bump' && v == 1 && (_prevSensorVals[id] ?? 0) != 1) _bumpObstacle(id);
      _prevSensorVals[id] = v;
    }
  }

  /// A bumper hit: something is there that the lidar didn't see. Put it on the map, permanently.
  void _bumpObstacle(String sensorId) {
    if (locState != 'tracking' || _loadingMap) return;
    final sn = profile.byId(sensorId);
    final w = sn == null ? null : _sensorWorld(sn);
    if (w == null) return;
    // a little beyond the bumper face, in the direction it faces
    final p = robotPose!;
    final a = p.heading + sn!.yawDeg * math.pi / 180;
    final ox = w.dx + math.cos(a) * 0.05, oy = w.dy + math.sin(a) * 0.05;
    active.edits.add({'type': 'obstacle', 'id': active.nextEditId++, 'stroke': 'bump${active.nextEditId}', 'x': ox, 'y': oy, 'r': 0.07});
    grid.markCircle(ox, oy, 0.07);
    active.edited = true;
    navNote = 'Bumped something - marked it on the map';
  }

  // Drop-offs seen repeatedly become permanent map obstacles (stairs don't move).
  final Map<int, int> _dropSeen = {};
  double _dropSeenResetMs = 0;
  static const int _dropConfirmFrames = 15, _maxDropEdits = 300;
  // where each candidate drop-off was first seen: it must also be seen from 30 cm away
  final Map<int, Offset> _dropFirstSeenFrom = {};

  void _rememberDropOffs() {
    if (locState != 'tracking' || _loadingMap || _rebuilding || !mapping || !_mapAllowed) return;
    final now = appClockMs();
    if (now - _dropSeenResetMs > 20000) {
      _dropSeen.clear();
      _dropFirstSeenFrom.clear();
      _dropSeenResetMs = now;
    }
    final cliffs = _depthWorldSplit(true);
    if (cliffs.isEmpty) return;
    final existing = [
      for (final e in active.edits)
        if (e['type'] == 'obstacle' && e['kind'] == 'dropoff') Offset((e['x'] as num).toDouble(), (e['y'] as num).toDouble())
    ];
    var added = false;
    for (final c in cliffs) {
      final key = (c.dx / 0.1).round() * 100000 + (c.dy / 0.1).round();
      final n = (_dropSeen[key] ?? 0) + 1;
      _dropSeen[key] = n;
      final here = robotPose == null ? Offset.zero : Offset(robotPose!.x, robotPose!.y);
      final from = _dropFirstSeenFrom.putIfAbsent(key, () => here);
      if (n < _dropConfirmFrames || (here - from).distance < 0.3) continue;
      _dropSeen[key] = -100000; // remember once
      if (existing.length >= _maxDropEdits) break;
      if (existing.any((e) => (e - c).distance < 0.12)) continue;
      final id = active.nextEditId++;
      active.edits.add({'type': 'obstacle', 'kind': 'dropoff', 'id': id, 'stroke': 'drop$id', 'x': c.dx, 'y': c.dy, 'r': 0.08});
      grid.markCircle(c.dx, c.dy, 0.08);
      existing.add(c);
      added = true;
    }
    if (added) {
      active.edited = true;
      navNote = 'Drop-off remembered on the map';
    }
  }

  /// Depth-camera obstacles and drop-offs in map coordinates (platform frame -> tracked point -> world).
  List<Offset> _depthWorld() {
    final p = robotPose;
    if (p == null || !depth.fresh(appClockMs())) return const [];
    final (tx, ty) = profile.trackedPoint;
    final c = math.cos(p.heading), sd = math.sin(p.heading);
    Offset toWorld(Offset q) {
      final fwd = q.dx - tx, left = q.dy - ty;
      return Offset(p.x + c * fwd - sd * left, p.y + sd * fwd + c * left);
    }
    return [for (final q in depth.obstacles) toWorld(q), for (final q in depth.cliffs) toWorld(q)];
  }

  List<Offset> _depthWorldSplit(bool cliffs) {
    final p = robotPose;
    if (p == null || !depth.fresh(appClockMs())) return const [];
    final (tx, ty) = profile.trackedPoint;
    final c = math.cos(p.heading), sd = math.sin(p.heading);
    final src = cliffs ? depth.cliffs : depth.obstacles;
    return [
      for (final q in src)
        Offset(p.x + c * (q.dx - tx) - sd * (q.dy - ty), p.y + sd * (q.dx - tx) + c * (q.dy - ty))
    ];
  }

  /// Every forward/side/rear ranger (ToF, ultrasonic) used for obstacles: its reading as a point on
  /// the map (low obstacles the lidar misses).
  List<Offset> _rangerPoints() {
    final p = robotPose;
    if (p == null) return const [];
    final out = <Offset>[];
    for (final sn in profile.sensors) {
      if (!sn.enabled || sn.role != 'obstacle' || sn.floorTilt || (sn.type != 'tof' && sn.type != 'ultrasonic')) continue;
      final mm = sensors.value(sn.id);
      final maxMm = sn.type == 'ultrasonic' ? 800 : 1500;
      if (mm == null || mm < 30 || mm > maxMm) continue;
      final w = _sensorWorld(sn);
      if (w == null) continue;
      final a = p.heading + sn.yawDeg * math.pi / 180;
      out.add(Offset(w.dx + math.cos(a) * mm / 1000, w.dy + math.sin(a) * mm / 1000));
    }
    return out;
  }

  // ---------- tap-to-go navigation ----------
  List<List<double>> get _nogoLines => [
        for (final e in active.edits)
          if (e['type'] == 'nogo')
            [(e['x1'] as num).toDouble(), (e['y1'] as num).toDouble(), (e['x2'] as num).toDouble(), (e['y2'] as num).toDouble()]
      ];

  /// What the lidar sees right now, within 2.5 m, in map coordinates.
  List<Offset> _liveObstacles() {
    final p = robotPose, sc = scan;
    if (p == null || sc == null || stale) return const [];
    final c = math.cos(p.heading), sn = math.sin(p.heading);
    return [
      for (final q in ScanMatcher.robotFrame(sc.points, fwdM: lidarFwdM, leftM: lidarLeftM, stride: 2, maxR: 2.5))
        Offset(p.x + c * q.dx - sn * q.dy, p.y + sn * q.dx + c * q.dy),
      ..._rangerPoints(),
      ..._depthWorld(),
    ];
  }

  void _navGoto(double x, double y) {
    if (locState != 'tracking') {
      navState = 'failed';
      navNote = "I don't know where I am on the map yet";
      return;
    }
    if (server.clientCount == 0) return;
    navGoal = Offset(x, y);
    _navFails = 0;
    _navRotating = false;
    _onRelease(); // start from a standstill
    if (_navReplan()) {
      navNote = 'Driving to the goal';
    }
    _navTimer ??= Timer.periodic(const Duration(milliseconds: 100), (_) => _navStep());
  }

  bool _navReplan() {
    final p = robotPose, goal = navGoal;
    if (p == null || goal == null || _rebuilding) return false;
    _navLastPlanMs = appClockMs();
    navReplans++;
    final r = Planner.plan(grid, _nogoLines, _liveObstacles(), p.x, p.y, goal.dx, goal.dy,
        robotRadius: profile.inflationRadiusM);
    if (r.error != null || r.path.length < 2) {
      navState = 'blocked';
      navNote = r.error ?? 'No route';
      navPath = [];
      return false;
    }
    navPath = r.path;
    _navIdx = 1;
    navState = 'driving';
    return true;
  }

  void _navStopMotors() {
    _cmdF = 0;
    _cmdT = 0;
    motion.release();
  }

  void _navCancel(String why) {
    _cmdF = 0;
    _cmdT = 0;
    navState = 'idle';
    navNote = why;
    navPath = [];
    navGoal = null;
    _navRotating = false;
    motion.release();
    if (mounted) setState(() {});
  }

  void _navFinish(String msg, {bool failed = false}) {
    _cmdF = 0;
    _cmdT = 0;
    navState = failed ? 'failed' : 'arrived';
    navNote = msg;
    navPath = [];
    _navRotating = false;
    motion.release();
    if (mounted) setState(() {});
  }

  /// Does anything the lidar sees now sit on the next 1.5 m of the route?
  bool _pathBlocked(Offset pos, List<Offset> live) {
    if (navPath.length < 2 || live.isEmpty) return false;
    final segs = <List<Offset>>[];
    var a = pos, along = 0.0;
    for (var i = _navIdx; i < navPath.length && along < 1.5; i++) {
      segs.add([a, navPath[i]]);
      along += (navPath[i] - a).distance;
      a = navPath[i];
    }
    for (final o in live) {
      if ((o - pos).distance > 2.0) continue;
      for (final sg in segs) {
        if (_segDist(o, sg[0], sg[1]) < profile.bodyRadiusM - 0.03) return true;
      }
    }
    return false;
  }

  static double _segDist(Offset p, Offset a, Offset b) {
    final d = b - a;
    final len2 = d.dx * d.dx + d.dy * d.dy;
    var t = len2 < 1e-9 ? 0.0 : ((p.dx - a.dx) * d.dx + (p.dy - a.dy) * d.dy) / len2;
    t = t.clamp(0.0, 1.0);
    return (p - Offset(a.dx + d.dx * t, a.dy + d.dy * t)).distance;
  }

  /// 10 times a second while navigating: steer along the route, re-plan around surprises.
  void _navStep() {
    if (!navActive) return;
    final now = appClockMs();
    if (server.clientCount == 0) {
      _navCancel('Stopped: no controller connected (someone needs to be watching)');
      return;
    }
    if (locState != 'tracking' || _rebuilding) {
      _navStopMotors();
      navNote = _rebuilding ? 'Updating the map...' : 'Lost my position - waiting';
      return;
    }
    final p = robotPose, goal = navGoal;
    if (p == null || goal == null) return;
    final pos = Offset(p.x, p.y);
    if ((goal - pos).distance < 0.2) {
      _navFinish('Arrived');
      return;
    }
    final live = _liveObstacles();

    if (navState == 'blocked') {
      _navStopMotors();
      if (now - _navLastPlanMs > 1500) {
        if (_navReplan()) {
          navNote = 'Found a way - driving';
          _navFails = 0;
        } else if (++_navFails >= 12) {
          _navFinish("Couldn't find a way there: ${navNote.toLowerCase()}", failed: true);
        }
      }
      return;
    }
    // re-plan now and then (the map changes), and when the lidar sees something on the route
    // (at most once a second, so it doesn't thrash)
    var replan = now - _navLastPlanMs > 4000;
    if (!replan && now - _navLastBlockReplanMs > 1000 && _pathBlocked(pos, live)) {
      replan = true;
      _navLastBlockReplanMs = now;
    }
    if (replan && !_navReplan()) {
      _navStopMotors();
      return;
    }

    // next waypoint: move on only once we are really at the current one (less corner cutting)
    while (_navIdx < navPath.length - 1 && (navPath[_navIdx] - pos).distance < 0.08) {
      _navIdx++;
    }
    final target = navPath[math.min(_navIdx, navPath.length - 1)];
    final alpha = _angDiff(math.atan2(target.dy - p.y, target.dx - p.x), p.heading);

    // Only power levels this robot can act on: zero, or at least its minimum-to-move.
    // Straight runs at cruise power; when off course, stop and turn on the spot (no steering blend,
    // which would starve one track below its threshold and just make it whine).
    final cruise = profile.cruisePower;
    final turnP = math.max(profile.minPower, 0.9); // turning on the spot needs nearly everything
    double f = 0, t = 0;
    // Turn on the spot when more than 20 deg off; keep going until within 4 deg. The direction is
    // re-checked every tick (an overshoot turns back), and for the last 20 deg the turn is pulsed:
    // 120 ms on, 200 ms settle, so full-power turns can't spin past the target.
    final dir = alpha > 0 ? -1.0 : 1.0; // turn command: positive = clockwise/right
    if (_navRotating) {
      if (alpha.abs() <= 0.07 && now - _navTurnStartMs >= 150) {
        _navRotating = false;
        f = cruise;
      } else if (alpha.abs() > 0.35) {
        t = dir * turnP;
      } else {
        final phase = ((now - _navTurnStartMs) % 320).toInt();
        t = phase < 120 ? dir * turnP : 0;
      }
    } else if (alpha.abs() > 0.35) {
      _navRotating = true;
      _navTurnStartMs = now;
      t = dir * turnP;
    } else {
      f = cruise;
    }
    // autonomy always asks the guardian before moving forward (whatever the manual setting)
    final g = guard;
    if (f > 0 && !g.forwardClear) {
      navFrontBlocks++;
      f = 0;
      if (now - _navLastBlockReplanMs > 1000) {
        _navLastBlockReplanMs = now;
        if (!_navReplan()) {
          _navStopMotors();
          return;
        }
        navNote = 'In the way (${g.reason}) - going around';
      }
    }
    _cmdF = f;
    _cmdT = t;
    motion.drive(_cmdF, _cmdT);
    _lastDriveMs = now;
  }

  // ---------- mounting ----------
  bool get _mapAllowed => !robotMode || mountState == 'mounted';
  bool get _motorsIdle => !motion.driving && appClockMs() - _lastDriveMs > 700;

  void _enterRobotMode() {
    setState(() {
      robotMode = true;
      mountState = 'mounting';
      mountNote = 'Mount me - mapping starts once I sit still in the cradle';
      _robotModeSinceMs = appClockMs();
      _pending.clear();
      _sigs.clear();
    });
  }

  void _exitRobotMode() => setState(() {
        robotMode = false;
        mountState = 'off';
        mountNote = '';
      });

  /// Mounted = camera level (within ~15 deg), still for 3 s, motors idle.
  void _checkMounted() {
    if (!robotMode || mountState != 'mounting') return;
    final now = appClockMs();
    if (now - _robotModeSinceMs < 5000 || !_motorsIdle) return;
    final h = poses.history;
    if (h.isEmpty) return;
    final last = h.last;
    if (last.t - h.first.t < 3000) return;
    for (var i = h.length - 1; i >= 0; i--) {
      final p = h[i];
      if (last.t - p.t > 3000) break;
      if (!p.good || p.fy.abs() > 0.26) return;
      if (math.sqrt(_sq(p.x - last.x) + _sq(p.y - last.y)) > 0.015) return;
      if (_angDiff(p.heading, last.heading).abs() > 0.026) return;
    }
    _onMounted();
  }

  /// Robot idle, but ARKit says the phone moved while the lidar scene did not change:
  /// the phone moved on the robot. Pause mapping until it settles again.
  void _checkDisturbance(LidarScan s) {
    final t = s.appMs;
    if (!robotMode || t == null) return;
    final sig = _ScanSig(t, _bins(s.points));
    _sigs.add(sig);
    while (_sigs.isNotEmpty && sig.t - _sigs.first.t > 1500) {
      _sigs.removeAt(0);
    }
    if (mountState != 'mounted' || !_motorsIdle) return;
    _ScanSig? old;
    for (final o in _sigs) {
      if (sig.t - o.t >= 450) old = o;
    }
    if (old == null) return;
    final p0 = poses.at(old.t), p1 = poses.at(sig.t) ?? poses.latest;
    if (p0 == null || p1 == null) return;
    final moved = math.sqrt(_sq(p1.x - p0.x) + _sq(p1.y - p0.y));
    final turned = _angDiff(p1.heading, p0.heading).abs();
    if (moved < 0.03 && turned < 0.052) return; // under 3 cm and 3 deg: fine
    if (_sceneChange(old.bins, sig.bins) > 0.15) return; // lidar saw the robot move too: consistent
    disturbances++;
    mountState = 'mounting';
    mountNote = 'Phone moved on the robot - mapping paused until it settles';
    _pending.clear();
  }

  Future<void> _startNewMap({String note = 'New map started here'}) async {
    if (robotMode) {
      mountState = 'mounted';
      mountNote = note;
    }
    await _saveActive(); // keep the map we had
    active = MapSession.fresh(lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
    await _resetMap();
    _sigs.clear();
    _broadcastMaps();
    if (mounted) setState(() {});
  }

  static Float32List _bins(List<LidarPoint> pts) {
    final sum = Float32List(180), cnt = Float32List(180);
    for (final p in pts) {
      final b = (p.angleDeg / 2).floor() % 180;
      sum[b] += p.distMm;
      cnt[b] += 1;
    }
    for (var i = 0; i < 180; i++) {
      sum[i] = cnt[i] > 0 ? sum[i] / cnt[i] : 0;
    }
    return sum;
  }

  /// Fraction of 2-degree bins whose range changed by more than 5 cm.
  static double _sceneChange(Float32List a, Float32List b) {
    var both = 0, changed = 0;
    for (var i = 0; i < 180; i++) {
      if (a[i] > 0 && b[i] > 0) {
        both++;
        if ((a[i] - b[i]).abs() > 50) changed++;
      }
    }
    return both < 30 ? 1.0 : changed / both;
  }

  // ---------- mapping ----------
  bool get _arLive {
    if (widget.role != AppRole.mounted) return false;
    final l = poses.latest;
    return l != null && l.good && appClockMs() - l.t < 600;
  }

  String get poseSource => _mode == 'ar' ? 'camera tracking + lidar' : 'lidar only';

  /// Robot pose in map coordinates (camera pose corrected by the lidar, or lidar alone).
  Pose? get robotPose {
    if (_mode == 'lidar') return _lidarPose;
    final l = poses.latest;
    return l == null ? null : _corr.apply(l);
  }

  /// Use camera tracking when it is healthy; fall back to lidar-only tracking when it is not,
  /// keeping the pose continuous across the switch.
  void _updateMode() {
    final now = appClockMs();
    if (_arLive) {
      _arBadSinceMs = -1;
      if (_mode == 'lidar') {
        final lp = _lidarPose;
        if (lp != null) _corr.setSoThat(poses.latest!, lp.x, lp.y, lp.heading);
        _mode = 'ar';
      }
    } else {
      if (_arBadSinceMs < 0) _arBadSinceMs = now;
      if (_mode == 'ar' && now - _arBadSinceMs > 1000) {
        _lidarPose = _lastRobotPose ?? Pose(now, 0, 0, math.pi / 2, true);
        _mode = 'lidar';
      }
    }
  }

  void _processPending() {
    _updateMode();
    final latest = poses.latest;
    while (_pending.isNotEmpty) {
      final s = _pending.first;
      final t = s.appMs;
      Pose? raw;
      if (_mode == 'ar') {
        if (t == null) {
          _pending.removeAt(0);
          skippedNoPose++;
          continue;
        }
        if (latest == null || latest.t < t) {
          if (latest != null && appClockMs() - t > 1000) {
            _pending.removeAt(0);
            skippedNoPose++;
            continue;
          }
          break; // the camera pose for this moment has not arrived yet
        }
        raw = poses.at(t);
        _pending.removeAt(0);
        if (raw == null || !raw.good) {
          skippedNoPose++;
          continue;
        }
      } else {
        _pending.removeAt(0);
      }
      if (!_mapAllowed || _loadingMap || locState != 'tracking') continue;
      final pose = _trackScan(s, raw);
      if (pose == null) continue;
      final now = t ?? appClockMs();
      final prev = _lastRobotPose;
      final dt = math.max(0.05, (now - _lastPoseMs) / 1000.0);
      final turnRate = prev == null ? 0.0 : _angDiff(pose.heading, prev.heading).abs() / dt;
      _lastRobotPose = pose;
      _lastPoseMs = now;
      if (_rebuilding) continue;
      if (turnRate > _maxMapTurnRate) {
        skippedTurning++; // the scan is smeared by the turn: track with it, but don't paint it
        continue;
      }
      grid.integrate(pose, s.points, lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
      _maybeKeyframe(pose, s.points);
    }
  }

  /// Best pose for this scan: the camera's guess (or the last lidar pose), refined by
  /// matching the scan against the walls already on the map.
  Pose? _trackScan(LidarScan s, Pose? raw) {
    final lidarOnly = raw == null;
    final guess = lidarOnly ? (_lidarPose ?? Pose(appClockMs(), 0, 0, math.pi / 2, true)) : _corr.apply(raw);
    if (_rebuilding || grid.scansIntegrated < 15) {
      if (lidarOnly) _lidarPose = guess;
      return guess; // not enough map yet to match against
    }
    final pts = ScanMatcher.robotFrame(s.points, fwdM: lidarFwdM, leftM: lidarLeftM, stride: 2);
    if (pts.length < 40) return lidarOnly ? null : guess;
    final r = matcher.localCoarseFine(pts, guess.x, guess.y, guess.heading,
        lin: lidarOnly ? 0.16 : 0.12, ang: lidarOnly ? 0.26 : 0.17);
    if (r.hitRatio < 0.35 || r.atEdge) {
      matchMisses++;
      return lidarOnly ? null : guess; // unsure: keep the camera's guess, or skip the scan
    }
    matchHits++;
    // Camera mode: move halfway to the match each scan (smooth, robust to one bad match).
    final a = lidarOnly ? 1.0 : 0.5;
    final nx = guess.x + (r.x - guess.x) * a, ny = guess.y + (r.y - guess.y) * a;
    final nh = guess.heading + _angDiff(r.h, guess.heading) * a;
    lastCorrCm = math.sqrt(_sq(r.x - guess.x) + _sq(r.y - guess.y)) * 100;
    final pose = Pose(guess.t, nx, ny, nh, true, guess.fy);
    if (lidarOnly) {
      _lidarPose = pose;
    } else {
      _corr.setSoThat(raw, nx, ny, nh);
    }
    return pose;
  }

  Future<void> _resetMap() async {
    grid.clear();
    trail.clear();
    mapImage = null;
    _pending.clear();
    _corr.reset();
    _lidarPose = null;
    _lastRobotPose = null;
    locState = 'tracking';
    locNote = '';
    await poses.reset();
    if (mounted) setState(() {});
  }

  // ---------- driving ----------
  bool get stale => scan == null || DateTime.now().difference(scan!.received) > const Duration(seconds: 1);

  /// The guardian's current verdict on forward motion (lidar around the body's front, ESP32 reflexes).
  GuardVerdict get guard => Guardian.evaluate(
        profile: profile,
        scan: scan,
        scanStale: stale,
        stopDistMm: stopDistMm,
        reflexBlock: sensors.block,
        lidarExpected: _robotHasLidar,
        depthObstacles: depth.fresh(appClockMs()) ? depth.obstacles : const [],
        dropOffs: depth.fresh(appClockMs()) ? depth.cliffs : const [],
        depthStopMm: profile.depthStopMm,
      );
  String blockReason = '';

  void _applyDrive() {
    var f = _wantF * maxSpeed;
    final t = _wantT * maxSpeed;
    final g = guard;
    // manual driving: the obstacle-stop setting can switch the lidar check off, never the reflexes
    final veto = f > 0 && !g.forwardClear && (obstacleStop || g.reason.startsWith('robot reflex'));
    _blocked = veto;
    blockReason = veto ? g.reason : '';
    if (_blocked) f = 0;
    motion.drive(f, t);
  }

  void _onStick(double f, double t) {
    if (navActive) _navCancel('Stopped: manual control');
    _lastDriveMs = appClockMs();
    _wantF = f;
    _wantT = t;
    _applyDrive();
  }

  void _onRelease() {
    _lastDriveMs = appClockMs();
    _wantF = 0;
    _wantT = 0;
    _blocked = false;
    motion.release();
    if (mounted) setState(() {});
  }

  Future<void> _enterIp() async {
    final ctrl = TextEditingController(text: client.address ?? '');
    final ip = await showDialog<String>(
      context: context,
      builder: (c) => AlertDialog(
        title: const Text('Robot address'),
        content: TextField(
          controller: ctrl,
          keyboardType: const TextInputType.numberWithOptions(decimal: true),
          decoration: const InputDecoration(hintText: 'e.g. 192.168.1.225 (blank = auto)'),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(c), child: const Text('Cancel')),
          FilledButton(onPressed: () => Navigator.pop(c, ctrl.text), child: const Text('Connect')),
        ],
      ),
    );
    if (ip != null) {
      client.start(manualIp: ip);
      motion.start(manualIp: ip);
      sensors.start(manualIp: ip);
    }
  }

  String _motionText() {
    final m = motionStatus;
    if (!motion.connected) return 'Motion: not connected';
    if (m == null) return 'Motion: waiting for robot...';
    final l = (m['left'] as num).toStringAsFixed(2);
    final r = (m['right'] as num).toStringAsFixed(2);
    return 'Motors L $l  R $r  (${m['src']})\nwatchdog stops: ${m['wd_trips']}';
  }

  String _trackingText() {
    final p = robotPose;
    final sync = client.robotOffsetMs == null ? 'sync...' : 'sync ±${(client.syncRttMs! / 2).toStringAsFixed(0)} ms';
    final pos = p == null ? '' : '  pos ${p.x.toStringAsFixed(2)}, ${p.y.toStringAsFixed(2)} m';
    return 'AR ${poses.state}$pos  |  $sync  |  ${active.name}: ${active.keyframes.length} keyframes${active.unsaved ? " (unsaved)" : ""}';
  }

  // ---------- UI ----------
  @override
  Widget build(BuildContext context) {
    if (robotMode) return _robotModeScreen(context);

    final rate = _recent.length / 2.0;
    final st = robotStatus;
    final statusText = [
      stale ? 'NO DATA' : '${rate.toStringAsFixed(1)} scans/s',
      if (scan != null) '${scan!.points.length} pts',
      'dropped ${client.droppedScans}',
      if (st != null) 'Wi-Fi ${st['rssi']} dBm',
    ].join('  |  ');
    const small = TextStyle(fontSize: 12);

    return Scaffold(
      appBar: AppBar(
        title: const Text('TankBot'),
        actions: [
          if (widget.role == AppRole.mounted)
            IconButton(
              icon: const Icon(Icons.smart_toy),
              tooltip: 'Robot mode',
              onPressed: _enterRobotMode,
            ),
          IconButton(
            icon: const Icon(Icons.swap_horiz),
            tooltip: 'Change role',
            onPressed: widget.onChangeRole,
          ),
          if (view == ViewMode.map)
            IconButton(
              icon: Icon(mapping ? Icons.pause_circle : Icons.play_circle),
              tooltip: mapping ? 'Pause mapping' : 'Resume mapping',
              onPressed: () => setState(() => mapping = !mapping),
            ),
          if (view == ViewMode.map)
            IconButton(icon: const Icon(Icons.delete_sweep), tooltip: 'New map here', onPressed: () => _startNewMap()),
          IconButton(icon: const Icon(Icons.wifi_find), tooltip: 'Set address', onPressed: _enterIp),
          IconButton(
            icon: const Icon(Icons.refresh),
            tooltip: 'Reconnect',
            onPressed: () {
              client.start();
              motion.start();
              sensors.start();
            },
          ),
        ],
      ),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 2, 12, 0),
              child: Text('$link   •   remote: ${server.url ?? server.error ?? "starting..."}',
                  style: Theme.of(context).textTheme.bodySmall),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(8, 4, 8, 0),
              child: Text(statusText,
                  style: TextStyle(color: stale ? Colors.redAccent : Colors.tealAccent, fontWeight: FontWeight.w500)),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(8, 2, 8, 4),
              child: Text(_trackingText(), style: const TextStyle(fontSize: 11, color: Colors.white70)),
            ),
            SegmentedButton<ViewMode>(
              segments: const [
                ButtonSegment(value: ViewMode.radar, label: Text('Radar'), icon: Icon(Icons.radar)),
                ButtonSegment(value: ViewMode.map, label: Text('Map'), icon: Icon(Icons.map)),
              ],
              selected: {view},
              onSelectionChanged: (s) => setState(() => view = s.first),
            ),
            Expanded(
              child: CustomPaint(
                painter: view == ViewMode.radar
                    ? RadarPainter(scan: scan, rangeMm: rangeMm, stale: stale)
                    : MapPainter(map: mapImage, pose: robotPose, trail: trail, rangeMm: rangeMm),
                size: Size.infinite,
              ),
            ),
            if (_blocked)
              Container(
                width: double.infinity,
                color: Colors.red.withValues(alpha: 0.8),
                padding: const EdgeInsets.all(6),
                child: Text('OBSTACLE AHEAD - forward blocked ($blockReason)',
                    textAlign: TextAlign.center, style: const TextStyle(fontWeight: FontWeight.w600)),
              ),
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 4, 12, 8),
              child: Row(
                children: [
                  Joystick(onChanged: _onStick, onReleased: _onRelease),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(_motionText(), style: small),
                        const SizedBox(height: 4),
                        Text('Max speed ${(maxSpeed * 100).round()}%', style: small),
                        Slider(
                            value: maxSpeed,
                            min: 0.2,
                            max: 1.0,
                            divisions: 8,
                            onChanged: (v) => setState(() => maxSpeed = v)),
                        Text('View range ${(rangeMm / 1000).toStringAsFixed(1)} m', style: small),
                        Slider(
                            value: rangeMm,
                            min: 1000,
                            max: 12000,
                            divisions: 22,
                            onChanged: (v) => setState(() => rangeMm = v)),
                        Row(children: [
                          const Expanded(child: Text('Obstacle stop', style: small)),
                          Switch(value: obstacleStop, onChanged: (v) => setState(() => obstacleStop = v)),
                        ]),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  /// Minimal screen for when the phone is mounted on the robot.
  Widget _robotModeScreen(BuildContext context) {
    final p = robotPose;
    final ok = !stale && poses.state == 'normal';
    final sync = client.robotOffsetMs == null ? 'syncing' : '±${(client.syncRttMs! / 2).toStringAsFixed(0)} ms';
    TextStyle s(double size, [Color c = Colors.white70]) => TextStyle(fontSize: size, color: c);
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(children: [
                Icon(Icons.smart_toy, color: ok ? Colors.tealAccent : Colors.orangeAccent, size: 36),
                const SizedBox(width: 12),
                Text(widget.role == AppRole.mounted ? 'TankBot Brain' : 'TankBot Brain (in hand)', style: s(26, Colors.white)),
              ]),
              const SizedBox(height: 16),
              if (mountNote.isNotEmpty)
                Text(mountNote, style: s(18, mountState == 'mounted' ? Colors.tealAccent : Colors.amberAccent)),
              if (locNote.isNotEmpty)
                Text(locNote, style: s(15, locState == 'tracking' ? Colors.tealAccent : Colors.amberAccent)),
              Text('Tracking: $poseSource', style: s(14)),
              if (navState != 'idle' || navNote.isNotEmpty)
                Text('Navigation: ${navNote.isEmpty ? navState : navNote}', style: s(15, navActive ? Colors.lightBlueAccent : Colors.white70)),
              const SizedBox(height: 24),
              Text('Control from any browser on this Wi-Fi:', style: s(14)),
              const SizedBox(height: 6),
              SelectableText(server.url ?? server.error ?? 'starting...',
                  style: const TextStyle(fontSize: 22, color: Colors.tealAccent, fontWeight: FontWeight.w600)),
              const SizedBox(height: 24),
              Text('Remotes connected: ${server.clientCount}', style: s(16)),
              Text('Lidar: ${stale ? "NO DATA" : "${(_recent.length / 2.0).toStringAsFixed(1)} scans/s"}', style: s(16)),
              Text('Tracking: ${poses.state}', style: s(16)),
              Text('Clock sync: $sync', style: s(16)),
              Text('Mapped scans: ${grid.scansIntegrated}', style: s(16)),
              if (p != null) Text('Position: ${p.x.toStringAsFixed(2)}, ${p.y.toStringAsFixed(2)} m', style: s(16)),
              if (_blocked) Text('OBSTACLE AHEAD', style: s(20, Colors.redAccent)),
              const Spacer(),
              Center(
                child: TextButton(
                  onLongPress: _exitRobotMode,
                  onPressed: () {},
                  child: Text('Long-press to exit robot mode', style: s(13, Colors.white38)),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class RadarPainter extends CustomPainter {
  RadarPainter({required this.scan, required this.rangeMm, required this.stale});
  final LidarScan? scan;
  final double rangeMm;
  final bool stale;

  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final radius = math.min(size.width, size.height) / 2 - 12;
    final ring = Paint()
      ..color = Colors.white24
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    final step = rangeMm <= 3000 ? 500.0 : 1000.0;
    for (var d = step; d <= rangeMm + 1; d += step) {
      final r = d / rangeMm * radius;
      canvas.drawCircle(c, r, ring);
      final tp = TextPainter(
        text: TextSpan(
            text: '${(d / 1000).toStringAsFixed(d % 1000 == 0 ? 0 : 1)} m',
            style: const TextStyle(color: Colors.white38, fontSize: 10)),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, c + Offset(3, -r - tp.height));
    }
    canvas.drawLine(c + Offset(0, -radius), c + Offset(0, radius), ring);
    canvas.drawLine(c + Offset(-radius, 0), c + Offset(radius, 0), ring);

    final robot = Path()
      ..moveTo(c.dx, c.dy - 12)
      ..lineTo(c.dx - 8, c.dy + 8)
      ..lineTo(c.dx + 8, c.dy + 8)
      ..close();
    canvas.drawPath(robot, Paint()..color = Colors.orangeAccent);

    final s = scan;
    if (s == null) return;
    final pts = <Offset>[];
    for (final p in s.points) {
      if (p.distMm > rangeMm) continue;
      final th = p.angleDeg * math.pi / 180;
      final r = p.distMm / rangeMm * radius;
      pts.add(Offset(c.dx + r * math.sin(th), c.dy - r * math.cos(th)));
    }
    canvas.drawPoints(
      ui.PointMode.points,
      pts,
      Paint()
        ..color = stale ? Colors.white30 : Colors.tealAccent
        ..strokeWidth = 3.5
        ..strokeCap = StrokeCap.round,
    );
  }

  @override
  bool shouldRepaint(RadarPainter old) => old.scan != scan || old.rangeMm != rangeMm || old.stale != stale;
}

/// Top-down map, north-up (map +y is up), centred on the robot.
class MapPainter extends CustomPainter {
  MapPainter({required this.map, required this.pose, required this.trail, required this.rangeMm});
  final MapImage? map;
  final Pose? pose;
  final List<Offset> trail;
  final double rangeMm;

  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final radius = math.min(size.width, size.height) / 2 - 12;
    final ppm = radius / (rangeMm / 1000.0);
    final px = pose?.x ?? 0, py = pose?.y ?? 0;
    Offset toScreen(double x, double y) => Offset(c.dx + (x - px) * ppm, c.dy - (y - py) * ppm);

    canvas.drawRect(Offset.zero & size, Paint()..color = const Color(0xFF15191C));
    final m = map;
    if (m != null) {
      final tl = toScreen(m.leftM, m.topM);
      final dst = Rect.fromLTWH(tl.dx, tl.dy, m.image.width * m.resolution * ppm, m.image.height * m.resolution * ppm);
      canvas.drawImageRect(
        m.image,
        Rect.fromLTWH(0, 0, m.image.width.toDouble(), m.image.height.toDouble()),
        dst,
        Paint()..filterQuality = FilterQuality.none,
      );
    }
    final gridPaint = Paint()
      ..color = Colors.white10
      ..strokeWidth = 1;
    final x0 = (px - rangeMm / 1000 * 2).floorToDouble(), x1 = (px + rangeMm / 1000 * 2).ceilToDouble();
    final y0 = (py - rangeMm / 1000 * 2).floorToDouble(), y1 = (py + rangeMm / 1000 * 2).ceilToDouble();
    for (var x = x0; x <= x1; x++) {
      canvas.drawLine(toScreen(x, y0), toScreen(x, y1), gridPaint);
    }
    for (var y = y0; y <= y1; y++) {
      canvas.drawLine(toScreen(x0, y), toScreen(x1, y), gridPaint);
    }
    if (trail.length > 1) {
      final first = toScreen(trail.first.dx, trail.first.dy);
      final path = Path()..moveTo(first.dx, first.dy);
      for (final t in trail.skip(1)) {
        final s = toScreen(t.dx, t.dy);
        path.lineTo(s.dx, s.dy);
      }
      canvas.drawPath(
          path,
          Paint()
            ..color = Colors.orangeAccent.withValues(alpha: 0.6)
            ..style = PaintingStyle.stroke
            ..strokeWidth = 2);
    }
    final p = pose;
    if (p != null) {
      final h = p.heading;
      final dir = Offset(math.cos(h), -math.sin(h));
      final side = Offset(-dir.dy, dir.dx);
      final robot = Path()
        ..moveTo(c.dx + dir.dx * 14, c.dy + dir.dy * 14)
        ..lineTo(c.dx - dir.dx * 9 + side.dx * 9, c.dy - dir.dy * 9 + side.dy * 9)
        ..lineTo(c.dx - dir.dx * 9 - side.dx * 9, c.dy - dir.dy * 9 - side.dy * 9)
        ..close();
      canvas.drawPath(robot, Paint()..color = p.good ? Colors.orangeAccent : Colors.grey);
    }
    final tp = TextPainter(
      text: const TextSpan(text: 'grid: 1 m', style: TextStyle(color: Colors.white38, fontSize: 10)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, const Offset(8, 8));
  }

  @override
  bool shouldRepaint(MapPainter old) => true;
}

class _ScanSig {
  final double t;
  final Float32List bins;
  _ScanSig(this.t, this.bins);
}

double _sq(double v) => v * v;

double _angDiff(double a, double b) {
  var d = a - b;
  while (d > math.pi) {
    d -= 2 * math.pi;
  }
  while (d < -math.pi) {
    d += 2 * math.pi;
  }
  return d;
}
