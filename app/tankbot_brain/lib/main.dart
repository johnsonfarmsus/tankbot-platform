import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'lidar_client.dart';
import 'motion_client.dart';
import 'joystick.dart';
import 'pose_client.dart';
import 'occupancy_grid.dart';

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
      home: const LidarScreen(),
    );
  }
}

enum ViewMode { radar, map }

class LidarScreen extends StatefulWidget {
  const LidarScreen({super.key});
  @override
  State<LidarScreen> createState() => _LidarScreenState();
}

class _LidarScreenState extends State<LidarScreen> with WidgetsBindingObserver {
  final client = LidarClient();
  final motion = MotionClient();
  final poses = PoseClient();
  final grid = OccupancyGrid();
  final List<StreamSubscription> _subs = [];
  LidarScan? scan;
  Map<String, dynamic>? robotStatus;
  Map<String, dynamic>? motionStatus;
  String link = 'Starting...';
  double rangeMm = 6000;
  double maxSpeed = 0.6;
  bool obstacleStop = false;
  static const double stopDistMm = 300, selfMaskMm = 150, frontHalfAngle = 25;
  double _wantF = 0, _wantT = 0;
  bool _blocked = false;
  final List<DateTime> _recent = [];
  Timer? _tick;

  // Mapping
  ViewMode view = ViewMode.radar;
  bool mapping = true;
  final List<LidarScan> _pending = [];
  final List<Offset> trail = [];
  MapImage? mapImage;
  bool _rendering = false;
  int skippedNoPose = 0;

  // Lidar position relative to the phone, metres. Set once the phone cradle is mounted.
  static const double lidarFwdM = 0.0, lidarLeftM = 0.0;

  @override
  void initState() {
    super.initState();
    _subs.add(client.scans.listen((s) {
      final now = DateTime.now();
      _recent.add(now);
      _recent.removeWhere((t) => now.difference(t) > const Duration(seconds: 2));
      scan = s;
      if (mapping) _pending.add(s);
      _processPending();
      if (motion.driving) _applyDrive();
      setState(() {});
    }));
    _subs.add(client.status.listen((s) => setState(() => robotStatus = s)));
    _subs.add(client.linkState.listen((s) => setState(() => link = s)));
    _subs.add(motion.status.listen((m) => setState(() => motionStatus = m)));
    _subs.add(poses.poses.listen((p) {
      if (p.good && (trail.isEmpty || (Offset(p.x, p.y) - trail.last).distance > 0.05)) {
        trail.add(Offset(p.x, p.y));
        if (trail.length > 5000) trail.removeAt(0);
      }
      _processPending();
    }));
    _tick = Timer.periodic(const Duration(milliseconds: 400), (_) async {
      if (grid.dirty && !_rendering && view == ViewMode.map) {
        _rendering = true;
        final img = await grid.render();
        _rendering = false;
        if (img != null) mapImage = img;
      }
      if (mounted) setState(() {});
    });
    WidgetsBinding.instance.addObserver(this);
    client.start();
    motion.start();
    poses.start();
  }

  @override
  void dispose() {
    for (final s in _subs) {
      s.cancel();
    }
    _tick?.cancel();
    WidgetsBinding.instance.removeObserver(this);
    motion.dispose();
    client.dispose();
    poses.dispose();
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state != AppLifecycleState.resumed) motion.release();
  }

  /// Place scans into the map once we have a pose for the moment they were taken.
  void _processPending() {
    final latest = poses.latest;
    while (_pending.isNotEmpty) {
      final s = _pending.first;
      final t = s.appMs;
      if (t == null) {
        _pending.removeAt(0);
        skippedNoPose++;
        continue;
      }
      if (latest == null || latest.t < t) {
        // pose for this moment not here yet; give up on very old scans
        if (latest != null && appClockMs() - t > 1000) {
          _pending.removeAt(0);
          skippedNoPose++;
          continue;
        }
        break;
      }
      _pending.removeAt(0);
      final p = poses.at(t);
      if (p == null || !p.good) {
        skippedNoPose++;
        continue;
      }
      grid.integrate(p, s.points, lidarFwdM: lidarFwdM, lidarLeftM: lidarLeftM);
    }
  }

  bool get stale => scan == null || DateTime.now().difference(scan!.received) > const Duration(seconds: 1);

  double? get frontClearanceMm {
    final sc = scan;
    if (sc == null || stale) return null;
    double? best;
    for (final p in sc.points) {
      final a = p.angleDeg > 180 ? p.angleDeg - 360 : p.angleDeg;
      if (a.abs() <= frontHalfAngle && p.distMm > selfMaskMm) {
        if (best == null || p.distMm < best) best = p.distMm;
      }
    }
    return best;
  }

  void _applyDrive() {
    var f = _wantF * maxSpeed;
    final t = _wantT * maxSpeed;
    final clear = frontClearanceMm;
    _blocked = obstacleStop && f > 0 && (stale || (clear != null && clear < stopDistMm));
    if (_blocked) f = 0;
    motion.drive(f, t);
  }

  void _onStick(double f, double t) {
    _wantF = f;
    _wantT = t;
    _applyDrive();
  }

  void _onRelease() {
    _wantF = 0;
    _wantT = 0;
    _blocked = false;
    motion.release();
    setState(() {});
  }

  Future<void> _resetMap() async {
    grid.clear();
    trail.clear();
    mapImage = null;
    _pending.clear();
    await poses.reset();
    setState(() {});
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
    final p = poses.latest;
    final sync = client.robotOffsetMs == null ? 'sync...' : 'sync ±${(client.syncRttMs! / 2).toStringAsFixed(0)} ms';
    final pos = p == null ? '' : '  pos ${p.x.toStringAsFixed(2)}, ${p.y.toStringAsFixed(2)} m';
    return 'AR ${poses.state}$pos  |  $sync  |  mapped ${grid.scansIntegrated}';
  }

  @override
  Widget build(BuildContext context) {
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
          if (view == ViewMode.map)
            IconButton(
              icon: Icon(mapping ? Icons.pause_circle : Icons.play_circle),
              tooltip: mapping ? 'Pause mapping' : 'Resume mapping',
              onPressed: () => setState(() => mapping = !mapping),
            ),
          if (view == ViewMode.map)
            IconButton(icon: const Icon(Icons.delete_sweep), tooltip: 'Clear map', onPressed: _resetMap),
          IconButton(icon: const Icon(Icons.wifi_find), tooltip: 'Set address', onPressed: _enterIp),
          IconButton(
            icon: const Icon(Icons.refresh),
            tooltip: 'Reconnect',
            onPressed: () {
              client.start();
              motion.start();
            },
          ),
        ],
      ),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 2, 12, 0),
              child: Text(link, style: Theme.of(context).textTheme.bodySmall),
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
                    : MapPainter(map: mapImage, pose: poses.latest, trail: trail, rangeMm: rangeMm),
                size: Size.infinite,
              ),
            ),
            if (_blocked)
              Container(
                width: double.infinity,
                color: Colors.red.withValues(alpha: 0.8),
                padding: const EdgeInsets.all(6),
                child: const Text('OBSTACLE AHEAD - forward blocked',
                    textAlign: TextAlign.center, style: TextStyle(fontWeight: FontWeight.w600)),
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
    final ppm = radius / (rangeMm / 1000.0); // pixels per metre
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
    // 1 m grid
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
    // trail
    if (trail.length > 1) {
      final path = Path()..moveTo(toScreen(trail.first.dx, trail.first.dy).dx, toScreen(trail.first.dx, trail.first.dy).dy);
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
    // robot arrow
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
    // scale label
    final tp = TextPainter(
      text: const TextSpan(text: 'grid: 1 m', style: TextStyle(color: Colors.white38, fontSize: 10)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, const Offset(8, 8));
  }

  @override
  bool shouldRepaint(MapPainter old) => true;
}
