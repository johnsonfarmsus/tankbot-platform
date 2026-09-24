import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'lidar_client.dart';
import 'motion_client.dart';
import 'joystick.dart';

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

class LidarScreen extends StatefulWidget {
  const LidarScreen({super.key});
  @override
  State<LidarScreen> createState() => _LidarScreenState();
}

class _LidarScreenState extends State<LidarScreen> with WidgetsBindingObserver {
  final client = LidarClient();
  final motion = MotionClient();
  final List<StreamSubscription> _subs = [];
  LidarScan? scan;
  Map<String, dynamic>? robotStatus;
  Map<String, dynamic>? motionStatus;
  String link = 'Starting...';
  double rangeMm = 6000;
  double maxSpeed = 0.6;
  bool obstacleStop = false; // off until the lidar is mounted on the robot
  static const double stopDistMm = 300, selfMaskMm = 150, frontHalfAngle = 25;
  double _wantF = 0, _wantT = 0;
  bool _blocked = false;
  final List<DateTime> _recent = [];
  Timer? _tick;

  @override
  void initState() {
    super.initState();
    _subs.add(client.scans.listen((s) {
      final now = DateTime.now();
      _recent.add(now);
      _recent.removeWhere((t) => now.difference(t) > const Duration(seconds: 2));
      setState(() => scan = s);
      if (motion.driving) _applyDrive(); // re-check obstacles on every new scan
    }));
    _subs.add(client.status.listen((s) => setState(() => robotStatus = s)));
    _subs.add(client.linkState.listen((s) => setState(() => link = s)));
    _subs.add(motion.status.listen((m) => setState(() => motionStatus = m)));
    _tick = Timer.periodic(const Duration(milliseconds: 500), (_) => setState(() {}));
    WidgetsBinding.instance.addObserver(this);
    client.start();
    motion.start();
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
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state != AppLifecycleState.resumed) motion.release(); // never drive from the background
  }

  bool get stale => scan == null || DateTime.now().difference(scan!.received) > const Duration(seconds: 1);

  /// Closest lidar point in the front cone, ignoring anything closer than the self-mask.
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
    // With obstacle stop on, forward needs fresh lidar data showing clear space ahead.
    _blocked = obstacleStop && f > 0 && (stale || (clear != null && clear < stopDistMm));
    if (_blocked) f = 0; // turning and reversing still allowed
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
              padding: const EdgeInsets.all(6),
              child: Text(statusText,
                  style: TextStyle(color: stale ? Colors.redAccent : Colors.tealAccent, fontWeight: FontWeight.w500)),
            ),
            Expanded(
              child: CustomPaint(
                painter: RadarPainter(scan: scan, rangeMm: rangeMm, stale: stale),
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
                        Text('Range ${(rangeMm / 1000).toStringAsFixed(1)} m', style: small),
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
