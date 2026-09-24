import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'lidar_client.dart';

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

class _LidarScreenState extends State<LidarScreen> {
  final client = LidarClient();
  final List<StreamSubscription> _subs = [];
  LidarScan? scan;
  Map<String, dynamic>? robotStatus;
  String link = 'Starting...';
  double rangeMm = 6000;
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
    }));
    _subs.add(client.status.listen((s) => setState(() => robotStatus = s)));
    _subs.add(client.linkState.listen((s) => setState(() => link = s)));
    _tick = Timer.periodic(const Duration(milliseconds: 500), (_) => setState(() {}));
    client.start();
  }

  @override
  void dispose() {
    for (final s in _subs) {
      s.cancel();
    }
    _tick?.cancel();
    client.dispose();
    super.dispose();
  }

  bool get stale => scan == null || DateTime.now().difference(scan!.received) > const Duration(seconds: 1);

  Future<void> _enterIp() async {
    final ctrl = TextEditingController(text: client.address ?? '');
    final ip = await showDialog<String>(
      context: context,
      builder: (c) => AlertDialog(
        title: const Text('Lidar bridge address'),
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
    if (ip != null) client.start(manualIp: ip);
  }

  @override
  Widget build(BuildContext context) {
    final rate = _recent.length / 2.0;
    final st = robotStatus;
    final statusText = [
      stale ? 'NO DATA' : '${rate.toStringAsFixed(1)} scans/s',
      if (scan != null) '${scan!.points.length} pts',
      'dropped ${client.droppedScans}',
      if (st != null) 'robot ${(st['hz'] as num).toStringAsFixed(1)} Hz',
      if (st != null) 'Wi-Fi ${st['rssi']} dBm',
    ].join('  |  ');

    return Scaffold(
      appBar: AppBar(
        title: const Text('TankBot lidar'),
        actions: [
          IconButton(icon: const Icon(Icons.wifi_find), tooltip: 'Set address', onPressed: _enterIp),
          IconButton(icon: const Icon(Icons.refresh), tooltip: 'Reconnect', onPressed: () => client.start()),
        ],
      ),
      body: SafeArea(
        child: Column(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 4, 12, 0),
              child: Text(link, style: Theme.of(context).textTheme.bodySmall),
            ),
            Padding(
              padding: const EdgeInsets.all(8),
              child: Text(statusText,
                  style: TextStyle(color: stale ? Colors.redAccent : Colors.tealAccent, fontWeight: FontWeight.w500)),
            ),
            Expanded(
              child: CustomPaint(
                painter: RadarPainter(scan: scan, rangeMm: rangeMm, stale: stale),
                size: Size.infinite,
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12),
              child: Row(
                children: [
                  const Text('Range'),
                  Expanded(
                    child: Slider(
                      value: rangeMm, min: 1000, max: 12000, divisions: 22,
                      label: '${(rangeMm / 1000).toStringAsFixed(1)} m',
                      onChanged: (v) => setState(() => rangeMm = v),
                    ),
                  ),
                  Text('${(rangeMm / 1000).toStringAsFixed(1)} m'),
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
    final ring = Paint()..color = Colors.white24..style = PaintingStyle.stroke..strokeWidth = 1;
    final step = rangeMm <= 3000 ? 500.0 : 1000.0;
    for (var d = step; d <= rangeMm + 1; d += step) {
      final r = d / rangeMm * radius;
      canvas.drawCircle(c, r, ring);
      final tp = TextPainter(
        text: TextSpan(text: '${(d / 1000).toStringAsFixed(d % 1000 == 0 ? 0 : 1)} m',
            style: const TextStyle(color: Colors.white38, fontSize: 10)),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, c + Offset(3, -r - tp.height));
    }
    canvas.drawLine(c + Offset(0, -radius), c + Offset(0, radius), ring);
    canvas.drawLine(c + Offset(-radius, 0), c + Offset(radius, 0), ring);

    // Robot marker: a small wedge pointing to the front (up)
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
