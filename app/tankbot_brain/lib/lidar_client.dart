// Client for the TankBot lidar bridge. See docs/protocol.md.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'pose_client.dart' show appClockMs;

class LidarPoint {
  final double angleDeg; // clockwise from the lidar's front
  final double distMm;
  final int quality;
  const LidarPoint(this.angleDeg, this.distMm, this.quality);
}

class LidarScan {
  final int rotation;
  final int robotMs;
  final List<LidarPoint> points;
  final DateTime received;
  /// Mid-rotation time on the app clock (ms), once clock sync has a fix.
  final double? appMs;
  LidarScan(this.rotation, this.robotMs, this.points, this.received, this.appMs);
}

class _Partial {
  final int chunkCount;
  final Map<int, List<LidarPoint>> chunks = {};
  _Partial(this.chunkCount);
}

class LidarClient {
  LidarClient({this.host = 'tankbot.local', this.port = 5601});

  final String host;
  final int port;

  InternetAddress? _address;
  RawDatagramSocket? _socket;
  Timer? _subTimer;
  final Map<int, _Partial> _partials = {};
  int? _lastRotation;

  int completeScans = 0;
  int droppedScans = 0;
  DateTime? lastPacket;

  // Clock sync (robot ms -> app ms)
  int _syncSeq = 0;
  final Map<int, double> _syncSent = {};
  final List<List<double>> _syncSamples = []; // [rtt, offset]
  double? robotOffsetMs; // robot_ms - app_ms
  double? syncRttMs;

  final _scanCtrl = StreamController<LidarScan>.broadcast();
  final _statusCtrl = StreamController<Map<String, dynamic>>.broadcast();
  final _stateCtrl = StreamController<String>.broadcast();

  Stream<LidarScan> get scans => _scanCtrl.stream;
  Stream<Map<String, dynamic>> get status => _statusCtrl.stream;
  Stream<String> get linkState => _stateCtrl.stream;
  String? get address => _address?.address;

  Future<void> start({String? manualIp}) async {
    await stop();
    completeScans = 0;
    droppedScans = 0;
    _lastRotation = null;
    _syncSamples.clear();
    robotOffsetMs = null;
    try {
      if (manualIp != null && manualIp.trim().isNotEmpty) {
        _address = InternetAddress(manualIp.trim());
      } else {
        _stateCtrl.add('Looking for $host...');
        final found = await InternetAddress.lookup(host, type: InternetAddressType.IPv4)
            .timeout(const Duration(seconds: 6));
        _address = found.first;
      }
    } catch (_) {
      _stateCtrl.add('Could not find $host - tap the Wi-Fi icon to enter its IP');
      return;
    }
    _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    _socket!.listen(_onEvent);
    _subscribe();
    _subTimer = Timer.periodic(const Duration(seconds: 1), (_) => _subscribe());
    _stateCtrl.add('Subscribed to ${_address!.address}');
  }

  void _subscribe() {
    final a = _address, s = _socket;
    if (a == null || s == null) return;
    s.send(ascii.encode('TLSUB'), a, port);
    // clock sync ping
    final seq = ++_syncSeq;
    final b = ByteData(9);
    for (var i = 0; i < 5; i++) {
      b.setUint8(i, 'TLSYN'.codeUnitAt(i));
    }
    b.setUint32(5, seq, Endian.little);
    _syncSent[seq] = appClockMs();
    _syncSent.removeWhere((k, _) => k < seq - 10);
    s.send(b.buffer.asUint8List(), a, port);
  }

  void _onEvent(RawSocketEvent e) {
    if (e != RawSocketEvent.read) return;
    Datagram? d;
    while ((d = _socket?.receive()) != null) {
      _handle(d!.data);
    }
  }

  void _handleSync(Uint8List data) {
    final now = appClockMs();
    final bd = ByteData.sublistView(data);
    final seq = bd.getUint32(5, Endian.little);
    final sent = _syncSent.remove(seq);
    if (sent == null) return;
    final robotMs = bd.getInt64(9, Endian.little) / 1000.0;
    final rtt = now - sent;
    _syncSamples.add([rtt, robotMs - (sent + now) / 2]);
    if (_syncSamples.length > 30) _syncSamples.removeAt(0);
    final best = _syncSamples.reduce((a, b) => a[0] <= b[0] ? a : b);
    syncRttMs = best[0];
    robotOffsetMs = best[1];
  }

  void _handle(Uint8List data) {
    if (data.length < 4) return;
    lastPacket = DateTime.now();
    if (data.length >= 17 && String.fromCharCodes(data.sublist(0, 5)) == 'TLSY1') {
      _handleSync(data);
      return;
    }
    final magic = String.fromCharCodes(data.sublist(0, 4));
    if (magic == 'TLH1') {
      try {
        _statusCtrl.add(jsonDecode(utf8.decode(data.sublist(4))) as Map<String, dynamic>);
      } catch (_) {}
      return;
    }
    if (magic != 'TLS1' || data.length < 16) return;
    final bd = ByteData.sublistView(data);
    final rot = bd.getUint32(4, Endian.little);
    final robotMs = bd.getUint32(8, Endian.little);
    final chunk = data[12];
    final count = data[13];
    final n = bd.getUint16(14, Endian.little);
    if (count == 0 || data.length < 16 + n * 5) return;
    final pts = List<LidarPoint>.generate(n, (i) {
      final o = 16 + i * 5;
      return LidarPoint(bd.getUint16(o, Endian.little) / 64.0,
          bd.getUint16(o + 2, Endian.little) / 4.0, data[o + 4]);
    });
    final p = _partials.putIfAbsent(rot, () => _Partial(count));
    p.chunks[chunk] = pts;
    if (p.chunks.length == p.chunkCount) {
      _partials.remove(rot);
      final all = <LidarPoint>[
        for (var k = 0; k < p.chunkCount; k++) ...?p.chunks[k],
      ];
      if (_lastRotation != null && rot < _lastRotation!) _lastRotation = null; // bridge restarted
      // Small gaps are real losses; big gaps mean we were paused/disconnected.
      if (_lastRotation != null) {
        final gap = rot - _lastRotation! - 1;
        if (gap > 0 && gap <= 20) droppedScans += gap;
      }
      _lastRotation = rot;
      completeScans++;
      final off = robotOffsetMs;
      // robotMs is the rotation start; a rotation takes ~100 ms, so use the middle.
      final appMs = off == null ? null : robotMs + 50 - off;
      _scanCtrl.add(LidarScan(rot, robotMs, all, DateTime.now(), appMs));
    }
    _partials.removeWhere((k, _) => k + 3 < rot);
  }

  Future<void> stop() async {
    _subTimer?.cancel();
    _subTimer = null;
    _socket?.close();
    _socket = null;
    _partials.clear();
  }

  void dispose() {
    stop();
    _scanCtrl.close();
    _statusCtrl.close();
    _stateCtrl.close();
  }
}
