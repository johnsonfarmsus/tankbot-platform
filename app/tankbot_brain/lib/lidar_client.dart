// Client for the TankBot lidar bridge. See docs/protocol.md.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

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
  LidarScan(this.rotation, this.robotMs, this.points, this.received);
}

class _Partial {
  final int chunkCount;
  final Map<int, List<LidarPoint>> chunks = {};
  _Partial(this.chunkCount);
}

class LidarClient {
  LidarClient({this.host = 'tanklidar.local', this.port = 5601});

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
  }

  void _onEvent(RawSocketEvent e) {
    if (e != RawSocketEvent.read) return;
    Datagram? d;
    while ((d = _socket?.receive()) != null) {
      _handle(d!.data);
    }
  }

  void _handle(Uint8List data) {
    if (data.length < 4) return;
    lastPacket = DateTime.now();
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
      if (_lastRotation != null && rot > _lastRotation! + 1) droppedScans += rot - _lastRotation! - 1;
      _lastRotation = rot;
      completeScans++;
      _scanCtrl.add(LidarScan(rot, robotMs, all, DateTime.now()));
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
