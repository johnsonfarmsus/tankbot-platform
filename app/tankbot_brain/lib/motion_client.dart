// Client for the TankBot motion channel (UDP 5602). See docs/protocol.md.
// Sends TMC1 at 20 Hz while driving; the robot stops itself if these stop for 300 ms.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

class MotionClient {
  MotionClient({this.host = 'tankbot.local', this.port = 5602});
  final String host;
  final int port;

  InternetAddress? _addr;
  RawDatagramSocket? _sock;
  Timer? _timer;
  int _idleTicks = 0;

  double forward = 0, turn = 0;
  bool driving = false;

  final _statusCtrl = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get status => _statusCtrl.stream;
  bool get connected => _sock != null && _addr != null;
  String? get address => _addr?.address;

  Future<void> start({String? manualIp}) async {
    stop();
    try {
      _addr = (manualIp != null && manualIp.trim().isNotEmpty)
          ? InternetAddress(manualIp.trim())
          : (await InternetAddress.lookup(host, type: InternetAddressType.IPv4)
                  .timeout(const Duration(seconds: 6)))
              .first;
    } catch (_) {
      _addr = null;
      return;
    }
    _sock = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    _sock!.listen((e) {
      if (e != RawSocketEvent.read) return;
      Datagram? d;
      while ((d = _sock?.receive()) != null) {
        final data = d!.data;
        if (data.length > 4 && String.fromCharCodes(data.sublist(0, 4)) == 'TMH1') {
          try {
            _statusCtrl.add(jsonDecode(utf8.decode(data.sublist(4))) as Map<String, dynamic>);
          } catch (_) {}
        }
      }
    });
    _timer = Timer.periodic(const Duration(milliseconds: 50), (_) => _tick());
  }

  void drive(double f, double t) {
    forward = f.clamp(-1.0, 1.0);
    turn = t.clamp(-1.0, 1.0);
    driving = true;
  }

  void release() {
    driving = false;
    forward = 0;
    turn = 0;
    _send(ascii.encode('TMS1'));
  }

  void _tick() {
    if (driving) {
      final b = ByteData(12);
      b.setUint8(0, 0x54); b.setUint8(1, 0x4D); b.setUint8(2, 0x43); b.setUint8(3, 0x31); // 'TMC1'
      b.setFloat32(4, forward, Endian.little);
      b.setFloat32(8, turn, Endian.little);
      _send(b.buffer.asUint8List());
      _idleTicks = 0;
    } else if (++_idleTicks >= 20) {
      _idleTicks = 0;
      _send(ascii.encode('TMP1')); // ping: keeps motion status flowing, never moves the robot
    }
  }

  void _send(List<int> bytes) {
    final a = _addr, s = _sock;
    if (a == null || s == null) return;
    s.send(bytes, a, port);
  }

  void stop() {
    if (driving) release();
    _timer?.cancel();
    _timer = null;
    _sock?.close();
    _sock = null;
  }

  void dispose() {
    stop();
    _statusCtrl.close();
  }
}
