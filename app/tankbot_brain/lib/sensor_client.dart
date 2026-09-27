// Client for the robot's sensor feed (UDP 5603): capability announce + live readings from the ESP32.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'pose_client.dart' show appClockMs;

class SensorClient {
  SensorClient({this.host = 'tankbot.local', this.port = 5603});
  final String host;
  final int port;

  InternetAddress? _addr;
  RawDatagramSocket? _sock;
  Timer? _timer;
  Map<String, dynamic>? caps; // what the robot says it has
  Map<String, dynamic>? latest; // last readings
  double lastMs = -1e9;
  final _ctrl = StreamController<Map<String, dynamic>>.broadcast();
  Stream<Map<String, dynamic>> get readings => _ctrl.stream;

  bool get fresh => appClockMs() - lastMs < 1000;
  String get block => fresh ? (latest?['block'] as String? ?? 'none') : 'none';

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
        _handle(d!.data);
      }
    });
    _subscribe();
    _timer = Timer.periodic(const Duration(seconds: 2), (_) => _subscribe());
  }

  void _subscribe() {
    final a = _addr, s = _sock;
    if (a == null || s == null) return;
    s.send(ascii.encode('TSSUB'), a, port);
  }

  void _handle(Uint8List data) {
    if (data.length < 6) return;
    final tag = String.fromCharCodes(data.sublist(0, 5));
    try {
      if (tag == 'TCAP1') {
        caps = jsonDecode(utf8.decode(data.sublist(5))) as Map<String, dynamic>;
      } else if (tag.startsWith('TSN1')) {
        latest = jsonDecode(utf8.decode(data.sublist(4))) as Map<String, dynamic>;
        lastMs = appClockMs();
        _ctrl.add(latest!);
      }
    } catch (_) {}
  }

  void stop() {
    _timer?.cancel();
    _timer = null;
    _sock?.close();
    _sock = null;
  }

  void dispose() {
    stop();
    _ctrl.close();
  }
}
