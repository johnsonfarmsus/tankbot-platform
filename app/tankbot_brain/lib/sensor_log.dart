// Sensor log: timestamped CSV of the robot's tracked pose plus GPS fixes and compass readings,
// for measuring how useful those phone sensors are before relying on them.
// Documents/logs/log_YYYYMMDD_HHMMSS.csv; one row per reading: appMs,kind,fields...
import 'dart:io';
import 'package:flutter/services.dart';
import 'pose_client.dart' show appClockMs;

class SensorLog {
  static const _native = MethodChannel('tankbot/arkit');
  IOSink? _out;
  String? name;
  int lines = 0;

  bool get recording => _out != null;

  Future<String?> start() async {
    try {
      final p = await _native.invokeMethod<String>('documentsDir');
      if (p == null) return null;
      final d = Directory('$p/logs');
      await d.create(recursive: true);
      final now = DateTime.now();
      String two(int v) => v.toString().padLeft(2, '0');
      name = 'log_${now.year}${two(now.month)}${two(now.day)}_${two(now.hour)}${two(now.minute)}${two(now.second)}.csv';
      _out = File('${d.path}/$name').openWrite();
      lines = 0;
      _out!.writeln('# appMs,kind,... | pose: x,y,heading,loc,source | '
          'gps: t,lat,lon,hAcc,alt,vAcc,speed,course,x,y,heading | heading: t,mag,true,acc,bx,by,bz,x,y,heading | auth: status,precise');
      return name;
    } catch (_) {
      _out = null;
      return null;
    }
  }

  void write(String kind, List<Object?> fields) {
    final o = _out;
    if (o == null) return;
    o.writeln([
      appClockMs().toStringAsFixed(0),
      kind,
      for (final f in fields) f is double ? f.toStringAsFixed(7) : (f ?? ''),
    ].join(','));
    lines++;
  }

  Future<void> stop() async {
    final o = _out;
    _out = null;
    if (o != null) {
      await o.flush();
      await o.close();
    }
  }
}
