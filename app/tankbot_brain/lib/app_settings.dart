// App role: mounted brain, brain in hand, or controller. Persisted in Documents/app_role.json.
import 'dart:convert';
import 'dart:io';
import 'package:flutter/services.dart';

enum AppRole { mounted, brain, controller }

class AppSettings {
  AppRole? role;
  String brainUrl = ''; // controller role: last brain address, e.g. http://192.168.1.199:8080
  String lastRobot = 'TankBot'; // brain roles: which robot's profile and maps to open at startup

  static const _native = MethodChannel('tankbot/arkit');

  static Future<File?> _file() async {
    try {
      final p = await _native.invokeMethod<String>('documentsDir');
      return p == null ? null : File('$p/app_role.json');
    } catch (_) {
      return null;
    }
  }

  static Future<AppSettings> load() async {
    final s = AppSettings();
    final f = await _file();
    if (f == null || !await f.exists()) return s;
    try {
      final j = jsonDecode(await f.readAsString()) as Map<String, dynamic>;
      final r = j['role'];
      s.role = AppRole.values.cast<AppRole?>().firstWhere((e) => e!.name == r, orElse: () => null);
      s.brainUrl = (j['brainUrl'] as String?) ?? '';
      s.lastRobot = (j['lastRobot'] as String?) ?? 'TankBot';
    } catch (_) {}
    return s;
  }

  Future<void> save() async {
    final f = await _file();
    if (f == null) return;
    try {
      await f.writeAsString(jsonEncode({'role': role?.name, 'brainUrl': brainUrl, 'lastRobot': lastRobot}), flush: true);
    } catch (_) {}
  }
}
