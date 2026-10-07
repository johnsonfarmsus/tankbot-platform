import 'dart:convert';
import 'dart:io';
// First-launch role chooser, and the Controller role (the app as a remote for a brain).
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'app_settings.dart';

/// Shown every time the app opens: what is this device doing today? The last choice is highlighted,
/// and it checks whether the robot already has a brain, so two brains never fight over one robot.
class RoleChooser extends StatefulWidget {
  const RoleChooser({super.key, required this.onChosen, this.lastRole, this.lastRobotIp = ''});
  final void Function(AppRole role, {String? brainUrl}) onChosen;
  final AppRole? lastRole;
  final String lastRobotIp;
  @override
  State<RoleChooser> createState() => _RoleChooserState();
}

class _RoleChooserState extends State<RoleChooser> {
  bool _checking = true;
  String? _robotIp, _brainUrl, _myIp;

  @override
  void initState() {
    super.initState();
    _check();
  }

  Future<void> _check() async {
    setState(() => _checking = true);
    String? my, robot, brain;
    try {
      for (final ni in await NetworkInterface.list(type: InternetAddressType.IPv4)) {
        if (ni.name.startsWith('en')) {
          my = ni.addresses.first.address;
          break;
        }
      }
    } catch (_) {}
    try {
      final found = await InternetAddress.lookup('tankbot.local', type: InternetAddressType.IPv4)
          .timeout(const Duration(seconds: 3));
      if (found.isNotEmpty) robot = found.first.address;
    } catch (_) {}
    if (robot == null && widget.lastRobotIp.isNotEmpty) robot = widget.lastRobotIp;
    if (robot != null) {
      final c = HttpClient()..connectionTimeout = const Duration(seconds: 3);
      try {
        final req = await c.getUrl(Uri.parse('http://$robot/api/brain')).timeout(const Duration(seconds: 3));
        final res = await req.close().timeout(const Duration(seconds: 3));
        final j = jsonDecode(await res.transform(utf8.decoder).join());
        if (j is Map && j['url'] is String) brain = j['url'] as String;
      } catch (_) {
        robot = null; // didn't answer: not reachable right now
      } finally {
        c.close();
      }
    }
    if (!mounted) return;
    setState(() {
      _checking = false;
      _myIp = my;
      _robotIp = robot;
      _brainUrl = brain;
    });
  }

  /// A brain is running on some other device.
  bool get _otherBrain => _brainUrl != null && (_myIp == null || !_brainUrl!.contains('//$_myIp:'));

  Future<void> _pick(AppRole role) async {
    if ((role == AppRole.mounted || role == AppRole.brain) && _otherBrain) {
      final choice = await showDialog<String>(
        context: context,
        builder: (ctx) => AlertDialog(
          title: const Text('This robot already has a brain'),
          content: Text('A brain is running at ${Uri.tryParse(_brainUrl!)?.host ?? _brainUrl}. Two brains on one robot '
              'fight over it. Use this device as a Controller for that brain instead?'),
          actions: [
            TextButton(onPressed: () => Navigator.pop(ctx, 'anyway'), child: const Text('Start a brain anyway')),
            FilledButton(onPressed: () => Navigator.pop(ctx, 'controller'), child: const Text('Use as Controller')),
          ],
        ),
      );
      if (!mounted || choice == null) return;
      if (choice == 'controller') {
        widget.onChosen(AppRole.controller, brainUrl: _brainUrl);
        return;
      }
    }
    widget.onChosen(role, brainUrl: role == AppRole.controller ? _brainUrl : null);
  }

  @override
  Widget build(BuildContext context) {
    final last = widget.lastRole;
    Widget card(AppRole role, IconData icon, String title, String body) {
      final isLast = role == last;
      return Card(
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
          side: BorderSide(color: isLast ? Colors.tealAccent : Colors.transparent, width: 2),
        ),
        child: InkWell(
          borderRadius: BorderRadius.circular(12),
          onTap: () => _pick(role),
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Row(children: [
              Icon(icon, size: 40, color: Colors.tealAccent),
              const SizedBox(width: 16),
              Expanded(
                child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                  Row(children: [
                    Flexible(child: Text(title, style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600))),
                    if (isLast) ...[
                      const SizedBox(width: 8),
                      const Text('last time', style: TextStyle(fontSize: 12, color: Colors.tealAccent)),
                    ],
                  ]),
                  const SizedBox(height: 4),
                  Text(body, style: const TextStyle(color: Colors.white70)),
                ]),
              ),
            ]),
          ),
        ),
      );
    }

    final brainHost = _brainUrl == null ? null : (Uri.tryParse(_brainUrl!)?.host ?? _brainUrl);
    final status = _checking
        ? 'Looking for the robot...'
        : _robotIp == null
            ? 'Robot not found on this Wi-Fi (is it on?)'
            : _brainUrl == null
                ? 'Robot found ($_robotIp) - no brain running'
                : _otherBrain
                    ? 'Robot found ($_robotIp) - brain running on $brainHost'
                    : 'Robot found ($_robotIp)';
    return Scaffold(
      appBar: AppBar(title: const Text('TankBot: what is this device doing?')),
      body: ListView(padding: const EdgeInsets.all(16), children: [
        Row(children: [
          Icon(_robotIp == null ? Icons.wifi_off : Icons.smart_toy,
              color: _checking ? Colors.white54 : (_robotIp == null ? Colors.amberAccent : Colors.tealAccent)),
          const SizedBox(width: 8),
          Expanded(child: Text(status, style: const TextStyle(color: Colors.white70))),
          IconButton(onPressed: _checking ? null : _check, icon: const Icon(Icons.refresh), tooltip: 'Look again'),
        ]),
        const SizedBox(height: 8),
        card(AppRole.mounted, Icons.smart_toy, 'Mounted brain',
            'This phone rides on the robot. Camera tracking, the status face, and it hosts the controls.'),
        card(AppRole.brain, Icons.psychology, 'Brain in hand',
            'This device is the brain but stays off the robot: lidar-only tracking, mapping and tap-to-go still work.'),
        card(AppRole.controller, Icons.sports_esports, 'Controller',
            brainHost != null && _otherBrain
                ? 'Drive, map and navigate with the brain running on $brainHost.'
                : 'Connect to a brain on this Wi-Fi and drive, map and navigate from here.'),
        if (_otherBrain && !_checking)
          const Padding(
            padding: EdgeInsets.only(top: 12),
            child: Text('Only one brain per robot: this robot already has one, so this device is best as a Controller.',
                style: TextStyle(color: Colors.amberAccent)),
          ),
      ]),
    );
  }
}

class ControllerScreen extends StatefulWidget {
  const ControllerScreen({super.key, required this.settings, required this.onChangeRole});
  final AppSettings settings;
  final VoidCallback onChangeRole;
  @override
  State<ControllerScreen> createState() => _ControllerScreenState();
}

class _ControllerScreenState extends State<ControllerScreen> {
  static const _native = MethodChannel('tankbot/arkit');
  WebViewController? _wv;
  late final TextEditingController _url = TextEditingController(text: widget.settings.brainUrl);
  String _error = '';
  bool _loading = false;
  String _loadError = '';

  @override
  void initState() {
    super.initState();
    _native.invokeMethod('keepAwake', true).catchError((_) {});
    if (widget.settings.brainUrl.isNotEmpty) _connect(widget.settings.brainUrl);
  }

  void _connect(String raw) {
    var u = raw.trim();
    if (u.isEmpty) return;
    if (!u.startsWith('http')) u = 'http://$u';
    final uri = Uri.tryParse(u);
    if (uri == null || uri.host.isEmpty) {
      setState(() => _error = 'That does not look like an address');
      return;
    }
    if (!uri.hasPort) u = '$u:8080';
    widget.settings.brainUrl = u;
    widget.settings.save();
    final c = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(const Color(0xFF101416))
      ..setNavigationDelegate(NavigationDelegate(
        onPageStarted: (_) => setState(() {
          _loading = true;
          _loadError = '';
        }),
        onPageFinished: (_) => setState(() => _loading = false),
        onWebResourceError: (e) => setState(() {
          _loading = false;
          if (e.isForMainFrame ?? true) _loadError = 'Could not load the brain page: ${e.description}';
        }),
      ))
      ..loadRequest(Uri.parse(u));
    setState(() {
      _wv = c;
      _error = '';
      _loading = true;
      _loadError = '';
    });
  }

  @override
  Widget build(BuildContext context) {
    final wv = _wv;
    if (wv == null) {
      return Scaffold(
        appBar: AppBar(
          title: const Text('Controller'),
          actions: [IconButton(icon: const Icon(Icons.swap_horiz), tooltip: 'Change role', onPressed: widget.onChangeRole)],
        ),
        body: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            const Text('Brain address', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            const SizedBox(height: 6),
            const Text('Shown on the brain phone\'s screen, e.g. 192.168.1.199:8080', style: TextStyle(color: Colors.white70)),
            const SizedBox(height: 12),
            TextField(
              controller: _url,
              keyboardType: TextInputType.url,
              autocorrect: false,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: '192.168.1.199:8080'),
              onSubmitted: _connect,
            ),
            if (_error.isNotEmpty) Padding(padding: const EdgeInsets.only(top: 8), child: Text(_error, style: const TextStyle(color: Colors.redAccent))),
            const SizedBox(height: 12),
            FilledButton(onPressed: () => _connect(_url.text), child: const Text('Connect')),
          ]),
        ),
      );
    }
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.settings.brainUrl.replaceFirst('http://', ''), style: const TextStyle(fontSize: 15)),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), tooltip: 'Reload', onPressed: () => wv.reload()),
          IconButton(icon: const Icon(Icons.wifi_find), tooltip: 'Change brain', onPressed: () => setState(() => _wv = null)),
          IconButton(icon: const Icon(Icons.swap_horiz), tooltip: 'Change role', onPressed: widget.onChangeRole),
        ],
      ),
      body: Stack(children: [
        WebViewWidget(controller: wv),
        if (_loading) const LinearProgressIndicator(),
        if (_loadError.isNotEmpty)
          Container(
            color: const Color(0xFF101416),
            padding: const EdgeInsets.all(20),
            alignment: Alignment.center,
            child: Column(mainAxisSize: MainAxisSize.min, children: [
              Text(_loadError, style: const TextStyle(color: Colors.redAccent), textAlign: TextAlign.center),
              const SizedBox(height: 12),
              const Text('Check that this phone is on the same Wi-Fi as the brain, and that the brain app is open.',
                  style: TextStyle(color: Colors.white70), textAlign: TextAlign.center),
              const SizedBox(height: 12),
              FilledButton(onPressed: () => wv.reload(), child: const Text('Try again')),
            ]),
          ),
      ]),
    );
  }
}
