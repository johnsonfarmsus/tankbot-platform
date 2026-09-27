// First-launch role chooser, and the Controller role (the app as a remote for a brain).
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'app_settings.dart';

class RoleChooser extends StatelessWidget {
  const RoleChooser({super.key, required this.onChosen});
  final void Function(AppRole role) onChosen;

  @override
  Widget build(BuildContext context) {
    Widget card(AppRole role, IconData icon, String title, String body) => Card(
          child: InkWell(
            onTap: () => onChosen(role),
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Row(children: [
                Icon(icon, size: 40, color: Colors.tealAccent),
                const SizedBox(width: 16),
                Expanded(
                  child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                    Text(title, style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
                    const SizedBox(height: 4),
                    Text(body, style: const TextStyle(color: Colors.white70)),
                  ]),
                ),
              ]),
            ),
          ),
        );
    return Scaffold(
      appBar: AppBar(title: const Text('TankBot: what is this device doing?')),
      body: ListView(padding: const EdgeInsets.all(16), children: [
        card(AppRole.mounted, Icons.smart_toy, 'Mounted brain',
            'This phone rides on the robot. Camera tracking, the status face, and it hosts the controls.'),
        card(AppRole.brain, Icons.psychology, 'Brain in hand',
            'This device is the brain but stays off the robot: lidar-only tracking, mapping and tap-to-go still work.'),
        card(AppRole.controller, Icons.sports_esports, 'Controller',
            'Connect to a brain on this Wi-Fi and drive, map and navigate from here.'),
        const Padding(
          padding: EdgeInsets.only(top: 12),
          child: Text('You can change this later from the menu.', style: TextStyle(color: Colors.white54)),
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
      ..loadRequest(Uri.parse(u));
    setState(() {
      _wv = c;
      _error = '';
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
      body: WebViewWidget(controller: wv),
    );
  }
}
