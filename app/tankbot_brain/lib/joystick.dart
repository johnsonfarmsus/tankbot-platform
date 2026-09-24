import 'dart:math' as math;
import 'package:flutter/material.dart';

/// Thumb joystick. Calls onChanged(forward, turn) in -1..1 (turn + = right),
/// and onReleased when the thumb lifts.
class Joystick extends StatefulWidget {
  const Joystick({super.key, required this.onChanged, required this.onReleased, this.size = 170});
  final void Function(double forward, double turn) onChanged;
  final VoidCallback onReleased;
  final double size;
  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset knob = Offset.zero; // in -1..1 units
  bool active = false;

  void _update(Offset local) {
    final r = widget.size / 2;
    var v = (local - Offset(r, r)) / r;
    if (v.distance > 1) v = v / v.distance;
    setState(() { knob = v; active = true; });
    double dz(double x) => x.abs() < 0.08 ? 0 : x;
    widget.onChanged(dz(-v.dy), dz(v.dx));
  }

  void _end() {
    setState(() { knob = Offset.zero; active = false; });
    widget.onReleased();
  }

  @override
  Widget build(BuildContext context) {
    final r = widget.size / 2;
    final scheme = Theme.of(context).colorScheme;
    return GestureDetector(
      onPanStart: (d) => _update(d.localPosition),
      onPanUpdate: (d) => _update(d.localPosition),
      onPanEnd: (_) => _end(),
      onPanCancel: _end,
      child: SizedBox(
        width: widget.size,
        height: widget.size,
        child: Stack(children: [
          Container(
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: scheme.surfaceContainerHighest.withValues(alpha: 0.5),
              border: Border.all(color: active ? scheme.primary : Colors.white24, width: 2),
            ),
          ),
          Positioned(
            left: r + knob.dx * (r - 28) - 28,
            top: r + knob.dy * (r - 28) - 28,
            child: Container(
              width: 56, height: 56,
              decoration: BoxDecoration(shape: BoxShape.circle, color: active ? scheme.primary : Colors.white38),
              child: Transform.rotate(angle: math.pi, child: const SizedBox()),
            ),
          ),
        ]),
      ),
    );
  }
}
