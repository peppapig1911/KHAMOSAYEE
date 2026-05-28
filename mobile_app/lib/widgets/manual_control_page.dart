import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_debouncer/flutter_debouncer.dart';

import '../ble/ble.dart';

class ManualControlPage extends StatefulWidget {
  const ManualControlPage({super.key, required this.bleClient});

  final WindBleClient bleClient;

  @override
  State<ManualControlPage> createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  double _frontWheelAngle = 50.0;
  double _sailOpening = 50.0;

  final Debouncer _frontWheeldebouncer = Debouncer();
  final Debouncer _sailOpeningDebouncer = Debouncer();

  @override
  void initState() {
    super.initState();
    unawaited(_setControlModeSafely(BleControlMode.manual));
    _onFrontWheelAngleChanged(_frontWheelAngle);
    _onSailOpeningChanged(_sailOpening);
  }

  @override
  void dispose() {
    unawaited(_setControlModeSafely(BleControlMode.automatic));
    _frontWheeldebouncer.cancel();
    _sailOpeningDebouncer.cancel();
    super.dispose();
  }

  void _onFrontWheelAngleChanged(double value) {
    setState(() {
      _frontWheelAngle = value;
    });
    _frontWheeldebouncer.debounce(
      duration: Duration(milliseconds: 300),
      onDebounce: () {
        unawaited(_setFrontWheelOffsetSafely(value));
      },
    );
  }

  void _onSailOpeningChanged(double value) {
    setState(() {
      _sailOpening = value;
    });
    _sailOpeningDebouncer.debounce(
      duration: Duration(milliseconds: 300),
      onDebounce: () {
        unawaited(_setSailOpeningSafely(value));
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF4F7FA),
      appBar: AppBar(
        backgroundColor: const Color(0xFF5ECCC0),
        elevation: 0,
        title: const Text('Manual Control'),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 8),
              Text(
                'Direct control mode',
                style: Theme.of(
                  context,
                ).textTheme.headlineSmall?.copyWith(fontWeight: FontWeight.w700),
              ),
              const SizedBox(height: 8),
              Text(
                'Adjust the front wheel and sail opening manually.',
                style: Theme.of(context).textTheme.bodyMedium?.copyWith(color: Colors.grey[700]),
              ),
              const SizedBox(height: 24),
              _ControlCard(
                title: 'Front wheel',
                valueLabel: '${_frontWheelAngle.toStringAsFixed(0)}°',
                child: Slider(
                  value: _frontWheelAngle,
                  min: 0,
                  max: 100,
                  divisions: 100,
                  label: '${_frontWheelAngle.toStringAsFixed(0)}°',
                  onChanged: _onFrontWheelAngleChanged,
                ),
              ),
              const SizedBox(height: 16),
              _ControlCard(
                title: 'Sail opening',
                valueLabel: '${_sailOpening.toStringAsFixed(0)}%',
                child: Slider(
                  value: _sailOpening,
                  min: 0,
                  max: 100,
                  divisions: 100,
                  label: '${_sailOpening.toStringAsFixed(0)}%',
                  onChanged: _onSailOpeningChanged,
                ),
              ),
              const SizedBox(height: 24),
              Container(
                width: double.infinity,
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(16),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withValues(alpha: 0.06),
                      blurRadius: 20,
                      offset: const Offset(0, 8),
                    ),
                  ],
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text(
                      'Current setpoints',
                      style: TextStyle(fontSize: 14, fontWeight: FontWeight.w700),
                    ),
                    const SizedBox(height: 8),
                    Text('Front wheel: ${_frontWheelAngle.toStringAsFixed(0)}%'),
                    Text('Sail opening: ${_sailOpening.toStringAsFixed(0)}%'),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<void> _setControlModeSafely(BleControlMode mode) async {
    try {
      await widget.bleClient.setControlMode(mode);
    } catch (_) {
      // Manual mode falls back to the visible UI state if the BLE write fails.
    }
  }

  Future<void> _setFrontWheelOffsetSafely(double value) async {
    try {
      await widget.bleClient.setFrontWheelOffset(value);
    } catch (_) {
      // Ignore transient BLE errors while dragging the slider.
    }
  }

  Future<void> _setSailOpeningSafely(double value) async {
    try {
      await widget.bleClient.setSailOpeningPercent(value);
    } catch (_) {
      // Ignore transient BLE errors while dragging the slider.
    }
  }
}

class _ControlCard extends StatelessWidget {
  const _ControlCard({required this.title, required this.valueLabel, required this.child});

  final String title;
  final String valueLabel;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.06),
            blurRadius: 20,
            offset: const Offset(0, 8),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(title, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w700)),
              Text(valueLabel, style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
            ],
          ),
          const SizedBox(height: 12),
          child,
        ],
      ),
    );
  }
}
