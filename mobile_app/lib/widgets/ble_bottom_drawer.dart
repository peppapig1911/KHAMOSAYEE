import 'package:flutter/material.dart';

import '../ble.dart';

class BleBottomDrawer extends StatelessWidget {
  const BleBottomDrawer({
    super.key,
    required this.isExpanded,
    required this.bleState,
    required this.windData,
    required this.bleError,
    required this.isLocating,
    required this.onToggleExpanded,
    required this.onReconnect,
  });

  final bool isExpanded;
  final BleConnectionState bleState;
  final WindGattData? windData;
  final String? bleError;
  final bool isLocating;
  final VoidCallback onToggleExpanded;
  final VoidCallback onReconnect;

  String _stateText(BleConnectionState state) {
    switch (state) {
      case BleConnectionState.idle:
        return 'Idle';
      case BleConnectionState.scanning:
        return 'Scanning';
      case BleConnectionState.connecting:
        return 'Connecting';
      case BleConnectionState.connected:
        return 'Connected';
      case BleConnectionState.disconnected:
        return 'Disconnected';
      case BleConnectionState.error:
        return 'Error';
    }
  }

  Color _stateColor(BleConnectionState state) {
    switch (state) {
      case BleConnectionState.connected:
        return Colors.greenAccent;
      case BleConnectionState.scanning:
      case BleConnectionState.connecting:
        return Colors.amberAccent;
      case BleConnectionState.error:
        return Colors.redAccent;
      case BleConnectionState.disconnected:
        return Colors.orangeAccent;
      case BleConnectionState.idle:
        return Colors.white70;
    }
  }

  Widget _buildMetricChip({required IconData icon, required String label, required Color color}) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.12),
        borderRadius: BorderRadius.circular(999),
        border: Border.all(color: color.withValues(alpha: 0.28)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 16, color: color),
          const SizedBox(width: 6),
          Text(
            label,
            style: TextStyle(color: color, fontWeight: FontWeight.w700, fontSize: 12),
          ),
        ],
      ),
    );
  }

  Widget _buildMetricRow({
    required IconData icon,
    required String label,
    required String value,
    required Color iconColor,
  }) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Row(
        children: [
          Container(
            width: 38,
            height: 38,
            decoration: BoxDecoration(
              color: Colors.white.withValues(alpha: 0.08),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(icon, color: iconColor, size: 20),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  label,
                  style: const TextStyle(
                    color: Colors.white70,
                    fontSize: 12,
                    fontWeight: FontWeight.w500,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  value,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 15,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final backgroundGradient = LinearGradient(
      begin: Alignment.topLeft,
      end: Alignment.bottomRight,
      colors: [
        const Color(0xFF111827),
        const Color(0xFF0B1220),
        Colors.black.withValues(alpha: 0.96),
      ],
    );

    Widget drawerContent() {
      final header = Row(
        children: [
          Container(
            width: 38,
            height: 38,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.blueAccent, Colors.cyanAccent.withValues(alpha: 0.85)],
              ),
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Icon(Icons.bluetooth, color: Colors.white),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Bluetooth Link',
                  style: TextStyle(
                    color: Colors.white.withValues(alpha: 0.92),
                    fontSize: 16,
                    fontWeight: FontWeight.w800,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  isExpanded ? 'Tap to collapse' : 'Tap to expand',
                  style: const TextStyle(color: Colors.white54, fontSize: 12),
                ),
              ],
            ),
          ),
          IconButton(
            onPressed: onToggleExpanded,
            tooltip: isExpanded ? 'Hide drawer' : 'Show drawer',
            icon: AnimatedRotation(
              turns: isExpanded ? 0.5 : 0.0,
              duration: const Duration(milliseconds: 220),
              child: const Icon(Icons.keyboard_arrow_up_rounded, color: Colors.white),
            ),
          ),
        ],
      );

      final handle = Center(
        child: Container(
          width: 44,
          height: 4,
          decoration: BoxDecoration(color: Colors.white24, borderRadius: BorderRadius.circular(99)),
        ),
      );

      if (!isExpanded) {
        return Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            handle,
            const SizedBox(height: 10),
            header,
            const SizedBox(height: 6),
            Row(
              children: [
                Icon(Icons.radio_button_checked_rounded, color: _stateColor(bleState), size: 14),
                const SizedBox(width: 8),
                Text(
                  'Status: ${_stateText(bleState)}',
                  style: const TextStyle(color: Colors.white70, fontWeight: FontWeight.w600),
                ),
              ],
            ),
          ],
        );
      }

      return SingleChildScrollView(
        physics: const BouncingScrollPhysics(),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            handle,
            const SizedBox(height: 10),
            header,
            const SizedBox(height: 10),
            Row(
              children: [
                _buildMetricChip(
                  icon: Icons.link_rounded,
                  label: _stateText(bleState),
                  color: _stateColor(bleState),
                ),
                const SizedBox(width: 8),
                _buildMetricChip(
                  icon: isLocating ? Icons.my_location : Icons.gps_fixed,
                  label: isLocating ? 'Locating' : 'GPS Ready',
                  color: isLocating ? Colors.amberAccent : Colors.lightGreenAccent,
                ),
                const Spacer(),
                IconButton.filledTonal(
                  onPressed: onReconnect,
                  tooltip: 'Scan & reconnect',
                  icon: const Icon(Icons.refresh_rounded),
                ),
              ],
            ),
            const SizedBox(height: 12),
            _buildMetricRow(
              icon: Icons.air_rounded,
              label: 'Wind speed',
              value: '${windData?.windSpeedMs?.toStringAsFixed(2) ?? '--'} m/s',
              iconColor: Colors.cyanAccent,
            ),
            _buildMetricRow(
              icon: Icons.explore_rounded,
              label: 'Wind direction',
              value: '${windData?.windDirectionDeg?.toStringAsFixed(2) ?? '--'} deg',
              iconColor: Colors.lightBlueAccent,
            ),
            _buildMetricRow(
              icon: Icons.battery_full_rounded,
              label: 'Battery',
              value: '${windData?.batteryPercent?.toString() ?? '--'} %',
              iconColor: Colors.lightGreenAccent,
            ),
            if (bleError != null) ...[
              const SizedBox(height: 2),
              Container(
                width: double.infinity,
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.redAccent.withValues(alpha: 0.12),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.redAccent.withValues(alpha: 0.25)),
                ),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Icon(Icons.warning_rounded, color: Colors.redAccent, size: 20),
                    const SizedBox(width: 10),
                    Expanded(
                      child: Text(bleError!, style: const TextStyle(color: Colors.white70)),
                    ),
                  ],
                ),
              ),
            ],
          ],
        ),
      );
    }

    return SafeArea(
      top: false,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 260),
        curve: Curves.easeInOutCubic,
        height: isExpanded ? 230 : 76,
        width: double.infinity,
        decoration: BoxDecoration(
          gradient: backgroundGradient,
          borderRadius: const BorderRadius.vertical(top: Radius.circular(24)),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withValues(alpha: 0.32),
              blurRadius: 18,
              offset: const Offset(0, -6),
            ),
          ],
          border: Border(top: BorderSide(color: Colors.white.withValues(alpha: 0.08))),
        ),
        child: Material(
          color: Colors.transparent,
          child: InkWell(
            onTap: onToggleExpanded,
            child: Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 14),
              child: drawerContent(),
            ),
          ),
        ),
      ),
    );
  }
}
