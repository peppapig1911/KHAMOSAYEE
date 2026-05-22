import 'package:flutter/material.dart';
import '../ble/data.dart';

class WindDataCardsSection extends StatelessWidget {
  final WindGattData? windData;

  const WindDataCardsSection({super.key, required this.windData});

  @override
  Widget build(BuildContext context) {
    return Container(
      color: const Color(0xFF5ECCC0),
      padding: const EdgeInsets.fromLTRB(16, 16, 16, 24),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          WindSpeedCard(speed: windData?.windSpeedMs),
          const SizedBox(width: 12),
          WindDirectionCard(direction: windData?.windDirectionDeg),
          const SizedBox(width: 12),
          BatteryCard(percentage: windData?.batteryPercent),
        ],
      ),
    );
  }
}

class DataMetricCard extends StatelessWidget {
  final IconData icon;
  final Color iconColor;
  final String label;
  final String value;
  final String unit;
  final Color valueColor;

  const DataMetricCard({
    super.key,
    required this.icon,
    required this.iconColor,
    required this.label,
    required this.value,
    required this.unit,
    required this.valueColor,
  });

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: Container(
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(12),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.1),
              blurRadius: 8,
              offset: const Offset(0, 2),
            ),
          ],
        ),
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Icon(icon, color: iconColor, size: 20),
                const SizedBox(width: 8),
                Text(label, style: const TextStyle(color: Colors.grey, fontSize: 12)),
              ],
            ),
            const SizedBox(height: 12),
            Text(
              value,
              style: TextStyle(fontSize: 28, fontWeight: FontWeight.bold, color: valueColor),
            ),
            Text(unit, style: const TextStyle(color: Colors.grey, fontSize: 12)),
          ],
        ),
      ),
    );
  }
}

class WindSpeedCard extends StatelessWidget {
  final double? speed;

  const WindSpeedCard({super.key, required this.speed});

  @override
  Widget build(BuildContext context) {
    return DataMetricCard(
      icon: Icons.air,
      iconColor: Colors.blue[400]!,
      label: 'Wind Speed',
      value: speed?.toStringAsFixed(2) ?? '0.00',
      unit: 'm/s',
      valueColor: Colors.blue,
    );
  }
}

class WindDirectionCard extends StatelessWidget {
  final double? direction;

  const WindDirectionCard({super.key, required this.direction});

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: Container(
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(12),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.1),
              blurRadius: 8,
              offset: const Offset(0, 2),
            ),
          ],
        ),
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Icon(Icons.navigation, color: const Color(0xFF5ECCC0), size: 20),
                const SizedBox(width: 8),
                const Text('Wind Dir', style: TextStyle(color: Colors.grey, fontSize: 12)),
              ],
            ),
            const SizedBox(height: 12),
            Text(
              direction?.toStringAsFixed(1) ?? '0.0',
              style: const TextStyle(
                fontSize: 28,
                fontWeight: FontWeight.bold,
                color: Color(0xFF5ECCC0),
              ),
            ),
            const Text(
              '°\nN',
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.grey, fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }
}

class BatteryCard extends StatelessWidget {
  final int? percentage;

  const BatteryCard({super.key, required this.percentage});

  @override
  Widget build(BuildContext context) {
    return DataMetricCard(
      icon: Icons.battery_full,
      iconColor: Colors.green[400]!,
      label: 'Battery',
      value: percentage?.toString() ?? '0',
      unit: '%',
      valueColor: Colors.green,
    );
  }
}
