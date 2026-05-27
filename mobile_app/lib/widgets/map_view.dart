import 'dart:ui' as ui;
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

class MapViewWidget extends StatelessWidget {
  final MapController mapController;
  final LatLng? userPosition;
  final LatLng? blePosition;
  final double? headingDeg;
  final LatLng? targetPosition;
  final List<LatLng> userTrail;
  final List<LatLng> bleTrail;
  final LatLng center;
  final double zoom;
  final Function(TapPosition, LatLng) onMapTap;
  final ValueChanged<(LatLng, double)> onMapMoved;

  const MapViewWidget({
    super.key,
    required this.mapController,
    required this.userPosition,
    required this.blePosition,
    required this.headingDeg,
    required this.targetPosition,
    required this.userTrail,
    required this.bleTrail,
    required this.center,
    required this.zoom,
    required this.onMapTap,
    required this.onMapMoved,
  });

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: FlutterMap(
        mapController: mapController,
        options: MapOptions(
          center: center,
          zoom: zoom,
          minZoom: 3.0,
          maxZoom: 19.0,
          onTap: onMapTap,
          onPositionChanged: (position, hasGesture) {
            if (position.center != null) {
              onMapMoved((position.center!, position.zoom ?? zoom));
            }
          },
        ),
        children: [
          TileLayer(
            urlTemplate: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
            userAgentPackageName: 'com.example.khamosayee_map_ble',
            minZoom: 3.0,
            maxZoom: 19.0,
            subdomains: const ['a', 'b', 'c'],
          ),
          if (userTrail.length > 1 || bleTrail.length > 1)
            PolylineLayer(
              polylines: [
                if (userTrail.length > 1)
                  Polyline(
                    points: userTrail,
                    color: const Color(0xFF1E88E5).withOpacity(0.75),
                    strokeWidth: 4.0,
                  ),
                if (bleTrail.length > 1)
                  Polyline(
                    points: bleTrail,
                    color: Colors.blueAccent.withOpacity(0.65),
                    strokeWidth: 4.0,
                  ),
              ],
            ),
          MarkerLayer(
            markers: [
              if (userPosition != null)
                Marker(
                  width: 40.0,
                  height: 40.0,
                  point: userPosition!,
                  builder: (context) =>
                      const Icon(Icons.navigation, color: Color(0xFF1E88E5), size: 34),
                ),
              if (blePosition != null)
                Marker(
                  width: 64.0,
                  height: 64.0,
                  point: blePosition!,
                  builder: (context) => _HeadingConeMarker(headingDeg: headingDeg),
                ),
              if (targetPosition != null)
                Marker(
                  width: 44.0,
                  height: 44.0,
                  point: targetPosition!,
                  builder: (context) => const Icon(Icons.flag, color: Color(0xFFE65100), size: 36),
                ),
            ],
          ),
        ],
      ),
    );
  }
}

class _HeadingConeMarker extends StatelessWidget {
  const _HeadingConeMarker({required this.headingDeg});

  final double? headingDeg;

  @override
  Widget build(BuildContext context) {
    final rotation = ((headingDeg ?? 0.0) - 90.0) * math.pi / 180.0;

    return Transform.rotate(
      angle: rotation,
      child: CustomPaint(
        size: const Size(64, 64),
        painter: _HeadingConePainter(color: const Color(0xFF1565C0)),
      ),
    );
  }
}

class _HeadingConePainter extends CustomPainter {
  const _HeadingConePainter({required this.color});

  final Color color;

  @override
  void paint(Canvas canvas, Size size) {
    final center = ui.Offset(size.width / 2, size.height / 2);
    final conePath = ui.Path()
      ..moveTo(center.dx, size.height * 0.06)
      ..lineTo(size.width * 0.78, size.height * 0.82)
      ..lineTo(size.width * 0.22, size.height * 0.82)
      ..close();

    final fillPaint = ui.Paint()
      ..color = color.withValues(alpha: 0.20)
      ..style = PaintingStyle.fill;
    canvas.drawPath(conePath, fillPaint);

    final outlinePaint = ui.Paint()
      ..color = color.withValues(alpha: 0.85)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawPath(conePath, outlinePaint);

    canvas.drawCircle(center, 4.0, ui.Paint()..color = color);
  }

  @override
  bool shouldRepaint(covariant _HeadingConePainter oldDelegate) {
    return oldDelegate.color != color;
  }
}
