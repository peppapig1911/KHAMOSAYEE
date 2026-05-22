import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

class MapViewWidget extends StatelessWidget {
  final MapController mapController;
  final LatLng? userPosition;
  final LatLng? blePosition;
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
                  width: 40.0,
                  height: 40.0,
                  point: blePosition!,
                  builder: (context) => const Icon(Icons.location_on, color: Colors.blue, size: 40),
                ),
            ],
          ),
        ],
      ),
    );
  }
}
