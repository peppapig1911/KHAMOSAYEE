import 'package:flutter/material.dart';
import 'package:latlong2/latlong.dart';

class LocationOverlay extends StatelessWidget {
  final LatLng? userPosition;
  final String? locationError;
  final bool isLocating;
  final VoidCallback onRecenter;

  const LocationOverlay({
    super.key,
    required this.userPosition,
    required this.locationError,
    required this.isLocating,
    required this.onRecenter,
  });

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // Position info
        if (userPosition != null)
          Positioned(
            top: 240,
            left: 16,
            child: Container(
              padding: const EdgeInsets.all(12),
              color: Colors.black54,
              child: Text(
                'User position: ${userPosition!.latitude.toStringAsFixed(6)}, ${userPosition!.longitude.toStringAsFixed(6)}',
                style: const TextStyle(color: Colors.white, fontSize: 12),
              ),
            ),
          ),
        // Error message
        if (locationError != null)
          Positioned(
            top: userPosition != null ? 290 : 240,
            left: 16,
            right: 16,
            child: Container(
              padding: const EdgeInsets.all(10),
              color: Colors.black54,
              child: Text(
                locationError!,
                style: const TextStyle(color: Colors.orangeAccent, fontSize: 12),
              ),
            ),
          ),
        // Recenter button
        Positioned(
          bottom: 24,
          right: 16,
          child: FloatingActionButton(
            backgroundColor: Colors.black54,
            foregroundColor: Colors.white,
            mini: true,
            onPressed: isLocating ? null : onRecenter,
            child: isLocating
                ? const SizedBox(
                    width: 18,
                    height: 18,
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                    ),
                  )
                : const Icon(Icons.my_location),
          ),
        ),
      ],
    );
  }
}
