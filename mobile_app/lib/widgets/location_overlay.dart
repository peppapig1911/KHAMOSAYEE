import 'package:flutter/material.dart';

class LocationOverlay extends StatelessWidget {
  final String? locationError;
  final bool isLocating;
  final VoidCallback onRecenter;

  const LocationOverlay({
    super.key,
    required this.locationError,
    required this.isLocating,
    required this.onRecenter,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.92),
        borderRadius: BorderRadius.circular(18),
        border: Border.all(color: const Color(0xFFE5E7EB)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.08),
            blurRadius: 16,
            offset: const Offset(0, 8),
          ),
        ],
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          FloatingActionButton.small(
            backgroundColor: const Color(0xFF0F766E),
            foregroundColor: Colors.white,
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
          if (locationError != null) ...[
            const SizedBox(height: 8),
            SizedBox(
              width: 120,
              child: Text(
                locationError!,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  fontSize: 11,
                  color: Color(0xFF92400E),
                  fontWeight: FontWeight.w600,
                ),
                maxLines: 2,
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ],
        ],
      ),
    );
  }
}
