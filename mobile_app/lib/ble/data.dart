class WindGattData {
  const WindGattData({
    required this.windSpeedMs,
    required this.windDirectionDeg,
    required this.batteryPercent,
    required this.timestamp,
  });

  /// Apparent wind speed in m/s (source unit is 0.01 m/s).
  final double? windSpeedMs;

  /// Apparent wind direction in degrees (source unit is 0.01 degrees).
  final double? windDirectionDeg;

  /// Battery level in percent (0-100).
  final int? batteryPercent;

  final DateTime timestamp;

  WindGattData copyWith({
    double? windSpeedMs,
    double? windDirectionDeg,
    int? batteryPercent,
    DateTime? timestamp,
  }) {
    return WindGattData(
      windSpeedMs: windSpeedMs ?? this.windSpeedMs,
      windDirectionDeg: windDirectionDeg ?? this.windDirectionDeg,
      batteryPercent: batteryPercent ?? this.batteryPercent,
      timestamp: timestamp ?? this.timestamp,
    );
  }
}

class GPSData {
  const GPSData({
    required this.latitude,
    required this.longitude,
    required this.altitudeM,
    required this.accuracyM,
    required this.timestamp,
  });

  final double latitude;
  final double longitude;
  final double altitudeM;
  final double accuracyM;
  final DateTime timestamp;

  GPSData copyWith({
    double? latitude,
    double? longitude,
    double? altitudeM,
    double? accuracyM,
    DateTime? timestamp,
  }) {
    return GPSData(
      latitude: latitude ?? this.latitude,
      longitude: longitude ?? this.longitude,
      altitudeM: altitudeM ?? this.altitudeM,
      accuracyM: accuracyM ?? this.accuracyM,
      timestamp: timestamp ?? this.timestamp,
    );
  }
}
