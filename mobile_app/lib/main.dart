import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:flutter_compass/flutter_compass.dart';
import 'package:geolocator/geolocator.dart';
import 'package:latlong2/latlong.dart';

import 'ble/ble.dart';
import 'ble/data.dart';
import 'widgets/ble_device_selection.dart';
import 'widgets/location_overlay.dart';
import 'widgets/map_view.dart';
import 'widgets/manual_control_page.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'KHAMOSAYEE',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF0F766E)),
      ),
      home: const MapBlePage(),
    );
  }
}

class MapBlePage extends StatefulWidget {
  const MapBlePage({super.key});

  @override
  State<MapBlePage> createState() => _MapBlePageState();
}

class _MapBlePageState extends State<MapBlePage> {
  LatLng? _userPosition;
  double? _phoneHeadingDeg;
  LatLng? _blePosition;
  LatLng? _targetPosition;
  final List<LatLng> _userTrail = <LatLng>[];
  final List<LatLng> _bleTrail = <LatLng>[];
  LatLng _mapCenter = LatLng(0.0, 0.0);
  double _mapZoom = 12.0;
  String? _locationError;
  bool _isLocating = false;
  bool _followVessel = true;

  final WindBleClient _bleClient = WindBleClient();
  final MapController _mapController = MapController();
  StreamSubscription<BleConnectionState>? _bleStateSub;
  StreamSubscription<BleData>? _bleDataSub;
  StreamSubscription<String>? _bleErrorSub;
  StreamSubscription<Position>? _positionSub;
  StreamSubscription<CompassEvent>? _compassSub;
  BleConnectionState _bleState = BleConnectionState.idle;
  BleData? _data;
  String? _bleError;
  bool _locationStreamsStarted = false;

  @override
  void initState() {
    super.initState();
    _setupBle();
  }

  void _setupBle() {
    _bleStateSub = _bleClient.stateStream.listen((state) {
      if (!mounted) return;
      setState(() {
        _bleState = state;
      });
    });

    _bleDataSub = _bleClient.dataStream.listen((data) {
      if (!mounted) return;

      final lat = data.gps.latitude;
      final lng = data.gps.longitude;
      final boatPosition = (lat != 0.0 || lng != 0.0) ? LatLng(lat, lng) : null;

      setState(() {
        _data = data;
        _bleError = null;
        if (boatPosition != null) {
          _blePosition = boatPosition;
          _appendTrailPoint(_bleTrail, boatPosition);
        }
      });

      if (_followVessel && boatPosition != null) {
        _mapController.move(boatPosition, _mapZoom);
      }
    });

    _bleErrorSub = _bleClient.errorStream.listen((message) {
      if (!mounted) return;
      setState(() {
        _bleError = message;
      });
    });
  }

  void _startCompassHeadingStream() {
    _compassSub?.cancel();

    try {
      final compassEvents = FlutterCompass.events;
      if (compassEvents == null) {
        if (!mounted) return;
        setState(() {
          _locationError = 'Compass sensor unavailable on this device.';
        });
        return;
      }

      _compassSub = compassEvents.listen(
        (event) {
          if (!mounted) return;

          final heading = event.heading;
          if (heading == null || heading.isNaN) {
            return;
          }

          setState(() {
            _phoneHeadingDeg = heading;
          });
        },
        onError: (error) {
          if (!mounted) return;
          setState(() {
            _locationError = 'Compass heading unavailable: $error';
          });
        },
      );
    } catch (error) {
      if (!mounted) return;
      setState(() {
        _locationError = 'Compass heading unavailable: $error';
      });
    }
  }

  void _startPositionStream() {
    _positionSub?.cancel();

    try {
      _positionSub =
          Geolocator.getPositionStream(
            locationSettings: const LocationSettings(
              accuracy: LocationAccuracy.high,
              distanceFilter: 1,
            ),
          ).listen(
            (position) {
              if (!mounted) return;

              final target = LatLng(position.latitude, position.longitude);

              setState(() {
                _userPosition = target;
                _appendTrailPoint(_userTrail, target);
              });

              if (_followVessel) {
                _mapController.move(target, _mapZoom);
              }
            },
            onError: (error) {
              if (!mounted) return;
              setState(() {
                _locationError = 'Live location stream unavailable: $error';
              });
            },
          );
    } catch (error) {
      if (!mounted) return;
      setState(() {
        _locationError = 'Live location stream unavailable: $error';
      });
    }
  }

  Future<void> _initializeLocation() async {
    if (_locationStreamsStarted) {
      return;
    }

    _locationStreamsStarted = true;
    await _refreshUserLocation(moveMap: true);
    _startPositionStream();
    _startCompassHeadingStream();
  }

  Future<void> _refreshUserLocation({bool moveMap = false}) async {
    setState(() {
      _isLocating = true;
      _locationError = null;
    });

    final hasService = await Geolocator.isLocationServiceEnabled();
    if (!hasService) {
      if (!mounted) return;
      setState(() {
        _isLocating = false;
        _locationError = 'Location service is disabled.';
      });
      return;
    }

    var permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }

    if (permission == LocationPermission.denied) {
      if (!mounted) return;
      setState(() {
        _isLocating = false;
        _locationError = 'Location permission denied.';
      });
      return;
    }

    if (permission == LocationPermission.deniedForever) {
      if (!mounted) return;
      setState(() {
        _isLocating = false;
        _locationError = 'Location permission permanently denied.';
      });
      return;
    }

    try {
      final position = await Geolocator.getCurrentPosition(
        desiredAccuracy: LocationAccuracy.high,
        timeLimit: const Duration(seconds: 10),
      );
      final target = LatLng(position.latitude, position.longitude);
      if (!mounted) return;

      setState(() {
        _userPosition = target;
        _appendTrailPoint(_userTrail, target);
        _isLocating = false;
        if (moveMap) {
          _mapCenter = target;
        }
      });

      if (moveMap) {
        _mapController.move(target, _mapController.zoom);
      }
    } on TimeoutException {
      final lastKnown = await Geolocator.getLastKnownPosition();
      if (!mounted) return;

      if (lastKnown != null) {
        final target = LatLng(lastKnown.latitude, lastKnown.longitude);
        setState(() {
          _userPosition = target;
          _appendTrailPoint(_userTrail, target);
          _isLocating = false;
          _locationError = 'Using last known location.';
          if (moveMap) {
            _mapCenter = target;
          }
        });
        if (moveMap) {
          _mapController.move(target, _mapController.zoom);
        }
      } else {
        setState(() {
          _isLocating = false;
          _locationError = 'Timed out while getting current location.';
        });
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _isLocating = false;
        _locationError = 'Failed to get location: $e';
      });
    }
  }

  @override
  void dispose() {
    _bleStateSub?.cancel();
    _bleDataSub?.cancel();
    _bleErrorSub?.cancel();
    _positionSub?.cancel();
    _compassSub?.cancel();
    unawaited(_bleClient.dispose());
    super.dispose();
  }

  void _onMapTap(TapPosition tapPosition, LatLng point) {
    setState(() {
      _targetPosition = point;
    });
    _mapController.move(point, _mapController.zoom);
    unawaited(_sendTargetWaypoint(point));
  }

  void _onMapMoved((LatLng, double, bool) camera) {
    setState(() {
      _mapCenter = camera.$1;
      _mapZoom = camera.$2;
      if (camera.$3) {
        _followVessel = false;
      }
    });
  }

  void _appendTrailPoint(List<LatLng> trail, LatLng point) {
    if (trail.isNotEmpty) {
      final lastPoint = trail.last;
      if (lastPoint.latitude == point.latitude && lastPoint.longitude == point.longitude) {
        return;
      }
    }

    trail.add(point);
    if (trail.length > 300) {
      trail.removeAt(0);
    }
  }

  Future<void> _sendTargetWaypoint(LatLng point) async {
    if (!_bleClient.isConnected) {
      return;
    }

    try {
      await _bleClient.setTargetWaypoint(point.latitude, point.longitude);
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('Failed to send target waypoint: $e')));
    }
  }

  Future<void> _recenterToUserPosition() async {
    await _refreshUserLocation(moveMap: true);
    if (_userPosition != null && mounted) {
      _mapController.move(_userPosition!, _mapController.zoom);
    }
  }

  void _toggleFollowVessel() {
    setState(() {
      _followVessel = !_followVessel;
    });

    if (_followVessel && _blePosition != null) {
      _mapController.move(_blePosition!, _mapZoom);
    }
  }

  double? _distanceToTargetMeters() {
    final target = _targetPosition;
    final boat = _blePosition;
    if (target == null || boat == null) {
      return null;
    }

    return const Distance().as(LengthUnit.Meter, boat, target);
  }

  Future<void> _connectToSelectedDevice(BluetoothDevice device) async {
    try {
      await _bleClient.connect(device);
      await _bleClient.setControlMode(BleControlMode.automatic);
      unawaited(_initializeLocation());
      if (mounted) {
        setState(() {});
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(SnackBar(content: Text('Failed to connect: $e')));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    if (!_bleClient.isConnected) {
      return BleDeviceSelectionPage(
        bleClient: _bleClient,
        onDeviceSelected: _connectToSelectedDevice,
      );
    }

    final deviceName = _bleClient.device?.platformName ?? 'Not Connected';
    final deviceId = _bleClient.device?.remoteId.str ?? 'Unknown';
    final bleStatus = switch (_bleState) {
      BleConnectionState.scanning => 'Scanning',
      BleConnectionState.connecting => 'Connecting',
      BleConnectionState.connected => 'Connected',
      BleConnectionState.disconnected => 'Disconnected',
      BleConnectionState.error => 'Error',
      BleConnectionState.idle => 'Idle',
    };

    final gpsAccuracy = _data?.gps.accuracyM;
    final gpsAge = _data == null ? null : DateTime.now().difference(_data!.gps.timestamp);
    final heading = _data?.navigation.headingDeg;
    final targetDistance = _distanceToTargetMeters();

    return Scaffold(
      backgroundColor: const Color(0xFFF3F7FA),
      appBar: AppBar(
        elevation: 0,
        backgroundColor: const Color(0xFF0F766E),
        title: const Text('KHAMOSAYEE', style: TextStyle(fontWeight: FontWeight.w800)),
        actions: [
          IconButton(
            tooltip: _followVessel ? 'Stop following vessel' : 'Follow vessel',
            icon: Icon(_followVessel ? Icons.gps_fixed : Icons.gps_not_fixed),
            onPressed: _toggleFollowVessel,
          ),
          IconButton(
            tooltip: 'Manual control',
            icon: const Icon(Icons.tune),
            onPressed: () {
              Navigator.of(context).push(
                MaterialPageRoute<void>(builder: (_) => ManualControlPage(bleClient: _bleClient)),
              );
            },
          ),
        ],
      ),
      body: Stack(
        children: [
          Positioned.fill(
            child: Container(
              decoration: const BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [Color(0xFFE7F6F4), Color(0xFFF7F8FB)],
                ),
              ),
              child: SafeArea(
                child: Column(
                  children: [
                    Flexible(
                      fit: FlexFit.loose,
                      child: SingleChildScrollView(
                        padding: const EdgeInsets.only(bottom: 8),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Padding(
                              padding: const EdgeInsets.fromLTRB(16, 12, 16, 8),
                              child: _HeaderPanel(
                                deviceName: deviceName,
                                deviceId: deviceId,
                                bleStatus: bleStatus,
                                followVessel: _followVessel,
                                targetDistanceM: targetDistance,
                              ),
                            ),
                            if (_bleError != null || _locationError != null)
                              Padding(
                                padding: const EdgeInsets.symmetric(horizontal: 16),
                                child: _AlertBanner(
                                  bleError: _bleError,
                                  locationError: _locationError,
                                ),
                              ),
                            Padding(
                              padding: const EdgeInsets.fromLTRB(16, 12, 16, 8),
                              child: _TelemetryGrid(
                                windData: _data?.wind,
                                gpsAccuracyM: gpsAccuracy,
                                gpsAge: gpsAge,
                                phoneHeadingDeg: _phoneHeadingDeg,
                                boatPosition: _blePosition,
                                targetPosition: _targetPosition,
                                headingDeg: heading,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.fromLTRB(16, 0, 16, 16),
                        child: ClipRRect(
                          borderRadius: BorderRadius.circular(24),
                          child: Stack(
                            children: [
                              MapViewWidget(
                                mapController: _mapController,
                                userPosition: _userPosition,
                                userHeadingDeg: _phoneHeadingDeg,
                                blePosition: _blePosition,
                                headingDeg: heading,
                                targetPosition: _targetPosition,
                                userTrail: _userTrail,
                                bleTrail: _bleTrail,
                                center: _mapCenter,
                                zoom: _mapZoom,
                                onMapTap: _onMapTap,
                                onMapMoved: _onMapMoved,
                              ),
                              Positioned(
                                right: 14,
                                bottom: 14,
                                child: LocationOverlay(
                                  locationError: _locationError,
                                  isLocating: _isLocating,
                                  onRecenter: _recenterToUserPosition,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _HeaderPanel extends StatelessWidget {
  const _HeaderPanel({
    required this.deviceName,
    required this.deviceId,
    required this.bleStatus,
    required this.followVessel,
    required this.targetDistanceM,
  });

  final String deviceName;
  final String deviceId;
  final String bleStatus;
  final bool followVessel;
  final double? targetDistanceM;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: const LinearGradient(
          colors: [Color(0xFF0F766E), Color(0xFF155E75)],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(24),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.12),
            blurRadius: 24,
            offset: const Offset(0, 10),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text(
                      'Live vessel dashboard',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 18,
                        fontWeight: FontWeight.w800,
                      ),
                    ),
                    const SizedBox(height: 4),
                    Text(
                      '$deviceName · $deviceId',
                      style: const TextStyle(color: Colors.white70, fontSize: 12),
                      overflow: TextOverflow.ellipsis,
                    ),
                  ],
                ),
              ),
              _StatusPill(
                label: bleStatus,
                color: const Color(0xFFECFEFF),
                textColor: const Color(0xFF0F766E),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              _StatusPill(
                label: followVessel ? 'Following vessel' : 'Map free pan',
                color: followVessel ? const Color(0xFFDBF4FF) : const Color(0xFFF1F5F9),
                textColor: const Color(0xFF0F172A),
              ),
              _StatusPill(
                label: targetDistanceM == null
                    ? 'No target'
                    : 'Target ${targetDistanceM!.toStringAsFixed(1)} m',
                color: const Color(0xFFF8FAFC),
                textColor: const Color(0xFF0F172A),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class _TelemetryGrid extends StatelessWidget {
  const _TelemetryGrid({
    required this.windData,
    required this.gpsAccuracyM,
    required this.gpsAge,
    required this.boatPosition,
    required this.targetPosition,
    required this.headingDeg,
    required this.phoneHeadingDeg,
  });

  final WindGattData? windData;
  final double? gpsAccuracyM;
  final Duration? gpsAge;
  final LatLng? boatPosition;
  final LatLng? targetPosition;
  final double? headingDeg;
  final double? phoneHeadingDeg;

  @override
  Widget build(BuildContext context) {
    final cards = <Widget>[
      _InfoCard(
        title: 'GPS fix',
        value: boatPosition == null
            ? 'Waiting'
            : '${boatPosition!.latitude.toStringAsFixed(6)}, ${boatPosition!.longitude.toStringAsFixed(6)}',
        subtitle: gpsAge == null ? 'No GNSS data yet' : 'Age ${gpsAge!.inSeconds}s',
        icon: Icons.my_location,
        accentColor: const Color(0xFF0EA5E9),
      ),
      _InfoCard(
        title: 'Accuracy',
        value: gpsAccuracyM == null ? '--' : '${gpsAccuracyM!.toStringAsFixed(1)} m',
        subtitle: 'Horizontal uncertainty',
        icon: Icons.radar,
        accentColor: const Color(0xFF14B8A6),
      ),
      _InfoCard(
        title: 'Heading',
        value: headingDeg == null ? '--' : '${headingDeg!.toStringAsFixed(1)}°',
        subtitle: 'CMPS12 compass',
        icon: Icons.explore,
        accentColor: const Color(0xFF6366F1),
      ),
      _InfoCard(
        title: 'Wind',
        value: windData?.windSpeedMs == null
            ? '--'
            : '${windData!.windSpeedMs!.toStringAsFixed(2)} m/s',
        subtitle: windData?.windDirectionDeg == null
            ? 'Direction unavailable'
            : 'Dir ${windData!.windDirectionDeg!.toStringAsFixed(1)}°',
        icon: Icons.air,
        accentColor: const Color(0xFFF97316),
      ),
      _InfoCard(
        title: 'Battery',
        value: windData?.batteryPercent == null ? '--' : '${windData!.batteryPercent}%',
        subtitle: 'BLE-reported power',
        icon: Icons.battery_full,
        accentColor: const Color(0xFF22C55E),
      ),
      _InfoCard(
        title: 'Target',
        value: targetPosition == null
            ? 'Tap map'
            : '${targetPosition!.latitude.toStringAsFixed(6)}, ${targetPosition!.longitude.toStringAsFixed(6)}',
        subtitle: targetPosition == null ? 'No active waypoint' : 'Waypoint set from map tap',
        icon: Icons.flag,
        accentColor: const Color(0xFFEA580C),
      ),
      _InfoCard(
        title: 'Phone heading',
        value: phoneHeadingDeg == null ? '--' : '${phoneHeadingDeg!.toStringAsFixed(1)}°',
        subtitle: 'Direction of your phone',
        icon: Icons.screen_rotation,
        accentColor: const Color(0xFF8B5CF6),
      ),
    ];

    return LayoutBuilder(
      builder: (context, constraints) {
        final cardWidth = constraints.maxWidth >= 700
            ? (constraints.maxWidth - 12) / 2
            : constraints.maxWidth;
        return Wrap(
          spacing: 12,
          runSpacing: 12,
          children: cards
              .map((card) => SizedBox(width: cardWidth, child: card))
              .toList(growable: false),
        );
      },
    );
  }
}

class _InfoCard extends StatelessWidget {
  const _InfoCard({
    required this.title,
    required this.value,
    required this.subtitle,
    required this.icon,
    required this.accentColor,
  });

  final String title;
  final String value;
  final String subtitle;
  final IconData icon;
  final Color accentColor;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: const Color(0xFFE5E7EB)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            blurRadius: 18,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            width: 42,
            height: 42,
            decoration: BoxDecoration(
              color: accentColor.withOpacity(0.12),
              borderRadius: BorderRadius.circular(14),
            ),
            child: Icon(icon, color: accentColor, size: 22),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title.toUpperCase(),
                  style: const TextStyle(
                    fontSize: 11,
                    letterSpacing: 0.7,
                    fontWeight: FontWeight.w700,
                    color: Color(0xFF64748B),
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  value,
                  style: const TextStyle(
                    fontSize: 20,
                    fontWeight: FontWeight.w800,
                    color: Color(0xFF0F172A),
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
                const SizedBox(height: 4),
                Text(
                  subtitle,
                  style: const TextStyle(fontSize: 12, color: Color(0xFF64748B)),
                  overflow: TextOverflow.ellipsis,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _StatusPill extends StatelessWidget {
  const _StatusPill({required this.label, required this.color, required this.textColor});

  final String label;
  final Color color;
  final Color textColor;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(color: color, borderRadius: BorderRadius.circular(999)),
      child: Text(
        label,
        style: TextStyle(color: textColor, fontSize: 12, fontWeight: FontWeight.w700),
      ),
    );
  }
}

class _AlertBanner extends StatelessWidget {
  const _AlertBanner({required this.bleError, required this.locationError});

  final String? bleError;
  final String? locationError;

  @override
  Widget build(BuildContext context) {
    final message = <String>[
      if (bleError != null) bleError!,
      if (locationError != null) locationError!,
    ].join(' • ');

    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: const Color(0xFFFFF7ED),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: const Color(0xFFF59E0B).withOpacity(0.35)),
      ),
      child: Row(
        children: [
          const Icon(Icons.warning_amber_rounded, color: Color(0xFFD97706), size: 20),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              message,
              style: const TextStyle(
                color: Color(0xFF92400E),
                fontSize: 12,
                fontWeight: FontWeight.w600,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
