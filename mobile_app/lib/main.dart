import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:geolocator/geolocator.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'ble/ble.dart';
import 'widgets/wind_data_card.dart';
import 'widgets/map_view.dart';
import 'widgets/location_overlay.dart';
import 'widgets/ble_device_selection.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'KHAMOSAYEE Map & BLE',
      theme: ThemeData(colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple)),
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
  final List<LatLng> _userTrail = <LatLng>[];
  final List<LatLng> _bleTrail = <LatLng>[];
  LatLng _mapCenter = LatLng(0.0, 0.0);
  double _mapZoom = 12.0;
  String? _locationError;
  bool _isLocating = false;

  final WindBleClient _bleClient = WindBleClient();
  StreamSubscription<BleConnectionState>? _bleStateSub;
  StreamSubscription<BleData>? _bleDataSub;
  StreamSubscription<String>? _bleErrorSub;
  BleConnectionState _bleState = BleConnectionState.idle;
  BleData? _data;
  String? _bleError;
  final MapController _mapController = MapController();

  @override
  void initState() {
    super.initState();
    unawaited(_initializeLocation());
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
      setState(() {
        _data = data;
        _bleError = null;
        var lat = data.gps.latitude;
        var lng = data.gps.longitude;
        if (lat != 0 && lng != 0) {
          _appendTrailPoint(_bleTrail, LatLng(lat, lng));
        }
      });
    });

    _bleErrorSub = _bleClient.errorStream.listen((message) {
      if (!mounted) return;
      setState(() {
        _bleError = message;
      });
    });
  }

  Future<void> _initializeLocation() async {
    await _refreshUserLocation(moveMap: true);
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
    unawaited(_bleClient.dispose());
    super.dispose();
  }

  void _onMapTap(TapPosition tapPosition, LatLng point) {
    // Keep user location tied to GPS; tapping only pans the map.
    _mapController.move(point, _mapController.zoom);
  }

  void _onMapMoved((LatLng, double) camera) {
    setState(() {
      _mapCenter = camera.$1;
      _mapZoom = camera.$2;
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

  Future<void> _recenterToUserPosition() async {
    await _refreshUserLocation(moveMap: true);
    if (_userPosition != null && mounted) {
      _mapController.move(_userPosition!, _mapController.zoom);
    }
  }

  @override
  Widget build(BuildContext context) {
    // Show device selection page if not connected
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

    return Scaffold(
      backgroundColor: Colors.grey[100],
      appBar: AppBar(
        elevation: 0,
        backgroundColor: const Color(0xFF5ECCC0),
        title: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Calypso Anemometer',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            Text(
              '$deviceName · $deviceId',
              style: const TextStyle(fontSize: 12, fontWeight: FontWeight.normal),
            ),
            Text(bleStatus, style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w500)),
          ],
        ),
      ),
      body: Stack(
        children: [
          Column(
            children: [
              WindDataCardsSection(windData: _data?.wind),
              MapViewWidget(
                mapController: _mapController,
                userPosition: _userPosition,
                blePosition: LatLng(_data!.gps.latitude, _data!.gps.longitude),
                userTrail: _userTrail,
                bleTrail: _bleTrail,
                center: _mapCenter,
                zoom: _mapZoom,
                onMapTap: _onMapTap,
                onMapMoved: _onMapMoved,
              ),
            ],
          ),
          if (_bleError != null)
            Positioned(
              top: 12,
              left: 12,
              right: 12,
              child: SafeArea(
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                  decoration: BoxDecoration(
                    color: Colors.black87,
                    borderRadius: BorderRadius.circular(10),
                  ),
                  child: Text(
                    _bleError!,
                    style: const TextStyle(color: Colors.white, fontSize: 12),
                  ),
                ),
              ),
            ),
          LocationOverlay(
            userPosition: _userPosition,
            locationError: _locationError,
            isLocating: _isLocating,
            onRecenter: _recenterToUserPosition,
          ),
        ],
      ),
    );
  }

  Future<void> _connectToSelectedDevice(BluetoothDevice device) async {
    try {
      await _bleClient.connect(device);
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
}
