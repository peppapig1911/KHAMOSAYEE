import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:geolocator/geolocator.dart';

import 'ble.dart';
import 'widgets/ble_bottom_drawer.dart';

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
  static final LatLng _defaultCenter = LatLng(0.0, 0.0);
  static const double _drawerCollapsedHeight = 76;
  static const double _drawerExpandedHeight = 230;

  LatLng? _userPosition;
  String? _locationError;
  bool _isLocating = false;
  bool _isDrawerExpanded = true;

  final WindBleClient _bleClient = WindBleClient();
  StreamSubscription<BleConnectionState>? _bleStateSub;
  StreamSubscription<WindGattData>? _bleDataSub;
  StreamSubscription<String>? _bleErrorSub;
  BleConnectionState _bleState = BleConnectionState.idle;
  WindGattData? _windData;
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
        _windData = data;
        _bleError = null;
      });
    });

    _bleErrorSub = _bleClient.errorStream.listen((message) {
      if (!mounted) return;
      setState(() {
        _bleError = message;
      });
    });

    unawaited(_scanAndConnect());
  }

  Future<void> _scanAndConnect() async {
    await _bleClient.startScanAndConnect(deviceId: "2C:CF:67:09:0F:C4");
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
        _isLocating = false;
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
          _isLocating = false;
          _locationError = 'Using last known location.';
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

  Future<void> _recenterToUserPosition() async {
    await _refreshUserLocation(moveMap: true);
    if (_userPosition != null && mounted) {
      _mapController.move(_userPosition!, _mapController.zoom);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('KHAMOSAYEE Map & BLE'),
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
      ),
      bottomSheet: BleBottomDrawer(
        isExpanded: _isDrawerExpanded,
        bleState: _bleState,
        windData: _windData,
        bleError: _bleError,
        isLocating: _isLocating,
        onToggleExpanded: () => setState(() {
          _isDrawerExpanded = !_isDrawerExpanded;
        }),
        onReconnect: _scanAndConnect,
      ),
      body: Stack(
        children: [
          FlutterMap(
            mapController: _mapController,
            options: MapOptions(
              center: _userPosition ?? _defaultCenter,
              zoom: 12.0,
              onTap: _onMapTap,
            ),
            children: [
              TileLayer(
                urlTemplate: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
                userAgentPackageName: 'com.example.khamosayee_map_ble',
                subdomains: const ['a', 'b', 'c'],
              ),
              MarkerLayer(
                markers: [
                  if (_userPosition != null)
                    Marker(
                      width: 40.0,
                      height: 40.0,
                      point: _userPosition!,
                      builder: (context) =>
                          const Icon(Icons.location_on, color: Colors.red, size: 40),
                    ),
                ],
              ),
            ],
          ),
          if (_userPosition != null)
            Positioned(
              top: 16,
              left: 16,
              child: Container(
                padding: const EdgeInsets.all(12),
                color: Colors.black54,
                child: Text(
                  'User position: ${_userPosition!.latitude.toStringAsFixed(6)}, ${_userPosition!.longitude.toStringAsFixed(6)}',
                  style: const TextStyle(color: Colors.white),
                ),
              ),
            ),
          if (_locationError != null)
            Positioned(
              top: _userPosition != null ? 76 : 16,
              left: 16,
              right: 16,
              child: Container(
                padding: const EdgeInsets.all(10),
                color: Colors.black54,
                child: Text(_locationError!, style: const TextStyle(color: Colors.orangeAccent)),
              ),
            ),
          Positioned(
            bottom: _isDrawerExpanded ? _drawerExpandedHeight + 16 : _drawerCollapsedHeight + 16,
            right: 16,
            child: FloatingActionButton(
              backgroundColor: Colors.black54,
              foregroundColor: Colors.white,
              mini: true,
              onPressed: _isLocating ? null : _recenterToUserPosition,
              child: _isLocating
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
      ),
    );
  }
}
