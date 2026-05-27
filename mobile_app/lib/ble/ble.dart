import 'dart:async';
import 'dart:developer';
import 'dart:typed_data';

import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'data.dart';

enum BleConnectionState { idle, scanning, connecting, connected, disconnected, error }

enum BleControlMode {
  automatic(0),
  manual(1);

  const BleControlMode(this.value);
  final int value;
}

class BleData {
  final WindGattData wind;
  final GPSData gps;
  final NavigationData navigation;

  BleData({required this.wind, required this.gps, required this.navigation});
}

/// BLE manager for scanning, connecting and consuming wind GATT data.
class WindBleClient {
  static final Guid environmentalSensingService = Guid('0000181A-0000-1000-8000-00805F9B34FB');
  static final Guid batteryService = Guid('0000180F-0000-1000-8000-00805F9B34FB');
  static final Guid manualControlService = Guid('0000FFF0-0000-1000-8000-00805F9B34FB');

  static final Guid apparentWindSpeedChar = Guid('00002A72-0000-1000-8000-00805F9B34FB');
  static final Guid apparentWindDirectionChar = Guid('00002A73-0000-1000-8000-00805F9B34FB');
  static final Guid batteryLevelChar = Guid('00002A19-0000-1000-8000-00805F9B34FB');
  static final Guid latitudeChar = Guid('00002A69-0000-1000-8000-00805F9B34FB');
  static final Guid longitudeChar = Guid('00002A6A-0000-1000-8000-00805F9B34FB');
  static final Guid altitudeChar = Guid('00002A6B-0000-1000-8000-00805F9B34FB');
  static final Guid headingChar = Guid('0000FFF5-0000-1000-8000-00805F9B34FB');
  static final Guid gpsAccuracyChar = Guid('0000FFF6-0000-1000-8000-00805F9B34FB');
  static final Guid manualModeChar = Guid('0000FFF1-0000-1000-8000-00805F9B34FB');
  static final Guid frontWheelOffsetChar = Guid('0000FFF2-0000-1000-8000-00805F9B34FB');
  static final Guid sailOpeningChar = Guid('0000FFF3-0000-1000-8000-00805F9B34FB');
  static final Guid targetWaypointChar = Guid('0000FFF4-0000-1000-8000-00805F9B34FB');

  final _stateController = StreamController<BleConnectionState>.broadcast();
  final _dataController = StreamController<BleData>.broadcast();
  final _errorController = StreamController<String>.broadcast();

  Stream<BleConnectionState> get stateStream => _stateController.stream;
  Stream<BleData> get dataStream => _dataController.stream;
  Stream<String> get errorStream => _errorController.stream;

  BluetoothDevice? _device;
  BluetoothCharacteristic? _windSpeedCharacteristic;
  BluetoothCharacteristic? _windDirectionCharacteristic;
  BluetoothCharacteristic? _batteryCharacteristic;
  BluetoothCharacteristic? _latitudeCharacteristic;
  BluetoothCharacteristic? _longitudeCharacteristic;
  BluetoothCharacteristic? _altitudeCharacteristic;
  BluetoothCharacteristic? _headingCharacteristic;
  BluetoothCharacteristic? _gpsAccuracyCharacteristic;
  BluetoothCharacteristic? _manualModeCharacteristic;
  BluetoothCharacteristic? _frontWheelOffsetCharacteristic;
  BluetoothCharacteristic? _sailOpeningCharacteristic;
  BluetoothCharacteristic? _targetWaypointCharacteristic;

  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<BluetoothConnectionState>? _deviceStateSub;
  final List<StreamSubscription<List<int>>> _notifySubs = <StreamSubscription<List<int>>>[];

  WindGattData _latestWindData = WindGattData(
    windSpeedMs: null,
    windDirectionDeg: null,
    batteryPercent: null,
    timestamp: DateTime.now(),
  );

  GPSData _latestGpsData = GPSData(
    latitude: 0.0,
    longitude: 0.0,
    altitudeM: 0.0,
    accuracyM: 0.0,
    timestamp: DateTime.now(),
  );

  NavigationData _latestNavigationData = NavigationData(
    headingDeg: null,
    timestamp: DateTime.now(),
  );

  bool get isConnected => _device != null;
  BluetoothDevice? get device => _device;

  Stream<List<ScanResult>> get scanResults => FlutterBluePlus.scanResults;

  Future<void> startScanning({Duration timeout = const Duration(seconds: 15)}) async {
    _emitState(BleConnectionState.scanning);
    try {
      await FlutterBluePlus.startScan(timeout: timeout);
    } catch (e) {
      _emitError('Failed to start scan: $e');
      _emitState(BleConnectionState.error);
    }
  }

  Future<void> stopScanning() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (e) {
      _emitError('Failed to stop scan: $e');
    }
    _emitState(BleConnectionState.idle);
  }

  Future<void> startScanAndConnect({
    String? deviceId,
    Duration timeout = const Duration(seconds: 8),
  }) async {
    await disconnect();

    _emitState(BleConnectionState.scanning);
    await _scanSub?.cancel();

    final completer = Completer<BluetoothDevice>();
    _scanSub = FlutterBluePlus.scanResults.listen((results) {
      for (final result in results) {
        final d = result.device;
        final idMatch = deviceId == null || d.remoteId.str == deviceId;

        if (idMatch && !completer.isCompleted) {
          log('Selected device: ${d.platformName} (${d.remoteId})');
          completer.complete(d);
          break;
        }
      }
    });

    try {
      await FlutterBluePlus.startScan(timeout: timeout);
      final foundDevice = await completer.future.timeout(timeout);
      await FlutterBluePlus.stopScan();
      await _scanSub?.cancel();
      _scanSub = null;

      await connect(foundDevice);
    } on TimeoutException {
      _emitState(BleConnectionState.error);
      _emitError('No matching BLE device found before timeout.');
      await FlutterBluePlus.stopScan();
    } catch (e) {
      _emitState(BleConnectionState.error);
      _emitError('Scan/connect failed: $e');
      await FlutterBluePlus.stopScan();
    }
  }

  Future<void> connect(BluetoothDevice device) async {
    _emitState(BleConnectionState.connecting);

    try {
      await device.connect();
    } catch (_) {
      // Ignored: connect throws when already connected on some platforms.
    }

    _device = device;
    _deviceStateSub?.cancel();
    _deviceStateSub = device.connectionState.listen((state) {
      if (state == BluetoothConnectionState.disconnected) {
        _emitState(BleConnectionState.disconnected);
      }
    });

    await _discoverAndSubscribe();
    _emitState(BleConnectionState.connected);
  }

  Future<void> disconnect() async {
    await _scanSub?.cancel();
    _scanSub = null;

    for (final sub in _notifySubs) {
      await sub.cancel();
    }
    _notifySubs.clear();

    await _deviceStateSub?.cancel();
    _deviceStateSub = null;

    if (_device != null) {
      try {
        await _device!.disconnect();
      } catch (_) {
        // Device may already be disconnected.
      }
    }

    _device = null;
    _windSpeedCharacteristic = null;
    _windDirectionCharacteristic = null;
    _batteryCharacteristic = null;
    _latitudeCharacteristic = null;
    _longitudeCharacteristic = null;
    _altitudeCharacteristic = null;
    _headingCharacteristic = null;
    _gpsAccuracyCharacteristic = null;
    _manualModeCharacteristic = null;
    _frontWheelOffsetCharacteristic = null;
    _sailOpeningCharacteristic = null;
    _targetWaypointCharacteristic = null;
    _emitState(BleConnectionState.idle);
  }

  Future<void> dispose() async {
    await disconnect();
    await _stateController.close();
    await _dataController.close();
    await _errorController.close();
  }

  Future<void> _discoverAndSubscribe() async {
    final device = _device;
    if (device == null) {
      throw StateError('No connected device.');
    }

    final services = await device.discoverServices();
    for (final service in services) {
      for (final c in service.characteristics) {
        if (c.uuid == apparentWindSpeedChar) {
          _windSpeedCharacteristic = c;
        } else if (c.uuid == apparentWindDirectionChar) {
          _windDirectionCharacteristic = c;
        } else if (c.uuid == batteryLevelChar) {
          _batteryCharacteristic = c;
        } else if (c.uuid == latitudeChar) {
          _latitudeCharacteristic = c;
        } else if (c.uuid == longitudeChar) {
          _longitudeCharacteristic = c;
        } else if (c.uuid == altitudeChar) {
          _altitudeCharacteristic = c;
        } else if (c.uuid == headingChar) {
          _headingCharacteristic = c;
        } else if (c.uuid == gpsAccuracyChar) {
          _gpsAccuracyCharacteristic = c;
        } else if (c.uuid == manualModeChar) {
          _manualModeCharacteristic = c;
        } else if (c.uuid == frontWheelOffsetChar) {
          _frontWheelOffsetCharacteristic = c;
        } else if (c.uuid == sailOpeningChar) {
          _sailOpeningCharacteristic = c;
        } else if (c.uuid == targetWaypointChar) {
          _targetWaypointCharacteristic = c;
        }
      }
    }

    if (_windSpeedCharacteristic == null || _windDirectionCharacteristic == null) {
      throw StateError('Required wind characteristics (0x2A72 / 0x2A73) were not found.');
    }

    await _subscribeCharacteristic(
      _windSpeedCharacteristic!,
      onData: (value) {
        final raw = _readUint16LittleEndian(value);
        if (raw == null) return;

        _latestWindData = _latestWindData.copyWith(
          windSpeedMs: raw / 100.0,
          timestamp: DateTime.now(),
        );
        _dataController.add(
          BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
        );
      },
    );

    await _subscribeCharacteristic(
      _windDirectionCharacteristic!,
      onData: (value) {
        final raw = _readUint16LittleEndian(value);
        if (raw == null) return;

        _latestWindData = _latestWindData.copyWith(
          windDirectionDeg: raw / 100.0,
          timestamp: DateTime.now(),
        );
        _dataController.add(
          BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
        );
      },
    );

    if (_batteryCharacteristic != null) {
      await _subscribeCharacteristic(
        _batteryCharacteristic!,
        onData: (value) {
          if (value.isEmpty) return;

          _latestWindData = _latestWindData.copyWith(
            batteryPercent: value.first,
            timestamp: DateTime.now(),
          );
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }

    if (_latitudeCharacteristic != null) {
      await _subscribeCharacteristic(
        _latitudeCharacteristic!,
        onData: (value) {
          final raw = _readInt32LittleEndian(value);
          if (raw == null) return;

          final latitude = raw * 1e-7;
          _latestGpsData = _latestGpsData.copyWith(latitude: latitude, timestamp: DateTime.now());
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }

    if (_longitudeCharacteristic != null) {
      await _subscribeCharacteristic(
        _longitudeCharacteristic!,
        onData: (value) {
          final raw = _readInt32LittleEndian(value);
          if (raw == null) return;

          final longitude = raw * 1e-7;
          _latestGpsData = _latestGpsData.copyWith(longitude: longitude, timestamp: DateTime.now());
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }

    if (_altitudeCharacteristic != null) {
      await _subscribeCharacteristic(
        _altitudeCharacteristic!,
        onData: (value) {
          final raw = _readInt32LittleEndian(value);
          if (raw == null) return;

          _latestGpsData = _latestGpsData.copyWith(
            altitudeM: raw / 1000.0,
            timestamp: DateTime.now(),
          );
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }

    if (_headingCharacteristic != null) {
      await _subscribeCharacteristic(
        _headingCharacteristic!,
        onData: (value) {
          final raw = _readUint16LittleEndian(value);
          if (raw == null) return;

          _latestNavigationData = _latestNavigationData.copyWith(
            headingDeg: raw / 100.0,
            timestamp: DateTime.now(),
          );
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }

    if (_gpsAccuracyCharacteristic != null) {
      await _subscribeCharacteristic(
        _gpsAccuracyCharacteristic!,
        onData: (value) {
          final raw = _readUint16LittleEndian(value);
          if (raw == null) return;

          _latestGpsData = _latestGpsData.copyWith(
            accuracyM: raw / 100.0,
            timestamp: DateTime.now(),
          );
          _dataController.add(
            BleData(wind: _latestWindData, gps: _latestGpsData, navigation: _latestNavigationData),
          );
        },
      );
    }
  }

  Future<void> setControlMode(BleControlMode mode) async {
    await _writeUint8(_manualModeCharacteristic, mode.value);
  }

  Future<void> setFrontWheelOffset(double percent) async {
    final clamped = percent.round().clamp(0, 100);
    final characteristic = _frontWheelOffsetCharacteristic;
    if (characteristic == null) {
      throw StateError('Manual control characteristic is not available.');
    }

    await _writeUint8(characteristic, clamped);
  }

  Future<void> setSailOpeningPercent(double percent) async {
    final clamped = percent.round().clamp(0, 100);
    await _writeUint8(_sailOpeningCharacteristic, clamped);
  }

  Future<void> setTargetWaypoint(double latitude, double longitude) async {
    final characteristic = _targetWaypointCharacteristic;
    if (characteristic == null) {
      throw StateError('Target waypoint characteristic is not available.');
    }

    final byteData = ByteData(8);
    byteData.setInt32(0, (latitude * 1e7).round(), Endian.little);
    byteData.setInt32(4, (longitude * 1e7).round(), Endian.little);
    await _writeBytes(characteristic, byteData.buffer.asUint8List());
  }

  Future<void> _subscribeCharacteristic(
    BluetoothCharacteristic characteristic, {
    required void Function(List<int> value) onData,
  }) async {
    if (characteristic.properties.notify || characteristic.properties.indicate) {
      await characteristic.setNotifyValue(true);
      final sub = characteristic.lastValueStream.listen(
        onData,
        onError: (e) {
          _emitError('Characteristic ${characteristic.uuid.str} error: $e');
        },
      );
      _notifySubs.add(sub);
    }

    if (characteristic.properties.read) {
      final value = await characteristic.read();
      onData(value);
    }
  }

  Future<void> _writeUint8(BluetoothCharacteristic? characteristic, int value) async {
    if (characteristic == null) {
      throw StateError('Manual control characteristic is not available.');
    }

    await _writeBytes(characteristic, [value & 0xFF]);
  }

  Future<void> _writeBytes(BluetoothCharacteristic characteristic, List<int> value) async {
    if (!characteristic.properties.write && !characteristic.properties.writeWithoutResponse) {
      throw StateError('Characteristic ${characteristic.uuid.str} is not writable.');
    }

    await characteristic.write(value, withoutResponse: false);
  }

  int? _readUint16LittleEndian(List<int> value) {
    if (value.length < 2) {
      return null;
    }

    final bytes = Uint8List.fromList(value.sublist(0, 2));
    return ByteData.sublistView(bytes).getUint16(0, Endian.little);
  }

  int? _readInt32LittleEndian(List<int> value) {
    if (value.length < 4) {
      return null;
    }

    final bytes = Uint8List.fromList(value.sublist(0, 4));
    return ByteData.sublistView(bytes).getInt32(0, Endian.little);
  }

  void _emitState(BleConnectionState state) {
    if (!_stateController.isClosed) {
      _stateController.add(state);
    }
  }

  void _emitError(String message) {
    if (!_errorController.isClosed) {
      _errorController.add(message);
    }
  }
}
