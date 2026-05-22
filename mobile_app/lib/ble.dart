import 'dart:async';
import 'dart:developer';
import 'dart:typed_data';

import 'package:flutter_blue_plus/flutter_blue_plus.dart';

/// Parsed values coming from the BLE wind peripheral.
class WindGattData {
  const WindGattData({
    required this.windSpeedMs,
    required this.windDirectionDeg,
    required this.batteryPercent,
    required this.receivedAt,
  });

  /// Apparent wind speed in m/s (source unit is 0.01 m/s).
  final double? windSpeedMs;

  /// Apparent wind direction in degrees (source unit is 0.01 degrees).
  final double? windDirectionDeg;

  /// Battery level in percent (0-100).
  final int? batteryPercent;

  final DateTime receivedAt;

  WindGattData copyWith({
    double? windSpeedMs,
    double? windDirectionDeg,
    int? batteryPercent,
    DateTime? receivedAt,
  }) {
    return WindGattData(
      windSpeedMs: windSpeedMs ?? this.windSpeedMs,
      windDirectionDeg: windDirectionDeg ?? this.windDirectionDeg,
      batteryPercent: batteryPercent ?? this.batteryPercent,
      receivedAt: receivedAt ?? this.receivedAt,
    );
  }
}

enum BleConnectionState { idle, scanning, connecting, connected, disconnected, error }

/// BLE manager for scanning, connecting and consuming wind GATT data.
class WindBleClient {
  static final Guid environmentalSensingService = Guid('0000181A-0000-1000-8000-00805F9B34FB');
  static final Guid batteryService = Guid('0000180F-0000-1000-8000-00805F9B34FB');

  static final Guid apparentWindSpeedChar = Guid('00002A72-0000-1000-8000-00805F9B34FB');
  static final Guid apparentWindDirectionChar = Guid('00002A73-0000-1000-8000-00805F9B34FB');
  static final Guid batteryLevelChar = Guid('00002A19-0000-1000-8000-00805F9B34FB');

  final _stateController = StreamController<BleConnectionState>.broadcast();
  final _dataController = StreamController<WindGattData>.broadcast();
  final _errorController = StreamController<String>.broadcast();

  Stream<BleConnectionState> get stateStream => _stateController.stream;
  Stream<WindGattData> get dataStream => _dataController.stream;
  Stream<String> get errorStream => _errorController.stream;

  BluetoothDevice? _device;
  BluetoothCharacteristic? _windSpeedCharacteristic;
  BluetoothCharacteristic? _windDirectionCharacteristic;
  BluetoothCharacteristic? _batteryCharacteristic;

  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<BluetoothConnectionState>? _deviceStateSub;
  final List<StreamSubscription<List<int>>> _notifySubs = <StreamSubscription<List<int>>>[];

  WindGattData _latest = WindGattData(
    windSpeedMs: null,
    windDirectionDeg: null,
    batteryPercent: null,
    receivedAt: DateTime.now(),
  );

  bool get isConnected => _device != null;
  BluetoothDevice? get device => _device;

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

        _latest = _latest.copyWith(windSpeedMs: raw / 100.0, receivedAt: DateTime.now());
        _dataController.add(_latest);
      },
    );

    await _subscribeCharacteristic(
      _windDirectionCharacteristic!,
      onData: (value) {
        final raw = _readUint16LittleEndian(value);
        if (raw == null) return;

        _latest = _latest.copyWith(windDirectionDeg: raw / 100.0, receivedAt: DateTime.now());
        _dataController.add(_latest);
      },
    );

    if (_batteryCharacteristic != null) {
      await _subscribeCharacteristic(
        _batteryCharacteristic!,
        onData: (value) {
          if (value.isEmpty) return;

          _latest = _latest.copyWith(batteryPercent: value.first, receivedAt: DateTime.now());
          _dataController.add(_latest);
        },
      );
    }
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

  int? _readUint16LittleEndian(List<int> value) {
    if (value.length < 2) {
      return null;
    }

    final bytes = Uint8List.fromList(value.sublist(0, 2));
    return ByteData.sublistView(bytes).getUint16(0, Endian.little);
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
