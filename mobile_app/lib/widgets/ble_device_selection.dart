import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../ble/ble.dart';

class BleDeviceSelectionPage extends StatefulWidget {
  final WindBleClient bleClient;
  final Function(BluetoothDevice) onDeviceSelected;

  const BleDeviceSelectionPage({
    super.key,
    required this.bleClient,
    required this.onDeviceSelected,
  });

  @override
  State<BleDeviceSelectionPage> createState() => _BleDeviceSelectionPageState();
}

class _BleDeviceSelectionPageState extends State<BleDeviceSelectionPage> {
  bool _isScanning = false;
  late TextEditingController _searchController;
  String _searchQuery = '';

  @override
  void initState() {
    super.initState();
    _searchController = TextEditingController();
    _searchController.addListener(() {
      setState(() {
        _searchQuery = _searchController.text.toLowerCase();
      });
    });
    _startScanning();
  }

  @override
  void dispose() {
    _searchController.dispose();
    _stopScanning();
    super.dispose();
  }

  Future<void> _startScanning() async {
    if (_isScanning) return;
    setState(() => _isScanning = true);
    await widget.bleClient.startScanning();
  }

  Future<void> _stopScanning() async {
    await widget.bleClient.stopScanning();
    if (mounted) {
      setState(() => _isScanning = false);
    }
  }

  bool _deviceMatches(BluetoothDevice device, String query) {
    if (query.isEmpty) return true;
    final name = device.platformName.toLowerCase();
    final address = device.remoteId.str.toLowerCase();
    return name.contains(query) || address.contains(query);
  }

  void _connectToDevice(BluetoothDevice device) {
    _stopScanning();
    widget.onDeviceSelected(device);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Select BLE Device'),
        backgroundColor: const Color(0xFF5ECCC0),
        elevation: 0,
      ),
      body: Column(
        children: [
          Container(
            color: const Color(0xFF5ECCC0),
            padding: const EdgeInsets.all(16),
            child: Row(
              children: [
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text(
                        'Available Devices',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 16,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      const SizedBox(height: 4),
                      Text(
                        _isScanning ? 'Scanning...' : 'Tap to rescan',
                        style: const TextStyle(color: Colors.white70, fontSize: 12),
                      ),
                    ],
                  ),
                ),
                FloatingActionButton.small(
                  backgroundColor: Colors.white,
                  foregroundColor: const Color(0xFF5ECCC0),
                  onPressed: _isScanning ? null : _startScanning,
                  child: _isScanning
                      ? const SizedBox(
                          width: 20,
                          height: 20,
                          child: CircularProgressIndicator(
                            strokeWidth: 2,
                            valueColor: AlwaysStoppedAnimation<Color>(Color(0xFF5ECCC0)),
                          ),
                        )
                      : const Icon(Icons.refresh),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(16),
            child: TextField(
              controller: _searchController,
              decoration: InputDecoration(
                hintText: 'Search by name or MAC address',
                prefixIcon: const Icon(Icons.search),
                suffixIcon: _searchQuery.isNotEmpty
                    ? IconButton(
                        icon: const Icon(Icons.clear),
                        onPressed: () {
                          _searchController.clear();
                          setState(() => _searchQuery = '');
                        },
                      )
                    : null,
                border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
                contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              ),
            ),
          ),
          Expanded(
            child: StreamBuilder<List<ScanResult>>(
              stream: widget.bleClient.scanResults,
              builder: (context, snapshot) {
                if (!snapshot.hasData || snapshot.data!.isEmpty) {
                  return Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.bluetooth_disabled, size: 64, color: Colors.grey[300]),
                        const SizedBox(height: 16),
                        Text(
                          _isScanning ? 'Scanning for devices...' : 'No devices found',
                          style: TextStyle(fontSize: 16, color: Colors.grey[600]),
                        ),
                      ],
                    ),
                  );
                }

                // Deduplicate devices by MAC address
                final seenIds = <String>{};
                final uniqueDevices = <ScanResult>[];

                for (final result in snapshot.data!) {
                  final deviceId = result.device.remoteId.str;
                  if (!seenIds.contains(deviceId)) {
                    seenIds.add(deviceId);
                    uniqueDevices.add(result);
                  }
                }

                // Filter devices based on search query
                final filteredDevices = uniqueDevices
                    .where((result) => _deviceMatches(result.device, _searchQuery))
                    .toList();

                if (filteredDevices.isEmpty) {
                  return Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.search_off, size: 64, color: Colors.grey[300]),
                        const SizedBox(height: 16),
                        Text(
                          'No devices match "$_searchQuery"',
                          style: TextStyle(fontSize: 16, color: Colors.grey[600]),
                        ),
                      ],
                    ),
                  );
                }

                return ListView.builder(
                  itemCount: filteredDevices.length,
                  itemBuilder: (context, index) {
                    final result = filteredDevices[index];
                    final device = result.device;
                    final deviceId = device.remoteId.str;

                    return ListTile(
                      contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                      title: Text(
                        device.platformName.isEmpty ? 'Unknown Device' : device.platformName,
                        style: const TextStyle(fontWeight: FontWeight.bold),
                      ),
                      subtitle: Text(
                        deviceId,
                        style: TextStyle(
                          fontSize: 12,
                          color: Colors.grey[600],
                          fontFamily: 'monospace',
                        ),
                      ),
                      trailing: result.rssi != 0
                          ? Text(
                              '${result.rssi} dBm',
                              style: TextStyle(fontSize: 12, color: Colors.grey[600]),
                            )
                          : null,
                      onTap: () => _connectToDevice(device),
                    );
                  },
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
