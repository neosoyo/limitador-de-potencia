import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import '../services/pm100_ble_controller.dart';
import '../widgets/theme_toggle.dart';

/// Device discovery and connection UI.
class ScanScreen extends StatelessWidget {
  const ScanScreen({
    super.key,
    required this.controller,
    required this.themeMode,
  });

  final Pm100BleController controller;
  final ValueNotifier<ThemeMode> themeMode;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('PM100 Power Limiter'),
        actions: [ThemeModeToggle(themeMode: themeMode)],
      ),
      body: Column(
        children: [
          if (controller.error != null)
            _ErrorBanner(
              message: controller.error!,
              onDismiss: controller.clearError,
            ),
          Expanded(child: _buildBody(context)),
        ],
      ),
      floatingActionButton: _buildScanButton(),
    );
  }

  Widget? _buildScanButton() {
    if (controller.adapterState != BluetoothAdapterState.on ||
        controller.isConnecting) {
      return null;
    }
    return FloatingActionButton.extended(
      onPressed: controller.isScanning
          ? controller.stopScan
          : controller.startScan,
      icon: Icon(
        controller.isScanning ? Icons.stop : Icons.bluetooth_searching,
      ),
      label: Text(controller.isScanning ? 'Stop' : 'Scan'),
    );
  }

  Widget _buildBody(BuildContext context) {
    if (!controller.isSupported) {
      return const _CenteredMessage(
        icon: Icons.bluetooth_disabled,
        title: 'Bluetooth not supported',
        message: 'This device does not support Bluetooth Low Energy.',
      );
    }

    switch (controller.adapterState) {
      case BluetoothAdapterState.off:
      case BluetoothAdapterState.turningOff:
        return _BluetoothOff(onEnable: controller.turnOnBluetooth);
      case BluetoothAdapterState.turningOn:
        return const _CenteredMessage(
          icon: Icons.bluetooth,
          title: 'Turning Bluetooth on…',
          showSpinner: true,
        );
      case BluetoothAdapterState.unauthorized:
        return const _CenteredMessage(
          icon: Icons.block,
          title: 'Permission required',
          message: 'Bluetooth permission was denied. Enable it in system settings to scan.',
        );
      case BluetoothAdapterState.unavailable:
        return const _CenteredMessage(
          icon: Icons.bluetooth_disabled,
          title: 'Bluetooth unavailable',
          message: 'Bluetooth is not available on this device.',
        );
      case BluetoothAdapterState.unknown:
      case BluetoothAdapterState.on:
        break;
    }

    if (controller.isConnecting) {
      return _CenteredMessage(
        icon: Icons.bluetooth_connected,
        title: 'Connecting to ${_deviceName(controller.device!)}…',
        showSpinner: true,
      );
    }

    final results = List<ScanResult>.from(controller.scanResults)
      ..sort((a, b) => b.rssi.compareTo(a.rssi));

    if (results.isEmpty) {
      return _CenteredMessage(
        icon: Icons.radar,
        title: controller.isScanning
            ? 'Scanning for devices…'
            : 'No devices found',
        message: controller.isScanning
            ? 'Keep the PM100 powered on and nearby.'
            : 'Tap the Scan button to search for the PM100.',
        showSpinner: controller.isScanning,
      );
    }

    return ListView.separated(
      itemCount: results.length,
      separatorBuilder: (_, _) => const Divider(height: 1),
      itemBuilder: (context, index) {
        final result = results[index];
        return _DeviceTile(
          result: result,
          onTap: () => controller.connect(result.device),
        );
      },
    );
  }
}

String _deviceName(BluetoothDevice device) {
  final name = device.advName.isNotEmpty
      ? device.advName
      : device.platformName.isNotEmpty
      ? device.platformName
      : '';
  return name.isEmpty ? 'Unknown device' : name;
}

class _DeviceTile extends StatelessWidget {
  const _DeviceTile({required this.result, required this.onTap});

  final ScanResult result;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    final device = result.device;
    return ListTile(
      leading: const CircleAvatar(child: Icon(Icons.devices_other)),
      title: Text(_deviceName(device)),
      subtitle: Text(device.remoteId.str),
      trailing: Text(
        '${result.rssi} dBm',
        style: Theme.of(context).textTheme.bodySmall,
      ),
      onTap: onTap,
    );
  }
}

class _BluetoothOff extends StatelessWidget {
  const _BluetoothOff({required this.onEnable});

  final VoidCallback onEnable;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              Icons.bluetooth_disabled,
              size: 64,
              color: theme.colorScheme.primary,
            ),
            const SizedBox(height: 16),
            Text('Bluetooth is off', style: theme.textTheme.titleMedium),
            const SizedBox(height: 8),
            Text(
              'Enable Bluetooth to scan for the PM100 device.',
              textAlign: TextAlign.center,
              style: theme.textTheme.bodyMedium,
            ),
            const SizedBox(height: 24),
            FilledButton.icon(
              onPressed: onEnable,
              icon: const Icon(Icons.power_settings_new),
              label: const Text('Turn on Bluetooth'),
            ),
          ],
        ),
      ),
    );
  }
}

class _CenteredMessage extends StatelessWidget {
  const _CenteredMessage({
    required this.icon,
    required this.title,
    this.message,
    this.showSpinner = false,
  });

  final IconData icon;
  final String title;
  final String? message;
  final bool showSpinner;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            if (showSpinner)
              const CircularProgressIndicator()
            else
              Icon(icon, size: 64, color: theme.colorScheme.primary),
            const SizedBox(height: 16),
            Text(
              title,
              style: theme.textTheme.titleMedium,
              textAlign: TextAlign.center,
            ),
            if (message != null) ...[
              const SizedBox(height: 8),
              Text(
                message!,
                style: theme.textTheme.bodyMedium,
                textAlign: TextAlign.center,
              ),
            ],
          ],
        ),
      ),
    );
  }
}

class _ErrorBanner extends StatelessWidget {
  const _ErrorBanner({required this.message, required this.onDismiss});

  final String message;
  final VoidCallback onDismiss;

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    return Material(
      color: scheme.errorContainer,
      child: Padding(
        padding: const EdgeInsets.only(left: 16, right: 4, top: 4, bottom: 4),
        child: Row(
          children: [
            Icon(Icons.error_outline, color: scheme.onErrorContainer),
            const SizedBox(width: 8),
            Expanded(
              child: Text(
                message,
                style: TextStyle(color: scheme.onErrorContainer),
              ),
            ),
            IconButton(
              icon: Icon(Icons.close, color: scheme.onErrorContainer),
              onPressed: onDismiss,
            ),
          ],
        ),
      ),
    );
  }
}
