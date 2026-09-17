import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import '../models/pm100_spec.dart';
import '../services/pm100_ble_controller.dart';
import '../widgets/theme_toggle.dart';

/// Connected-device view: shows live telemetry and the "blink" command.
class DeviceScreen extends StatelessWidget {
  const DeviceScreen({
    super.key,
    required this.controller,
    required this.themeMode,
  });

  final Pm100BleController controller;
  final ValueNotifier<ThemeMode> themeMode;

  @override
  Widget build(BuildContext context) {
    final device = controller.device!;
    return Scaffold(
      appBar: AppBar(
        title: Text(_deviceName(device)),
        actions: [
          ThemeModeToggle(themeMode: themeMode),
          IconButton(
            icon: const Icon(Icons.link_off),
            tooltip: 'Disconnect',
            onPressed: controller.disconnect,
          ),
        ],
      ),
      body: Column(
        children: [
          if (controller.error != null)
            _ErrorBanner(
              message: controller.error!,
              onDismiss: controller.clearError,
            ),
          Expanded(
            child: ListView(
              padding: const EdgeInsets.all(16),
              children: [
                _HeaderCard(controller: controller),
                const SizedBox(height: 16),
                Text(
                  'Telemetry',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                ..._buildTelemetryItems(controller),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

List<Widget> _buildTelemetryItems(Pm100BleController controller) {
  final items = <Widget>[];
  for (final spec in pm100Variables) {
    if (spec.id == Pm100Variable.teamName ||
        spec.id == Pm100Variable.teamNumber) {
      continue; // Shown in the header card instead of the telemetry list.
    }
    if (spec.id == Pm100Variable.pwmSignals) {
      items.add(_PwmTile(controller: controller));
      continue;
    }
    if (spec.id == Pm100Variable.controlState) {
      items.add(
        _ControlStateTile(
          phrase: controller.valueFor(Pm100Variable.controlState),
        ),
      );
      continue;
    }
    items.add(_TelemetryTile(spec: spec, value: controller.valueFor(spec.id)));
    if (spec.id == Pm100Variable.power) {
      items.add(_ActualPowerTile(value: controller.actualPowerLabel));
    }
  }
  return items;
}

String _deviceName(BluetoothDevice device) {
  final name = device.advName.isNotEmpty
      ? device.advName
      : device.platformName.isNotEmpty
      ? device.platformName
      : '';
  return name.isEmpty ? 'PM100' : name;
}

String _teamSubtitle(String teamName, String teamNumber) {
  final parts = <String>[];
  if (teamName != '—') parts.add(teamName);
  if (teamNumber != '—') parts.add('Team #$teamNumber');
  return parts.isEmpty ? '—' : parts.join(' · ');
}

class _HeaderCard extends StatelessWidget {
  const _HeaderCard({required this.controller});

  final Pm100BleController controller;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final subtitle = _teamSubtitle(
      controller.valueFor(Pm100Variable.teamName),
      controller.valueFor(Pm100Variable.teamNumber),
    );
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(subtitle, style: theme.textTheme.titleMedium),
            const SizedBox(height: 16),
            FilledButton.icon(
              onPressed: controller.isBlinking ? null : controller.blink,
              icon: controller.isBlinking
                  ? const SizedBox(
                      width: 16,
                      height: 16,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    )
                  : const Icon(Icons.flash_on),
              label: Text(controller.isBlinking ? 'Blinking…' : 'Blink'),
            ),
          ],
        ),
      ),
    );
  }
}

class _PwmTile extends StatelessWidget {
  const _PwmTile({required this.controller});

  final Pm100BleController controller;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final pwm = controller.pwmSignals;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.sync, size: 20, color: theme.colorScheme.primary),
                const SizedBox(width: 12),
                Text('PWM Signals', style: theme.textTheme.titleMedium),
              ],
            ),
            const SizedBox(height: 16),
            _PwmBar(label: 'IN', value: pwm?.pwmIn),
            const SizedBox(height: 10),
            _PwmBar(label: 'OUT', value: pwm?.pwmOut),
            const SizedBox(height: 10),
            _PwmBar(label: 'CTRL', value: pwm?.pwmCtrl),
          ],
        ),
      ),
    );
  }
}

class _PwmBar extends StatelessWidget {
  const _PwmBar({required this.label, required this.value});

  final String label;
  final int? value;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final progress = value == null ? 0.0 : (value! / 2000.0).clamp(0.0, 1.0);
    return Row(
      children: [
        SizedBox(
          width: 44,
          child: Text(label, style: theme.textTheme.labelLarge),
        ),
        Expanded(
          child: ClipRRect(
            borderRadius: BorderRadius.circular(4),
            child: LinearProgressIndicator(value: progress, minHeight: 10),
          ),
        ),
        const SizedBox(width: 8),
        SizedBox(
          width: 76,
          child: Text(
            value == null ? '—' : '$value µs',
            textAlign: TextAlign.right,
            style: theme.textTheme.bodyMedium?.copyWith(
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
      ],
    );
  }
}

class _ControlStateTile extends StatelessWidget {
  const _ControlStateTile({required this.phrase});

  final String phrase;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Card(
      child: ListTile(
        leading: Icon(Icons.bolt, size: 20, color: theme.colorScheme.primary),
        title: Text(
          phrase,
          style: theme.textTheme.bodyLarge?.copyWith(
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }
}

class _ActualPowerTile extends StatelessWidget {
  const _ActualPowerTile({required this.value});

  final String value;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Card(
      child: ListTile(
        leading: Icon(
          Icons.calculate_outlined,
          size: 20,
          color: theme.colorScheme.primary,
        ),
        title: const Text('Actual Power'),
        trailing: Text(
          value,
          style: theme.textTheme.bodyLarge?.copyWith(
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }
}

class _TelemetryTile extends StatelessWidget {
  const _TelemetryTile({required this.spec, required this.value});

  final Pm100VariableSpec spec;
  final String value;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Card(
      child: ListTile(
        leading: spec.notifies
            ? Icon(Icons.sync, size: 20, color: theme.colorScheme.primary)
            : const Icon(Icons.info_outline, size: 20),
        title: Text(spec.label),
        trailing: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 180),
          child: Text(
            value,
            textAlign: TextAlign.right,
            style: theme.textTheme.bodyLarge?.copyWith(
              fontWeight: FontWeight.w600,
            ),
          ),
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
