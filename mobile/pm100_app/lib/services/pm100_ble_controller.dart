import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import '../models/pm100_spec.dart';

/// Owns the BLE lifecycle for a single PM100 Power Limiter connection and
/// exposes device state + formatted telemetry values to the UI.
class Pm100BleController extends ChangeNotifier {
  BluetoothAdapterState adapterState = BluetoothAdapterState.unknown;
  bool isSupported = true;
  bool isScanning = false;
  bool isConnecting = false;
  bool isDiscovering = false;
  bool isReady = false;
  bool isBlinking = false;
  List<ScanResult> scanResults = [];
  BluetoothDevice? device;
  BluetoothConnectionState connectionState =
      BluetoothConnectionState.disconnected;
  String? error;

  final Map<Pm100Variable, String> _values = {};
  final Map<Pm100Variable, BluetoothCharacteristic> _chars = {};
  BluetoothCharacteristic? _blinkChar;
  PwmSignals? _pwmSignals;
  int? _voltageMv;
  int? _currentMa;

  final List<StreamSubscription<dynamic>> _globalSubs = [];
  final List<StreamSubscription<dynamic>> _telemetrySubs = [];

  bool get isConnected => connectionState == BluetoothConnectionState.connected;

  String valueFor(Pm100Variable variable) => _values[variable] ?? '—';

  /// Raw PWM values, exposed for the progress-bar UI.
  PwmSignals? get pwmSignals => _pwmSignals;

  /// Computed instantaneous power (voltage × current) in watts, or `null` if
  /// either input has not been received yet.
  double? get actualPowerWatts {
    final voltageMv = _voltageMv;
    final currentMa = _currentMa;
    if (voltageMv == null || currentMa == null) return null;
    return calculateActualPowerWatts(voltageMv, currentMa);
  }

  String get actualPowerLabel {
    final watts = actualPowerWatts;
    return watts == null ? '—' : '${watts.toStringAsFixed(1)} W';
  }

  void _updateValue(Pm100Variable variable, List<int> bytes) {
    _values[variable] = formatPm100Value(variable, bytes);
    if (variable == Pm100Variable.pwmSignals) {
      _pwmSignals = decodePwmSignals(bytes);
    } else if (variable == Pm100Variable.voltage) {
      _voltageMv = decodeUint16(bytes);
    } else if (variable == Pm100Variable.current) {
      _currentMa = decodeUint16(bytes);
    }
  }

  /// Starts listening to global BLE state. Call once during app startup.
  void init() {
    _globalSubs.add(
      FlutterBluePlus.adapterState.listen((state) {
        adapterState = state;
        notifyListeners();
      }),
    );

    FlutterBluePlus.isSupported.then((supported) {
      isSupported = supported;
      notifyListeners();
    });

    _globalSubs.add(
      FlutterBluePlus.isScanning.listen((scanning) {
        isScanning = scanning;
        notifyListeners();
      }),
    );

    _globalSubs.add(
      FlutterBluePlus.scanResults.listen((results) {
        scanResults = results.where(_matchesDeviceName).toList();
        notifyListeners();
      }),
    );
  }

  bool _matchesDeviceName(ScanResult result) {
    final advertised = result.advertisementData.advName;
    if (advertised.isNotEmpty) {
      return pm100DeviceNamePattern.hasMatch(advertised);
    }
    final platform = result.device.platformName;
    return platform.isNotEmpty && pm100DeviceNamePattern.hasMatch(platform);
  }

  Future<void> startScan() async {
    if (isScanning) return;
    clearError();
    scanResults = [];
    notifyListeners();
    try {
      await FlutterBluePlus.startScan(timeout: const Duration(seconds: 15));
    } catch (e) {
      error = 'Scan failed: $e';
      notifyListeners();
    }
  }

  Future<void> stopScan() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (_) {
      // Already stopped; nothing to do.
    }
  }

  Future<void> turnOnBluetooth() async {
    if (defaultTargetPlatform == TargetPlatform.android) {
      try {
        await FlutterBluePlus.turnOn();
        return;
      } catch (e) {
        error = 'Could not turn on Bluetooth: $e';
        notifyListeners();
        return;
      }
    }
    error = 'Please enable Bluetooth in system settings.';
    notifyListeners();
  }

  Future<void> connect(BluetoothDevice target) async {
    if (isConnecting) return;
    clearError();
    isConnecting = true;
    device = target;
    _resetTelemetryState();
    notifyListeners();

    final connectionSub = target.connectionState.listen((state) {
      connectionState = state;
      if (state == BluetoothConnectionState.disconnected) {
        isReady = false;
      }
      notifyListeners();
    });
    // Auto-cancel this subscription the next time the device disconnects.
    target.cancelWhenDisconnected(connectionSub, next: true);

    try {
      await target.connect(
        license: License.nonprofit,
        timeout: const Duration(seconds: 30),
        // We never exchange more than ~32 bytes, so skip the MTU request to
        // avoid an extra GATT operation right after connecting.
        mtu: null,
      );
      await _setupDevice(target);
    } catch (e) {
      error = 'Connection failed: $e';
    } finally {
      isConnecting = false;
      notifyListeners();
    }
  }

  Future<void> _setupDevice(BluetoothDevice target) async {
    isDiscovering = true;
    notifyListeners();
    try {
      final services = await target.discoverServices();
      _mapCharacteristics(services);
      await _activateTelemetry();
      isReady = true;
    } catch (e) {
      error = 'Reading telemetry failed: $e';
    } finally {
      isDiscovering = false;
      notifyListeners();
    }
  }

  void _mapCharacteristics(List<BluetoothService> services) {
    final byUuid = <Guid, BluetoothCharacteristic>{};
    for (final service in services) {
      for (final characteristic in service.characteristics) {
        byUuid[characteristic.uuid] = characteristic;
      }
    }

    for (final spec in pm100Variables) {
      final characteristic = byUuid[spec.uuid];
      if (characteristic != null) {
        _chars[spec.id] = characteristic;
      }
    }
    _blinkChar = byUuid[Pm100Uuids.whiteBlink];
  }

  Future<void> _activateTelemetry() async {
    _cancelTelemetrySubs();

    // Phase 1: subscribe to notifications. The PM100 firmware processes one
    // GATT operation at a time and replies with GATT_WRITE_REQUEST_BUSY if a
    // CCCD write arrives while it is still handling a previous operation, so
    // we retry busy writes and leave a short gap between each one.
    for (final spec in pm100Variables) {
      final characteristic = _chars[spec.id];
      if (characteristic == null) continue;

      final supportsNotify =
          characteristic.properties.notify ||
          characteristic.properties.indicate;
      if (!spec.notifies || !supportsNotify) continue;

      try {
        await _retryWrite(() => characteristic.setNotifyValue(true));
        _telemetrySubs.add(
          characteristic.onValueReceived.listen((bytes) {
            _updateValue(spec.id, bytes);
            notifyListeners();
          }),
        );
      } catch (_) {
        // Best effort: skip this subscription and keep setting up the rest.
      }
      await _sleep(const Duration(milliseconds: 100));
    }

    // Phase 2: read once for a current snapshot. Kept separate from the
    // subscribe phase so the device isn't hit with a read while it is still
    // finishing a CCCD write.
    for (final spec in pm100Variables) {
      final characteristic = _chars[spec.id];
      if (characteristic == null) continue;
      if (!spec.reads || !characteristic.properties.read) continue;

      try {
        final bytes = await characteristic.read();
        _updateValue(spec.id, bytes);
      } catch (_) {
        // An encrypted read may fail while pairing is still in progress; the
        // refresh button lets the user retry READ-only values.
      }
      await _sleep(const Duration(milliseconds: 60));
    }

    notifyListeners();
  }

  Future<void> _sleep(Duration duration) => Future<void>.delayed(duration);

  Future<void> _retryWrite(Future<void> Function() operation) async {
    const maxAttempts = 3;
    for (var attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        await operation();
        return;
      } catch (_) {
        if (attempt == maxAttempts) rethrow;
        await _sleep(Duration(milliseconds: 150 * attempt));
      }
    }
  }

  /// Re-reads all readable characteristics on demand.
  Future<void> refreshReads() async {
    if (device == null || !isReady) return;
    clearError();
    try {
      for (final spec in pm100Variables) {
        final characteristic = _chars[spec.id];
        if (characteristic == null ||
            !spec.reads ||
            !characteristic.properties.read) {
          continue;
        }
        final bytes = await characteristic.read();
        _updateValue(spec.id, bytes);
      }
      notifyListeners();
    } catch (e) {
      error = 'Refresh failed: $e';
      notifyListeners();
    }
  }

  /// Writes `0x01` to the White Blink characteristic to trigger 10 white blinks.
  Future<void> blink() async {
    final characteristic = _blinkChar;
    if (characteristic == null) {
      error = 'Blink characteristic not found on this device.';
      notifyListeners();
      return;
    }
    if (!characteristic.properties.write &&
        !characteristic.properties.writeWithoutResponse) {
      error = 'Blink characteristic is not writable.';
      notifyListeners();
      return;
    }

    clearError();
    isBlinking = true;
    notifyListeners();
    try {
      final useWithoutResponse =
          !characteristic.properties.write &&
          characteristic.properties.writeWithoutResponse;
      await characteristic.write([0x01], withoutResponse: useWithoutResponse);
    } catch (e) {
      error = 'Blink failed: $e';
    } finally {
      isBlinking = false;
      notifyListeners();
    }
  }

  Future<void> disconnect() async {
    final current = device;
    _resetTelemetryState();
    device = null;
    clearError();
    notifyListeners();
    if (current != null) {
      try {
        await current.disconnect();
      } catch (_) {
        // Already disconnected; nothing to do.
      }
    }
  }

  void _resetTelemetryState() {
    _cancelTelemetrySubs();
    _chars.clear();
    _values.clear();
    _blinkChar = null;
    _pwmSignals = null;
    _voltageMv = null;
    _currentMa = null;
    connectionState = BluetoothConnectionState.disconnected;
    isReady = false;
  }

  void _cancelTelemetrySubs() {
    for (final subscription in _telemetrySubs) {
      subscription.cancel();
    }
    _telemetrySubs.clear();
  }

  void clearError() {
    if (error != null) {
      error = null;
      notifyListeners();
    }
  }

  @override
  void dispose() {
    _cancelTelemetrySubs();
    for (final subscription in _globalSubs) {
      subscription.cancel();
    }
    super.dispose();
  }
}
