import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter_blue_plus/flutter_blue_plus.dart';

/// UUID constants for the PM100 Power Limiter GATT service.
///
/// Standard 16-bit Bluetooth SIG characteristics (Voltage, Current, Energy,
/// Uptime) are expressed against the Bluetooth Base UUID
/// `0000XXXX-0000-1000-8000-00805F9B34FB`, which [Guid] expands automatically
/// for 16-bit inputs. Custom characteristics use their full 128-bit UUID.
class Pm100Uuids {
  Pm100Uuids._();

  /// PM100 Power Limiter primary GATT service.
  static final Guid service = Guid('E20A1A00-473B-4444-9F6D-BE083A8BD92D');

  static final Guid voltage = Guid('2B18'); // Voltage (mV)
  static final Guid current = Guid('2B17'); // Current (mA)
  static final Guid energy = Guid('2B06'); // Energy, total consumption (J)
  static final Guid uptime = Guid('2A2B'); // Time ms (ms)
  static final Guid pwmSignals = Guid('E20A1A03-473B-4444-9F6D-BE083A8BD92D');
  static final Guid teamName = Guid('E20A1A04-473B-4444-9F6D-BE083A8BD92D');
  static final Guid teamNumber = Guid('E20A1A05-473B-4444-9F6D-BE083A8BD92D');
  static final Guid whiteBlink = Guid('E20A1A06-473B-4444-9F6D-BE083A8BD92D');
  static final Guid controlState = Guid('E20A1A07-473B-4444-9F6D-BE083A8BD92D');
  static final Guid power = Guid('E20A1A08-473B-4444-9F6D-BE083A8BD92D');
}

/// Matches the PM100 dynamic device name (`team_name_<team number>_…`).
///
/// Examples: `TeamFalcon_123_F1A2`, `name_1_1d12`, `name_11_2d2f`.
final RegExp pm100DeviceNamePattern = RegExp(r'(\w)+_([0-9]+)_');

/// The telemetry variables exposed by the PM100 device.
enum Pm100Variable {
  voltage,
  current,
  energy,
  uptime,
  pwmSignals,
  teamName,
  teamNumber,
  power,
  controlState,
}

/// Describes how a single PM100 characteristic is presented to the user.
class Pm100VariableSpec {
  const Pm100VariableSpec({
    required this.id,
    required this.uuid,
    required this.label,
    required this.notifies,
    required this.reads,
  });

  final Pm100Variable id;
  final Guid uuid;
  final String label;

  /// Whether the device pushes updates for this value via NOTIFY.
  final bool notifies;

  /// Whether the value can be read on demand.
  final bool reads;

  String format(List<int> bytes) => formatPm100Value(id, bytes);
}

/// Canonical display order for the telemetry list.
final List<Pm100VariableSpec> pm100Variables = [
  Pm100VariableSpec(
    id: Pm100Variable.voltage,
    uuid: Pm100Uuids.voltage,
    label: 'Voltage',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.current,
    uuid: Pm100Uuids.current,
    label: 'Current',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.power,
    uuid: Pm100Uuids.power,
    label: 'Maximum Power',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.energy,
    uuid: Pm100Uuids.energy,
    label: 'Energy',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.uptime,
    uuid: Pm100Uuids.uptime,
    label: 'Uptime',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.pwmSignals,
    uuid: Pm100Uuids.pwmSignals,
    label: 'PWM Signals',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.controlState,
    uuid: Pm100Uuids.controlState,
    label: 'Control State',
    notifies: true,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.teamName,
    uuid: Pm100Uuids.teamName,
    label: 'Team Name',
    notifies: false,
    reads: true,
  ),
  Pm100VariableSpec(
    id: Pm100Variable.teamNumber,
    uuid: Pm100Uuids.teamNumber,
    label: 'Team Number',
    notifies: false,
    reads: true,
  ),
];

/// Converts a raw characteristic payload into a human-readable string.
///
/// All multi-byte integers use little-endian encoding, matching the Bluetooth
/// GATT convention.
String formatPm100Value(Pm100Variable variable, List<int> bytes) {
  switch (variable) {
    case Pm100Variable.voltage:
      return '${(_uint16(bytes) / 1000).toStringAsFixed(2)} V';
    case Pm100Variable.current:
      return '${(_uint16(bytes) / 1000).toStringAsFixed(2)} A';
    case Pm100Variable.power:
      return '${(_uint32(bytes) / 1000).toStringAsFixed(1)} W';
    case Pm100Variable.energy:
      return '${_uint32(bytes)} J';
    case Pm100Variable.uptime:
      return '${_uint32(bytes) ~/ 1000}s';
    case Pm100Variable.pwmSignals:
      return _formatPwm(bytes);
    case Pm100Variable.teamName:
      return _utf8String(bytes);
    case Pm100Variable.teamNumber:
      return '${_uint32(bytes)}';
    case Pm100Variable.controlState:
      return bytes.isEmpty ? '—' : controlStatePhrase(bytes[0]);
  }
}

/// Maps the 1-byte `ctrl_state` enum value to a short user-facing phrase.
String controlStatePhrase(int value) {
  switch (value) {
    case 0:
      return 'Ready';
    case 1:
      return 'Limiting power';
    case 2:
      return 'No input signal';
    case 3:
      return 'No battery';
    case 4:
      return 'Blinking';
    default:
      return 'Unknown';
  }
}

int _uint16(List<int> bytes) {
  if (bytes.length < 2) return 0;
  return ByteData.sublistView(Uint8List.fromList(bytes))
      .getUint16(0, Endian.little);
}

int _uint32(List<int> bytes) {
  if (bytes.length < 4) return 0;
  return ByteData.sublistView(Uint8List.fromList(bytes))
      .getUint32(0, Endian.little);
}

String _utf8String(List<int> bytes) {
  if (bytes.isEmpty) return '';
  final terminator = bytes.indexOf(0);
  final text = terminator >= 0 ? bytes.sublist(0, terminator) : bytes;
  return utf8.decode(text, allowMalformed: true);
}

String _formatPwm(List<int> bytes) {
  if (bytes.length < 6) return '—';
  final pwmIn = _uint16(bytes.sublist(0, 2));
  final pwmOut = _uint16(bytes.sublist(2, 4));
  final pwmCtrl = _uint16(bytes.sublist(4, 6));
  return 'IN $pwmIn µs · OUT $pwmOut µs · CTRL $pwmCtrl µs';
}

/// Decoded PWM telemetry: 3 channels in microseconds (typical range 0–2000 µs).
class PwmSignals {
  const PwmSignals({
    required this.pwmIn,
    required this.pwmOut,
    required this.pwmCtrl,
  });

  final int pwmIn;
  final int pwmOut;
  final int pwmCtrl;
}

/// Decodes the 3×`uint16` PWM payload. Returns `null` if the payload is too
/// short or invalid.
PwmSignals? decodePwmSignals(List<int> bytes) {
  if (bytes.length < 6) return null;
  return PwmSignals(
    pwmIn: _uint16(bytes.sublist(0, 2)),
    pwmOut: _uint16(bytes.sublist(2, 4)),
    pwmCtrl: _uint16(bytes.sublist(4, 6)),
  );
}

/// Decodes a little-endian `uint16` payload, or `null` if it is too short.
int? decodeUint16(List<int> bytes) => bytes.length < 2 ? null : _uint16(bytes);

/// Computes instantaneous power in watts from raw millivolts and milliamperes.
double calculateActualPowerWatts(int voltageMv, int currentMa) {
  return voltageMv * currentMa / 1e6;
}
