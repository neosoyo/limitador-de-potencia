import 'package:flutter_test/flutter_test.dart';

import 'package:pm100_app/models/pm100_spec.dart';

void main() {
  group('Pm100 value formatting', () {
    test('voltage is scaled from millivolts', () {
      expect(formatPm100Value(Pm100Variable.voltage, [0x18, 0x2E]), '11.80 V');
    });

    test('current is scaled from milliamperes', () {
      expect(formatPm100Value(Pm100Variable.current, [0xD8, 0x27]), '10.20 A');
    });

    test('power is scaled from milliwatts', () {
      expect(
        formatPm100Value(Pm100Variable.power, [0xC0, 0x27, 0x09, 0x00]),
        '600.0 W',
      );
    });

    test('energy is shown in joules', () {
      expect(
        formatPm100Value(Pm100Variable.energy, [0xC2, 0x01, 0x00, 0x00]),
        '450 J',
      );
    });

    test('uptime is shown as total seconds', () {
      // 70000 ms = 70 s
      expect(
        formatPm100Value(Pm100Variable.uptime, [0x70, 0x11, 0x01, 0x00]),
        '70s',
      );
    });

    test('team number decodes uint32 little-endian', () {
      expect(
        formatPm100Value(Pm100Variable.teamNumber, [0xD2, 0x04, 0x00, 0x00]),
        '1234',
      );
    });

    test('control state maps the enum value to a short phrase', () {
      expect(formatPm100Value(Pm100Variable.controlState, [0]), 'Ready');
      expect(
        formatPm100Value(Pm100Variable.controlState, [1]),
        'Limiting power',
      );
      expect(
        formatPm100Value(Pm100Variable.controlState, [2]),
        'No input signal',
      );
      expect(formatPm100Value(Pm100Variable.controlState, [3]), 'No battery');
      expect(formatPm100Value(Pm100Variable.controlState, [4]), 'Blinking');
      expect(formatPm100Value(Pm100Variable.controlState, [9]), 'Unknown');
    });

    test('PWM signals decode three uint16 values', () {
      expect(
        formatPm100Value(Pm100Variable.pwmSignals, [
          0xDC, 0x05, // 1500
          0xB0, 0x04, // 1200
          0xDC, 0x05, // 1500
        ]),
        'IN 1500 µs · OUT 1200 µs · CTRL 1500 µs',
      );
    });

    test('decodePwmSignals exposes the three raw values', () {
      final pwm = decodePwmSignals([
        0xDC, 0x05, // 1500
        0xB0, 0x04, // 1200
        0xDC, 0x05, // 1500
      ]);
      expect(pwm, isNotNull);
      expect(pwm!.pwmIn, 1500);
      expect(pwm.pwmOut, 1200);
      expect(pwm.pwmCtrl, 1500);
    });

    test('decodePwmSignals returns null for short payloads', () {
      expect(decodePwmSignals([0xDC, 0x05]), isNull);
    });

    test('team name decodes null-terminated UTF-8', () {
      final bytes = 'TeamFalcon'.codeUnits.toList()..add(0);
      expect(formatPm100Value(Pm100Variable.teamName, bytes), 'TeamFalcon');
    });
  });

  group('PM100 device name pattern', () {
    test('matches the dynamic device name', () {
      expect(pm100DeviceNamePattern.hasMatch('TeamFalcon_123_F1A2'), isTrue);
      expect(pm100DeviceNamePattern.hasMatch('Team_000_A1B2'), isTrue);
      expect(pm100DeviceNamePattern.hasMatch('name_1_1d12'), isTrue);
      expect(pm100DeviceNamePattern.hasMatch('name_11_2d2f'), isTrue);
      expect(pm100DeviceNamePattern.hasMatch('TeamFalcon_1234_F1A2'), isTrue);
    });

    test('rejects unrelated names', () {
      expect(pm100DeviceNamePattern.hasMatch('iPhone'), isFalse);
      expect(pm100DeviceNamePattern.hasMatch(''), isFalse);
    });
  });

  group('Actual power calculation', () {
    test('computes watts from millivolts and milliamperes', () {
      expect(calculateActualPowerWatts(11800, 10200), closeTo(120.36, 0.0001));
      expect(calculateActualPowerWatts(12000, 500), closeTo(6.0, 0.0001));
    });

    test('decodeUint16 reads little-endian and handles short payloads', () {
      expect(decodeUint16([0x18, 0x2E]), 11800);
      expect(decodeUint16([0x18]), isNull);
      expect(decodeUint16([]), isNull);
    });
  });
}
