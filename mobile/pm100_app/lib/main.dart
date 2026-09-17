import 'package:dynamic_color/dynamic_color.dart';
import 'package:flutter/material.dart';

import 'screens/device_screen.dart';
import 'screens/scan_screen.dart';
import 'services/pm100_ble_controller.dart';

const Color _seedColor = Color(0xFF0066CC);

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  final controller = Pm100BleController()..init();
  final initialBrightness =
      WidgetsBinding.instance.platformDispatcher.platformBrightness;
  final themeMode = ValueNotifier(
    initialBrightness == Brightness.dark ? ThemeMode.dark : ThemeMode.light,
  );
  runApp(Pm100App(controller: controller, themeMode: themeMode));
}

class Pm100App extends StatelessWidget {
  const Pm100App({
    super.key,
    required this.controller,
    required this.themeMode,
  });

  final Pm100BleController controller;
  final ValueNotifier<ThemeMode> themeMode;

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<ThemeMode>(
      valueListenable: themeMode,
      builder: (context, mode, _) {
        return DynamicColorBuilder(
          builder: (lightDynamic, darkDynamic) {
            return MaterialApp(
              title: 'PM100 Power Limiter',
              debugShowCheckedModeBanner: false,
              theme: ThemeData(
                useMaterial3: true,
                colorScheme: lightDynamic ?? _fallbackScheme(Brightness.light),
              ),
              darkTheme: ThemeData(
                useMaterial3: true,
                colorScheme: darkDynamic ?? _fallbackScheme(Brightness.dark),
              ),
              themeMode: mode,
              home: HomeScreen(controller: controller, themeMode: themeMode),
            );
          },
        );
      },
    );
  }
}

ColorScheme _fallbackScheme(Brightness brightness) {
  return ColorScheme.fromSeed(seedColor: _seedColor, brightness: brightness);
}

/// Switches between the scan view and the connected-device view based on
/// controller state.
class HomeScreen extends StatelessWidget {
  const HomeScreen({
    super.key,
    required this.controller,
    required this.themeMode,
  });

  final Pm100BleController controller;
  final ValueNotifier<ThemeMode> themeMode;

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: controller,
      builder: (context, _) {
        if (controller.isReady && controller.device != null) {
          return DeviceScreen(controller: controller, themeMode: themeMode);
        }
        return ScanScreen(controller: controller, themeMode: themeMode);
      },
    );
  }
}
