# PM100 Configurator - Desktop Architecture & Context

This document serves as the high-level system architecture, design blueprint, and development guide for the **PM100 Power Limiter Configurator App**. It integrates the hardware specs (`hardware.md`), the core project goals (`projeto.md`), and the user interface requirements (`mockup.png`).

---

## 1. System Architecture Overview

The PM100 Configurator is a **desktop** application written in **Rust** using the **Dioxus** framework. It acts as the user-facing interface for the PM100 smart power limiter hardware, communicating over a USB CDC ACM virtual serial port:

```
                 +-----------------------------------+
                 |        Dioxus App UI Core         |
                 |  - State: Dioxus Signals          |
                 |  - Desktop View                   |
                 +-----------------+-----------------+
                                   |
                   Polymorphic connection interface
                   (via Hardware Abstraction Trait)
                                   |
                                   v
                 +-----------------------------------+
                 |   USB CDC ACM Serial Port         |
                 |   Crate: serialport               |
                 +-----------------+-----------------+
                                   |
                            USB CDC ACM (10Hz)
                                   |
                                   v
                 +-----------------------------------+
                 |    PM100 MCU (nRF52840)           |
                 |  - Zephyr RTOS Firmware           |
                 |  - INA226 Power Sensor            |
                 +-----------------------------------+
```

---

## 2. Hardware Interface & Communication Protocol

### USB CDC ACM Console

The app connects to the PM100 via its Virtual COM Port (`cdc_acm_uart0`).

*   **Initialization:** The app opens the serial port (at 115200 bps) and must send the shell command:
    ```bash
    stream on\n
    ```
    This triggers the PM100 firmware to continuously output telemetry CSV lines at 10Hz.
*   **Data Parsing:** Telemetry is received as a UTF-8 CSV string:
    ```csv
    <time_ms>,<power_w>,<current_a>,<voltage_v>,<total_consumption_j>,<pwm_input_us>,<pwm_output_us>,<pwm_control_us>
    ```
*   **Config Writing:** To write settings, the app sends CLI commands followed by a newline:
    *   **ADRC Gains:** `update_adrc_gains <dt> <wo> <b0> <kp> <kd>` (e.g. `update_adrc_gains 0.002 100 1.0 1.2 0.1\n`)
    *   **Power Target:** `target <power_watts>`
    *   **Team Configuration:** `set_team <number> <name>`
    *   **6-Digit PIN Code:** `set_pin <pin_code>`

---

## 3. Best-Practice Rust Design

To share code across connection implementations, we use a **Polymorphic Connection Pattern** (using a Rust `Trait`). This separates the UI layout from the hardware communication library.

### A. The `DeviceConnection` Trait
Define this trait in a shared module (`src/connection/mod.rs`):

```rust
use async_trait::async_trait;
use tokio::sync::mpsc::Receiver;

#[derive(Debug, Clone, Default)]
pub struct TelemetryData {
    pub time_ms: u32,
    pub power_w: f32,
    pub current_a: f32,
    pub voltage_v: f32,
    pub total_consumption_j: f32,
    pub pwm_input_us: u16,
    pub pwm_output_us: u16,
    pub pwm_control_us: u16,
}

#[derive(Debug, Clone)]
pub struct DeviceConfigData {
    pub dt: f32,
    pub wo: f32,
    pub b0: f32,
    pub kp: f32,
    pub kd: f32,
    pub target_power: f32,
    pub team_name: String,
    pub team_number: u32,
    pub pin_code: String,
}

#[async_trait]
pub trait DeviceConnection: Send + Sync {
    // Connection Lifecycle
    async fn connect(&self) -> Result<(), String>;
    async fn disconnect(&self) -> Result<(), String>;
    async fn is_connected(&self) -> bool;

    // Telemetry Interface
    // Returns a receiver channel for real-time 10Hz updates
    async fn subscribe_telemetry(&self) -> Result<Receiver<TelemetryData>, String>;

    // Configuration / Writes
    async fn update_adrc_gains(&self, dt: f32, wo: f32, b0: f32, kp: f32, kd: f32) -> Result<(), String>;
    async fn update_target_power(&self, target_watts: f32) -> Result<(), String>;
    async fn update_team_config(&self, team_number: u32, team_name: &str) -> Result<(), String>;
    async fn update_pin_code(&self, pin_code: &str) -> Result<(), String>;
    async fn trigger_blink(&self) -> Result<(), String>;
}
```

### B. Concrete Implementations

1.  **`UsbConnection` (Desktop):**
    *   Uses the `serialport` crate to open the virtual COM port.
    *   Spawns a background thread that reads the serial buffer line-by-line, parses incoming CSV strings, and pushes parsed `TelemetryData` to the telemetry channel.
    *   Writes commands as formatted CLI text (e.g. `format!("target {}\n", target_watts)`).
2.  **`MockConnection` (Simulation / Offline Development):**
    *   *Extremely critical for rapid prototyping.*
    *   Does not talk to hardware; instead, it runs an asynchronous timer loop simulating random/smooth walk voltages, currents, and control parameters (e.g., simulating motor revving and power drops).
    *   Allows 100% of UI design, layout, forms, animations, and stream plotting to be built and verified offline without physical MCU units.

---

## 4. Concurrency & State Management in Dioxus

To maintain an ultra-smooth **60 FPS interface** while dealing with rapid 10Hz serial telemetry, we must **never** perform hardware polling or blocking operations on Dioxus's UI rendering thread.

### Concurrency Pipeline Blueprint
```
+-------------------------------------------------------------+
|                     BACKGROUND TASK                         |
|  - Continuous USB CDC Serial Reader                         |
|  - Receives Raw Packets -> Maps to TelemetryData struct     |
+------------------------------+------------------------------+
                               |
                   Rust async channel (mpsc)
                               |
                               v
+-------------------------------------------------------------+
|                   DIOXUS COROUTINE (Task)                   |
|  - Spawns async loop using `spawn()` in Dioxus              |
|  - Awaits channel `.recv().await`                           |
|  - Updates global UI signals atomically                     |
+------------------------------+------------------------------+
                               |
                     Dioxus Signal Writes
                               |
                               v
+-------------------------------------------------------------+
|                        RENDER TREE                          |
|  - Re-renders ONLY the telemetry components (fast path)     |
|  - Pushes metrics to dynamic Stream Data Plot               |
+-------------------------------------------------------------+
```

### Reactive UI State Design
Use **Dioxus Signals** (`use_signal`) to hold active state:
*   `active_telemetry`: Stores the latest `TelemetryData` containing current power, current, voltage, and uptime.
*   `connection_status`: Represents enum states `Disconnected`, `Connecting`, or `Connected`.
*   `telemetry_history`: A double-ended queue (`VecDeque`) storing the last $N$ telemetry data points (typically last 100-200 frames) used to feed the stream chart dynamically.

---

## 5. UI Layout & Component Organization

### Desktop View
*   **Stream Data Plot (Left Column - 60% Width):** Uses an interactive, high-performance SVG viewport or specialized UI chart rendering the history queue (`telemetry_history`). Displays curves for the PWM signals, real-time power, max power, and energy in real-time.
*   **Live Metrics Panel (Center Column - 20% Width):** Vertically stacked typography presenting Live Power ($W$), Max Power ($W$), Voltage ($V$), Current ($A$), and Joules ($J$).
*   **Analog Levels (Right Column - 20% Width):** Three dynamic, colored progress bars representing raw input control levels:
    *   `IN`: Input throttle signal ($PWM_{in}$)
    *   `CTRL`: Controller calculated limits ($PWM_{ctrl}$)
    *   `OUT`: Active output throttle signal sent to ESC ($PWM_{out}$)
*   **Bottom Config Row:** Form inputs for ADRC coefficients (4 active parameters like $w_o, b_0, K_p, K_d$) and Team Data (Number, Name), alongside the security PIN change form.

---

## 6. Prototyping Checklist & Phase Map

### Phase 1: Shared Core & Abstraction
- [ ] Create `src/connection/mod.rs` defining `TelemetryData`, `DeviceConfigData` and `DeviceConnection` trait.
- [ ] Implement `MockConnection` with realistic sensor emulation (sinusoidal power signals, tracking responses to simulate ADRC action).

### Phase 2: Dioxus UI & Interactive Prototyping
- [ ] Create core UI shell with responsiveness built using standard CSS flex/grid structures.
- [ ] Implement custom Dioxus component for **Live Metrics** (optimized text updates).
- [ ] Build **Analog Levels** (vertical/horizontal CSS progress indicators).
- [ ] Design **Stream Data Plot** using standard Dioxus SVG elements (`<svg>` viewport drawing `<polyline>` or `<path>` paths mapped from `telemetry_history`).
- [ ] Build the forms (ADRC parameter sliders/inputs, Team setup) and wire them to the active Connection implementation.

### Phase 3: Desktop Native & Serial Communication
- [ ] Implement `UsbConnection` using `serialport` crate.
- [ ] Set up the background reader thread to monitor serial buffers and parse incoming CLI lines.
- [ ] Validate connection lifecycle: auto-discover Xiao BLE virtual COM ports by VID/PID, send `stream on`, and handle reconnections.
