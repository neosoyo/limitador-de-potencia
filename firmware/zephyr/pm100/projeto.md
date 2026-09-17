# Power Limiter (Limitador de Potência) - Project Specifications

This project implements a smart power limiter for RC aircraft electric propulsion using the nRF Connect SDK (NCS) / Zephyr RTOS framework. It runs on a Seeed Studio Xiao BLE (nRF52840) MCU, measuring power consumption via an INA226 sensor, processing an input PWM throttle signal, and regulating a PWM output signal to the Electronic Speed Controller (ESC) using an Active Disturbance Rejection Control (ADRC) law.

---

## Architecture Overview

The system is designed around **three concurrent threads** to handle hard real-time control, high-frequency telemetry, and user feedback efficiently.

```
       +--------------------------------------------------------+
       |                  Thread 1: Control (500Hz)             |
       |  - Read INA226 Power        - Read Input PWM Pulse     |
       |  - Compute ADRC Law         - Safe Min-Limit PWM Out   |
       +---------------------------+----------------------------+
                                   |
                                   | (Shared State / Variables)
                                   v
       +--------------------------------------------------------+
       |                 Thread 2: Telemetry (10Hz)             |
       |  - Format Controller State to CSV                      |
       |  - Output to Default Port (USB CDC ACM Console)       |
       +---------------------------+----------------------------+
                                   |
                                   | (Status & Error Flags)
                                   v
       +--------------------------------------------------------+
       |                  Thread 3: Status LED (4Hz)            |
       |  - Update WS2812 LED color and blink frequency         |
       +--------------------------------------------------------+
```

---

## 1. Thread Specifications

### Thread 1: Control Loop (500Hz / Period: 2ms)
*   **Purpose:** Perform real-time power limiting to prevent motor/battery overload while respecting the pilot's throttle commands.
*   **Control Logic:**
    1.  Read the active throttle input signal ($PWM_{in}$) in microseconds.
    2.  Read the current bus voltage, current, and power ($P_{meas}$) from the INA226 sensor.
    3.  Compute the ADRC control effort command ($u_{ctrl}$) to drive $P_{meas}$ toward the desired $P_{target}$.
    4.  Apply the output limit:
        $$PWM_{out} = \min(PWM_{in}, u_{ctrl})$$
        This guarantees that the controller only restricts power when exceeding the limit and never exceeds the pilot's requested throttle.
    5.  Update status flags (OK, Limiting, or Error) based on throttle input boundaries and control state.
*   **Scheduling:** Set with high preemptive or cooperative priority to ensure minimal jitter.

### Thread 2: Telemetry & Interface (10Hz / Period: 100ms)
*   Purpose: Format, update, and transmit device diagnostics over the communication interfaces (USB CDC ACM console, BLE, and a dedicated physical UART), and handle the BLE interface.
*   Outputs & Ports:
    1.  **USB CDC ACM Console:** The virtual COM port (`cdc_acm_uart0` / Console) of the Seeed Studio Xiao BLE. The CSV stream on this port is toggleable via the shell (activated with `stream on`).
    2.  **Dedicated Physical UART Output:** A physical UART TX output stream (the "third UART / auxiliary port" at 115200 bps, configured via hardware devicetree pins) that continuously transmits the CSV telemetry.
    3.  **Bluetooth BLE:** Broadcasts/transmits the same telemetry buffer packet at 10Hz.
*   Data Format (CSV):
    ```csv
    <time_ms>,<power_w>,<current_a>,<voltage_v>,<total_consumption_j>,<pwm_input_us>,<pwm_output_us>,<pwm_control_us>,<control_state>
    ```
    Where `<control_state>` is represented as an integer:
    * `0` = **`READY`**
    * `1` = **`LIMITING_POWER`**
    * `2` = **`ERROR_NO_INPUT`**
    * `3` = **`ERROR_NO_BATTERY`**
    * `4` = **`BLINK`**

    *Example Output:*
    ```csv
    34200,120.5,10.2,11.8,450.2,1500,1420,1420,0
    ```
*   Control Shell Features:
    *   Enable/disable the USB console's CSV stream via shell commands.
    *   Maintain Bluetooth BLE transmission and physical UART streaming at the same 10Hz rate.

### Thread 3: Status & Feedback LED (4Hz / Period: 250ms)
*   **Purpose:** Update the single WS2812 RGB LED to provide clear, real-time diagnostic states to the operator.
*   **State Machine & LED Indicators:**

| State | State Integer Value | Condition | LED Color | Blink Frequency | Blink Period |
|---|---|---|---|---|---|
| **READY** | `0` | System ready and healthy, fallback if other states are false | **Green** (e.g., `#00FF00`) | **1 Hz** | Blinks every 1s (500ms ON, 500ms OFF) |
| **LIMITING_POWER**| `1` | ADRC actively limiting power, $PWM_{out} < PWM_{in}$ | **Blue** (e.g., `#0000FF`) | **4 Hz** | Blinks every 250ms (125ms ON, 125ms OFF) |
| **ERROR_NO_INPUT** | `2` | $PWM_{in} < 900\,\mu\text{s}$ or $PWM_{in} > 2000\,\mu\text{s}$ | **Red** (e.g., `#FF0000`) | **2 Hz** | Blinks every 500ms (250ms ON, 250ms OFF) |
| **ERROR_NO_BATTERY** | `3` | $Voltage < 5.0\,\text{V}$ | **Red** (e.g., `#FF0000`) | **2 Hz** | Blinks every 500ms (250ms ON, 250ms OFF) |
| **BLINK** | `4` | Triggered via CLI or BLE, overrides all other states | **White** (e.g., `#FFFFFF`) | **5 Hz** | Blinks every 200ms (100ms ON, 100ms OFF) for 5 seconds |

---

## 2. Non-Volatile Storage (NVS) & Settings Subsystem Configuration

To ensure parameter persistence across power cycles, all device configuration variables and Bluetooth bonding keys are stored in the internal flash memory using Zephyr's unified **Settings Subsystem** backed by the Non-Volatile Storage (NVS) file system. This prevents double-mounting conflicts on the shared flash partition.

### Configuration Structure

All parameters are packed into a single serializable C struct:

```c
struct __attribute__((packed)) device_config {
    /* ADRC Controller Parameters */
    float dt;              // Controller sampling time step (seconds)
    float wo;              // Observer bandwidth (rad/s)
    float b0;              // Controller input gain scaling factor
    float kp;              // Proportional gain
    float kd;              // Derivative gain
    float target_power;    // Initial/Default Power Target in Watts

    /* Team & Identification Information */
    char team_name[32];    // Team name string (up to 32 bytes, null-terminated)
    uint32_t team_number;  // Unique identifier for the team / controller
    char PIN_code[7];      // 6-character security PIN (+1 byte for null terminator)
};
```

### Operational Rules
*   **Mounting and Storage:** On startup, the Settings Subsystem automatically mounts and manages the flash storage partition (typically `storage_partition`) using NVS. Both application parameters and Bluetooth stack state (bonding keys, CCCDs) share this unified storage.
*   **Loading Configuration:**
    *   The system initializes the settings backend via `settings_subsys_init()` and registers a static handler under the path `"app/device"`.
    *   `settings_load()` is called to retrieve and load the stored `device_config` structure.
    *   If no config is found (first-time boot), a set of sensible default values is written, and the device proceeds with these defaults.
*   **Writing Configuration:** Any update to the ADRC gains, target power, team info, or PIN via the shell triggers an immediate write back to persistent storage using `settings_save_one("app/device", ...)` to ensure durability.

---

## 3. Shell Interface Specifications

The shell interface is accessible over the default USB serial terminal. It supports the following commands:

### Configuration Commands

#### `update_adrc_gains <dt> <wo> <b0> <kp> <kd>`
*   **Description:** Updates the active ADRC controller gains and saves them directly to NVS.
*   **Arguments:** Floating-point parameters for $dt, w_o, b_0, K_p, K_d$.

#### `target <power_watts>`
*   **Description:** Sets the active power target limit.
*   **Arguments:** Floating-point power target in Watts. Saves to NVS immediately.

#### `set_team <number> <name>`
*   **Description:** Configures the team identification parameters.
*   **Arguments:**
    *   `<number>`: Unsigned integer team number.
    *   `<name>`: Character string (up to 31 characters) representing the team name.
*   **Storage:** Saves to NVS immediately.

#### `set_pin <pin_code>`
*   **Description:** Updates the device's 6-digit PIN code.
*   **Arguments:** A 6-character string representing the security PIN.
*   **Storage:** Saves to NVS immediately.

#### `stream <on|off>`
*   **Description:** Activates or deactivates the 10Hz real-time CSV telemetry streaming on the default console port.

#### `readings`
*   Description: Triggers a single print of the current telemetry values in CSV format.

---

## 4. Bluetooth BLE Service Specification

To optimize bandwidth and power consumption, the device exposes telemetry parameters using standard Bluetooth SIG 16-bit characteristics and compact, scaled binary integers instead of ASCII CSV strings. Custom metadata and raw signals employ dedicated 128-bit characteristics.

### Custom GATT Service

The PM100 Power Limiter GATT Service is defined by the following custom 128-bit Service UUID:
*   **PM100 Service UUID:** `E20A1A00-473B-4444-9F6D-BE083A8BD92D`

### BLE Link Encryption, Bonding & PIN-Passkey Security

For operator privacy and security, all telemetry characteristics require **Encrypted Link Pairing with Man-in-the-Middle (MITM) protection** and **Persistent Bonding**.

*   **Passkey Pairing:** The 6-digit PIN code configured in the storage `PIN_code` structure acts as the Bluetooth pairing password (Passkey).
*   **Encrypted Attributes:** All GATT characteristics are guarded with `BT_GATT_PERM_READ_ENCRYPT` permissions. If a client attempts to read or subscribe to telemetry without pairing first, the BLE stack automatically halts the request, triggers the numeric passcode entry on the client's screen, and encrypts the communication link.
*   **Persistent Bonding:** Pairing is upgraded to a permanent **Bond**. Pairing keys (bonding keys) and client configuration descriptor (CCCD) subscriptions are saved persistently to flash via the Settings Subsystem. When a previously paired client reconnects, the connection is encrypted automatically without prompting the user to enter the passkey again.
*   **Bond Capacity and Rotation:** To protect resource consumption in flash, the system maintains up to 5 paired devices. If the bond table is full and a new device attempts to bond, the Bluetooth stack automatically deletes and replaces the oldest bond keys (Least Recently Used) to prevent pairing failures.

### Dynamic Device Name Composition

The BLE device name is compiled dynamically at runtime during startup using the following pattern:
$$\text{Device Name} = \text{team\_name}\_\text{team\_number}\_\text{MAC\_last\_4}$$
*Example Compiled Name:* `TeamFalcon_1234_F1A2`

*   `team_name`: String loaded from NVS.
*   `team_number`: Unsigned integer loaded from NVS.
*   `MAC_last_4`: The last 4 hexadecimal characters of the Bluetooth MAC address of the Seeed Studio Xiao BLE.

### Telemetry Characteristics & Standard UUID Mapping

| Parameter | Standard GATT Name | UUID (16-bit or 128-bit) | Properties | Binary Format & Scaling | Bandwidth Size |
|---|---|---|---|---|---|
| **Voltage** | Voltage | `0x2B18` (16-bit) | `NOTIFY`, `READ` | `uint16_t` in Millivolts (mV) (e.g. $11.8\text{V} \rightarrow 11800$) *(Requires Encryption)* | 2 Bytes |
| **Current** | Current | `0x2B17` (16-bit) | `NOTIFY`, `READ` | `uint16_t` in Milliamperes (mA) (e.g. $10.2\text{A} \rightarrow 10200$) *(Requires Encryption)* | 2 Bytes |
| **Energy** | Energy (Total Cons.) | `0x2B06` (16-bit) | `NOTIFY`, `READ` | `uint32_t` in Joules (J) (e.g. $450.2\text{J} \rightarrow 450$) *(Requires Encryption)* | 4 Bytes |
| **Uptime** | Time ms | `0x2A2B` (16-bit) | `READ` | `uint32_t` in milliseconds (ms) from startup *(Requires Encryption)* | 4 Bytes |
| **PWM Signals** | PWM Signals State | `E20A1A03-473B-4444-9F6D-BE083A8BD92D` | `NOTIFY`, `READ` | Array of 3x `uint16_t` representing $PWM_{in}$, $PWM_{out}$, $PWM_{ctrl}$ in microseconds *(Requires Encryption)* | 6 Bytes |
| **Team Name** | Team Name | `E20A1A04-473B-4444-9F6D-BE083A8BD92D` | `READ` | UTF-8 encoded string (up to 32 bytes, null-terminated) *(Requires Encryption)* | Up to 32 Bytes |
| **Team Number** | Team Number | `E20A1A05-473B-4444-9F6D-BE083A8BD92D` | `READ` | `uint32_t` representing the unique team index *(Requires Encryption)* | 4 Bytes |
| **White Blink Cmd**| White Blink Trigger | `E20A1A06-473B-4444-9F6D-BE083A8BD92D` | `WRITE` | 1-byte Boolean (`0x01` to trigger 10x white blinks) *(Requires Encryption)* | 1 Byte |
| **Power** | Power | `E20A1A08-473B-4444-9F6D-BE083A8BD92D` | `NOTIFY`, `READ` | `uint32_t` in Milliwatts (mW) (e.g. $600.0\text{W} \rightarrow 600000$) *(Requires Encryption)* | 4 Bytes |
| **Control State** | Control State | `E20A1A07-473B-4444-9F6D-BE083A8BD92D` | `NOTIFY`, `READ` | `uint8_t` representing the `ctrl_state` enum value *(Requires Encryption)* | 1 Byte |

### Bandwidth Comparison & Optimization

*   **ASCII CSV Telemetry Stream:** ~55 bytes per packet $\times$ 10Hz = **550 bytes/second** (4400 bps).
*   **Split Binary Integer Stream:** 19 bytes total per packet $\times$ 10Hz = **190 bytes/second** (1520 bps).
*   **Net Bandwidth Saving:** **~65% reduction** in radio airtime, lowering power consumption and eliminating frame fragmentation. Static identification data (Team Name and Number) are read once on connection, avoiding unnecessary periodic bandwidth consumption.

---

## 5. Implementation Guidelines (AI Instructions)

1.  **Scope Restrictions:**
    *   Only modify files inside the `/src` folder.
    *   **Do not** make any modifications to files within `/src/lipe` or outside of `/src` (with the exception of verifying headers/interfaces).
2.  **Documentation Standards:**
    *   Provide brief, clear documentation headers for all newly created functions.
    *   Follow the naming conventions already present in the codebase.
3.  **Concurrency & Safety:**
    *   Utilize Zephyr thread primitives (e.g., `K_THREAD_DEFINE`) to schedule the three threads.
    *   Implement basic thread safety (e.g., mutexes, atomic flags, or volatile variables) for variables shared across Thread 1, Thread 2, and Thread 3.
