# Power Limiter (Limitador de Potência) - Project Specifications

This project implements a smart power limiter for RC aircraft electric propulsion using the nRF Connect SDK (NCS) / Zephyr RTOS framework. It runs on a Seeed Studio Xiao BLE (nRF52840) MCU, measuring power consumption via an INA226 sensor, processing an input PWM throttle signal, and regulating a PWM output signal to the Electronic Speed Controller (ESC) using an Active Disturbance Rejection Control (ADRC) law.

---

## Architecture Overview

The system is designed around **four concurrent threads** to handle hard real-time control, high-frequency telemetry, user feedback, and the high-rate binary data stream efficiently.

```
       +---------------------------------------------------------+
       |                  Thread 1: Control (1kHz)               |
       |  - Read INA226 Power        - Read Input PWM Pulse      |
       |  - Compute ADRC Law         - Safe Min-Limit PWM Out    |
       +-------+--------------------------------+----------------+
               |                                |
               | (Shared State,                 | (1 kHz samples +
               |  10 Hz poll)                   |  controller snapshot)
               v                                v
  +-----------------------------+   +-------------------------------------+
  |     Thread 2: Telemetry     |   |     Thread 4: Binary Stream         |
  |           (10 Hz)           |   |      (4 ms default period)          |
  |  - Format controller state  |   |  - Pack 44 B samples / 84 B meta    |
  |    to CSV, 9 fields         |   |    into framed binary packets       |
  |  - USB CDC ACM port 0       |   |  - USB CDC ACM port 1               |
  +--------------+--------------+   +----------------+--------------------+
                 |                                   |
                 | (Status & Error Flags)            | (binary frames)
                 v                                   v
  +-----------------------------+   +-------------------------------------+
  |    Thread 3: Status LED     |   |  Host: /dev/ttyACM1 (port 1)        |
  |           (4 Hz)            |   |  Desktop application (parser)       |
  +-----------------------------+   +-------------------------------------+
```

---

## 1. Thread Specifications

### Thread 1: Control Loop (1kHz / Period: 1ms)
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
    <time_ms>,<peak_power_w>,<current_a>,<voltage_v>,<total_consumption_j>,<pwm_input_us>,<pwm_output_us>,<pwm_control_us>,<control_state>
    ```
    Where `<peak_power_w>` is the peak power measured since boot, in Watts, and `<control_state>` is represented as an integer:
    * `0` = **`READY`**
    * `1` = **`LIMITING_POWER`**
    * `2` = **`ERROR_NO_INPUT`**
    * `3` = **`ERROR_NO_BATTERY`**
    * `4` = **`BLINK`**
    * `5` = **`LEARNING`**

    *Example Output:*
    ```csv
    34200,120.5,10.2,11.8,450.2,1500,1420,1420,0
    ```
*   Control Shell Features:
    *   Enable/disable the USB console's CSV stream via shell commands.
    *   Maintain Bluetooth BLE transmission and physical UART streaming at the same 10Hz rate.
*   **Transport note:** the 10 Hz CSV stream above is a *human-readable convenience* output. The high-rate machine-readable telemetry is produced by **Thread 4** on a **second USB CDC ACM port** (see §4); the two paths are fully independent.

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
| **LEARNING** | `5` | ADRC b0 learning mode active ($b_0 = 0$) | **Orange** (e.g., `#FF7F00`) | **4 Hz** | Blinks every 250ms (125ms ON, 125ms OFF) |
### Thread 4: Binary Telemetry Stream (4ms default period / 250Hz framing)
*   **Purpose:** Ship *every* 1 kHz control-loop sample plus the ADRC internals to a desktop application without disturbing the shell or the control loop.
*   **Inputs:** Lock-free single-producer/single-consumer (SPSC) ring buffer filled by Thread 1.
*   **Behaviour:**
    1.  Thread 1 converts each control tick to a 44-byte fixed-point sample and pushes it into an 8 KiB ring (two small `memcpy`s, no formatting, no allocation).
    2.  Thread 4 wakes every `period_ms` (default 4 ms), drains the ring, and emits one frame of up to 48 samples.
    3.  Frames are handed to the port with a single `uart_fifo_fill()` call, i.e. one call per frame instead of one work-item per byte (which is what the `printf`/console path costs).
    4.  A controller/configuration snapshot (84 bytes, 1 Hz *and* after every parameter change) is interleaved between sample frames so a capture is self-describing.
*   **Scheduling:** `K_PRIO_PREEMPT(8)`, 2048-byte stack. Thread 1 is never blocked by the stream: if the host stalls, the ring simply overflows and the drop is counted.
*   **Details:** see §4.

**Latency/bandwidth trade-off:** `period_ms` sets how many samples share one frame. Total bandwidth stays between ~44 and ~52 kB/s regardless (the header is only 8 bytes), so the knob only trades frame count against latency — 1 ms gives ~1 ms latency at 1000 frames/s, 48 ms gives ~48 ms latency at ~21 frames/s.
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

#### `update_adrc_gains <wo> <b0> <kp> <kd>`
*   **Description:** Updates the active ADRC controller gains and saves them directly to NVS.
*   **Arguments:** Floating-point parameters for $w_o, b_0, K_p, K_d$. The sampling time step is fixed at 1 ms by the 1 kHz control loop.
*   **b0 Learning Mode:** Passing `0` as `<b0>` enables the identification mode. The controller passes the pilot throttle through (with a 1.2× target-power safety cap) while collecting moving averages of power in the 10-50% and 75-100% throttle bands. Three alternating throttle steps (low→high, high→low, low→high) are captured; each step's time constant $\tau_m$ is estimated with a log-linear least-squares fit, and the median of the three estimates is used. The controller then derives and saves $b_0 = \Delta P/(\tau_m^2 \Delta U)$, $w_c = 2/\tau_m$, $K_p = w_c^2$, $K_d = 2 w_c$, and $w_o = 5 w_c$. Inconsistent step estimates restart the learning automatically. The default `b0 = 0` triggers this on first boot.

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

#### `stream <on|off>` | `stream csv <on|off>`
*   **Description:** Activates or deactivates the 10 Hz real-time CSV telemetry streaming on the default console port (port 1). `stream csv <on|off>` is the explicit spelling of the same thing.

#### `stream bin <on|off> [period_ms]`
*   **Description:** Activates or deactivates the binary telemetry stream on the **second CDC ACM port** (port 2, see §4).
*   **Arguments:**
    *   `<on|off>`: enables or disables the stream. Normally open this port **before** enabling, otherwise the first frames are dropped.
    *   `[period_ms]`: optional frame period, `1`–`48`, default `4`. Because the stream is fed by the 1 kHz control loop, this is also the number of samples per frame (4 ms → 4 samples/frame at 250 frames/s).
*   **Behaviour:** Disabling only stops new samples; a frame already in flight is completed so the host never sees a truncated frame. Enabling starts from an empty ring, so the host sees current data, not a stale backlog.

#### `stream status`
*   **Description:** Prints the state of both streams plus the binary stream diagnostics: frame and sample counters (sent/dropped), mid-frame TX stalls and framing errors. Use it to tell "the host is not reading" (drops rising, stalls rising) from "the device is not producing" (counters flat).

#### `readings`
*   Description: Triggers a single print of the current telemetry values in CSV format.

---

## 4. Binary Telemetry Stream (Second USB CDC ACM Port)

### 4.1 Motivation and port topology

The console port must stay interactive, and in this build it is shared with *all* log output (the log backend is the shell backend, `CONFIG_SHELL_LOG_BACKEND=y`), which makes it line-oriented, DTR-gated and unsuitable for binary traffic. Instead of multiplexing binary data into it, the device instantiates a **second CDC ACM interface** and dedicates a port to telemetry.

| Property | Port 1 — console/shell | Port 2 — telemetry |
|---|---|---|
| Devicetree node | `board_cdc_acm_uart` | `stream_cdc_acm_uart` |
| USB interface numbers | 0 (control) + 1 (data) | 2 (control) + 3 (data) |
| Linux | `/dev/ttyACM0` | `/dev/ttyACM1` |
| Windows device instance | `...&MI_00` | `...&MI_02` |
| Carries | shell, `printf`, LOG, 10 Hz CSV | binary frames only |
| Shell available | yes | no |

Both interfaces belong to **one USB device**, therefore they share a single `idVendor`/`idProduct` (`CONFIG_USB_DEVICE_VID=0x2FE3`, `CONFIG_USB_DEVICE_PID=0x0100`) and one serial number; a VID/PID pair cannot be assigned per interface. Distinguish the ports by interface number:

```
# /etc/udev/rules.d/60-pm100.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2fe3", ATTRS{idProduct}=="0100", \
  ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="pm100_shell"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2fe3", ATTRS{idProduct}=="0100", \
  ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="pm100_stream"
```

Zephyr's `cdc_interface_config()` rebases the interface numbers per instance at build time, so the device enumerates as a single configuration with two IADs (interfaces 0-1 and 2-3). The device never reads the telemetry port, so the host may open it without asserting DTR. Baud rate is meaningless for both ports: USB CDC stores the host's line coding but no clock is derived from it.

### 4.2 Wire format

Little-endian, packed, no padding. `frame := header payload`.

**Header — 8 bytes**

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `u8` | `magic` | always `0xA5` |
| 1 | `u8` | `version` | currently `0x01` |
| 2 | `u8` | `type` | `1` = samples, `2` = controller snapshot |
| 3 | `u16` | `seq` | frame counter, +1 per frame, wraps at 65536 |
| 5 | `u8` | `count` | items in the payload (samples 1..48, snapshot 1) |
| 6 | `u16` | `len` | payload length in bytes |

**Payload type 1 — `pm100_stream_sample`, 44 bytes per item**

| Offset | Type | Field | Unit / scaling | Meaning |
|---|---|---|---|---|
| 0 | `u32` | `t_ms` | ms | uptime (wraps after ~49.7 days) |
| 4 | `i32` | `e_j` | J | accumulated energy |
| 8 | `i32` | `z1_mw` | mW | LESO $z_1$ — estimated power |
| 12 | `i32` | `z2_mws` | mW/s | LESO $z_2$ — estimated $\dot P$ |
| 16 | `i32` | `z3_mws` | mW/s | LESO $z_3$ — total disturbance |
| 20 | `u16` | `v_cv` | 10 mV (`2550` = 25.50 V) | bus voltage |
| 22 | `i16` | `i_ca` | 10 mA (`1250` = 12.50 A) | current |
| 24 | `i16` | `y_dw` | 0.1 W | measured power — controller input $y$ |
| 26 | `u16` | `pmax_w` | W | peak power since boot |
| 28 | `i16` | `tgt_dw` | 0.1 W | power target — reference $r$ |
| 30 | `u16` | `pwm_in` | µs | pilot throttle input |
| 32 | `u16` | `pwm_out` | µs | applied ESC pulse $\min(PWM_{in}, u)$ |
| 34 | `u16` | `pwm_ctrl` | µs | ADRC effort after clamping |
| 36 | `i16` | `pwm_ctrl_raw` | µs | ADRC effort **before** clamping (wind-up) |
| 38 | `u8` | `state` | enum | `ctrl_state`, same numbering as the CSV |
| 39 | `u8` | `flags` | bitfield | see below |
| 40 | `u8[4]` | `_rsvd` | — | reserved, keeps the item 4-byte aligned |

`flags` bits:

| Bit | Mask | Name | Meaning |
|---|---|---|---|
| 0 | `0x01` | `INPUT_VALID` | throttle pulse inside 900–2000 µs |
| 1 | `0x02` | `BATTERY_VALID` | bus voltage ≥ 5.0 V |
| 2 | `0x04` | `SAFE` | both of the above, so the control law ran |
| 3 | `0x08` | `LEARNING` | b0 identification mode active |
| 4 | `0x10` | `LEARN_POWER_CUT` | learning safety cap tripped |
| 5 | `0x20` | `ADRC_SATURATED` | raw effort was clamped to 1000–2000 µs |
| 6 | `0x40` | `SENSOR_ERROR` | INA226 read failed this tick (v/i forced to 0) |

> `z1_mw`, `z2_mws`, `z3_mws` and `pwm_ctrl_raw` are only meaningful on ticks where the control law actually ran (`SAFE` set and `LEARNING` clear); they hold the previous value otherwise.

**Payload type 2 — `pm100_stream_meta`, 84 bytes** (sent once per second *and* immediately after any parameter change)

| Offset | Type | Field | Unit | Notes |
|---|---|---|---|---|
| 0 | `f32` | `wo` | rad/s | observer bandwidth |
| 4 | `f32` | `b0` | — | controller input gain |
| 8 | `f32` | `kp` | — | proportional gain |
| 12 | `f32` | `kd` | — | derivative gain |
| 16 | `f32` | `l1` | — | $3 w_o$ |
| 20 | `f32` | `l2` | — | $3 w_o^2$ |
| 24 | `f32` | `l3` | — | $w_o^3$ |
| 28 | `f32` | `target_power` | W | active power target |
| 32 | `f32` | `shunt_mohm` | mΩ | configured shunt |
| 36 | `f32` | `dt` | s | control period |
| 40 | `u32` | `uptime_ms` | ms | when the snapshot was taken |
| 44 | `u32` | `team_number` | — | team / controller number |
| 48 | `char[32]` | `team_name` | — | NUL-padded |
| 80 | `u8` | `learning_stage` | enum | `learning_stage` |
| 81 | `u8[3]` | `_rsvd` | — | reserved |

**Reference Python decoder** (the sizes are enforced at compile time by the `BUILD_ASSERT`s in `src/stream.h` / `src/stream.c`):

```python
import struct

HDR    = struct.Struct("<BBBHBH")               # 8 B
SAMPLE = struct.Struct("<IiiiihhHhHHHhBB4x")    # 44 B
META   = struct.Struct("<10fII32sB3x")          # 84 B

TYPE_SAMPLES, TYPE_META = 1, 2
MAGIC, VERSION = 0xA5, 0x01

def parse(fh):
    """Yield (type, tuple) per frame; resynchronises on garbage."""
    buf = bytearray()
    while True:
        chunk = fh.read(4096)
        if not chunk:
            return
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                del buf[:max(0, len(buf) - 1)]   # keep a possibly split magic
                break
            del buf[:i]
            if len(buf) < HDR.size:
                break
            magic, ver, ftype, seq, count, plen = HDR.unpack_from(buf)
            item = {TYPE_SAMPLES: SAMPLE, TYPE_META: META}.get(ftype)
            if ver != VERSION or item is None or plen != count * item.size:
                del buf[:1]                       # false magic, resync
                continue
            if len(buf) < HDR.size + plen:
                break
            payload = buf[HDR.size:HDR.size + plen]
            for n in range(count):
                yield ftype, item.unpack_from(payload, n * item.size)
            del buf[:HDR.size + plen]
```

### 4.3 Host parser rules

1. Scan for `0xA5`; accept a frame only if `version == 1` and `len == count × item_size` — that is enough to resynchronise after any noise.
2. A jump in `seq` means whole **frames** were lost (e.g. the host stopped reading).
3. A jump in `t_ms` means individual **samples** were dropped on-device (ring overflow).
4. No per-frame CRC is needed: USB bulk already carries a link-layer CRC with retransmission. `magic` + `len` + `seq` cover framing and loss detection.

### 4.4 Rates, latency and bandwidth

| Metric | Value |
|---|---|
| Sample rate | 1000 samples/s (every 1 kHz control tick) |
| Default period | 4 ms → 4 samples/frame, 250 frames/s |
| Frame size (`period_ms = 4`) | 8 + 4 × 44 = 184 B |
| Sample stream | ≈46 kB/s (≈368 kbit/s, ≈3 % of Full-Speed USB) |
| Snapshot stream | 92 B/s (1 Hz) |
| Latency | ≈ `period_ms` + USB transfer + host read |
| Ring slack | 8 KiB ≈ 186 samples ≈ 186 ms of host stall absorbed |

Total bandwidth stays between ≈44 and ≈52 kB/s for any `period_ms` (the 8-byte header is negligible), so the knob only trades **latency against frame count**: `period_ms = 1` gives ≈1 ms latency at 1000 frames/s, `period_ms = 48` gives ≈48 ms at ≈21 frames/s.

For comparison, the 10 Hz CSV on the console port is ≈0.7 kB/s; streaming that same CSV at 1 kHz would be ≈70 kB/s *and* would be limited by `%f` software-double formatting rather than by the wire.

### 4.5 Build configuration

`prj.conf`:
```
CONFIG_USB_CDC_ACM_RINGBUF_SIZE=4096
```
The CDC ACM ring buffer size is global, i.e. it applies to TX **and** RX of **both** instances (16 kB of RAM). It is much larger than the largest frame (2120 B) so that a whole frame is normally accepted by a single `uart_fifo_fill()` call.

`boards/xiao_ble_nrf52840.overlay`:
```dts
&zephyr_udc0 {
	stream_cdc_acm_uart: stream_cdc_acm_uart {
		compatible = "zephyr,cdc-acm-uart";
	};
};
```

Endpoint budget: each CDC ACM instance takes 2 IN + 1 OUT → 4 IN + 2 OUT of the 7 IN / 7 OUT the nRF52840 USBD exposes (EP0 bidir aside).

### 4.6 Implementation map

| File | Role |
|---|---|
| `src/stream.h` | Wire format definition and public API — the reference for the desktop parser |
| `src/stream.c` | 8 KiB SPSC ring, record framing, frame assembly, `uart_fifo_fill()` TX, Thread 4 |
| `src/control.c` | Per-tick sample production, controller snapshot, `g_adrc.u_raw` |
| `src/main.c` | `stream_init()` call and shell commands |
| `boards/xiao_ble_nrf52840.overlay` | Second `zephyr,cdc-acm-uart` node |
| `prj.conf` | CDC ACM ring buffer size |
| `CMakeLists.txt` | Adds `src/stream.c` to the build |

### 4.7 Error handling

| Condition | Behaviour |
|---|---|
| Second port missing from the devicetree | `stream_init()` returns `-ENODEV`, stream stays disabled, everything else works |
| Host not reading the port | Frames are dropped (counted) instead of written partially, so the host never sees a truncated frame; the ring then overflows and samples are dropped (counted) |
| USB back-pressure mid-frame | The frame is completed before anything else is written, so frames stay frame-aligned |
| Ring misuse / torn record | Detected by the record length check, ring is reset by its owner, `corrupt_records` is bumped |

Diagnostics are exposed with `stream status` (see §3).

---

## 5. Bluetooth BLE Service Specification

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
| **Peak Power** | Peak Power (since boot) | `E20A1A08-473B-4444-9F6D-BE083A8BD92D` | `NOTIFY`, `READ` | `uint32_t` in Milliwatts (mW) representing the peak power measured since boot (e.g. $600.0\text{W} \rightarrow 600000$) *(Requires Encryption)* | 4 Bytes |
| **Control State** | Control State | `E20A1A07-473B-4444-9F6D-BE083A8BD92D` | `NOTIFY`, `READ` | `uint8_t` representing the `ctrl_state` enum value *(Requires Encryption)* | 1 Byte |

### Bandwidth Comparison & Optimization

*   **ASCII CSV Telemetry Stream:** ~55 bytes per packet $\times$ 10Hz = **550 bytes/second** (4400 bps).
*   **Split Binary Integer Stream:** 19 bytes total per packet $\times$ 10Hz = **190 bytes/second** (1520 bps).
*   **Net Bandwidth Saving:** **~65% reduction** in radio airtime, lowering power consumption and eliminating frame fragmentation. Static identification data (Team Name and Number) are read once on connection, avoiding unnecessary periodic bandwidth consumption.

---

## 6. Implementation Guidelines (AI Instructions)

1.  **Scope Restrictions:**
    *   Only modify files inside the `/src` folder.
    *   **Do not** make any modifications to files within `/src/lipe` or outside of `/src` (with the exception of verifying headers/interfaces).
    *   **Documented exception — binary telemetry stream (§4):** adding the second USB CDC ACM port necessarily touches build/configuration files, namely `boards/xiao_ble_nrf52840.overlay` (the new `zephyr,cdc-acm-uart` node), `prj.conf` (`CONFIG_USB_CDC_ACM_RINGBUF_SIZE`) and `CMakeLists.txt` (adding `src/stream.c`). Keep such edits to the minimum the feature requires and do not extend them to unrelated settings.
2.  **Documentation Standards:**
    *   Provide brief, clear documentation headers for all newly created functions.
    *   Follow the naming conventions already present in the codebase.
3.  **Concurrency & Safety:**
    *   Utilize Zephyr thread primitives (e.g., `K_THREAD_DEFINE`) to schedule the four threads.
    *   Implement basic thread safety (e.g., mutexes, atomic flags, or volatile variables) for variables shared across Thread 1, Thread 2, Thread 3, and Thread 4.
    *   Respect the producer/consumer contract of the telemetry stream: `stream_push_sample()` and `stream_push_meta()` may only be called from Thread 1 (the sole producer), the sample ring is a lock-free SPSC structure, and `ring_buf_reset()` may only run on the producer side (gated by `g_stream_reset_req`).
