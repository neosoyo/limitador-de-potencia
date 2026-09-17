use async_trait::async_trait;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::mpsc;

use crate::connection::{DeviceConfigData, DeviceConnection, DeviceStatus, TelemetryData};

/// Baud rate used by the PM100 USB CDC ACM console.
const BAUD_RATE: u32 = 115_200;

/// Zephyr shell prompt emitted by the PM100 console.
const SHELL_PROMPT: &str = "uart:~$ ";

/// USB Vendor ID of the PM100 (Zephyr CDC ACM console).
pub const PM100_USB_VID: u16 = 0x2FE3;

/// USB Product ID of the PM100 (Zephyr CDC ACM console).
pub const PM100_USB_PID: u16 = 0x0100;

/// Connection over the PM100 USB CDC ACM virtual serial port (Desktop).
///
/// The device exposes a Zephyr shell over its USB CDC ACM console. On
/// connect this driver rescans the USB devices, resolves the PM100 port by
/// matching its VID/PID, opens the serial port, and when telemetry
/// subscription starts it sends `stream on` and parses incoming CSV lines
/// into [`TelemetryData`] frames. Configuration is written back using the
/// actual shell commands discovered via `help`/`pm100 --help`.
pub struct UsbConnection {
    port_name: Mutex<String>,
    connected: Arc<AtomicBool>,
    port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
}

impl UsbConnection {
    /// Creates a serial connection for an explicit port (e.g. `/dev/ttyACM0` or `COM3`).
    pub fn new(port_name: &str) -> Self {
        Self {
            port_name: Mutex::new(port_name.to_string()),
            connected: Arc::new(AtomicBool::new(false)),
            port: Arc::new(Mutex::new(None)),
        }
    }

    /// Detects the PM100 serial port by matching its USB Vendor ID and
    /// Product ID (`PM100_USB_VID` / `PM100_USB_PID`). The device path
    /// (e.g. `/dev/ttyACM0`) is not assumed — it is resolved at connect time
    /// by VID/PID, so it stays correct even if the device re-enumerates.
    pub fn auto() -> Self {
        let port_name = detect_pm100_port().unwrap_or_else(fallback_port_name);
        Self::new(&port_name)
    }

    /// Returns the currently available serial devices (rescans the system).
    #[allow(dead_code)]
    pub fn available_devices() -> Vec<serialport::SerialPortInfo> {
        list_serial_ports()
    }

    /// Returns the configured serial port name.
    #[allow(dead_code)]
    pub fn port_name(&self) -> String {
        self.port_name
            .lock()
            .map(|name| name.clone())
            .unwrap_or_default()
    }

    /// Writes a raw shell command (terminated with `\r\n`) to the open serial port.
    fn write_command(&self, command: &str) -> Result<(), String> {
        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        let port = guard
            .as_mut()
            .ok_or_else(|| "Serial port is not open".to_string())?;

        port.write_all(command.as_bytes())
            .and_then(|_| port.write_all(b"\r\n"))
            .and_then(|_| port.flush())
            .map_err(|e| format!("Failed to write serial command: {}", e))
    }

    /// Sends a command and reads back the first shell line beginning with
    /// `Active ` (the human-readable config/read response).
    fn send_and_read_active(&self, command: &str) -> Result<String, String> {
        self.write_command(command)?;

        let deadline = Instant::now() + Duration::from_millis(3000);
        let mut line = String::new();
        let mut byte = [0u8; 1];

        loop {
            if Instant::now() >= deadline {
                return Err(format!("Timeout waiting for response to '{}'", command));
            }

            let read = {
                let mut guard = self.port.lock().map_err(|e| e.to_string())?;
                let port = guard
                    .as_mut()
                    .ok_or_else(|| "Serial port is not open".to_string())?;
                port.read(&mut byte)
            };

            match read {
                Ok(1) => {
                    if byte[0] == b'\n' {
                        let trimmed = line.trim();
                        if trimmed.starts_with("Active ") {
                            return Ok(trimmed.to_string());
                        }
                        line.clear();
                    } else if byte[0] != b'\r' {
                        line.push(byte[0] as char);
                    }
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(format!("Serial read error: {}", e)),
            }
        }
    }
}

/// Returns the current list of available serial ports.
fn list_serial_ports() -> Vec<serialport::SerialPortInfo> {
    serialport::available_ports().unwrap_or_default()
}

/// Resolves the PM100 serial port name by matching its USB VID/PID.
fn detect_pm100_port() -> Option<String> {
    list_serial_ports()
        .iter()
        .find(|port| is_pm100_usb(port))
        .map(|port| port.port_name.clone())
}

/// Returns a common CDC ACM port name used as a last-resort fallback.
fn fallback_port_name() -> String {
    #[cfg(target_os = "windows")]
    {
        "COM3".to_string()
    }
    #[cfg(not(target_os = "windows"))]
    {
        "/dev/ttyACM0".to_string()
    }
}

/// Returns true if the serial port belongs to the PM100 USB CDC ACM device.
fn is_pm100_usb(port: &serialport::SerialPortInfo) -> bool {
    matches!(
        &port.port_type,
        serialport::SerialPortType::UsbPort(info)
            if info.vid == PM100_USB_VID && info.pid == PM100_USB_PID
    )
}

/// Parses the `Active ADRC Gains: dt=..., w0=..., b0=..., kp=..., kd=...` response.
fn parse_gains(line: &str) -> Option<(f32, f32, f32, f32, f32)> {
    let body = line.split(':').nth(1)?;
    let mut dt = None;
    let mut wo = None;
    let mut b0 = None;
    let mut kp = None;
    let mut kd = None;

    for item in body.split(',') {
        let (key, value) = item.split_once('=')?;
        let value = value.trim().parse::<f32>().ok()?;
        match key.trim() {
            "dt" => dt = Some(value),
            "w0" | "wo" => wo = Some(value),
            "b0" => b0 = Some(value),
            "kp" => kp = Some(value),
            "kd" => kd = Some(value),
            _ => {}
        }
    }

    Some((dt?, wo?, b0?, kp?, kd?))
}

/// Parses the `Active Power Target: 600.00 W` response.
fn parse_target(line: &str) -> Option<f32> {
    line.split(':')
        .nth(1)?
        .trim()
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

/// Parses the `Active Team Name: <name>` response.
fn parse_team_name(line: &str) -> Option<String> {
    let name = line.split(':').nth(1)?.trim();
    if name.is_empty() {
        None
    } else {
        Some(name.to_string())
    }
}

/// Parses the `Active Team Number: <number>` response.
fn parse_team_number(line: &str) -> Option<u32> {
    line.split(':').nth(1)?.trim().parse().ok()
}

/// Parses the `Active BLE Pairing PIN: <pin>` response.
fn parse_pin(line: &str) -> Option<String> {
    let pin = line.split(':').nth(1)?.trim();
    if pin.is_empty() {
        None
    } else {
        Some(pin.to_string())
    }
}

/// Parses a PM100 CSV telemetry line into a [`TelemetryData`] frame.
///
/// Expected format (8 or 9 fields):
/// `<time_ms>,<power_w>,<current_a>,<voltage_v>,<total_consumption_j>,<pwm_input_us>,<pwm_output_us>,<pwm_control_us>[,<status>]`
///
/// The Zephyr shell prompt (`uart:~$ `) may be prepended to the first streamed
/// line, so it is stripped before parsing.
fn parse_telemetry_line(line: &str) -> Option<TelemetryData> {
    let trimmed = line.trim();
    let trimmed = trimmed.strip_prefix(SHELL_PROMPT).unwrap_or(trimmed).trim();

    let parts: Vec<&str> = trimmed.split(',').map(str::trim).collect();
    if parts.len() != 8 && parts.len() != 9 {
        return None;
    }

    let status = if parts.len() == 9 {
        parts[8]
            .parse::<u32>()
            .map(DeviceStatus::from_raw)
            .unwrap_or(DeviceStatus::Unknown)
    } else {
        DeviceStatus::Unknown
    };

    Some(TelemetryData {
        time_ms: parts[0].parse().ok()?,
        power_w: parts[1].parse().ok()?,
        current_a: parts[2].parse().ok()?,
        voltage_v: parts[3].parse().ok()?,
        total_consumption_j: parts[4].parse().ok()?,
        pwm_input_us: parts[5].parse().ok()?,
        pwm_output_us: parts[6].parse().ok()?,
        pwm_control_us: parts[7].parse().ok()?,
        status,
    })
}

#[async_trait]
impl DeviceConnection for UsbConnection {
    async fn connect(&self) -> Result<(), String> {
        if self.connected.load(Ordering::SeqCst) {
            return Ok(());
        }

        // Rescan the USB devices before connecting: the PM100 may have been
        // plugged in, unplugged, or re-enumerated on a different port since the
        // app started.
        let port_name = detect_pm100_port()
            .ok_or_else(|| "PM100 USB device not found. Check that it is connected.".to_string())?;

        let port = serialport::new(&port_name, BAUD_RATE)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| format!("Failed to open serial port {}: {}", port_name, e))?;

        {
            let mut guard = self.port.lock().map_err(|e| e.to_string())?;
            *guard = Some(port);
        }

        // Remember the resolved port for reference.
        {
            let mut name = self.port_name.lock().map_err(|e| e.to_string())?;
            *name = port_name;
        }

        self.connected.store(true, Ordering::SeqCst);

        // Blink the LED so the user can identify which physical device was
        // connected. This is the first shell command sent, and is best-effort:
        // a blink failure should not tear down an otherwise healthy connection.
        let _ = self.write_command("blink");

        Ok(())
    }

    async fn disconnect(&self) -> Result<(), String> {
        // Stop the streaming before dropping the port.
        let _ = self.write_command("stream off");

        self.connected.store(false, Ordering::SeqCst);

        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        *guard = None;

        Ok(())
    }

    async fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn subscribe_telemetry(&self) -> Result<mpsc::Receiver<TelemetryData>, String> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err("USB serial port is not connected".to_string());
        }

        // Enable the 10Hz CSV telemetry stream right before we start reading.
        self.write_command("stream on")?;

        let (tx, rx) = mpsc::channel(256);
        let port = Arc::clone(&self.port);
        let connected = Arc::clone(&self.connected);

        // Read incoming serial bytes on a dedicated OS thread. The PM100 emits
        // CSV telemetry at 10Hz, which is a light load, so byte-wise reading is
        // both simple and more than fast enough.
        std::thread::spawn(move || {
            let mut pending = String::new();
            let mut buf = [0u8; 256];

            while connected.load(Ordering::SeqCst) {
                let read_result = {
                    let mut guard = match port.lock() {
                        Ok(g) => g,
                        Err(_) => break,
                    };
                    let Some(serial) = guard.as_mut() else {
                        break;
                    };
                    serial.read(&mut buf)
                };

                match read_result {
                    Ok(n) if n > 0 => {
                        pending.push_str(&String::from_utf8_lossy(&buf[..n]));

                        // Emit every complete line as a telemetry frame.
                        while let Some(newline) = pending.find('\n') {
                            let line: String = pending.drain(..=newline).collect();
                            let trimmed = line.trim();
                            if trimmed.is_empty() {
                                continue;
                            }
                            if let Some(frame) = parse_telemetry_line(trimmed) {
                                if tx.blocking_send(frame).is_err() {
                                    return;
                                }
                            }
                        }
                    }
                    // Timeouts and transient errors are expected; keep listening.
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(_) => {
                        // Avoid a busy loop if the port is in a bad state.
                        std::thread::sleep(Duration::from_millis(5));
                    }
                }
            }
        });

        Ok(rx)
    }

    async fn update_adrc_gains(
        &self,
        dt: f32,
        wo: f32,
        b0: f32,
        kp: f32,
        kd: f32,
    ) -> Result<(), String> {
        self.write_command(&format!("pm100 gains {} {} {} {} {}", dt, wo, b0, kp, kd))
    }

    async fn update_target_power(&self, target_watts: f32) -> Result<(), String> {
        self.write_command(&format!("pm100 target {}", target_watts))
    }

    async fn update_team_config(&self, team_number: u32, team_name: &str) -> Result<(), String> {
        self.write_command(&format!("team_name {}", team_name))?;
        self.write_command(&format!("team_number {}", team_number))
    }

    async fn update_pin_code(&self, pin_code: &str) -> Result<(), String> {
        if pin_code.len() != 6 || !pin_code.chars().all(|c| c.is_ascii_digit()) {
            return Err("PIN code must be exactly 6 numeric digits".to_string());
        }
        self.write_command(&format!("ble_pin {}", pin_code))
    }

    async fn read_config(&self) -> Result<DeviceConfigData, String> {
        let gains_line = self.send_and_read_active("pm100 gains")?;
        let (dt, wo, b0, kp, kd) = parse_gains(&gains_line)
            .ok_or_else(|| format!("Failed to parse gains response: {}", gains_line))?;

        let target_line = self.send_and_read_active("pm100 target")?;
        let target_power = parse_target(&target_line)
            .ok_or_else(|| format!("Failed to parse target response: {}", target_line))?;

        let team_name_line = self.send_and_read_active("team_name")?;
        let team_name = parse_team_name(&team_name_line)
            .ok_or_else(|| format!("Failed to parse team name response: {}", team_name_line))?;

        let team_number_line = self.send_and_read_active("team_number")?;
        let team_number = parse_team_number(&team_number_line)
            .ok_or_else(|| format!("Failed to parse team number response: {}", team_number_line))?;

        let pin_line = self.send_and_read_active("ble_pin")?;
        let pin_code = parse_pin(&pin_line)
            .ok_or_else(|| format!("Failed to parse PIN response: {}", pin_line))?;

        Ok(DeviceConfigData {
            dt,
            wo,
            b0,
            kp,
            kd,
            target_power,
            team_name,
            team_number,
            pin_code,
        })
    }

    async fn trigger_blink(&self) -> Result<(), String> {
        // Triggers 10x white LED blinks on the device via the `blink` shell command.
        self.write_command("blink")
    }
}
