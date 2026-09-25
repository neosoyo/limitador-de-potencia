use async_trait::async_trait;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::mpsc;

use crate::connection::{
    CsvRecorder, DeviceConfigData, DeviceConnection, DeviceStatus, TelemetryData,
};

/// Baud rate used by the PM100 USB CDC ACM console.
const BAUD_RATE: u32 = 115_200;

/// Zephyr shell prompt emitted by the PM100 console.
const SHELL_PROMPT: &str = "uart:~$ ";

/// USB Vendor ID of the PM100 (Zephyr CDC ACM console).
pub const PM100_USB_VID: u16 = 0x2FE3;

/// USB Product ID of the PM100 (Zephyr CDC ACM console).
pub const PM100_USB_PID: u16 = 0x0100;

/// Binary telemetry wire-format constants (see `docs/hardware.md` §4).
const STREAM_MAGIC: u8 = 0xA5;
const STREAM_VERSION: u8 = 0x01;
const STREAM_TYPE_SAMPLES: u8 = 1;
const STREAM_TYPE_META: u8 = 2;
const STREAM_HEADER_SIZE: usize = 8;
const STREAM_SAMPLE_SIZE: usize = 44;
const STREAM_META_SIZE: usize = 84;

/// Downsample factor applied to the 1 kHz binary stream before it reaches
/// the UI (1000 / 20 = 50 Hz), keeping the 30 s history window light.
const STREAM_DECIMATION: usize = 20;

/// Connection over the PM100 USB CDC ACM ports (Desktop).
///
/// The device exposes a Zephyr shell on the console port and a dedicated
/// high-rate binary telemetry stream on a second CDC ACM port. On connect this
/// driver resolves both ports by VID/PID. Telemetry subscription opens the
/// stream port, enables it with `stream bin on`, and parses the 44-byte fixed
/// point samples into [`TelemetryData`] frames; if the second port is absent it
/// falls back to the 10 Hz CSV stream on the shell port. Configuration is
/// written back using the shell commands.
pub struct UsbConnection {
    port_name: Mutex<String>,
    connected: Arc<AtomicBool>,
    port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
    stream_port_name: Mutex<String>,
    stream_port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
}

impl UsbConnection {
    /// Creates a serial connection for an explicit shell port (e.g. `/dev/ttyACM0`).
    pub fn new(port_name: &str) -> Self {
        Self {
            port_name: Mutex::new(port_name.to_string()),
            connected: Arc::new(AtomicBool::new(false)),
            port: Arc::new(Mutex::new(None)),
            stream_port_name: Mutex::new(String::new()),
            stream_port: Arc::new(Mutex::new(None)),
        }
    }

    /// Detects the PM100 serial ports by matching its USB Vendor ID and
    /// Product ID (`PM100_USB_VID` / `PM100_USB_PID`). The first (lowest
    /// numbered) port is the shell console, the second is the binary stream.
    /// The device path is not assumed — it is resolved at connect time by
    /// VID/PID, so it stays correct even if the device re-enumerates.
    pub fn auto() -> Self {
        let ports = detect_pm100_ports();
        let shell = ports.first().cloned().unwrap_or_else(fallback_port_name);
        let stream = ports.get(1).cloned().unwrap_or_default();
        let mut conn = Self::new(&shell);
        conn.stream_port_name = Mutex::new(stream);
        conn
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
    ///
    /// Holds the port lock across the whole write+read cycle so the shell-log
    /// reader thread cannot steal bytes mid-response.
    fn send_and_read_active(&self, command: &str) -> Result<String, String> {
        let deadline = Instant::now() + Duration::from_millis(3000);
        let mut line = String::new();
        let mut byte = [0u8; 1];

        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        let port = guard
            .as_mut()
            .ok_or_else(|| "Serial port is not open".to_string())?;

        port.write_all(command.as_bytes())
            .and_then(|_| port.write_all(b"\r\n"))
            .and_then(|_| port.flush())
            .map_err(|e| format!("Failed to write serial command: {}", e))?;

        loop {
            if Instant::now() >= deadline {
                return Err(format!("Timeout waiting for response to '{}'", command));
            }

            match port.read(&mut byte) {
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

/// Resolves all PM100 serial port names by matching their USB VID/PID,
/// sorted so the shell console (lowest interface number) comes first.
fn detect_pm100_ports() -> Vec<String> {
    let mut names: Vec<String> = list_serial_ports()
        .iter()
        .filter(|port| is_pm100_usb(port))
        .map(|port| port.port_name.clone())
        .collect();
    names.sort();
    names
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

/// Parses the `Active ADRC (1st-order plant): dt=..., wo=..., b0=..., kp=...`
/// response. Only `wo`, `b0` and `kp` are returned: `dt` is fixed by the
/// firmware and `kd` is unused by the first-order law.
fn parse_gains(line: &str) -> Option<(f32, f32, f32)> {
    let body = line.split(':').nth(1)?;
    let mut wo = None;
    let mut b0 = None;
    let mut kp = None;

    for item in body.split(',') {
        let (key, value) = item.split_once('=')?;
        // Values carry a unit suffix (e.g. "0.0010 s", "0.0000 (W/s)/us",
        // "0.00 1/s"); parse only the leading numeric token.
        let value = value
            .trim()
            .split_whitespace()
            .next()?
            .parse::<f32>()
            .ok()?;
        match key.trim() {
            "w0" | "wo" => wo = Some(value),
            "b0" => b0 = Some(value),
            "kp" => kp = Some(value),
            _ => {}
        }
    }

    Some((wo?, b0?, kp?))
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
        // Control variables are not present in the CSV stream.
        flags: 0,
        z1_mw: 0,
        z2_mws: 0,
        z3_mws: 0,
        pwm_ctrl_raw: 0,
        measured_power_w: 0.0,
        target_power_w: 0.0,
    })
}

/// Decodes a single 44-byte binary stream sample into a [`TelemetryData`].
/// Field offsets and scalings follow `docs/hardware.md` §4.2 (payload type 1).
fn parse_stream_sample(bytes: &[u8]) -> TelemetryData {
    let t_ms = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let e_j = i32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
    let z1_mw = i32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
    let z2_mws = i32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
    let z3_mws = i32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
    let v_cv = u16::from_le_bytes([bytes[20], bytes[21]]);
    let i_ca = i16::from_le_bytes([bytes[22], bytes[23]]);
    let y_dw = i16::from_le_bytes([bytes[24], bytes[25]]);
    let pmax_w = u16::from_le_bytes([bytes[26], bytes[27]]);
    let tgt_dw = i16::from_le_bytes([bytes[28], bytes[29]]);
    let pwm_in = u16::from_le_bytes([bytes[30], bytes[31]]);
    let pwm_out = u16::from_le_bytes([bytes[32], bytes[33]]);
    let pwm_ctrl = u16::from_le_bytes([bytes[34], bytes[35]]);
    let pwm_ctrl_raw = i16::from_le_bytes([bytes[36], bytes[37]]);
    let state = bytes[38];
    let flags = bytes[39];

    TelemetryData {
        time_ms: t_ms,
        power_w: pmax_w as f32,
        current_a: i_ca as f32 / 100.0, // 10 mA units
        voltage_v: v_cv as f32 / 100.0, // 10 mV units
        total_consumption_j: e_j as f32,
        pwm_input_us: pwm_in,
        pwm_output_us: pwm_out,
        pwm_control_us: pwm_ctrl,
        status: DeviceStatus::from_raw(state as u32),
        flags,
        z1_mw,
        z2_mws,
        z3_mws,
        pwm_ctrl_raw,
        measured_power_w: y_dw as f32 / 10.0, // 0.1 W units
        target_power_w: tgt_dw as f32 / 10.0, // 0.1 W units
    }
}

/// Spawns a reader thread for the 10 Hz CSV telemetry stream on the shell port.
fn spawn_csv_reader(
    shell_port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
    connected: Arc<AtomicBool>,
    tx: mpsc::Sender<TelemetryData>,
    recorder: CsvRecorder,
) {
    std::thread::spawn(move || {
        let mut pending = String::new();
        let mut buf = [0u8; 256];

        while connected.load(Ordering::SeqCst) {
            let read_result = {
                let mut guard = match shell_port.lock() {
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
                            recorder.write_sample(&frame);
                            if tx.blocking_send(frame).is_err() {
                                return;
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => {
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        }
    });
}

/// Spawns a reader thread for the binary telemetry stream (port 2).
///
/// Frames are located by scanning for the `0xA5` magic byte and validated with
/// `version == 1` and `len == count * item_size`; invalid bytes are dropped one
/// at a time to resynchronise (mirrors the reference parser in §4.2).
///
/// Every sample is written to the [`CsvRecorder`] (buffered, so the reader never
/// blocks on disk), while only a decimated subset is forwarded to the UI via
/// `try_send` (dropped if the UI can't keep up, so no back-pressure reaches the
/// device ring and no sample is lost from the recording).
fn spawn_binary_reader(
    stream_port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
    connected: Arc<AtomicBool>,
    tx: mpsc::Sender<TelemetryData>,
    recorder: CsvRecorder,
) {
    std::thread::spawn(move || {
        let mut buf: Vec<u8> = Vec::with_capacity(4096);
        let mut read_buf = [0u8; 512];
        let mut sample_counter = 0usize;
        let mut first_frame_logged = false;

        while connected.load(Ordering::SeqCst) {
            let read_result = {
                let mut guard = match stream_port.lock() {
                    Ok(g) => g,
                    Err(_) => break,
                };
                let Some(serial) = guard.as_mut() else {
                    break;
                };
                serial.read(&mut read_buf)
            };

            match read_result {
                Ok(n) if n > 0 => {
                    buf.extend_from_slice(&read_buf[..n]);

                    loop {
                        let Some(pos) = buf.iter().position(|&b| b == STREAM_MAGIC) else {
                            buf.clear();
                            break;
                        };
                        if pos > 0 {
                            buf.drain(..pos);
                        }
                        if buf.len() < STREAM_HEADER_SIZE {
                            break;
                        }

                        let version = buf[1];
                        let ftype = buf[2];
                        let count = buf[5] as usize;
                        let plen = u16::from_le_bytes([buf[6], buf[7]]) as usize;

                        let item_size = match ftype {
                            STREAM_TYPE_SAMPLES => STREAM_SAMPLE_SIZE,
                            STREAM_TYPE_META => STREAM_META_SIZE,
                            _ => 0,
                        };

                        if version != STREAM_VERSION || item_size == 0 || plen != count * item_size
                        {
                            // False magic — drop one byte and resync.
                            buf.drain(..1);
                            continue;
                        }

                        if buf.len() < STREAM_HEADER_SIZE + plen {
                            break;
                        }

                        if ftype == STREAM_TYPE_SAMPLES {
                            if !first_frame_logged {
                                eprintln!("binary stream: first sample frame ({} samples)", count);
                                first_frame_logged = true;
                            }
                            let base = STREAM_HEADER_SIZE;
                            for n in 0..count {
                                sample_counter += 1;
                                let off = base + n * STREAM_SAMPLE_SIZE;
                                let frame =
                                    parse_stream_sample(&buf[off..off + STREAM_SAMPLE_SIZE]);

                                // Record every sample (buffered, no data loss).
                                recorder.write_sample(&frame);

                                // Decimate for the UI, dropping if it lags.
                                if sample_counter % STREAM_DECIMATION == 0 {
                                    let _ = tx.try_send(frame);
                                }
                            }
                        }
                        // Meta frames (type 2) are the config snapshot and are
                        // not needed by the UI, so they are skipped here.

                        buf.drain(..STREAM_HEADER_SIZE + plen);
                    }
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => {
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        }
    });
}

/// Spawns a reader thread that extracts human-readable shell/log lines from the
/// console port. Prompt/echo lines and CSV telemetry lines are skipped; the
/// remaining LOG output is forwarded to the UI.
fn spawn_shell_log_reader(
    shell_port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
    connected: Arc<AtomicBool>,
    tx: mpsc::Sender<String>,
) {
    std::thread::spawn(move || {
        let mut pending = String::new();
        let mut buf = [0u8; 256];

        while connected.load(Ordering::SeqCst) {
            let read_result = {
                let mut guard = match shell_port.lock() {
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

                    while let Some(newline) = pending.find('\n') {
                        let line: String = pending.drain(..=newline).collect();
                        let trimmed = line.trim();
                        if trimmed.is_empty() {
                            continue;
                        }
                        // Skip shell prompt/echo lines (they start with `uart:~$`).
                        if trimmed.starts_with(SHELL_PROMPT.trim()) {
                            continue;
                        }
                        // Skip CSV telemetry lines (only present in fallback mode).
                        if parse_telemetry_line(trimmed).is_some() {
                            continue;
                        }
                        if tx.blocking_send(trimmed.to_string()).is_err() {
                            return;
                        }
                    }
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => {
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        }
    });
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
        let ports = detect_pm100_ports();
        let port_name = ports
            .first()
            .cloned()
            .ok_or_else(|| "PM100 USB device not found. Check that it is connected.".to_string())?;
        let stream_name = ports.get(1).cloned().unwrap_or_default();

        let port = serialport::new(&port_name, BAUD_RATE)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| format!("Failed to open serial port {}: {}", port_name, e))?;

        {
            let mut guard = self.port.lock().map_err(|e| e.to_string())?;
            *guard = Some(port);
        }

        // Remember the resolved ports for reference.
        {
            let mut name = self.port_name.lock().map_err(|e| e.to_string())?;
            *name = port_name;
        }
        {
            let mut name = self.stream_port_name.lock().map_err(|e| e.to_string())?;
            *name = stream_name;
        }

        self.connected.store(true, Ordering::SeqCst);

        // Blink the LED so the user can identify which physical device was
        // connected. This is the first shell command sent, and is best-effort:
        // a blink failure should not tear down an otherwise healthy connection.
        let _ = self.write_command("blink");

        Ok(())
    }

    async fn disconnect(&self) -> Result<(), String> {
        // Stop both streams before dropping the ports.
        let _ = self.write_command("stream bin off");
        let _ = self.write_command("stream off");

        self.connected.store(false, Ordering::SeqCst);

        {
            let mut guard = self.port.lock().map_err(|e| e.to_string())?;
            *guard = None;
        }
        {
            let mut guard = self.stream_port.lock().map_err(|e| e.to_string())?;
            *guard = None;
        }

        Ok(())
    }

    async fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn subscribe_telemetry(
        &self,
        recorder: CsvRecorder,
    ) -> Result<mpsc::Receiver<TelemetryData>, String> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err("USB serial port is not connected".to_string());
        }

        let stream_name = self
            .stream_port_name
            .lock()
            .map(|n| n.clone())
            .unwrap_or_default();

        let (tx, rx) = mpsc::channel(1024);

        if stream_name.is_empty() {
            // No second CDC ACM port: fall back to the 10 Hz CSV stream.
            self.write_command("stream on")?;
            spawn_csv_reader(
                Arc::clone(&self.port),
                Arc::clone(&self.connected),
                tx,
                recorder,
            );
            return Ok(rx);
        }

        // Preferred path: open the dedicated binary telemetry port and parse
        // fixed-point sample frames. If it cannot be opened, fall back to CSV.
        let stream = match serialport::new(&stream_name, BAUD_RATE)
            .timeout(Duration::from_millis(50))
            .open()
        {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to open binary stream port {}: {}", stream_name, e);
                self.write_command("stream on")?;
                spawn_csv_reader(
                    Arc::clone(&self.port),
                    Arc::clone(&self.connected),
                    tx,
                    recorder,
                );
                return Ok(rx);
            }
        };

        {
            let mut guard = self.stream_port.lock().map_err(|e| e.to_string())?;
            *guard = Some(stream);
        }

        // Enable the binary stream right before reading (default 4 ms period).
        self.write_command("stream bin on")?;

        spawn_binary_reader(
            Arc::clone(&self.stream_port),
            Arc::clone(&self.connected),
            tx,
            recorder,
        );

        Ok(rx)
    }

    async fn subscribe_logs(&self) -> Result<mpsc::Receiver<String>, String> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err("USB serial port is not connected".to_string());
        }

        let (tx, rx) = mpsc::channel(256);

        // The shell port is only free for log capture when the binary stream is
        // used; in CSV fallback the port is already consumed by the telemetry
        // reader, so no logs are captured there.
        let stream_name = self
            .stream_port_name
            .lock()
            .map(|n| n.clone())
            .unwrap_or_default();
        if stream_name.is_empty() {
            return Ok(rx);
        }

        spawn_shell_log_reader(Arc::clone(&self.port), Arc::clone(&self.connected), tx);

        Ok(rx)
    }

    async fn update_adrc_gains(&self, wo: f32, b0: f32, kp: f32) -> Result<(), String> {
        // `dt` is fixed at 1 ms by the firmware and `kd` is unused by the
        // first-order law, so only `wo`, `b0` and `kp` are sent (kd = 0).
        self.write_command(&format!("pm100 gains {} {} {} 0", wo, b0, kp))
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
        let (wo, b0, kp) = parse_gains(&gains_line)
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
            wo,
            b0,
            kp,
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
