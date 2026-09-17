use async_trait::async_trait;
use tokio::sync::mpsc::Receiver;

pub mod mock;
pub mod usb;

/// Runtime state reported by the PM100 control loop.
///
/// Integer mapping (as sent in the telemetry stream):
/// `0 = READY`, `1 = LIMITING_POWER`, `2 = ERROR_NO_INPUT`,
/// `3 = ERROR_NO_BATTERY`, `4 = BLINK`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DeviceStatus {
    /// System is ready and healthy.
    Ready,
    /// ADRC is actively limiting output power.
    LimitingPower,
    /// No valid PWM input signal (throttle absent or out of range).
    ErrorNoInput,
    /// No battery voltage measured.
    ErrorNoBattery,
    /// Device is blinking its LED (identification feedback).
    Blink,
    /// Status has not yet been received or is unrecognized.
    #[default]
    Unknown,
}

impl DeviceStatus {
    /// Maps the raw integer sent by the firmware to a [`DeviceStatus`].
    #[allow(dead_code)]
    pub fn from_raw(raw: u32) -> Self {
        match raw {
            0 => Self::Ready,
            1 => Self::LimitingPower,
            2 => Self::ErrorNoInput,
            3 => Self::ErrorNoBattery,
            4 => Self::Blink,
            _ => Self::Unknown,
        }
    }

    /// Maps a [`DeviceStatus`] back to the raw integer sent by the firmware.
    pub fn to_raw(self) -> u32 {
        match self {
            Self::Ready => 0,
            Self::LimitingPower => 1,
            Self::ErrorNoInput => 2,
            Self::ErrorNoBattery => 3,
            Self::Blink => 4,
            Self::Unknown => 255,
        }
    }

    /// Returns a human-readable label for the UI.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Ready => "PRONTO",
            Self::LimitingPower => "LIMITANDO POTÊNCIA",
            Self::ErrorNoInput => "ERRO: SEM ENTRADA",
            Self::ErrorNoBattery => "ERRO: SEM BATERIA",
            Self::Blink => "PISCANDO",
            Self::Unknown => "DESCONHECIDO",
        }
    }
}

/// Data package sent at 10Hz containing real-time measurements and PWM states.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TelemetryData {
    pub time_ms: u32,
    pub power_w: f32,
    pub current_a: f32,
    pub voltage_v: f32,
    pub total_consumption_j: f32,
    pub pwm_input_us: u16,
    pub pwm_output_us: u16,
    pub pwm_control_us: u16,
    pub status: DeviceStatus,
}

/// Local representation of PM100 Non-Volatile Storage (NVS) parameter configurations.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
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

/// High-level trait representing a connection to the PM100 power limiter.
/// This allows the UI to interface with a Mock simulator or the USB CDC ACM
/// serial port.
#[async_trait]
pub trait DeviceConnection: Send + Sync {
    /// Establish a connection to the device.
    async fn connect(&self) -> Result<(), String>;

    /// Terminate the active connection.
    async fn disconnect(&self) -> Result<(), String>;

    /// Check if the connection is currently active.
    async fn is_connected(&self) -> bool;

    /// Subscribe to real-time 10Hz telemetry updates.
    /// Spawns a channel reader transmitting parsed values to the UI loop.
    async fn subscribe_telemetry(&self) -> Result<Receiver<TelemetryData>, String>;

    /// Send a request to update the active ADRC Gains on the device (and save to NVS).
    async fn update_adrc_gains(
        &self,
        dt: f32,
        wo: f32,
        b0: f32,
        kp: f32,
        kd: f32,
    ) -> Result<(), String>;

    /// Update the desired target limit in Watts.
    async fn update_target_power(&self, target_watts: f32) -> Result<(), String>;

    /// Configure the unique team registration (number and name string).
    async fn update_team_config(&self, team_number: u32, team_name: &str) -> Result<(), String>;

    /// Update the security PIN code.
    async fn update_pin_code(&self, pin_code: &str) -> Result<(), String>;

    /// Read the current device configuration (ADRC gains, target power,
    /// team info, and pairing PIN) back from the device.
    async fn read_config(&self) -> Result<DeviceConfigData, String>;

    /// Send a command to trigger a white blink feedback sequence (10x flashes) for physical unit testing.
    async fn trigger_blink(&self) -> Result<(), String>;
}
