use async_trait::async_trait;
use rand::Rng;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex};
use tokio::time::{sleep, Duration};

use crate::connection::{
    CsvRecorder, DeviceConfigData, DeviceConnection, DeviceStatus, TelemetryData,
};

/// Emulated connection simulating physical aircraft dynamics under ADRC control.
#[allow(dead_code)]
pub struct MockConnection {
    connected: Arc<AtomicBool>,
    config: Arc<Mutex<DeviceConfigData>>,
}

impl MockConnection {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            connected: Arc::new(AtomicBool::new(false)),
            config: Arc::new(Mutex::new(DeviceConfigData {
                wo: 100.0,
                b0: 1.0,
                kp: 1.5,
                target_power: 150.0, // Default 150W target limit
                team_name: "Mock Falcon".to_string(),
                team_number: 404,
                pin_code: "123456".to_string(),
            })),
        }
    }
}

#[async_trait]
impl DeviceConnection for MockConnection {
    async fn connect(&self) -> Result<(), String> {
        self.connected.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn disconnect(&self) -> Result<(), String> {
        self.connected.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn subscribe_telemetry(
        &self,
        recorder: CsvRecorder,
    ) -> Result<mpsc::Receiver<TelemetryData>, String> {
        if !self.is_connected().await {
            return Err("Device is not connected".to_string());
        }

        let (tx, rx) = mpsc::channel(100);
        let connected = Arc::clone(&self.connected);
        let config_clone = Arc::clone(&self.config);

        // Spawn async simulator task running at 10Hz (100ms period)
        tokio::spawn(async move {
            let mut uptime_ms = 0u32;
            let mut energy_j = 0.0f32;

            // Internal physical model parameters
            let mut throttle_phase = 0.0f32;
            // Session peak power and LESO estimator state for the control plot.
            let mut peak_power_w = 0.0f32;
            let mut z1_ema = 0.0f32;

            while connected.load(Ordering::SeqCst) {
                sleep(Duration::from_millis(100)).await;
                uptime_ms += 100;

                // 1. Emulate pilot's command (sinusoidal throttle demand)
                throttle_phase += 0.05;
                let pilot_throttle_pct = (throttle_phase.sin() + 1.0) / 2.0; // 0.0 to 1.0
                let pwm_input_us = 1000 + (pilot_throttle_pct * 1000.0) as u16; // 1000us to 2000us

                // 2. Fetch current config parameters (target power, etc.)
                let current_config = {
                    let c = config_clone.lock().await;
                    c.clone()
                };

                // 3. Emulate power measurement depending on the throttle and battery status
                // Volts: Let's simulate a 3S LiPo battery dropping slightly with high current draw
                let base_voltage = 12.0f32; // 12V LiPo
                let internal_resistance = 0.015f32; // Ohms

                // Emulate load power before limiting
                let raw_unlimited_current = pilot_throttle_pct * 30.0; // Max 30 Amps draw
                let voltage_v = base_voltage - (raw_unlimited_current * internal_resistance)
                    + rand::thread_rng().gen_range(-0.05..0.05);

                // Emulate ADRC controller behavior restricting PWM output if simulated power exceeds target
                let max_possible_power = raw_unlimited_current * voltage_v;

                let (pwm_output_us, pwm_control_us, power_w, current_a) = if max_possible_power
                    > current_config.target_power
                {
                    // Limiting is ACTIVE
                    // Target power is capped. Calculate necessary current
                    let target_current = current_config.target_power / voltage_v;
                    let target_throttle_pct = target_current / 30.0;

                    let pwm_control_us = (1000.0 + (target_throttle_pct * 1000.0)) as u16;
                    let pwm_output_us = pwm_control_us; // output is constrained

                    let current_a = target_current + rand::thread_rng().gen_range(-0.1..0.1);
                    let power_w = current_a * voltage_v;

                    (pwm_output_us, pwm_control_us, power_w, current_a)
                } else {
                    // Limiting is INACTIVE (pilot commands less than power limit)
                    let pwm_control_us = 2000; // control throttle is wide open
                    let pwm_output_us = pwm_input_us; // outputs matches pilot's direct input

                    let current_a = raw_unlimited_current + rand::thread_rng().gen_range(-0.1..0.1);
                    let power_w = current_a * voltage_v;

                    (pwm_output_us, pwm_control_us, power_w, current_a)
                };

                // Accumulate total consumption (Joules = Watts * seconds)
                energy_j += power_w * 0.1;

                let status = if pwm_output_us < pwm_input_us {
                    DeviceStatus::LimitingPower
                } else {
                    DeviceStatus::Ready
                };

                // Instantaneous measured power (the controller input `y`).
                let measured_power_w = power_w;

                // Running peak power since boot.
                peak_power_w = peak_power_w.max(measured_power_w);

                // Crude LESO emulation: z1 tracks the measured power with a
                // first-order lag; z2 is its derivative and z3 the disturbance
                // (here approximated by the power error to the target).
                let z1_prev_ema = z1_ema;
                z1_ema = z1_ema * 0.9 + measured_power_w * 0.1;
                let z1_mw = (z1_ema * 1000.0) as i32;
                let z2_mws = ((z1_ema - z1_prev_ema) * 10_000.0) as i32; // dP/dt in mW/s
                let z3_mws = ((measured_power_w - current_config.target_power) * 1000.0) as i32;

                // Pre-clamp effort: when limiting the raw effort exceeds the
                // clamped value (wind-up), otherwise it sits at the wide-open
                // 2000 us ceiling.
                let pwm_ctrl_raw = if pwm_output_us < pwm_input_us {
                    (pwm_control_us as i16).saturating_add(150)
                } else {
                    2000
                };

                let data = TelemetryData {
                    time_ms: uptime_ms,
                    power_w: peak_power_w,
                    current_a: current_a.max(0.0),
                    voltage_v: voltage_v.max(0.0),
                    total_consumption_j: energy_j,
                    pwm_input_us,
                    pwm_output_us,
                    pwm_control_us,
                    status,
                    flags: 0,
                    z1_mw,
                    z2_mws,
                    z3_mws,
                    pwm_ctrl_raw,
                    measured_power_w,
                    target_power_w: current_config.target_power,
                };

                recorder.write_sample(&data);

                if tx.send(data).await.is_err() {
                    break; // Receiver disconnected, terminate loop
                }
            }
        });

        Ok(rx)
    }

    async fn subscribe_logs(&self) -> Result<mpsc::Receiver<String>, String> {
        if !self.is_connected().await {
            return Err("Device is not connected".to_string());
        }

        let (tx, rx) = mpsc::channel(64);
        let connected = Arc::clone(&self.connected);

        tokio::spawn(async move {
            let mut tick = 0u32;
            while connected.load(Ordering::SeqCst) {
                sleep(Duration::from_millis(2000)).await;
                tick += 1;
                let lines = [
                    format!("[00:{:02}:00.000] <inf> app: ADRC loop at 1 kHz", tick),
                    format!(
                        "[00:{:02}:00.000] <inf> app: LESO converged (z1 ~150 W)",
                        tick
                    ),
                ];
                for line in lines {
                    if tx.send(line).await.is_err() {
                        return;
                    }
                }
            }
        });

        Ok(rx)
    }

    async fn update_adrc_gains(&self, wo: f32, b0: f32, kp: f32) -> Result<(), String> {
        let mut c = self.config.lock().await;
        c.wo = wo;
        c.b0 = b0;
        c.kp = kp;
        println!("Mock Update: ADRC Gains saved to NVS!");
        Ok(())
    }

    async fn update_target_power(&self, target_watts: f32) -> Result<(), String> {
        let mut c = self.config.lock().await;
        c.target_power = target_watts;
        println!(
            "Mock Update: Target Power set to {}W and saved to NVS!",
            target_watts
        );
        Ok(())
    }

    async fn update_team_config(&self, team_number: u32, team_name: &str) -> Result<(), String> {
        let mut c = self.config.lock().await;
        c.team_number = team_number;
        c.team_name = team_name.to_string();
        println!(
            "Mock Update: Team configured as #{} - '{}'!",
            team_number, team_name
        );
        Ok(())
    }

    async fn update_pin_code(&self, pin_code: &str) -> Result<(), String> {
        if pin_code.len() != 6 {
            return Err("PIN code must be exactly 6 digits".to_string());
        }
        let mut c = self.config.lock().await;
        c.pin_code = pin_code.to_string();
        println!("Mock Update: BLE pairing PIN updated to '{}'!", pin_code);
        Ok(())
    }

    async fn read_config(&self) -> Result<DeviceConfigData, String> {
        Ok(self.config.lock().await.clone())
    }

    async fn trigger_blink(&self) -> Result<(), String> {
        println!("Mock Cmd: Triggering 10x WS2812 status flashes (WHITE)!");
        Ok(())
    }
}
