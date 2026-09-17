use dioxus::prelude::*;
use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

mod components;
mod connection;

use components::{ConfigForms, LevelBars, LiveMetrics, StreamPlot};
use connection::usb::UsbConnection;
use connection::{DeviceConnection, DeviceStatus, TelemetryData};

// Hand-crafted high-performance offline monochrome CSS stylesheet (Dual-Theme)
const BASE_STYLE_CSS: &str = include_str!("../assets/style.css");

#[derive(Clone, Debug, PartialEq)]
enum ConnStatus {
    Disconnected,
    Connecting,
    Connected,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum RecordingState {
    Stopped,
    Recording,
    Paused,
}

fn main() {
    dioxus::LaunchBuilder::new()
        .with_cfg(desktop! {
            dioxus::desktop::Config::new().with_menu(None).with_window(
                dioxus::desktop::WindowBuilder::new()
                    .with_title("Configurador PM100")
            )
        })
        .launch(app);
}

fn app() -> Element {
    let mut connection_status = use_signal(|| ConnStatus::Disconnected);
    let mut telemetry_data = use_signal(|| TelemetryData::default());
    let mut telemetry_history = use_signal(|| Vec::<TelemetryData>::new());

    // Store session peak power (red metric readout)
    let mut max_power = use_signal(|| 0.0f32);

    let mut target_power = use_signal(|| 150.0f32);
    let mut team_number = use_signal(|| 404u32);
    let mut team_name = use_signal(|| "Mock Falcon".to_string());
    let mut pin_code = use_signal(|| "123456".to_string());

    let mut adrc_dt = use_signal(|| 0.002f32);
    let mut adrc_wo = use_signal(|| 100.0f32);
    let mut adrc_b0 = use_signal(|| 1.0f32);
    let mut adrc_kp = use_signal(|| 1.0f32);
    let mut adrc_kd = use_signal(|| 0.10f32);

    // Active UI Theme selection (Default: Dark theme)
    let mut is_dark_theme = use_signal(|| true);

    let device_conn = use_signal(|| {
        let conn: Arc<dyn DeviceConnection> = Arc::new(UsbConnection::auto());
        conn
    });

    // CSV stream recording state + shared file handle (the Arc is created once;
    // the inner `Option<File>` is opened/closed by the record controls).
    let mut recording_state = use_signal(|| RecordingState::Stopped);
    let csv_file = use_signal(|| Arc::new(Mutex::new(None::<File>)));
    let mut record_path = use_signal(|| String::new());

    // Reactive telemetry subscription. `use_resource` runs this in the current
    // component scope and re-runs it whenever `connection_status` changes. The
    // inner loop re-subscribes automatically if the stream ends while the device
    // is still connected (e.g. after Android freezes/resumes the process), which
    // previously froze the readouts after the app was backgrounded and reopened.
    let _telemetry_task = use_resource(move || async move {
        if *connection_status.read() == ConnStatus::Connected {
            let conn = device_conn.cloned();

            loop {
                if !conn.is_connected().await {
                    break;
                }

                match conn.subscribe_telemetry().await {
                    Ok(mut rx) => {
                        let mut local_history = VecDeque::new();
                        while let Some(data) = rx.recv().await {
                            telemetry_data.set(data.clone());

                            // Write the frame to the CSV log when recording and
                            // not paused. `peek()` avoids subscribing the resource
                            // to `recording_state` (which would restart the stream).
                            if *recording_state.peek() == RecordingState::Recording {
                                let file_arc = (*csv_file.peek()).clone();
                                let mut guard = match file_arc.lock() {
                                    Ok(g) => g,
                                    Err(poisoned) => poisoned.into_inner(),
                                };
                                if let Some(file) = guard.as_mut() {
                                    // `power_w` from the firmware is the running
                                    // peak (max) power. The real-time power is
                                    // not sent directly, so it is calculated as
                                    // current × voltage.
                                    let _ = writeln!(
                                        file,
                                        "{},{},{},{},{},{},{},{},{},{}",
                                        data.time_ms,
                                        data.power_w,
                                        data.current_a * data.voltage_v,
                                        data.current_a,
                                        data.voltage_v,
                                        data.total_consumption_j,
                                        data.pwm_input_us,
                                        data.pwm_output_us,
                                        data.pwm_control_us,
                                        data.status.to_raw(),
                                    );
                                }
                            }

                            // Session peak tracker using peek() to avoid subscription loop
                            if data.power_w > *max_power.peek() {
                                max_power.set(data.power_w);
                            }

                            // 300 points (30 seconds window at 10Hz)
                            local_history.push_back(data);
                            if local_history.len() > 300 {
                                local_history.pop_front();
                            }

                            telemetry_history.set(local_history.iter().cloned().collect());
                        }
                    }
                    Err(e) => {
                        eprintln!("subscribe_telemetry: {}", e);
                    }
                }

                // Stream ended while still connected; retry shortly.
                tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
            }
        }
    });

    let connect_action = move |_| {
        spawn(async move {
            connection_status.set(ConnStatus::Connecting);
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

            match device_conn.cloned().connect().await {
                Ok(()) => {
                    // Read the device's current configuration and populate the UI.
                    match device_conn.cloned().read_config().await {
                        Ok(config) => {
                            target_power.set(config.target_power);
                            team_name.set(config.team_name);
                            team_number.set(config.team_number);
                            pin_code.set(config.pin_code);
                            adrc_dt.set(config.dt);
                            adrc_wo.set(config.wo);
                            adrc_b0.set(config.b0);
                            adrc_kp.set(config.kp);
                            adrc_kd.set(config.kd);
                        }
                        Err(e) => {
                            eprintln!("Failed to read device config: {}", e);
                        }
                    }
                    connection_status.set(ConnStatus::Connected);
                }
                Err(e) => {
                    eprintln!("Connect failed: {}", e);
                    connection_status.set(ConnStatus::Disconnected);
                }
            }
        });
    };

    let disconnect_action = move |_| {
        spawn(async move {
            let _ = device_conn.cloned().disconnect().await;
            connection_status.set(ConnStatus::Disconnected);
            telemetry_data.set(TelemetryData::default());
            telemetry_history.set(Vec::new());

            // Clean slate for the next telemetry session
            max_power.set(0.0);
        });
    };

    let trigger_blink_action = move |_| {
        spawn(async move {
            let _ = device_conn.cloned().trigger_blink().await;
        });
    };

    let update_adrc_action =
        move |(dt_val, wo_val, b0_val, kp_val, kd_val): (f32, f32, f32, f32, f32)| {
            spawn(async move {
                let _ = device_conn
                    .cloned()
                    .update_adrc_gains(dt_val, wo_val, b0_val, kp_val, kd_val)
                    .await;
            });
        };

    let update_target_action = move |target_val: f32| {
        spawn(async move {
            if device_conn
                .cloned()
                .update_target_power(target_val)
                .await
                .is_ok()
            {
                target_power.set(target_val);
            }
        });
    };

    let update_team_action = move |(num, name): (u32, String)| {
        spawn(async move {
            if device_conn
                .cloned()
                .update_team_config(num, &name)
                .await
                .is_ok()
            {
                team_number.set(num);
                team_name.set(name);
            }
        });
    };

    let update_pin_action = move |pin_val: String| {
        spawn(async move {
            if device_conn.cloned().update_pin_code(&pin_val).await.is_ok() {
                pin_code.set(pin_val);
            }
        });
    };

    // --- CSV stream recording controls ---

    let start_recording = move |_| {
        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let path = format!("pm100_log_{}.csv", secs);

        match File::create(&path) {
            Ok(mut file) => {
                let header = "time_ms,max_power_w,power_w,current_a,voltage_v,total_consumption_j,pwm_input_us,pwm_output_us,pwm_control_us,status\n";
                if file.write_all(header.as_bytes()).is_ok() {
                    let file_arc = csv_file.cloned();
                    let mut guard = match file_arc.lock() {
                        Ok(g) => g,
                        Err(poisoned) => poisoned.into_inner(),
                    };
                    *guard = Some(file);
                    record_path.set(path);
                    recording_state.set(RecordingState::Recording);
                }
            }
            Err(e) => eprintln!("Failed to create CSV log file: {}", e),
        }
    };

    let toggle_pause_recording = move |_| {
        let next = match *recording_state.peek() {
            RecordingState::Recording => RecordingState::Paused,
            RecordingState::Paused => RecordingState::Recording,
            RecordingState::Stopped => RecordingState::Stopped,
        };
        recording_state.set(next);
    };

    let stop_recording = move |_| {
        let file_arc = csv_file.cloned();
        let mut guard = match file_arc.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        *guard = None; // drop the file, flushing/closing it
        recording_state.set(RecordingState::Stopped);
        record_path.set(String::new());
    };

    let theme_label = if *is_dark_theme.read() {
        "Claro"
    } else {
        "Escuro"
    };
    let theme_class = if *is_dark_theme.read() {
        "theme-dark"
    } else {
        "theme-light"
    };

    let device_status = telemetry_data.read().status;
    let status_label = device_status.label();
    let status_class = match device_status {
        DeviceStatus::Ready => "status-ready",
        DeviceStatus::LimitingPower => "status-limiting",
        DeviceStatus::ErrorNoInput | DeviceStatus::ErrorNoBattery => "status-error",
        DeviceStatus::Blink => "status-blink",
        DeviceStatus::Unknown => "status-unknown",
    };

    rsx! {
        style { "{BASE_STYLE_CSS}" }

        div { class: "app-container {theme_class}",

            // 1. HEADER STATUS & CONTROLS
            header {
                class: "header-bar",

                span {
                    class: "brand-subtitle",
                    if *connection_status.read() == ConnStatus::Connected {
                        "Conectado: {team_name} (ID: {team_number})"
                    } else {
                        "Desconectado"
                    }
                }

                // Control Panel Action buttons
                div {
                    class: "flex-row-center",

                    // Light/Dark Theme Switcher
                    button {
                        class: "btn",
                        onclick: move |_| is_dark_theme.set(!is_dark_theme.cloned()),
                        "Tema: {theme_label}"
                    }

                    {match *connection_status.read() {
                        ConnStatus::Disconnected => rsx! {
                            button {
                                class: "btn btn-primary",
                                onclick: connect_action,
                                "Conectar"
                            }
                        },
                        ConnStatus::Connecting => rsx! {
                            button {
                                class: "btn btn-disabled",
                                disabled: true,
                                "Conectando"
                            }
                        },
                        ConnStatus::Connected => rsx! {
                            button {
                                class: "btn btn-primary",
                                onclick: disconnect_action,
                                "Desconectar"
                            }
                        }
                    }}

                    if *connection_status.read() == ConnStatus::Connected {
                        button {
                            class: "btn",
                            onclick: trigger_blink_action,
                            "Piscar"
                        }

                        // CSV stream recording controls
                        {match *recording_state.read() {
                            RecordingState::Stopped => rsx! {
                                button {
                                    class: "btn",
                                    onclick: start_recording,
                                    "Gravar CSV"
                                }
                            },
                            RecordingState::Recording => rsx! {
                                button {
                                    class: "btn",
                                    onclick: toggle_pause_recording,
                                    "Pausar"
                                }
                                button {
                                    class: "btn",
                                    onclick: stop_recording,
                                    "Parar"
                                }
                            },
                            RecordingState::Paused => rsx! {
                                button {
                                    class: "btn",
                                    onclick: toggle_pause_recording,
                                    "Retomar"
                                }
                                button {
                                    class: "btn",
                                    onclick: stop_recording,
                                    "Parar"
                                }
                            }
                        }}
                    }
                }

                // Recording status indicator (REC / PAUSED + filename)
                if *recording_state.read() != RecordingState::Stopped {
                    span {
                        style: "font-size: 9px; font-weight: bold; letter-spacing: 1px; text-transform: uppercase; color: #ef4444;",
                        if *recording_state.read() == RecordingState::Recording {
                            "GRAVANDO: {record_path}"
                        } else {
                            "PAUSADO: {record_path}"
                        }
                    }
                }
            }

            // 2. MAIN WORKSPACE
            main {
                class: "main-content",

                // ----------------------------------------------------
                // DESKTOP VIEW (Two-column layout)
                // ----------------------------------------------------
                div {
                    class: "desktop-grid",

                        // Column 1 (Plot + Config forms below it) - 3fr width
                        div {
                            class: "flex flex-col gap-6",
                            style: "grid-column: span 1;",
                            StreamPlot {
                                history: telemetry_history.cloned(),
                                max_power: max_power.cloned(),
                            }

                            // Device status readout (updated from the telemetry stream)
                            div {
                                class: "status-strip",
                                div { class: "status-label", "Status do Dispositivo" }
                                div { class: "status-value {status_class}", "{status_label}" }
                            }

                            ConfigForms {
                                target_power,
                                team_number,
                                team_name,
                                pin_code,
                                adrc_dt,
                                adrc_wo,
                                adrc_b0,
                                adrc_kp,
                                adrc_kd,
                                on_update_adrc: update_adrc_action,
                                on_update_target: update_target_action,
                                on_update_team: update_team_action,
                                on_update_pin: update_pin_action,
                            }
                        }

                        // Column 2 (Metrics on top + Level bars stacked below) - 1fr width
                        div {
                            class: "flex flex-col gap-6 h-full",
                            style: "grid-column: span 1;",

                            LiveMetrics {
                                data: telemetry_data.cloned(),
                                max_power: max_power.cloned(),
                            }

                            LevelBars {
                                pwm_input_us: telemetry_data.read().pwm_input_us,
                                pwm_output_us: telemetry_data.read().pwm_output_us,
                                pwm_control_us: telemetry_data.read().pwm_control_us,
                                vertical: true,
                            }
                        }
                    }
            }
        }
    }
}
