use dioxus::prelude::*;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

mod components;
mod connection;

use components::{ConfigForms, ControlPlot, LevelBars, LiveMetrics, StreamPlot};
use connection::usb::UsbConnection;
use connection::{CsvRecorder, DeviceConnection, DeviceStatus, RecordingState, TelemetryData};

// Hand-crafted high-performance offline monochrome CSS stylesheet (Dual-Theme)
const BASE_STYLE_CSS: &str = include_str!("../assets/style.css");

#[derive(Clone, Debug, PartialEq)]
enum ConnStatus {
    Disconnected,
    Connecting,
    Connected,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LeftTab {
    Plot,
    Control,
    Config,
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

    let mut adrc_wo = use_signal(|| 100.0f32);
    let mut adrc_b0 = use_signal(|| 1.0f32);
    let mut adrc_kp = use_signal(|| 1.0f32);

    // Active UI Theme selection (Default: Dark theme)
    let mut is_dark_theme = use_signal(|| true);

    // Active left-column tab (Plot vs. the 3 configuration boxes)
    let mut left_tab = use_signal(|| LeftTab::Plot);

    let device_conn = use_signal(|| {
        let conn: Arc<dyn DeviceConnection> = Arc::new(UsbConnection::auto());
        conn
    });

    // Shared CSV recorder (buffered) + UI-mirroring state/path signals.
    let recorder = CsvRecorder::new();
    let telemetry_recorder = recorder.clone();
    let mut recording_state = use_signal(|| RecordingState::Stopped);
    let mut record_path = use_signal(|| String::new());

    // Most recent shell/log message captured from the device console.
    let mut log_line = use_signal(|| String::new());

    // Reactive telemetry subscription. `use_resource` runs this in the current
    // component scope and re-runs it whenever `connection_status` changes. The
    // inner loop re-subscribes automatically if the stream ends while the device
    // is still connected (e.g. after Android freezes/resumes the process), which
    // previously froze the readouts after the app was backgrounded and reopened.
    let _telemetry_task = use_resource(move || {
        let recorder = telemetry_recorder.clone();
        async move {
            if *connection_status.read() == ConnStatus::Connected {
                let conn = device_conn.cloned();

                loop {
                    if !conn.is_connected().await {
                        break;
                    }

                    match conn.subscribe_telemetry(recorder.clone()).await {
                        Ok(mut rx) => {
                            let mut local_history = VecDeque::new();
                            while let Some(data) = rx.recv().await {
                                telemetry_data.set(data.clone());

                                // Session peak tracker using peek() to avoid subscription loop
                                if data.power_w > *max_power.peek() {
                                    max_power.set(data.power_w);
                                }

                                // 1500 points (30 seconds window at 50 Hz)
                                local_history.push_back(data);
                                if local_history.len() > 1500 {
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
        }
    });

    // Reactive shell/log subscription: collects console messages from the
    // device while connected, keeping only the most recent lines.
    let _log_task = use_resource(move || async move {
        if *connection_status.read() == ConnStatus::Connected {
            let conn = device_conn.cloned();
            match conn.subscribe_logs().await {
                Ok(mut rx) => {
                    while let Some(line) = rx.recv().await {
                        log_line.set(line);
                    }
                }
                Err(e) => eprintln!("subscribe_logs: {}", e),
            }
        }
    });

    // Reads the device configuration and populates the form signals. Returns
    // the future so callers can either await it (during connect) or spawn it
    // (the "Ler Configuração" button).
    let read_config_action = move |_| {
        let conn = device_conn.cloned();
        async move {
            match conn.read_config().await {
                Ok(config) => {
                    target_power.set(config.target_power);
                    team_name.set(config.team_name);
                    team_number.set(config.team_number);
                    pin_code.set(config.pin_code);
                    adrc_wo.set(config.wo);
                    adrc_b0.set(config.b0);
                    adrc_kp.set(config.kp);
                }
                Err(e) => {
                    eprintln!("Failed to read device config: {}", e);
                }
            }
        }
    };

    let connect_action = move |_| {
        spawn(async move {
            connection_status.set(ConnStatus::Connecting);
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

            match device_conn.cloned().connect().await {
                Ok(()) => {
                    // Read the device's current configuration and populate the UI.
                    read_config_action(()).await;
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

    let update_adrc_action = move |(wo_val, b0_val, kp_val): (f32, f32, f32)| {
        spawn(async move {
            let _ = device_conn
                .cloned()
                .update_adrc_gains(wo_val, b0_val, kp_val)
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

    let start_recorder = recorder.clone();
    let start_recording = move |_| {
        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let path = format!("pm100_log_{}.csv", secs);

        match start_recorder.start(&path) {
            Ok(()) => {
                record_path.set(path);
                recording_state.set(RecordingState::Recording);
            }
            Err(e) => eprintln!("Failed to create CSV log file: {}", e),
        }
    };

    let toggle_recorder = recorder.clone();
    let toggle_pause_recording = move |_| {
        let next = match *recording_state.peek() {
            RecordingState::Recording => RecordingState::Paused,
            RecordingState::Paused => RecordingState::Recording,
            RecordingState::Stopped => RecordingState::Stopped,
        };
        match next {
            RecordingState::Paused => toggle_recorder.pause(),
            RecordingState::Recording => toggle_recorder.resume(),
            RecordingState::Stopped => {}
        }
        recording_state.set(next);
    };

    let stop_recorder = recorder.clone();
    let stop_recording = move |_| {
        stop_recorder.stop();
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
        DeviceStatus::Learning => "status-learning",
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
                        style: "font-size: 11px; font-weight: bold; letter-spacing: 1px; text-transform: uppercase; color: #ef4444;",
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

                        // Column 1 (Tabbed: Plot / Control / Config forms) - 3fr width
                        div {
                            class: "flex flex-col gap-6",
                            style: "grid-column: span 1; min-height: 0;",

                            // Tab bar switching between the plot, control variables and config boxes
                            div {
                                class: "tab-bar",
                                button {
                                    class: if *left_tab.read() == LeftTab::Plot { "tab tab-active" } else { "tab" },
                                    onclick: move |_| left_tab.set(LeftTab::Plot),
                                    "Gráfico"
                                }
                                button {
                                    class: if *left_tab.read() == LeftTab::Control { "tab tab-active" } else { "tab" },
                                    onclick: move |_| left_tab.set(LeftTab::Control),
                                    "Controle"
                                }
                                button {
                                    class: if *left_tab.read() == LeftTab::Config { "tab tab-active" } else { "tab" },
                                    onclick: move |_| {
                                        left_tab.set(LeftTab::Config);
                                        if *connection_status.read() == ConnStatus::Connected {
                                            spawn(read_config_action(()));
                                        }
                                    },
                                    "Configuração"
                                }
                            }

                            // Device status readout (updated from the telemetry stream)
                            div {
                                class: "status-strip",
                                div { class: "status-label", "Status do Dispositivo" }
                                div { class: "status-value {status_class}", "{status_label}" }
                            }

                            // Most recent shell/log message from the device console
                            div {
                                class: "log-panel",
                                div { class: "log-panel-title", "Log do Dispositivo" }
                                div {
                                    class: "log-scroll",
                                    div { class: "log-line", "{log_line}" }
                                }
                            }

                            {match *left_tab.read() {
                                LeftTab::Plot => rsx! {
                                    StreamPlot {
                                        history: telemetry_history.cloned(),
                                        max_power: max_power.cloned(),
                                    }
                                },
                                LeftTab::Control => rsx! {
                                    ControlPlot {
                                        history: telemetry_history.cloned(),
                                    }
                                },
                                LeftTab::Config => rsx! {
                                    ConfigForms {
                                        target_power,
                                        team_number,
                                        team_name,
                                        pin_code,
                                        adrc_wo,
                                        adrc_b0,
                                        adrc_kp,
                                        on_update_adrc: update_adrc_action,
                                        on_update_target: update_target_action,
                                        on_update_team: update_team_action,
                                        on_update_pin: update_pin_action,
                                    }
                                },
                            }}
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
