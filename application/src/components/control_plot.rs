use crate::connection::TelemetryData;
use dioxus::prelude::*;

/// Second plot: the ADRC control-loop internals.
///
/// Shows the LESO estimated power (`z1`) tracking the measured power (`y`) and
/// the target (`r`) on the left axis, and the ADRC effort before/after clamping
/// (`pwm_ctrl_raw` vs `pwm_ctrl`) on the right axis so the wind-up is visible.
#[derive(Props, Clone, PartialEq)]
pub struct ControlPlotProps {
    pub history: Vec<TelemetryData>,
}

#[component]
pub fn ControlPlot(props: ControlPlotProps) -> Element {
    let width = 600.0f32;
    let height = 300.0f32;
    let padding_top = 20.0f32;
    let padding_bottom = 25.0f32;
    let padding_left = 50.0f32;
    let padding_right = 105.0f32;

    let chart_width = width - padding_left - padding_right;
    let chart_height = height - padding_top - padding_bottom;

    // Auto-scale the power axis from the data, with a 300 W floor so the chart
    // stays readable before any meaningful power is measured.
    let max_w = props
        .history
        .iter()
        .map(|d| {
            (d.z1_mw as f32 / 1000.0)
                .max(d.measured_power_w)
                .max(d.target_power_w)
        })
        .fold(300.0f32, f32::max);

    let map_x = |index: usize, total_count: usize| -> f32 {
        if total_count < 2 {
            return padding_left;
        }
        padding_left + (index as f32 / (total_count - 1) as f32) * chart_width
    };

    let map_y = |pct: f32| -> f32 { padding_top + (1.0 - pct.clamp(0.0, 1.0)) * chart_height };

    let make_path = |selector: fn(&TelemetryData) -> f32, val_min: f32, val_max: f32| -> String {
        if props.history.is_empty() {
            return String::new();
        }
        let total = props.history.len();
        let mut path_str = String::new();

        for (i, data) in props.history.iter().enumerate() {
            let val = selector(data);
            let pct = (val - val_min) / (val_max - val_min);
            let x = map_x(i, total);
            let y = map_y(pct);

            if i == 0 {
                path_str.push_str(&format!("M {} {}", x, y));
            } else {
                path_str.push_str(&format!(" L {} {}", x, y));
            }
        }
        path_str
    };

    let path_z1 = make_path(|d| d.z1_mw as f32 / 1000.0, 0.0, max_w);
    let path_measured = make_path(|d| d.measured_power_w, 0.0, max_w);
    let path_target = make_path(|d| d.target_power_w, 0.0, max_w);
    let path_ctrl_raw = make_path(|d| d.pwm_ctrl_raw as f32, 1000.0, 2000.0);
    let path_ctrl = make_path(|d| d.pwm_control_us as f32, 1000.0, 2000.0);

    let divisions = vec![0.0f32, 0.25, 0.5, 0.75, 1.0];
    let right_chart_edge = width - padding_right;
    let effort_axis_x = right_chart_edge + 35.0f32;

    rsx! {
        div {
            class: "card-panel plot-panel",

            div {
                class: "plot-header",
                span {
                    class: "card-title",
                    style: "margin-bottom: 0; padding-bottom: 0; border-bottom: none;",
                    "Controle ADRC"
                }

                div {
                    class: "plot-legend",
                    div { class: "legend-item",
                        div { class: "legend-line bg-power-stroke" }
                        span { "z1 (Estimada)" }
                    }
                    div { class: "legend-item",
                        div { class: "legend-line bg-ctrl-stroke" }
                        span { "Medida (y)" }
                    }
                    div { class: "legend-item",
                        div { class: "legend-line bg-max-power-stroke" }
                        span { "Alvo (r)" }
                    }
                    div { class: "legend-item",
                        div { class: "legend-line bg-out-stroke" }
                        span { "Esforço bruto" }
                    }
                    div { class: "legend-item",
                        div { class: "legend-line bg-pwm-stroke" }
                        span { "Esforço (µs)" }
                    }
                }
            }

            svg {
                view_box: "0 0 {width} {height}",
                width: "100%",
                height: "auto",
                style: "background-color: var(--chart-bg); border: 1px solid var(--border-color);",

                // 1. GRID LINES + AXIS LABELS
                for pct in &divisions {
                    {
                        let y = map_y(*pct);
                        let watts_val = (pct * max_w) as u32;
                        let us_val = 1000 + (pct * 1000.0) as u32;

                        rsx! {
                            line {
                                x1: "{padding_left}",
                                y1: "{y}",
                                x2: "{right_chart_edge}",
                                y2: "{y}",
                                stroke: "var(--border-subtle)",
                                stroke_width: "1",
                            }

                            // LEFT AXIS: Watts
                            line {
                                x1: "{padding_left - 4.0}",
                                y1: "{y}",
                                x2: "{padding_left}",
                                y2: "{y}",
                                stroke: "var(--text-muted)",
                                stroke_width: "1",
                            }
                            text {
                                x: "{padding_left - 8.0}",
                                y: "{y + 3.0}",
                                fill: "var(--text-muted)",
                                font_size: "10",
                                text_anchor: "end",
                                "{watts_val}W"
                            }

                            // RIGHT AXIS: PWM microseconds
                            line {
                                x1: "{effort_axis_x}",
                                y1: "{y}",
                                x2: "{effort_axis_x + 4.0}",
                                y2: "{y}",
                                stroke: "var(--pwm-stroke)",
                                stroke_width: "1",
                            }
                            text {
                                x: "{effort_axis_x + 8.0}",
                                y: "{y + 3.0}",
                                fill: "var(--pwm-stroke)",
                                font_size: "10",
                                text_anchor: "start",
                                "{us_val}µs"
                            }
                        }
                    }
                }

                // 2. VERTICAL AXIS BARS
                line {
                    x1: "{padding_left}",
                    y1: "{padding_top}",
                    x2: "{padding_left}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--text-muted)",
                    stroke_width: "1",
                }
                line {
                    x1: "{right_chart_edge}",
                    y1: "{padding_top}",
                    x2: "{right_chart_edge}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--border-subtle)",
                    stroke_width: "1",
                }
                line {
                    x1: "{effort_axis_x}",
                    y1: "{padding_top}",
                    x2: "{effort_axis_x}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--pwm-stroke)",
                    stroke_width: "1",
                }

                // 3. TRACES
                if !props.history.is_empty() {
                    path {
                        d: "{path_measured}",
                        fill: "none",
                        stroke: "var(--ctrl-stroke)",
                        stroke_width: "1.2",
                        stroke_dasharray: "4 4",
                    }
                    path {
                        d: "{path_target}",
                        fill: "none",
                        stroke: "var(--max-power-stroke)",
                        stroke_width: "1.2",
                        stroke_dasharray: "4 3",
                    }
                    path {
                        d: "{path_z1}",
                        fill: "none",
                        stroke: "var(--power-stroke)",
                        stroke_width: "1.5",
                    }
                    path {
                        d: "{path_ctrl_raw}",
                        fill: "none",
                        stroke: "var(--out-stroke)",
                        stroke_width: "1.2",
                        stroke_dasharray: "1 3",
                    }
                    path {
                        d: "{path_ctrl}",
                        fill: "none",
                        stroke: "var(--pwm-stroke)",
                        stroke_width: "1.5",
                    }
                } else {
                    text {
                        x: "{(right_chart_edge + padding_left) / 2.0}",
                        y: "{height / 2.0}",
                        fill: "var(--text-subtle)",
                        font_size: "13",
                        text_anchor: "middle",
                        "Telemetria Desconectada"
                    }
                }
            }
        }
    }
}
