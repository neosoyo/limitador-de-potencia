use crate::connection::TelemetryData;
use dioxus::prelude::*;

#[derive(Props, Clone, PartialEq)]
pub struct StreamPlotProps {
    pub history: Vec<TelemetryData>,
    pub max_power: f32,
}

#[component]
pub fn StreamPlot(props: StreamPlotProps) -> Element {
    let width = 600.0f32;
    let height = 300.0f32;
    let padding_top = 20.0f32;
    let padding_bottom = 25.0f32;

    let padding_left = 50.0f32;
    let padding_right = 105.0f32;

    let chart_width = width - padding_left - padding_right;
    let chart_height = height - padding_top - padding_bottom;

    // Calculate maximum Joules dynamically in history to auto-scale the Joule curve
    let max_joules = props
        .history
        .iter()
        .map(|d| d.total_consumption_j)
        .fold(300.0f32, |m, v| m.max(v));

    // Helper to map index to X coordinate
    let map_x = |index: usize, total_count: usize| -> f32 {
        if total_count < 2 {
            return padding_left;
        }
        padding_left + (index as f32 / (total_count - 1) as f32) * chart_width
    };

    // Helper to map y percentage (0.0 to 1.0) to Y coordinate
    let map_y = |pct: f32| -> f32 {
        let pct_clamped = pct.clamp(0.0, 1.0);
        padding_top + (1.0 - pct_clamped) * chart_height
    };

    // Build the SVG path string for custom selectors
    let make_path = |selector: fn(&TelemetryData) -> f32, val_min: f32, val_max: f32| -> String {
        if props.history.is_empty() {
            return "".to_string();
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

    // Construct paths
    let path_input = make_path(|d| d.pwm_input_us as f32, 1000.0, 2000.0);
    let path_control = make_path(|d| d.pwm_control_us as f32, 1000.0, 2000.0);
    let path_output = make_path(|d| d.pwm_output_us as f32, 1000.0, 2000.0);
    // The firmware sends `power_w` as the running peak (max) power, not the
    // instantaneous power. The real-time power is calculated here as
    // current × voltage.
    let path_power = make_path(|d| d.current_a * d.voltage_v, 0.0, 300.0);
    let path_joules = make_path(|d| d.total_consumption_j, 0.0, max_joules);

    // Session peak power reference line (shared Watts axis, 0–300 W).
    let max_power_y = map_y((props.max_power / 300.0).clamp(0.0, 1.0));

    // Divisions
    let divisions = vec![0.0f32, 0.25, 0.5, 0.75, 1.0];

    // Axis coordinate calculations
    let right_chart_edge = width - padding_right;
    let watts_axis_x = right_chart_edge + 35.0f32;
    let joules_axis_x = right_chart_edge + 75.0f32;

    rsx! {
        div {
            class: "card-panel plot-panel",

            // Minimalist Header
            div {
                class: "plot-header",
                span {
                    class: "card-title",
                    style: "margin-bottom: 0; padding-bottom: 0; border-bottom: none;",
                    "Gráfico de Dados"
                }

                // Minimalist Legend (Uses theme-reactive classes)
                div {
                    class: "plot-legend",
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-pwm-stroke" }
                        span { "Entrada" }
                    }
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-ctrl-stroke" }
                        span { "Controle" }
                    }
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-out-stroke" }
                        span { "Saída" }
                    }
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-power-stroke" }
                        span { "Potência (W)" }
                    }
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-max-power-stroke" }
                        span { "Potência Máxima (W)" }
                    }
                    div {
                        class: "legend-item",
                        div { class: "legend-line bg-joules-stroke" }
                        span { "Energia (J)" }
                    }
                }
            }

            // Minimalist SVG drawing
            svg {
                view_box: "0 0 {width} {height}",
                width: "100%",
                height: "auto",
                style: "background-color: var(--chart-bg); border: 1px solid var(--border-color);",

                // 1. GRID H-LINES & MULTI-AXIS LABELS
                for pct in &divisions {
                    {
                        let y = map_y(*pct);

                        // Numeric calculations
                        let pwm_val = 1000 + (pct * 1000.0) as u32;
                        let watts_val = (pct * 300.0) as u32;
                        let joules_val = (pct * max_joules) as u32;

                        rsx! {
                            // Primary horizontal grid line across the plot canvas (using theme variable)
                            line {
                                x1: "{padding_left}",
                                y1: "{y}",
                                x2: "{right_chart_edge}",
                                y2: "{y}",
                                stroke: "var(--border-subtle)",
                                stroke_width: "1",
                            }

                            // LEFT AXIS: PWM microsecond ticks (using theme variable)
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
                                "{pwm_val}us"
                            }

                            // FIRST RIGHT AXIS: Watts ticks (using theme variable)
                            line {
                                x1: "{watts_axis_x}",
                                y1: "{y}",
                                x2: "{watts_axis_x + 4.0}",
                                y2: "{y}",
                                stroke: "var(--power-stroke)",
                                stroke_width: "1",
                            }
                            text {
                                x: "{watts_axis_x + 8.0}",
                                y: "{y + 3.0}",
                                fill: "var(--power-stroke)",
                                font_size: "10",
                                text_anchor: "start",
                                "{watts_val}W"
                            }

                            // SECOND RIGHT AXIS: Joules ticks (using theme variable)
                            line {
                                x1: "{joules_axis_x}",
                                y1: "{y}",
                                x2: "{joules_axis_x + 4.0}",
                                y2: "{y}",
                                stroke: "var(--joules-stroke)",
                                stroke_width: "1",
                            }
                            text {
                                x: "{joules_axis_x + 8.0}",
                                y: "{y + 3.0}",
                                fill: "var(--joules-stroke)",
                                font_size: "10",
                                text_anchor: "start",
                                "{joules_val}J"
                            }
                        }
                    }
                }

                // 2. VERTICAL SCALE BARS
                // Left Border Line (PWM frame)
                line {
                    x1: "{padding_left}",
                    y1: "{padding_top}",
                    x2: "{padding_left}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--text-muted)",
                    stroke_width: "1",
                }
                // Right Chart Limit
                line {
                    x1: "{right_chart_edge}",
                    y1: "{padding_top}",
                    x2: "{right_chart_edge}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--border-subtle)",
                    stroke_width: "1",
                }
                // Second right vertical bar (Watts Scale)
                line {
                    x1: "{watts_axis_x}",
                    y1: "{padding_top}",
                    x2: "{watts_axis_x}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--power-stroke)",
                    stroke_width: "1",
                }
                // Third right vertical bar (Joules Scale)
                line {
                    x1: "{joules_axis_x}",
                    y1: "{padding_top}",
                    x2: "{joules_axis_x}",
                    y2: "{height - padding_bottom}",
                    stroke: "var(--joules-stroke)",
                    stroke_width: "1",
                }

                // 3. PLOT TRACES (THEMED CURVES)
                if !props.history.is_empty() {
                    // INPUT PWM (SOLID THEMED INPUT)
                    path {
                        d: "{path_input}",
                        fill: "none",
                        stroke: "var(--pwm-stroke)",
                        stroke_width: "1.5",
                    }
                    // CONTROL PWM (DASHED THEMED CONTROL)
                    path {
                        d: "{path_control}",
                        fill: "none",
                        stroke: "var(--ctrl-stroke)",
                        stroke_width: "1.2",
                        stroke_dasharray: "4 4",
                    }
                    // OUTPUT PWM (THIN DOTTED THEMED OUTPUT)
                    path {
                        d: "{path_output}",
                        fill: "none",
                        stroke: "var(--out-stroke)",
                        stroke_width: "1.2",
                        stroke_dasharray: "1 3",
                    }
                    // MEASURED POWER (THEMED POWER)
                    path {
                        d: "{path_power}",
                        fill: "none",
                        stroke: "var(--power-stroke)",
                        stroke_width: "1.5",
                    }
                    // SESSION MAX POWER (DASHED REFERENCE LINE, SHARED WATTS AXIS)
                    if props.max_power > 0.0 {
                        line {
                            x1: "{padding_left}",
                            y1: "{max_power_y}",
                            x2: "{right_chart_edge}",
                            y2: "{max_power_y}",
                            stroke: "var(--max-power-stroke)",
                            stroke_width: "1.2",
                            stroke_dasharray: "4 3",
                        }
                    }
                    // ACCUMULATED JOULES (THEMED JOULES)
                    path {
                        d: "{path_joules}",
                        fill: "none",
                        stroke: "var(--joules-stroke)",
                        stroke_width: "1.2",
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
