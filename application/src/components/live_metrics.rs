use crate::connection::TelemetryData;
use dioxus::prelude::*;

#[derive(Props, Clone, PartialEq)]
pub struct LiveMetricsProps {
    pub data: TelemetryData,
    pub max_power: f32,
}

#[component]
pub fn LiveMetrics(props: LiveMetricsProps) -> Element {
    let t = props.data;

    // Format uptime in seconds with 1 decimal digit
    let total_secs = t.time_ms as f32 / 1000.0;
    let uptime_str = format!("{:.1} s", total_secs);

    rsx! {
        div {
            class: "card-panel metrics-panel",

            // Header
            div {
                class: "card-title",
                "Medições"
            }

            // Metrics Stack
            div {
                class: "flex-grow",

                // Real-time Power (calculated as current × voltage)
                MetricRow { label: "Potência", value: format!("{:.1} W", t.current_a * t.voltage_v) }

                // Peak/Maximum Power (Red Text)
                MetricRow { label: "Potência Máxima", value: format!("{:.1} W", props.max_power), is_red: true }

                // Other Parameters
                MetricRow { label: "Tensão", value: format!("{:.2} V", t.voltage_v) }
                MetricRow { label: "Corrente", value: format!("{:.2} A", t.current_a) }
                MetricRow { label: "Energia", value: format!("{:.0} J", t.total_consumption_j) }
                MetricRow { label: "Tempo Total", value: uptime_str }
            }
        }
    }
}

#[derive(Props, Clone, PartialEq)]
struct MetricRowProps {
    label: &'static str,
    value: String,
    #[props(default = false)]
    is_red: bool,
}

#[component]
fn MetricRow(props: MetricRowProps) -> Element {
    let value_class = if props.is_red {
        "metric-value text-red"
    } else {
        "metric-value"
    };

    rsx! {
        div {
            class: "metric-row",
            span {
                class: "metric-label",
                "{props.label}"
            }
            span {
                class: "{value_class}",
                "{props.value}"
            }
        }
    }
}
