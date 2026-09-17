use dioxus::prelude::*;

#[derive(Props, Clone, PartialEq)]
pub struct LevelBarsProps {
    pub pwm_input_us: u16,
    pub pwm_output_us: u16,
    pub pwm_control_us: u16,
    #[props(default = true)]
    pub vertical: bool,
}

#[component]
pub fn LevelBars(props: LevelBarsProps) -> Element {
    let get_pct = |val: u16| -> f32 {
        let val_clamped = val.clamp(0, 2000);
        (val_clamped as f32 / 2000.0) * 100.0
    };

    let in_pct = get_pct(props.pwm_input_us);
    let ctrl_pct = get_pct(props.pwm_control_us);
    let out_pct = get_pct(props.pwm_output_us);

    let container_class = if props.vertical {
        "card-panel bars-container-v"
    } else {
        "card-panel bars-container-h" // Replaced clashing phone-drawer style with standard clean card-panel!
    };

    rsx! {
        div {
            class: "{container_class}",

            Bar {
                label: "PILOTO",
                value_us: props.pwm_input_us,
                pct: in_pct,
                color_class: "bg-level-in",
                vertical: props.vertical,
            }

            Bar {
                label: "CONTROLE",
                value_us: props.pwm_control_us,
                pct: ctrl_pct,
                color_class: "bg-level-ctrl",
                vertical: props.vertical,
            }

            Bar {
                label: "SAÍDA",
                value_us: props.pwm_output_us,
                pct: out_pct,
                color_class: "bg-level-out",
                vertical: props.vertical,
            }
        }
    }
}

#[derive(Props, Clone, PartialEq)]
struct BarProps {
    label: &'static str,
    value_us: u16,
    pct: f32,
    color_class: &'static str,
    vertical: bool,
}

#[component]
fn Bar(props: BarProps) -> Element {
    if props.vertical {
        rsx! {
            div {
                class: "bar-v-column",

                span {
                    class: "bar-v-value",
                    "{props.value_us}"
                }

                // Vertical track channel
                div {
                    class: "bar-track-v",
                    div {
                        style: "height: {props.pct}%;",
                        class: "bar-fill {props.color_class}"
                    }
                }

                span {
                    class: "bar-v-label",
                    "{props.label}"
                }
            }
        }
    } else {
        rsx! {
            div {
                class: "bar-h-row",

                div {
                    class: "bar-h-info",
                    span { "{props.label}" }
                    span { class: "bar-h-value", "{props.value_us}" }
                }

                // Horizontal track channel
                div {
                    class: "bar-track-h",
                    div {
                        style: "width: {props.pct}%;",
                        class: "bar-fill h-full {props.color_class}"
                    }
                }
            }
        }
    }
}
