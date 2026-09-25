use dioxus::prelude::*;

#[derive(Props, Clone, PartialEq)]
pub struct ConfigFormsProps {
    pub target_power: Signal<f32>,
    pub team_number: Signal<u32>,
    pub team_name: Signal<String>,
    pub pin_code: Signal<String>,
    pub adrc_wo: Signal<f32>,
    pub adrc_b0: Signal<f32>,
    pub adrc_kp: Signal<f32>,

    pub on_update_adrc: EventHandler<(f32, f32, f32)>,
    pub on_update_target: EventHandler<f32>,
    pub on_update_team: EventHandler<(u32, String)>,
    pub on_update_pin: EventHandler<String>,
}

#[component]
pub fn ConfigForms(props: ConfigFormsProps) -> Element {
    // Bind writable signals to mutable locals so they can be updated directly
    // from the input handlers (Signal::set requires `&mut self`).
    let mut target_power = props.target_power;
    let mut team_number = props.team_number;
    let mut team_name = props.team_name;
    let mut pin_code = props.pin_code;

    rsx! {
        div {
            class: "config-grid",

            // 1. ADRC & TARGET POWER CONTROLLER CONFIG
            div {
                class: "card-panel",

                div { class: "card-title",
                    "Config. controlador ADRC"
                }

                // Target Power Form
                div { class: "form-row",
                    label { class: "form-label", "Potência máxima" }
                    div { class: "flex-input-row",
                        input {
                            r#type: "number",
                            value: "{target_power}",
                            class: "text-input flex-grow",
                            oninput: move |e| {
                                if let Ok(val) = e.value().parse::<f32>() {
                                    target_power.set(val);
                                }
                            }
                        }
                        button {
                            class: "btn",
                            onclick: move |_| props.on_update_target.call(target_power.cloned()),
                            "Definir"
                        }
                    }
                }

                // ADRC Gains Number Forms list
                div { class: "flex flex-col gap-4",
                    NumberField {
                        label: "Largura de Banda do Observador (wo)",
                        help: "Largura de banda do observador de estados estendido (ESO) em rad/s. Valores maiores rastreiam distúrbios mais rápido, mas podem amplificar o ruído de medição.",
                        value: props.adrc_wo
                    }
                    NumberField {
                        label: "Fator de Escala de Entrada (b0)",
                        help: "Ganho estimado de entrada do sistema. Escala o esforço de controle para que o observador e o controlador usem unidades consistentes.",
                        value: props.adrc_b0
                    }
                    NumberField {
                        label: "Ganho Proporcional (Kp)",
                        help: "Ganho de realimentação proporcional. Aumenta a resposta e reduz o erro em regime permanente; muito alto causa sobressinal (overshoot).",
                        value: props.adrc_kp
                    }

                    button {
                        class: "btn btn-primary w-full mt-2",
                        onclick: move |_| props.on_update_adrc.call((props.adrc_wo.cloned(), props.adrc_b0.cloned(), props.adrc_kp.cloned())),
                        "Atualizar ADRC"
                    }
                }
            }

            // 2. TEAM CONFIGURATION FORM
            div {
                class: "card-panel justify-between",

                div { class: "card-title",
                    "Informações da Equipe"
                }

                div { class: "flex flex-col gap-4 flex-grow",
                    div { class: "form-row",
                        label { class: "form-label", "Número da Equipe" }
                        input {
                            r#type: "number",
                            value: "{team_number}",
                            class: "text-input",
                            oninput: move |e| {
                                if let Ok(val) = e.value().parse::<u32>() {
                                    team_number.set(val);
                                }
                            }
                        }
                    }

                    div { class: "form-row",
                        label { class: "form-label", "Nome da Equipe" }
                        input {
                            r#type: "text",
                            value: "{team_name}",
                            maxlength: "31",
                            class: "text-input",
                            oninput: move |e| team_name.set(e.value())
                        }
                    }
                }

                button {
                    class: "btn btn-primary w-full mt-6",
                    onclick: move |_| props.on_update_team.call((team_number.cloned(), team_name.cloned())),
                    "Salvar"
                }
            }

            // 3. SECURITY PIN CONFIGURATION
            div {
                class: "card-panel justify-between",

                div { class: "card-title",
                    "Segurança & PIN BLE"
                }

                div { class: "flex flex-col gap-4 flex-grow",
                    p { style: "color: var(--text-muted); line-height: 1.5; font-size: 12px; margin: 0 0 12px 0;",
                        "Este PIN de 6 dígitos é armazenado no dispositivo como senha de pareamento BLE. Defina-o aqui pela conexão serial USB."
                    }

                    div { class: "form-row mt-2",
                        label { class: "form-label", "PIN de Pareamento (6 Dígitos)" }
                        input {
                            r#type: "text",
                            value: "{pin_code}",
                            maxlength: "6",
                            placeholder: "123456",
                            class: "text-input text-center",
                            style: "font-size: 21px; letter-spacing: 6px; padding-left: 12px; font-weight: bold; border-color: #262626;",
                            oninput: move |e| {
                                let clean = e.value().chars().filter(|c| c.is_numeric()).collect::<String>();
                                pin_code.set(clean);
                            }
                        }
                    }
                }

                // Compute and render button directly inside rust block
                {
                    let pin_invalid = pin_code.read().len() != 6;
                    let btn_class = if pin_invalid { "btn btn-disabled w-full mt-6" } else { "btn btn-primary w-full mt-6" };
                    rsx! {
                        button {
                            class: "{btn_class}",
                            disabled: pin_invalid,
                            onclick: move |_| {
                                if pin_code.read().len() == 6 {
                                    props.on_update_pin.call(pin_code.cloned());
                                }
                            },
                            "Atualizar PIN de Pareamento"
                        }
                    }
                }
            }
        }
    }
}

#[derive(Props, Clone, PartialEq)]
struct NumberFieldProps {
    label: &'static str,
    help: &'static str,
    value: Signal<f32>,
}

#[component]
fn NumberField(props: NumberFieldProps) -> Element {
    let mut val_sig = props.value;
    rsx! {
        div { class: "form-row tooltip-wrap",
            label { class: "form-label", "{props.label}" }
            input {
                r#type: "number",
                step: "any",
                value: "{val_sig}",
                class: "text-input w-full",
                oninput: move |e| {
                    if let Ok(num) = e.value().parse::<f32>() {
                        val_sig.set(num);
                    }
                }
            }
            span { class: "tooltip", "{props.help}" }
        }
    }
}
