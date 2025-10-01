use rumqttc::{Client, MqttOptions, QoS, Event, Incoming};
use serde_json::Value;
use std::{fs, time::Duration};

fn main() {
    let config_str = fs::read_to_string("mqtt_config.json")
        .expect("Falha ao ler mqtt_config.json");
    let config: Value = serde_json::from_str(&config_str)
        .expect("Configuração JSON inválida");

    let broker = config["broker"].as_str().expect("broker não encontrado");
    let port = config["port"].as_u64().expect("port inválida") as u16;
    let keepalive = config.get("keepalive").and_then(|v| v.as_u64()).unwrap_or(60);
    let topic = config["topics"]["vehicle_parameters"].as_str().expect("tópico não encontrado");

    let mut mqttoptions = MqttOptions::new("rust_mqtt_client", broker, port);
    mqttoptions.set_keep_alive(Duration::from_secs(keepalive));

    let (mut client, mut connection) = Client::new(mqttoptions, 10);

    client.subscribe(topic, QoS::AtMostOnce).unwrap();
    println!("Subscribed to topic: {}", topic);

    let mut previous_cruise_control: Option<bool> = None;

    for notification in connection.iter() {
        match notification {
            Ok(Event::Incoming(Incoming::Publish(p))) => {
                let payload_str = String::from_utf8_lossy(&p.payload);
                match serde_json::from_str::<Value>(&payload_str) {
                    Ok(data) => {
                        println!(
                            "Received message on {}: {}",
                            p.topic,
                            serde_json::to_string_pretty(&data).unwrap()
                        );

                        let current_cruise_control = data.get("CruiseControl").and_then(|v| v.as_bool()).unwrap_or(false);
                        let speed = data.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.0);

                        match (previous_cruise_control, current_cruise_control) {
                            (Some(false), true) | (None, true) => {
                                println!("Cruise Control active. Speed: {}", speed);
                            }
                            (Some(true), false) => {
                                println!("Cruise Control deactivated.");
                            }
                            _ => {}
                        }

                        previous_cruise_control = Some(current_cruise_control);
                    }
                    Err(_) => {
                        println!("Received non-JSON message: {}", payload_str);
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                eprintln!("Erro na conexão: {:?}", e);
                break;
            }
        }
    }
}
