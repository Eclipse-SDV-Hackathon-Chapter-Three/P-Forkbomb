use rumqttc::{MqttOptions, Client, QoS};
use serde::{Deserialize, Serialize};
use std::{fs, thread, time::Duration};

#[derive(Deserialize)]
struct Config {
    broker: String,
    port: u16,
    keepalive: Option<u16>,
    topics: Topics,
}

#[derive(Deserialize)]
struct Topics {
    vehicle_parameters: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct Payload {
    speed: i32,
    cruise_control: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_str = fs::read_to_string("mqtt_config.json")?;
    let config: Config = serde_json::from_str(&config_str)?;

    let data_str = fs::read_to_string("data.json")?;
    let mut payload: Payload = serde_json::from_str(&data_str)?;
    println!("Loaded payload from data.json");

    let mut mqttoptions = MqttOptions::new("rust-client", config.broker, config.port);
    mqttoptions.set_keep_alive(Duration::from_secs(config.keepalive.unwrap_or(60) as u64));

    let (client, mut connection) = Client::new(mqttoptions, 10);

    thread::spawn(move || {
        for notification in connection.iter() {
            if let Err(e) = notification {
                eprintln!("MQTT connection error: {:?}", e);
            }
        }
    });

    let mut speed = payload.speed;
    let mut direction = 1;

    loop {
        speed += 5 * direction;

        if speed >= 100 {
            direction = -1;
        } else if speed <= 0 {
            direction = 1;
        }

        payload.speed = speed;

        let msg = serde_json::to_string(&payload)?;
        client
            .publish(&config.topics.vehicle_parameters, QoS::AtLeastOnce, false, msg)
            .unwrap();

        println!("Published Speed={} to {}", speed, config.topics.vehicle_parameters);

        thread::sleep(Duration::from_secs(1));
    }
}
