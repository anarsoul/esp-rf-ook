// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Vasily Khoruzhick <anarsoul@gmail.com>

use chrono::{DateTime, Utc};
use embedded_svc::mqtt::client::{EventPayload::*, QoS};
use esp_idf_hal::gpio::*;
use esp_idf_hal::task::watchdog::{TWDTConfig, TWDTDriver};
use esp_idf_hal::timer::{config, TimerDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::sntp::{EspSntp, SyncStatus};
use log::{info, warn};
use std::str;
use std::time::SystemTime;
use wifi::wifi;

const PREAMBLE_MIN: u64 = 2000; // us
const PREAMBLE_MAX: u64 = 8000; // us
const SIGNAL_END_MIN: u64 = 3000; // us
const SIGNAL_END_MAX: u64 = 8000; // us
const PULSE_MIN: u64 = 300; // us
const PULSE_MAX: u64 = 600; // us
const PAYLOAD_LEN: usize = 36;

const MIN_HIGH: u64 = 1650;
const MAX_HIGH: u64 = 2150;
const MIN_LOW: u64 = 800;
const MAX_LOW: u64 = 1100;

#[toml_cfg::toml_config]
pub struct Config {
    #[default("mqttserver")]
    mqtt_host: &'static str,
    #[default("")]
    mqtt_user: &'static str,
    #[default("")]
    mqtt_pass: &'static str,
    #[default("rtl_433/Nexus-TH")]
    mqtt_topic: &'static str,
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
    #[default(1)]
    channel: u8,
}

enum WaitingFor {
    PulseIdle,
    Preamble,
    Pulse,
    Data,
}

enum DecodeError {
    WrongPayloadLen(usize),
    SampleOutOfRange(u64),
    WrongChannel(u8),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            DecodeError::WrongPayloadLen(len) => write!(f, "Wrong payload len: {}", len),
            DecodeError::SampleOutOfRange(sample) => write!(f, "Sample out of range: {}", sample),
            DecodeError::WrongChannel(ch) => write!(f, "Wrong channel: {}", ch),
        }
    }
}

fn in_range(count: u64, min: u64, max: u64) -> bool {
    count >= min && count <= max
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();

    let app_config = CONFIG;

    let _wifi = wifi(
        app_config.wifi_ssid,
        app_config.wifi_psk,
        peripherals.modem,
        sysloop,
    )
    .unwrap();

    // Synchronize time
    let ntp = EspSntp::new_default().unwrap();
    info!("Synchronizing with NTP Server");
    while ntp.get_sync_status() != SyncStatus::Completed {}
    info!("Time Sync Completed");

    // Initialize MQTT
    let mqtt_config = MqttClientConfiguration::default();
    let broker_url = if !app_config.mqtt_user.is_empty() {
        format!(
            "mqtt://{}:{}@{}",
            app_config.mqtt_user, app_config.mqtt_pass, app_config.mqtt_host
        )
    } else {
        format!("mqtt://{}", app_config.mqtt_host)
    };
    info!("Broker URL: {}", broker_url);
    info!("Sensor channel: {}", app_config.channel);

    // Pump MQTT events. Warn on errors, publish will panic on unwrap,
    // but we'll have a chance to dump decoded data at least once
    let mut client =
        EspMqttClient::new_cb(
            &broker_url,
            &mqtt_config,
            move |message_event| match message_event.payload() {
                Error(e) => warn!("Received error from MQTT: {:?}", e),
                _ => info!("Received from MQTT: {:?}", message_event.payload()),
            },
        )
        .unwrap();

    let twdt_config = TWDTConfig {
        duration: core::time::Duration::from_secs(2),
        panic_on_trigger: true,
        subscribed_idle_tasks: enumset::EnumSet::empty(),
    };
    let mut twdt_driver = TWDTDriver::new(peripherals.twdt, &twdt_config).unwrap();
    let mut sub = twdt_driver.watch_current_task().unwrap();

    let pin = PinDriver::input(peripherals.pins.gpio21).unwrap();
    let config = config::Config::new();
    let mut timer = TimerDriver::new(peripherals.timer00, &config).unwrap();

    timer.set_counter(0_u64).unwrap();
    timer.enable(true).unwrap();

    let mut count: u64;
    let mut pin_current_level: Level;
    let mut pin_old_level: Level = Level::High;
    let mut samples: Vec<u64> = Vec::new();
    let mut state = WaitingFor::PulseIdle;
    loop {
        // Poke watchdog
        sub.feed().unwrap();
        pin_current_level = pin.get_level();

        // Wait for edge
        if pin_current_level == pin_old_level {
            continue;
        }

        count = timer.counter().unwrap();
        timer.set_counter(0_u64).unwrap();
        state = match state {
            WaitingFor::PulseIdle => {
                if pin_old_level == Level::High {
                    if in_range(count, PULSE_MIN, PULSE_MAX) {
                        WaitingFor::Preamble
                    } else {
                        WaitingFor::PulseIdle
                    }
                } else {
                    WaitingFor::PulseIdle
                }
            }
            WaitingFor::Preamble => {
                if in_range(count, PREAMBLE_MIN, PREAMBLE_MAX) {
                    WaitingFor::Pulse
                } else {
                    WaitingFor::PulseIdle
                }
            }
            WaitingFor::Pulse => {
                if in_range(count, PULSE_MIN, PULSE_MAX) {
                    WaitingFor::Data
                } else {
                    samples = Vec::new();
                    WaitingFor::PulseIdle
                }
            }
            WaitingFor::Data => {
                if in_range(count, SIGNAL_END_MIN, SIGNAL_END_MAX) {
                    // Don't attempt to decode if there is no samples
                    if !samples.is_empty() {
                        match decode(&samples, app_config.channel) {
                            Ok(decoded) => {
                                client
                                    .publish(
                                        app_config.mqtt_topic,
                                        QoS::AtMostOnce,
                                        false,
                                        decoded.as_bytes(),
                                    )
                                    .unwrap();
                            }
                            Err(why) => {
                                warn!("Decode failed: {}", why);
                            }
                        }
                        samples = Vec::new();
                    }
                    WaitingFor::PulseIdle
                } else if in_range(count, MIN_LOW, MAX_HIGH) {
                    samples.push(count);
                    WaitingFor::Pulse
                } else {
                    samples = Vec::new();
                    WaitingFor::PulseIdle
                }
            }
        };
        pin_old_level = pin_current_level;
    }
}

fn dump_samples(samples: &[u64]) {
    info!("!! BEGIN, {} samples", samples.len());
    for sample in samples {
        info!("{}", sample);
    }
    info!("!! END");
}

fn decode_range(samples: &[u64], start: usize, size: usize) -> Result<u32, DecodeError> {
    let mut value: u32 = 0;
    for sample in &samples[start..start + size] {
        if in_range(*sample, MIN_HIGH, MAX_HIGH) {
            value <<= 1;
            value |= 1;
        } else if in_range(*sample, MIN_LOW, MAX_LOW) {
            value <<= 1;
        } else {
            warn!("Range: {} - {}", start, start + size);
            dump_samples(samples);
            return Err(DecodeError::SampleOutOfRange(*sample));
        }
    }
    Ok(value)
}

fn decode(samples: &[u64], channel_to_use: u8) -> Result<String, DecodeError> {
    // Currently we support only Nexus-TH which has 36 bit of payload
    if samples.len() != PAYLOAD_LEN {
        return Err(DecodeError::WrongPayloadLen(samples.len()));
    }

    let mut temp_10x: i32 = decode_range(samples, 12, 12)? as i32;
    // Handle negative temp
    if temp_10x > 2048 {
        temp_10x = -(4096 - temp_10x);
    }
    let temp_int = temp_10x / 10;
    let temp_decimal = temp_10x.abs() % 10;

    let mut humidity: i32 = decode_range(samples, 28, 8)? as i32;
    // Clamp humidity
    if humidity > 100 {
        humidity = 100;
    }
    let battery_ok: u8 = decode_range(samples, 8, 1)? as u8;
    let channel: u8 = (decode_range(samples, 10, 2)? + 1) as u8;
    let id: u8 = decode_range(samples, 0, 8)? as u8;

    // Obtain System Time
    let st_now = SystemTime::now();
    // Convert to UTC Time
    let dt_now_utc: DateTime<Utc> = st_now.into();
    // Format Time String
    let formatted = format!("{}", dt_now_utc.format("%Y-%m-%d %H:%M:%S UTC"));
    // Print Time
    info!("{}", formatted);
    info!(
        "Temp: {}.{}, humidity: {}, channel: {}, ID: {}, battery_ok: {}",
        temp_int, temp_decimal, humidity, channel, id, battery_ok
    );

    if channel != channel_to_use {
        return Err(DecodeError::WrongChannel(channel));
    }

    Ok(format!("{{\"time\" : \"{formatted}\", \"model\" : \"Nexus-TH\", \"id\" : {id}, \"channel\" : {channel}, \"battery_ok\" : {battery_ok}, \"temperature_C\" : {temp_int}.{temp_decimal}, \"humidity\" : {humidity} }}"))
}
