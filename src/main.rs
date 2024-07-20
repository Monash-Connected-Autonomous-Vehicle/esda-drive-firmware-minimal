//! ESP-NOW Example
//!
//! Broadcasts, receives and sends messages via esp-now

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/esp-now
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::PeriodicTimer,
};
use esp_println::println;
use esp_wifi::{
    current_millis,
    esp_now::{PeerInfo, BROADCAST_ADDRESS},
    initialize,
    EspWifiInitFor,
};

use byteorder::{Byteorder, LittleEndian}; // Import Byteorder

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0, &clocks, None)
            .timer0
            .into(),
    );

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    let mut next_send_time = current_millis() + 5 * 1000;
    let mut cntr: u32 = 0; // Initializes an integer counter 

    loop {
        let r = esp_now.receive();
        if let Some(r) = r {

            println!("Received message from {:?}", r.info.src_address);
            let data = r.data;
            if data.len() >= 4 {
                let received_number = LittleEndian::read_u32(&data[0..4]);
                println!("Received number: {}", received_number);
            } else {
                println!("Received data is too short to decode as u32.");
            }
            
            // println!("Received data as {:?}", data);


            //println!("Received {:?}", r);

            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                let status = esp_now
                    .send(&r.info.src_address, b"Hello Peer")
                    .unwrap()
                    .wait();
                println!("Send hello to peer status: {:?}", status);
            }
        }

        if current_millis() >= next_send_time {
            next_send_time = current_millis() + 5 * 1000;
            println!("Send");
            // Increment counter and send
            let mut buffer = [0; 4];
            LittleEndian::write_u32(&mut buffer, cntr);
            let status = esp_now
                .send(&BROADCAST_ADDRESS, &buffer)
                .unwrap()
                .wait();
            println!("Send broadcast status: {:?}", status)
        }
    }
}