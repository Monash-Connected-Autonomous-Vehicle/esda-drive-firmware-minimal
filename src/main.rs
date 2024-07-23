//! Embassy ESP-NOW Example
//!
//! Broadcasts, receives and sends messages via esp-now in an async way
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2

//% FEATURES: async embassy embassy-generic-timers esp-wifi esp-wifi/async esp-wifi/embassy-net esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/esp-now
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]
#![allow(dead_code)]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Event, GpioPin, Input, Io, Pull},
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
    uart::{self, config::AtCmdConfig},
};

use esp_wifi::{
    esp_now::{PeerInfo, BROADCAST_ADDRESS},
    initialize, EspWifiInitFor,
};

use esp_println::println;
use static_cell::StaticCell;

/// Module containing interface types for communicating with controller (via esp-now) and the computer (via serial)
mod esda_interface;

// Don't ask
mod pwm_extension;

/// Module containing code for esda serial interface
mod esda_serial;

/// Module containing throttle pwm driver
mod esda_throttle;

/// Module for handling esda wireless implementation
mod esda_wireless;

// Speedometer derivation code
mod esda_speedo;

// Register the mk_static utility macro
// NOTE: If using nightly compiler then use use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html instead
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    println!("Beginning Asterius Firmware Initialisation...");

    println!("Initialising Runtime...");
    // Initialise Peripherals handle
    let peripherals = Peripherals::take();

    // Initialise system control handle
    let system = SystemControl::new(peripherals.SYSTEM);
    // Configure clocks and lock settings until next reboot
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    // Initialise main IO driver
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Initialise Timers and the embassy runtime
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    // Initialise rotary encoders
    println!("Initialisng Rotary Encoders...");
    let encoder_left_a = io.pins.gpio18;
    let encoder_left_d = io.pins.gpio19;
    let encoder_right_a = io.pins.gpio34;
    let encoder_right_d = io.pins.gpio33;
    let mut encoder_left_a = Input::new(encoder_left_a, Pull::Down);
    let encoder_left_d = Input::new(encoder_left_d, Pull::Down);
    let mut encoder_right_a = Input::new(encoder_right_a, Pull::Down);
    let encoder_right_d = Input::new(encoder_right_d, Pull::Down);

    // Initialise global handles to encoders
    // critical_section::with(|cs| {
    //     encoder_left_a.listen(Event::RisingEdge);
    //     esda_speedo::ENCODER_LEFT_A
    //         .borrow_ref_mut(cs)
    //         .replace(encoder_left_a);
    // });
    // critical_section::with(|cs| {
    //     encoder_right_a.listen(Event::RisingEdge);
    //     esda_speedo::ENCODER_RIGHT_A
    //         .borrow_ref_mut(cs)
    //         .replace(encoder_right_a);
    // });
    // critical_section::with(|cs| {
    //     esda_speedo::ENCODER_LEFT_D
    //         .borrow_ref_mut(cs)
    //         .replace(encoder_left_d);
    // });
    // critical_section::with(|cs| {
    //     esda_speedo::ENCODER_RIGHT_D
    //         .borrow_ref_mut(cs)
    //         .replace(encoder_right_d);
    // });
    println!("Spawning speedometer tasks...");
    // Initialise signal channel for throttle updates
    static SPEEDO_TICK_SIGNAL: StaticCell<Signal<NoopRawMutex, (f32, f32)>> = StaticCell::new();
    let speedo_tick_signal = &*SPEEDO_TICK_SIGNAL.init(Signal::new());

    // Spawn tick counters
    spawner.spawn(esda_speedo::tick_counter(&esda_speedo::ENCODER_LEFT_TICKS, encoder_left_a, encoder_left_d, esda_speedo::Direction::Clockwise)).unwrap();
    spawner.spawn(esda_speedo::tick_counter(&esda_speedo::ENCODER_RIGHT_TICKS, encoder_right_a, encoder_right_d, esda_speedo::Direction::Anticlockwise)).unwrap();

    // Spawn Main Speedometer Task
    spawner.spawn(esda_speedo::speedometer(&speedo_tick_signal)).ok();
    println!("Encoders Initialised...");

    println!("Initialising throttle pwm pins...");
    let left_throttle_pin: GpioPin<{ esda_throttle::THROTTLE_PWM_PIN_LEFT }> =
        GpioPin::<{ esda_throttle::THROTTLE_PWM_PIN_LEFT }>;
    let right_throttle_pin: GpioPin<{ esda_throttle::THROTTLE_PWM_PIN_RIGHT }> =
        GpioPin::<{ esda_throttle::THROTTLE_PWM_PIN_RIGHT }>;
    // Initialise pwm clock with esp32's primary XTAL clock frequency of 40MHz
    let pwm_clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40.MHz()).unwrap();
    let mut mcpwm_left = McPwm::new(peripherals.MCPWM0, pwm_clock_cfg);
    let mut mcpwm_right = McPwm::new(peripherals.MCPWM1, pwm_clock_cfg);
    // Link operators to timers
    mcpwm_left.operator0.set_timer(&mcpwm_left.timer0);
    mcpwm_right.operator0.set_timer(&mcpwm_right.timer0);
    // Link operators to pins
    let throttle_driver_left = mcpwm_left
        .operator0
        .with_pin_a(left_throttle_pin, PwmPinConfig::UP_ACTIVE_HIGH);
    let throttle_driver_right = mcpwm_right
        .operator0
        .with_pin_a(right_throttle_pin, PwmPinConfig::UP_ACTIVE_HIGH);
    // Finish initialising pwm clocks, use 50kHz (i may be off by an order of magnitude)
    let pwm_timer_clock_config = pwm_clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 50.kHz())
        .unwrap();
    mcpwm_left.timer0.start(pwm_timer_clock_config);
    mcpwm_right.timer0.start(pwm_timer_clock_config);

    critical_section::with(|cs| {
        esda_throttle::THROTTLE_PWM_HANDLE_LEFT
            .borrow_ref_mut(cs)
            .replace(throttle_driver_left);
        esda_throttle::THROTTLE_PWM_HANDLE_RIGHT
            .borrow_ref_mut(cs)
            .replace(throttle_driver_right);
    });

    // Initialise signal channel for throttle updates
    static THROTTLE_COMMAND_SIGNAL: StaticCell<
        Signal<NoopRawMutex, esda_throttle::ThrottleSetCommand>,
    > = StaticCell::new();
    let throttle_command_signal = &*THROTTLE_COMMAND_SIGNAL.init(Signal::new());

    // Spawn throttle driver task
    spawner
        .spawn(esda_throttle::throttle_driver(&throttle_command_signal))
        .unwrap();

    println!("Throttle driver initialised!");

    println!("Initialising UART Connection to ROS2 Stack...");
    // Initialise signal channel for forwarding espnow messages to serial
    static SERIAL_FORWARDING_SIGNAL: StaticCell<
    Signal<NoopRawMutex, esda_interface::ESDAMessage>,
> = StaticCell::new();
let serial_forwarding_signal = &*SERIAL_FORWARDING_SIGNAL.init(Signal::new());


    // Define pins for UART connection in IO MUX (Pins 1 and 3 are Standard)
    let (tx_pin, rx_pin) = (io.pins.gpio2, io.pins.gpio15);

    // Define configuration for UART
    let config =
        uart::config::Config::default().rx_fifo_full_threshold(esda_serial::READ_BUF_SIZE as u16);

    // Initialise UART
    let mut uart1 =
        uart::Uart::new_async_with_config(peripherals.UART1, config, &clocks, tx_pin, rx_pin)
            .unwrap();
    uart1.set_at_cmd(AtCmdConfig::new(
        None,
        None,
        None,
        esda_serial::AT_CMD,
        None,
    ));

    // Split UART handle into TX and RX Channels
    let (tx, rx) = uart1.split();

    println!("Spawning UART Tasks...");
    spawner
        .spawn(esda_serial::reader(rx, &throttle_command_signal))
        .ok();
    spawner
        .spawn(esda_serial::writer(tx, &serial_forwarding_signal, &speedo_tick_signal))
        .ok();
    println!("Finished UART Initialisation!");

    println!("Starting esp-now Initialisation");
    let timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None)
            .timer1
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

}
