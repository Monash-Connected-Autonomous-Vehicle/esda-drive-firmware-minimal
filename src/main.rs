//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers

#![no_std]
#![no_main]
#![allow(dead_code)]

use core::{cell::RefCell, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Event, GpioPin, Input, Io, Level, Pull},
    macros::ram,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    uart::{self, config::AtCmdConfig},
};
use esp_println::println;
use static_cell::StaticCell;

/// Module containing interface types for communicating with controller (via esp-now) and the computer (via serial)
mod esda_interface;

// Don't ask
mod pwm_extension;

/// Module containing code for esda serial interface
mod esda_serial;
mod esda_throttle;

// 18, 19, 21
const ENCODER_LEFT_A_PIN: u8 = 18;
const ENCODER_LEFT_D_PIN: u8 = 19;
static ENCODER_LEFT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_LEFT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_LEFT_TICKS: AtomicF32 = AtomicF32::new(0.0);

// 34, 33, 32
const ENCODER_RIGHT_A_PIN: u8 = 34;
const ENCODER_RIGHT_D_PIN: u8 = 33;
static ENCODER_RIGHT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_RIGHT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
static ENCODER_RIGHT_TICKS: AtomicF32 = AtomicF32::new(0.0);

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
    // Register General Hardware Interrupt Handler
    io.set_interrupt_handler(handler);

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
    critical_section::with(|cs| {
        encoder_left_a.listen(Event::RisingEdge);
        ENCODER_LEFT_A.borrow_ref_mut(cs).replace(encoder_left_a);
    });
    critical_section::with(|cs| {
        encoder_right_a.listen(Event::RisingEdge);
        ENCODER_RIGHT_A.borrow_ref_mut(cs).replace(encoder_right_a);
    });
    critical_section::with(|cs| {
        ENCODER_LEFT_D.borrow_ref_mut(cs).replace(encoder_left_d);
    });
    critical_section::with(|cs| {
        ENCODER_RIGHT_D.borrow_ref_mut(cs).replace(encoder_right_d);
    });

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
    // Initialise signal channel for estop
    static ESTOP_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();

    // Spawn throttle driver task
    spawner
        .spawn(esda_throttle::throttle_driver(&throttle_command_signal))
        .unwrap();
    println!("Fully Initialised!");

    println!("Encoder Init complete!");

    println!("Initialising UART Connection to PC");
    // Define pins for UART connection in IO MUX (Pins 1 and 3 are Standard)
    let (tx_pin, rx_pin) = (io.pins.gpio1, io.pins.gpio3);

    // Define configuration for UART
    let config =
        uart::config::Config::default().rx_fifo_full_threshold(esda_serial::READ_BUF_SIZE as u16);

    // Initialise UART
    let mut uart0 =
        uart::Uart::new_async_with_config(peripherals.UART0, config, &clocks, tx_pin, rx_pin)
            .unwrap();
    uart0.set_at_cmd(AtCmdConfig::new(
        None,
        None,
        None,
        esda_serial::AT_CMD,
        None,
    ));

    // Split UART handle into TX and RX Channels
    let (tx, rx) = uart0.split();

    println!("Spawning UART Tasks...");
    spawner
        .spawn(esda_serial::reader(rx, &throttle_command_signal))
        .ok();
    spawner.spawn(esda_serial::writer(tx)).ok();
    println!("Finished UART Initialisation!");

    println!("Fully Initialised!");
}

#[handler]
#[ram] // Equivalent of IRAM_ATTR
fn handler() {
    println!(
        "GPIO Interrupt with priority {}",
        esp_hal::xtensa_lx::interrupt::get_level()
    );

    critical_section::with(|cs| {
        // Check that the left encoder caused an interrupt (since interrupt is rising edge, does not debounce)
        if ENCODER_LEFT_A
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_high()
        {
            ENCODER_LEFT_TICKS
                .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |current_ticks| {
                    Some(
                        match ENCODER_LEFT_D.borrow_ref(cs).as_ref().unwrap().get_level() {
                            Level::Low => current_ticks + 1.0,
                            Level::High => current_ticks - 1.0,
                        },
                    )
                })
                .unwrap();
            // Reset the left encoder's interrupt status
            ENCODER_LEFT_A
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        }
    });

    critical_section::with(|cs| {
        // Check that the left encoder caused an interrupt (since interrupt is rising edge, does not debounce)
        if ENCODER_RIGHT_A
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_high()
        {
            ENCODER_RIGHT_TICKS
                .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |current_ticks| {
                    Some(
                        match ENCODER_RIGHT_D.borrow_ref(cs).as_ref().unwrap().get_level() {
                            Level::Low => current_ticks + 1.0,
                            Level::High => current_ticks - 1.0,
                        },
                    )
                })
                .unwrap();
            // Reset the right encoder's interrupt status
            ENCODER_RIGHT_A
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        }
    });
}
