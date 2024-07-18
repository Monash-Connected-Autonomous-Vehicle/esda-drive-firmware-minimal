//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers

#![no_std]
#![no_main]

use core::{cell::RefCell, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Event, Input, Io, Level, Pull},
    macros::ram,
    peripherals::{Peripherals, UART0},
    prelude::*,
    system::SystemControl,
    uart::{
        config::{AtCmdConfig, Config},
        Uart,
    }, Blocking,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use core::fmt::Write;

static SERIAL: Mutex<RefCell<Option<Uart<UART0, Blocking>>>> = Mutex::new(RefCell::new(None));

// 18, 19, 21
static ENCODER_LEFT_A: Mutex<RefCell<Option<Input<gpio::Gpio18>>>> = Mutex::new(RefCell::new(None));
static ENCODER_LEFT_D: Mutex<RefCell<Option<Input<gpio::Gpio19>>>> = Mutex::new(RefCell::new(None));
static ENCODER_LEFT_TICKS: AtomicF32 = AtomicF32::new(0.0);

// 34, 33, 32
static ENCODER_RIGHT_A: Mutex<RefCell<Option<Input<gpio::Gpio34>>>> = Mutex::new(RefCell::new(None));
static ENCODER_RIGHT_D: Mutex<RefCell<Option<Input<gpio::Gpio33>>>> = Mutex::new(RefCell::new(None));
static ENCODER_RIGHT_TICKS: AtomicF32 = AtomicF32::new(0.0);

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let delay = Delay::new(&clocks);

    io.set_interrupt_handler(handler);

    // Default pins for Uart/Serial communication
    let (tx_pin, rx_pin) = (io.pins.gpio1, io.pins.gpio3);

    let serial_config = Config { baudrate: 115200, ..Default::default()}.rx_fifo_full_threshold(30);
    let mut uart0 = Uart::new_with_config(
        peripherals.UART0,
        serial_config,
        &clocks,
        Some(serial_interrupt_handler),
        tx_pin,
        rx_pin,
    ).unwrap();

    critical_section::with(|cs| {
        uart0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
        uart0.listen_at_cmd();
        uart0.listen_rx_fifo_full();

        SERIAL.borrow_ref_mut(cs).replace(uart0);
    });

    let encoder_left_a = io.pins.gpio18;
    let encoder_left_d = io.pins.gpio19;
    let encoder_right_a = io.pins.gpio34;
    let encoder_right_d = io.pins.gpio33;
    let mut encoder_left_a = Input::new(encoder_left_a, Pull::Down);
    let encoder_left_d = Input::new(encoder_left_d, Pull::Down);
    let mut encoder_right_a = Input::new(encoder_right_a, Pull::Down);
    let encoder_right_d = Input::new(encoder_right_d, Pull::Down);

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

    esp_println::println!("Fully Initialised!");

    // let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    // let timer0 = OneShotTimer::new(timg0.timer0.into());
    // let timers = [timer0];
    // let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    // esp_hal_embassy::init(&clocks, timers);

    // spawner.spawn(run()).ok();

    // loop {
    //     esp_println::println!("Bing!");
    //     Timer::after(Duration::from_millis(5_000)).await;
    // }
    loop {
        critical_section::with(|cs| {
            let mut serial = SERIAL.borrow_ref_mut(cs);
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Hello World! Send a single `#` character or send at least 30 characters and see the interrupts trigger.").ok();
        });

        delay.delay(1.secs());
    }

}

#[handler]
#[ram]
fn handler() {
    esp_println::println!(
        "GPIO Interrupt with priority {}",
        esp_hal::xtensa_lx::interrupt::get_level()
    );
    
    critical_section::with(|cs| {
        // Check that the left encoder caused an interrupt (since interrupt is rising edge, does not debounce)
        if ENCODER_LEFT_A.borrow_ref_mut(cs).as_mut().unwrap().is_high() {
            ENCODER_LEFT_TICKS.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |current_ticks| { 
                Some(match ENCODER_LEFT_D.borrow_ref(cs).as_ref().unwrap().get_level() {
                    Level::Low => current_ticks+1.0,
                    Level::High => current_ticks-1.0,
                })
            }).unwrap();
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
        if ENCODER_RIGHT_A.borrow_ref_mut(cs).as_mut().unwrap().is_high() {
            ENCODER_RIGHT_TICKS.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |current_ticks| { 
                Some(match ENCODER_RIGHT_D.borrow_ref(cs).as_ref().unwrap().get_level() {
                    Level::Low => current_ticks+1.0,
                    Level::High => current_ticks-1.0,
                })
            }).unwrap();
            // Reset the right encoder's interrupt status
            ENCODER_RIGHT_A
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .clear_interrupt()
        }
    });
}

#[handler]
pub fn serial_interrupt_handler() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial = serial.as_mut().unwrap();

        let mut cnt = 0;
        while let nb::Result::Ok(_c) = serial.read_byte() {
            cnt += 1;
        }
        writeln!(serial, "Read {} bytes", cnt,).ok();

        writeln!(
            serial,
            "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
            serial.at_cmd_interrupt_set(),
            serial.rx_fifo_full_interrupt_set(),
        )
        .ok();

        serial.reset_at_cmd_interrupt();
        serial.reset_rx_fifo_full_interrupt();
    });
}