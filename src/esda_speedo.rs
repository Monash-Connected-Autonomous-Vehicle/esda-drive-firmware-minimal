use core::pin;
use core::{cell::RefCell, f32::consts::PI, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{InputPin, RtcInputPin};
use esp_hal::peripheral::PeripheralRef;
use esp_hal::prelude::ram;
use esp_hal::{
    gpio::{GpioPin, Input, Level},
    macros::handler,
};
use esp_println::println;

/// Number of encoder ticks per full axial revolution
const ENCODER_TICKS_PER_REVOLUTION: f32 = 1024.0;
/// Radius of wheels used to calculate linear velocity from angular velocity
const WHEEL_RADIUS: f32 = 0.13;
/// Frequency at which velocity updates are sent to ROS2
const VELOCITY_SAMPLE_FREQUENCY: f32 = 0.25;
/// Time between velocity updates sent to ROS2
const VELOCITY_SAMPLE_PERIOD: u64 = (1E3 / VELOCITY_SAMPLE_FREQUENCY as f32) as u64;

// Available Pins: 18, 19, 21
pub const ENCODER_LEFT_A_PIN: u8 = 18;
pub const ENCODER_LEFT_D_PIN: u8 = 19;
/// Which direction of rotation should be considered 'positive'
pub const ENCODER_LEFT_POS_DIR: Direction = Direction::Clockwise;
pub static ENCODER_LEFT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_TICKS: AtomicF32 = AtomicF32::new(0.0);

// Available Pins: 34, 33, 32
pub const ENCODER_RIGHT_A_PIN: u8 = 34;
pub const ENCODER_RIGHT_D_PIN: u8 = 33;
/// Which direction of rotation should be considered 'positive'
pub const ENCODER_RIGHT_POS_DIR: Direction = Direction::Clockwise;
pub static ENCODER_RIGHT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_TICKS: AtomicF32 = AtomicF32::new(0.0);

pub enum Direction {
    Clockwise,
    Anticlockwise
}

#[task]
pub async fn speedometer(speedo_transmit_signal: &'static Signal<NoopRawMutex, (f32, f32)>, ) {
    loop {
        // Read and then reset the left and right tick countersS
        let left_ticks = ENCODER_LEFT_TICKS
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |_| Some(0.0))
            .unwrap();
        let right_ticks = ENCODER_RIGHT_TICKS
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |_| Some(0.0))
            .unwrap();
        let left_velocity = (left_ticks * WHEEL_RADIUS * PI * VELOCITY_SAMPLE_FREQUENCY)
            / (ENCODER_TICKS_PER_REVOLUTION);
        let right_velocity = (right_ticks * WHEEL_RADIUS * PI * VELOCITY_SAMPLE_FREQUENCY)
            / (ENCODER_TICKS_PER_REVOLUTION);
        // println!("L: {left_ticks}t|{left_velocity}m/s R: {right_ticks}t|{right_velocity}m/s");
        speedo_transmit_signal.signal((left_velocity, right_velocity));
        // Sleep until we next need to derive velocity
        Timer::after(Duration::from_millis(VELOCITY_SAMPLE_PERIOD)).await;
    }
}

#[task]
pub async fn tick_counter(tick_counter: &'static AtomicF32, mut pin_a: Input<'static, impl InputPin>, pin_d: Input<'static, impl InputPin>, pos_direction: Direction) {
    loop {
        // Wait for rising edge on the pin
        pin_a.wait_for_rising_edge().await;
        // If the second pin is high then the tick was clockwise,
        // Otherwise it was anticlockwise
        tick_counter.fetch_update(Ordering::SeqCst, Ordering::SeqCst, | current_ticks: f32 | {
            Some(
                current_ticks + 
                match pin_d.get_level() {
                    Level::Low => -1.0,
                    Level::High => 1.0
                }
                // Invert if the direction is inverted
                * match pos_direction {
                    Direction::Clockwise => -1.0,
                    Direction::Anticlockwise => 1.0,
                },
            )
        }).unwrap();
    }
}