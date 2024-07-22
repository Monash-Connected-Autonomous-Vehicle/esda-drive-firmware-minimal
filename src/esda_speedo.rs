use core::{cell::RefCell, f32::consts::PI, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
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
pub static ENCODER_LEFT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_TICKS: AtomicF32 = AtomicF32::new(0.0);

// Available Pins: 34, 33, 32
pub const ENCODER_RIGHT_A_PIN: u8 = 34;
pub const ENCODER_RIGHT_D_PIN: u8 = 33;
pub static ENCODER_RIGHT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_TICKS: AtomicF32 = AtomicF32::new(0.0);

#[task]
pub async fn speedometer(speedo_transmit_signal: &'static Signal<NoopRawMutex, (f32, f32)>) {
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

#[handler]
#[ram] // Equivalent of IRAM_ATTR
pub fn interrupt_handler() {
    critical_section::with(|cs| {
        // Check that the left encoder caused an interrupt (since interrupt is rising edge, does not debounce)
        if ENCODER_LEFT_A
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
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
            .is_interrupt_set()
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
