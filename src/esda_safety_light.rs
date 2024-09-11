use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_hal::{gpio::{GpioPin, Level, Output}, mcpwm::operator::PwmPin};

pub const SAFETY_LIGHT_PIN: u8 = 27;
pub const SAFETY_LIGHT_ON_TIMESTAMP: u16 = 1000;
pub const SAFETY_LIGHT_BLINK_TIMESTAMP: u16 = 500;
pub const SAFETY_LIGHT_OFF_TIMESTAMP: u16 = 0;

/// Pin Driver handle for safety light
pub static SAFETY_LIGHT_HANDLE: Mutex<
    RefCell<
        Option<Output<'_, GpioPin<SAFETY_LIGHT_PIN>>>,
    >,
> = Mutex::new(RefCell::new(None));

pub enum SafetyLightMode {
    Off,
    Blink,
    On
}

#[task]
pub async fn safety_light_handler(safety_light_mode_signal: &'static Signal<NoopRawMutex, Level>) {
    

    // Default to SafetyLightMode::On
    critical_section::with(|cs| {
        let mut safety_light_handle_cell = SAFETY_LIGHT_HANDLE.borrow_ref_mut(cs);
        if let Some(mut safety_light_pwm_handle) = safety_light_handle_cell.take() {
            // 100% Duty Cycle
            safety_light_pwm_handle.set_level(Level::High);

            // Restore the mutex
            safety_light_handle_cell.replace(safety_light_pwm_handle);
        }
    });

    loop {
        // Wait for an instruction to change the mode of the safety light
        let new_mode = safety_light_mode_signal.wait().await;

        // Apply it
        critical_section::with(|cs| {
            let mut safety_light_handle_cell = SAFETY_LIGHT_HANDLE.borrow_ref_mut(cs);
            if let Some(mut safety_light_pwm_handle) = safety_light_handle_cell.take() {
                safety_light_pwm_handle.set_level(new_mode);

                // Restore the mutex
                safety_light_handle_cell.replace(safety_light_pwm_handle);
            }
        });
    }
}