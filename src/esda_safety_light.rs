use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_hal::{gpio::GpioPin, mcpwm::operator::PwmPin};

pub const SAFETY_LIGHT_PIN: u8 = 10;
pub const SAFETY_LIGHT_ON_TIMESTAMP: u16 = 1000;
pub const SAFETY_LIGHT_BLINK_TIMESTAMP: u16 = 500;
pub const SAFETY_LIGHT_OFF_TIMESTAMP: u16 = 0;

/// PWM Driver handle for safety light
pub static SAFETY_LIGHT_PWM_HANDLE: Mutex<
    RefCell<
        Option<PwmPin<'_, GpioPin<SAFETY_LIGHT_PIN>, esp_hal::peripherals::MCPWM1, 2, true>>,
    >,
> = Mutex::new(RefCell::new(None));

pub enum SafetyLightMode {
    Off,
    Blink,
    On
}

#[task]
pub async fn safety_light_handler(safety_light_mode_signal: &'static Signal<NoopRawMutex, SafetyLightMode>) {
    // Default to SafetyLightMode::On
    critical_section::with(|cs| {
        if let Some(mut safety_light_pwm_handle) = SAFETY_LIGHT_PWM_HANDLE.borrow_ref_mut(cs).take() {
            // 100% Duty Cycle
            safety_light_pwm_handle.set_timestamp(1000);

            // Restore the mutex
            SAFETY_LIGHT_PWM_HANDLE.replace(cs, Some(safety_light_pwm_handle));
        }
    });

    loop {
        // Wait for an instruction to change the mode of the safety light
        let new_mode = safety_light_mode_signal.wait().await;
        // Match the mode to the appropriate pwm timestamp
        let new_pwm_timestamp: u16 = match new_mode {
            SafetyLightMode::Off => SAFETY_LIGHT_OFF_TIMESTAMP,
            SafetyLightMode::Blink => SAFETY_LIGHT_BLINK_TIMESTAMP,
            SafetyLightMode::On => SAFETY_LIGHT_ON_TIMESTAMP,
        };

        // Apply it
        critical_section::with(|cs| {
            if let Some(mut safety_light_pwm_handle) = SAFETY_LIGHT_PWM_HANDLE.borrow_ref_mut(cs).take() {
                // 100% Duty Cycle
                safety_light_pwm_handle.set_timestamp(new_pwm_timestamp);

                // Restore the mutex
                SAFETY_LIGHT_PWM_HANDLE.replace(cs, Some(safety_light_pwm_handle));
            }
        });
    }
}