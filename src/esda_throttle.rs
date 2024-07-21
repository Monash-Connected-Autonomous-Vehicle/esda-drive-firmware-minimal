use core::cell::{RefCell, RefMut};

use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::{gpio::GpioPin, mcpwm::{operator::PwmPin, McPwm}, peripherals::{MCPWM0, MCPWM1}};
use esp_println::{dbg, println};

use crate::pwm_extension::PwmPinExtension;

pub enum ThrottleSetCommand {
    /// Arm both left and right ESCs
    ArmESCs,
    /// Set throttle amount on left wheel
    SetThrottleLeft { new_throttle: f32 },
    /// Set throttle amount on right wheel
    SetThrottleRight { new_throttle: f32 },
    /// Engage the E-STOP, setting both left and right throttles to neutral (wherein escs will idle brake)
    EngageEStop,
}

pub const THROTTLE_PWM_FREQUENCY_KHZ: u32 = 50;
/// Reference
pub const THROTTLE_PWM_MIN_POS_WIDTH_INCREMENT: f32 = 1.0/(THROTTLE_PWM_FREQUENCY_KHZ*1000*100) as f32;

pub const THROTTLE_PWM_PIN_LEFT: u8 = 26; 
pub const THROTTLE_PWM_PIN_RIGHT: u8 = 16; 

// PWM Driver handles for throttle
pub static THROTTLE_PWM_HANDLE_LEFT: Mutex<RefCell<Option<PwmPin<'_, GpioPin<THROTTLE_PWM_PIN_LEFT>, esp_hal::peripherals::MCPWM0, 0, true>>>> = Mutex::new(RefCell::new(None));
pub static THROTTLE_PWM_HANDLE_RIGHT: Mutex<RefCell<Option<PwmPin<'_, GpioPin<THROTTLE_PWM_PIN_RIGHT>, esp_hal::peripherals::MCPWM1, 0, true>>>> = Mutex::new(RefCell::new(None));

#[task]
pub async fn throttle_driver(throttle_command_signal: &'static Signal<NoopRawMutex, ThrottleSetCommand>) {
    loop {
        // Wait until we receive a command to change the throttle
        let received_throttle_command = throttle_command_signal.wait().await;
        throttle_command_signal.reset();

        critical_section::with(|cs| {
            // Process the command
            match received_throttle_command {
                // Set the escs to neutral for 3 seconds
                ThrottleSetCommand::ArmESCs | ThrottleSetCommand::EngageEStop => {
                        // Set left pwm
                        apply_throttle_set_command(THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs), 1500.0);
                        // Set right pwm
                        apply_throttle_set_command(THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs), 1500.0);
                }
                // Simple throttle changes can be applied as-is
                ThrottleSetCommand::SetThrottleLeft { new_throttle } => {
                        apply_throttle_set_command(THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs), new_throttle);
                }
                ThrottleSetCommand::SetThrottleRight { new_throttle } => {
                        apply_throttle_set_command(THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs), new_throttle);
                }
            }
        });
        
        // Wait three seconds for the escs to arm
        if matches!(received_throttle_command,ThrottleSetCommand::ArmESCs) {
            Timer::after(Duration::from_millis(3_000)).await;
            esp_println::println!("ESCs Armed");
        } else {
            return
        }
    }
}

/// Utility function for converting +width throttle descriptions (microseconds) into their equivalent duty cycle
fn pos_width_to_duty(pos_width: f32) -> u16 {
    (pos_width * THROTTLE_PWM_FREQUENCY_KHZ as f32) as u16
}

/// Utility function for setting throttle on a given side
fn apply_throttle_set_command<T>(mut pwm_driver_cell: RefMut<Option<T>>, new_throttle: f32) 
where T: PwmPinExtension {
    dbg!("Applying throttle set command {throttle_set_command:?}...");
    if let Some(mut pwm_driver_handle) = pwm_driver_cell.take() {
        // Apply the throttle to the pwm output
        pwm_driver_handle.set_timestamp(pos_width_to_duty(new_throttle));

        // Return the pwm driver to the mutex
        pwm_driver_cell.replace(pwm_driver_handle);
        dbg!("Applying throttle set command {throttle_set_command:?}...");
    }
    // Handle the event that the mutex had no handle in it
    // This can occur in two (theoretically impossible) cases:
    // - another task has taken the handle (there are no other tasks which use these mutexes)
    // - the pwm code has not even been initialised yet (this task is started after that happens)
    else {
        esp_println::println!("ERROR: Failed to set throttle for - pwm handle is uninitialised or has been taken by another task");
    }
}