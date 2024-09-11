// MCAV - Asterius MCU Firmware - esda_throttle
//
// Authors: BMCG0011

use core::{
    cell::{RefCell, RefMut},
    sync::atomic::{AtomicBool, Ordering},
};

use critical_section::Mutex;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::{gpio::{GpioPin, Input, InputPin, Level}, mcpwm::operator::PwmPin};
use esp_println::println;

use crate::{esda_interface, esda_safety_light, esda_serial, pwm_extension::PwmPinExtension};

pub const COMMAND_BUFFER_SIZE: usize = 4;

/// Utility enum used by tasks to [Signal](embassy_sync::signal::Signal) commands to the [throttle_driver](crate::esda_throttle::throttle_driver) task
pub enum ThrottleCommand {
    /// Arm both left and right ESCs
    ArmESCs,
    /// Set throttle amount on left wheel
    SetThrottleLeft { new_throttle: f32 },
    /// Set throttle amount on right wheel
    SetThrottleRight { new_throttle: f32 },
    /// Engage the E-STOP, setting both left and right throttles to neutral (wherein ESCs will idle brake)
    EngageEStop,
}

pub const THROTTLE_PWM_FREQUENCY_KHZ: u32 = 50;
/// Precomputed reference value
pub const THROTTLE_PWM_MIN_POS_WIDTH_INCREMENT: f32 =
    1.0 / (THROTTLE_PWM_FREQUENCY_KHZ * 1000 * 100) as f32;

pub const THROTTLE_PWM_PIN_LEFT: u8 = 16;
pub const THROTTLE_PWM_PIN_RIGHT: u8 = 26;

/// Static value used to track ESC arm state
pub static ESCS_ARMED: AtomicBool = AtomicBool::new(false);

/// PWM Driver handle for left throttle
pub static THROTTLE_PWM_HANDLE_LEFT: Mutex<
    RefCell<
        Option<PwmPin<'_, GpioPin<THROTTLE_PWM_PIN_LEFT>, esp_hal::peripherals::MCPWM0, 0, true>>,
    >,
> = Mutex::new(RefCell::new(None));
/// PWM Driver handle for left throttle
pub static THROTTLE_PWM_HANDLE_RIGHT: Mutex<
    RefCell<
        Option<PwmPin<'_, GpioPin<THROTTLE_PWM_PIN_RIGHT>, esp_hal::peripherals::MCPWM0, 1, true>>,
    >,
> = Mutex::new(RefCell::new(None));

/// Driver task responsible for enacting [ThrottleCommands](crate::esda_throttle::ThrottleCommand) (e.g. set throttle on side, arming/disarming the escs) per signals from other tasks (i.e. [serial_reader](crate::esda_serial::serial_reader), [wireless_receiver](crate::esda_wireless::wireless_receiver))
#[task]
pub async fn throttle_driver(
    throttle_command_channel: &'static Channel<NoopRawMutex, ThrottleCommand, {COMMAND_BUFFER_SIZE}>,
    safety_light_mode_signal: &'static Signal<NoopRawMutex, Level>
) {
    // Start by sending ourselves a signal to arm the escs
    throttle_command_channel.send(ThrottleCommand::ArmESCs).await;

    loop {
        // Wait until we receive a command to change the throttle
        let received_throttle_command = throttle_command_channel.receive().await;
        // println!("THROTTLE_DRIVER<DEBUG>: Received Throttle Command: {:?}", received_throttle_command);

        critical_section::with(|cs| {
            // Process the command
            match received_throttle_command {
                // Set the escs to neutral for 3 seconds
                ThrottleCommand::ArmESCs => {
                    println!("THROTTLE_DRIVER<DEBUG>: Arming ESCs...");
                    // Set left pwm to neutral
                    set_pwm_microseconds(THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs), 1500.0);
                    // Set right pwm to neutral
                    set_pwm_microseconds(THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs), 1500.0);
                },
                ThrottleCommand::EngageEStop => {
                    println!("THROTTLE_DRIVER: ESTOP SIGNAL RECEIVED - IDLING ESCS AND IGNORING ALL FURTHER THROTTLE INSTRUCTIONS!");
                    // Set left pwm to neutral
                    set_pwm_microseconds(THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs), 1500.0);
                    // Set right pwm to neutral
                    set_pwm_microseconds(THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs), 1500.0);
                    // Ignore any subsequent throttle commands that may have already been received

                    THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs).take();  // Disable the left motor
                    THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs).take(); // Disable the right motor
                    // throttle_command_channel.clear();
                    // // Set the safety light to solid
                    // safety_light_mode_signal.signal(Level::Low);
                    // Stop listening for further throttle commands
                    return
                }
                // Simple throttle changes can be applied as-is, provided the escs are armed
                ThrottleCommand::SetThrottleLeft { new_throttle } => {
                    if !ESCS_ARMED.load(Ordering::SeqCst) {
                        println!(
                            "THROTTLE_DRIVER<WARN>: Ignoring new left throttle value {new_throttle} as ESCs are not armed"
                        );
                    } else {
                        println!(
                            "THROTTLE_DRIVER<DEBUG>: Setting left throttle to {}",
                            new_throttle
                        );
                        set_pwm_microseconds(
                            THROTTLE_PWM_HANDLE_LEFT.borrow_ref_mut(cs),
                            new_throttle,
                        );
                    }
                }
                ThrottleCommand::SetThrottleRight { new_throttle } => {
                    if !ESCS_ARMED.load(Ordering::SeqCst) {
                        println!(
                            "THROTTLE_DRIVER<WARN>: Ignoring new right throttle value {new_throttle} as ESCs are not armed"
                        );
                    } else {
                        println!(
                            "THROTTLE_DRIVER<DEBUG>: Setting right throttle to {}",
                            new_throttle
                        );
                        set_pwm_microseconds(
                            THROTTLE_PWM_HANDLE_RIGHT.borrow_ref_mut(cs),
                            new_throttle,
                        );
                    }
                }
            }
        });

        // Wait three seconds for the escs to arm
        if matches!(received_throttle_command, ThrottleCommand::ArmESCs) {
            println!("THROTTLE_DRIVER: ESCs set to idle, Allowing 3 seconds for ESCs to Arm themselves...");
            Timer::after(Duration::from_millis(3_000)).await;
            ESCS_ARMED.store(true, Ordering::SeqCst);
            println!("THROTTLE_DRIVER: ESCs Armed!");
        }
    }
}

/// Utility function for converting +width throttle descriptions (microseconds) into their equivalent duty cycle
fn pos_width_to_duty(pos_width: f32) -> u16 {
    (pos_width * THROTTLE_PWM_FREQUENCY_KHZ as f32) as u16
}

/// Internal Utility function for setting throttle on a given side using a [RefMut<](core::cell::RefMut)[PwmPin](esp_hal::mcpwm::operator)[>](core::cell::RefMut) and the new throttle (+width in microseconds)
fn set_pwm_microseconds<T>(mut pwm_driver_cellref: RefMut<Option<T>>, new_throttle: f32)
where
    T: PwmPinExtension,
{
    if let Some(mut pwm_driver_handle) = pwm_driver_cellref.take() {
        let duty = pos_width_to_duty(new_throttle);
        println!("ESDA_THROTTLE: Setting pwm to {new_throttle} ({duty}%))");
        // Apply the throttle to the pwm output
        pwm_driver_handle.set_timestamp(new_throttle as u16);

        // Return the pwm driver to the mutex
        pwm_driver_cellref.replace(pwm_driver_handle);
    }
    // Handle the event that the mutex had no handle in it
    // This can occur in two (theoretically impossible) cases:
    // - another task has taken the handle (there are no other tasks which use these mutexes)
    // - the pwm code has not even been initialised yet (this task is started after that happens)
    else {
        println!("THROTTLE_DRIVER(SET_PWM_MICROSECONDS)<ERROR>: Failed to set throttle - pwm handle is uninitialised or has been taken by another task");
    }
}

#[task]
/// Task responsible for waiting for an ESTOP press, notifying the ROS2 stack and telling the throttle task to stop the motors.
pub async fn estop_button_handler(
    mut estop_button_pin: Input<'static, impl InputPin>, 
    throttle_command_channel: &'static Channel<NoopRawMutex, ThrottleCommand, {COMMAND_BUFFER_SIZE}>, 
    serial_forwarding_channel: &'static Channel<NoopRawMutex, esda_interface::ESDAMessage, {esda_serial::SERIAL_FORWARDING_BUFFER_SIZE}>,) {
    loop {
        // Wait for the estop to be pressed
        estop_button_pin.wait_for_falling_edge().await;
        println!("ESTOP_BUTTON_HANDLER: ESTOP BUTTON PRESSED, ENGAGING ESTOP!");
        // Engage the E-Stop
        throttle_command_channel.send(ThrottleCommand::EngageEStop).await;
        // Forward the E-Stop signal over serial to the ROS2 stack
        serial_forwarding_channel.send(esda_interface::ESDAMessage { id: esda_interface::ESDAMessageID::ESTOP, data: 1 }).await;
    }
}