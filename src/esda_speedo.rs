use core::{cell::RefCell, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use critical_section::Mutex;
use embassy_executor::task;
use esp_hal::{gpio::{GpioPin, Input, Level}, macros::handler};
use esp_hal::prelude::ram;
use esp_println::println;

// 18, 19, 21
pub const ENCODER_LEFT_A_PIN: u8 = 18;
pub const ENCODER_LEFT_D_PIN: u8 = 19;
pub static ENCODER_LEFT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_LEFT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_LEFT_TICKS: AtomicF32 = AtomicF32::new(0.0);

// 34, 33, 32
pub const ENCODER_RIGHT_A_PIN: u8 = 34;
pub const ENCODER_RIGHT_D_PIN: u8 = 33;
pub static ENCODER_RIGHT_A: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_A_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_D: Mutex<RefCell<Option<Input<GpioPin<{ ENCODER_RIGHT_D_PIN }>>>>> =
    Mutex::new(RefCell::new(None));
pub static ENCODER_RIGHT_TICKS: AtomicF32 = AtomicF32::new(0.0);

#[task]
pub async fn speedometer() {

}

#[handler]
#[ram] // Equivalent of IRAM_ATTR
pub fn encoder_tick_handler() {
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
