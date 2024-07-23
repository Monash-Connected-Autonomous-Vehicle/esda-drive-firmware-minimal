// MCAV - Asterius MCU Firmware - pwm_extension
//
// Contains extension trait for PwmPin, allowing for esda_throttle::apply_throttle_command to be pin-agnostic
//
// Authors: Nigecat, BMCG0011

use esp_hal::{
    gpio::OutputPin,
    mcpwm::{operator::PwmPin, PwmPeripheral},
};

pub trait PwmPinExtension {
    fn set_timestamp(&mut self, value: u16);
}

impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool> PwmPinExtension
    for PwmPin<'d, Pin, PWM, OP, IS_A>
{
    fn set_timestamp(&mut self, value: u16) {
        <PwmPin<'d, Pin, PWM, OP, IS_A>>::set_timestamp(self, value);
    }
}
