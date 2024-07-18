use core::{cell::RefCell, sync::atomic::Ordering};

use atomic_float::AtomicF32;
use core::fmt::Write;
use critical_section::Mutex;
use embassy_executor::{task, Spawner};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::Clocks,
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
    peripherals::{Peripherals, UART0},
    prelude::*,
    uart::{
        config::{AtCmdConfig, Config},
        Uart,
    },
    Blocking,
};

static SERIAL: Mutex<RefCell<Option<Uart<UART0, Blocking>>>> = Mutex::new(RefCell::new(None));

pub fn initialise_serial<TxPin: OutputPin, RxPin: InputPin>(
    uart: UART0,
    tx_pin: impl Peripheral<P = TxPin> + 'static,
    rx_pin: impl Peripheral<P = RxPin> + 'static,
    clocks: &Clocks,
) {
    let serial_config = Config {
        baudrate: 115200,
        ..Default::default()
    }
    .rx_fifo_full_threshold(30);
    let mut uart0 = Uart::new_with_config(
        uart,
        serial_config,
        &clocks,
        Some(serial_interrupt_handler),
        tx_pin,
        rx_pin,
    )
    .unwrap();

    critical_section::with(|cs| {
        uart0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
        uart0.listen_at_cmd();
        uart0.listen_rx_fifo_full();

        SERIAL.borrow_ref_mut(cs).replace(uart0);
    });
}

#[handler]
#[ram] // Equivalent of IRAM_ATTR
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
