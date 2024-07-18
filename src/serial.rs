use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    gpio::Io,
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
    peripherals::{self, Peripherals, TIMG1, UART0},
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroupInstance,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    uart::{
        config::{AtCmdConfig, Config},
        Uart, UartRx, UartTx,
    },
    Async,
    Blocking
};

use static_cell::StaticCell;

// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

pub fn init<TxPin: OutputPin, RxPin: InputPin, Timg: TimerGroupInstance>(
    uart: UART0,
    tx_pin: impl Peripheral<P = TxPin> + 'static,
    rx_pin: impl Peripheral<P = RxPin> + 'static,
    timer_group: impl Peripheral<P = Timg>,
    clocks: &Clocks,
    spawner: Spawner
) -> Spawner {
    let timg1 = TimerGroup::new(timer_group, &clocks, None);
    let timer0 = OneShotTimer::new(timg1.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let uart_config = Config::default().rx_fifo_full_threshold(READ_BUF_SIZE as u16);

    let mut uart0 =
    Uart::new_async_with_config(uart, uart_config, &clocks, tx_pin, rx_pin).unwrap();
uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));

    let (tx, rx) = uart0.split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
    return spawner
}

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(
        &mut tx,
        b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
    )
    .await
    .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                esp_println::println!("Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;
                signal.signal(len);
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}
