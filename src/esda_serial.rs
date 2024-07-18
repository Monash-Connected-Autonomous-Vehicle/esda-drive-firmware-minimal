use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    peripherals::UART0,
    uart::{UartRx, UartTx},
    Async,
};

use crate::esda_interface;

const MESSAGE_SIZE: usize = 8;

// rx_fifo_full_threshold
pub(crate) const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
pub(crate) const AT_CMD: u8 = 0x04;

#[embassy_executor::task]
pub(crate) async fn writer(
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
pub(crate) async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut read_buffer: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let read_result = embedded_io_async::Read::read(&mut rx, &mut read_buffer[offset..]).await;
        match read_result {
            // If we successfully read from the read buffer
            Ok(len) => {
                // Loop over the messages
                for message_offset in 0..len / MESSAGE_SIZE {
                    // NOTE: Should be little endian :fingers_crossed_emoji:
                    let message = esda_interface::ESDAMessage::from_le_bytes(
                        &read_buffer[0 + message_offset..MESSAGE_SIZE + message_offset],
                    );
                    esp_println::print!("Read message {message:?}")
                }
                esp_println::println!("RAW({len} bytes, data: {:?})", &read_buffer[..offset]);

                offset += len;
                // Signal the read length (not 100% sure what this achieves
                signal.signal(len);
            }
            // Otherwise log the error
            Err(e) => esp_println::println!("Serial RX Error: {:?}", e),
        }
    }
}
