use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    peripherals::UART1,
    uart::{self, UartRx, UartTx},
    Async,
};
use esp_println::println;

use crate::{
    esda_interface::{self},
    esda_throttle,
};

// rx_fifo_full_threshold
pub(crate) const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
pub(crate) const AT_CMD: u8 = 0x04;

pub const TX_PIN: u8 = 2;
pub const RX_PIN: u8 = 15;

#[embassy_executor::task]
pub(crate) async fn writer(tx: UartTx<'static, UART1, Async>,
    _serial_forwarding_signal: &'static Signal<NoopRawMutex, esda_interface::ESDAMessage>,
    speedo_transmit_signal: &'static Signal<NoopRawMutex, (f32, f32)>,
) {
    use core::fmt::Write;
    loop {
        // Wait until the speedo task tells us to send a speed update
        let (left_vel, right_vel) = speedo_transmit_signal.wait().await;
        speedo_transmit_signal.reset();

        let left = esda_interface::ESDAMessage {
            id: esda_interface::ESDAMessageID::CurrentVelLeft,
            data: left_vel,
        };
        let right = esda_interface::ESDAMessage {
            id: esda_interface::ESDAMessageID::CurrentVelRight,
            data: right_vel,
        };
        println!("Left {:?}, Right: {:?}\r\n", left, right);
        // write!(&mut tx, "{:?}\r\n", &left).unwrap()
    }
    // embedded_io_async::Write::write(
    //     &mut tx,
    //     b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
    // )
    // .await
    // .unwrap();
    // embedded_io_async::Write::flush(&mut tx).await.unwrap();
    // loop {
    //     let bytes_read = signal.wait().await;
    //     signal.reset();
    //     write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
    //     embedded_io_async::Write::flush(&mut tx).await.unwrap();
    // }
}

#[embassy_executor::task]
pub(crate) async fn reader(
    mut rx: UartRx<'static, UART1, Async>,
    throttle_command_signal: &'static Signal<NoopRawMutex, esda_throttle::ThrottleSetCommand>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut read_buffer: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let read_result = embedded_io_async::Read::read(&mut rx, &mut read_buffer[offset..]).await;
        match read_result {
            // If we successfully read from the read buffer
            Ok(len) => {
                esp_println::dbg!("RAW({len} bytes, data: {:?})", &read_buffer[..offset]);
                // Loop over the messages
                for message_offset in 0..len / esda_interface::MESSAGE_SIZE {
                    // NOTE: Should be little endian :fingers_crossed_emoji:
                    match esda_interface::ESDAMessage::from_le_bytes(
                        &read_buffer[0 + message_offset..esda_interface::MESSAGE_SIZE + message_offset],
                    ) {
                        // If we got a valid message
                        Ok(message) => {
                            match message.id {
                                // Forward throttle commands to throttle driver via signalling channel
                                esda_interface::ESDAMessageID::SetTargetVelLeft => {
                                    throttle_command_signal.signal(
                                        esda_throttle::ThrottleSetCommand::SetThrottleLeft {
                                            new_throttle: message.data,
                                        },
                                    )
                                }
                                esda_interface::ESDAMessageID::SetTargetVelRight => {
                                    throttle_command_signal.signal(
                                        esda_throttle::ThrottleSetCommand::SetThrottleRight {
                                            new_throttle: message.data,
                                        },
                                    )
                                }
                                esda_interface::ESDAMessageID::ESTOP => {
                                    println!("SERIAL_READER: Received E-Stop Signal, idling ESCs and ignoring all further throttle commands!");
                                    throttle_command_signal
                                        .signal(esda_throttle::ThrottleSetCommand::EngageEStop)
                                }
                                _ => {}
                            }
                        }
                        Err(invalid_data) => {
                            println!("SERIAL_READER: Got invalid message {invalid_data:?}")
                        }
                    }
                }

                offset += len;
            }
            // Otherwise log the error
            Err(e) => println!("Serial RX Error: {:?}", e),
        }
    }
}
