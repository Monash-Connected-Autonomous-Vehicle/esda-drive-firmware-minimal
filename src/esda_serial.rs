// MCAV - Asterius MCU Firmware - esda_serial
//
// Authors: BMCG0011

use core::{cell::RefCell, ffi::CStr, str};

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    peripherals::UART1,
    uart::{self, UartRx, UartTx},
    Async,
};
use esp_println::{dbg, println};

use crate::{
    esda_interface::{self},
    esda_throttle,
};

// // rx_fifo_full_threshold
pub(crate) const READ_BUF_SIZE: usize = 64;
// // EOT (CTRL-D)
pub(crate) const AT_CMD: u8 = 0x04;

pub const TX_PIN: u8 = 2;
pub const RX_PIN: u8 = 15;

// /// Globally accessible uart tx handle
pub static UART_TX: Mutex<RefCell<Option<UartTx<'static, UART1, Async>>>> = Mutex::new(RefCell::new(None));

// /// Writer task responsible for forwarding velocity data reported by the [speedometer](crate::esda_speedo::speedometer), to the ROS2 stack via UART
// /// TODO: handle serial forwarding from esp_now (put UartTx in a mutex and make a serial forwarder task)
#[embassy_executor::task]
pub(crate) async fn speedo_serial_writer(
    tx: &'static Mutex<RefCell<Option<UartTx<'static, UART1, Async>>>>,
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
        let mut success = false;
        while !success {
            critical_section::with(|cs| {
                if tx.borrow_ref(cs).is_some() {
                    let mut tx = tx.borrow_ref_mut(cs);
                    let tx = tx.as_mut().unwrap();
                    tx.write_bytes(&left.to_le_bytes()).unwrap();
                    write!(tx, "\r\n").unwrap();
                    tx.write_bytes(&right.to_le_bytes()).unwrap();
                    write!(tx, "\r\n").unwrap();
                    success = true;
                }
            });
        }
    }
}

// Writer task responsible for forwarding [ESDAMessages](crate::esda_interface::ESDAMessages) received [from esp_now](crate::esda_wireless::wireless_receiver), to the ROS2 stack via UART
#[embassy_executor::task]
pub(crate) async fn serial_forwarding_writer(
    tx: &'static Mutex<RefCell<Option<UartTx<'static, UART1, Async>>>>,
    serial_forwarding_signal: &'static Signal<NoopRawMutex, esda_interface::ESDAMessage>,
) {
    use core::fmt::Write;
    loop {
        // Wait for us to be forwarded an ESDAMessage from the serial reader
        let message = serial_forwarding_signal.wait().await;
        let mut success = false;
        while !success {
            critical_section::with(|cs| {
                if tx.borrow_ref(cs).is_some() {
                    write!(tx.borrow_ref_mut(cs).as_mut().unwrap(), "{:?}\r\n", message.to_le_bytes()).unwrap();
                    success = true;
                } 
            });
        } 
    }
}

/// Reader task responsible for decoding and handling [ESDAMessages](crate::esda_interface::ESDAMessage) from the ROS2 Stack, manipulating them as need be and propagating them to the appropriate Signals and Channels
#[embassy_executor::task]
pub(crate) async fn serial_reader(
    mut rx: UartRx<'static, UART1, Async>,
    throttle_command_signal: &'static Signal<NoopRawMutex, esda_throttle::ThrottleCommand>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut read_buffer: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let read_result = embedded_io_async::Read::read(&mut rx, &mut read_buffer[offset..]).await;
        match read_result {
            // If we successfully read from the read buffer
            Ok(len) => {
                // println!("Raw received data: {:?}", &read_buffer[..len]);

                println!(
                    "SERIAL_READER<DEBUG>: Received ({len} bytes, data: {:?})",
                    &read_buffer[..offset]
                );
                // Loop over the messages
                for message_offset in 0..len / esda_interface::MESSAGE_SIZE {
                    // NOTE: Should be little endian :fingers_crossed_emoji:
                    match esda_interface::ESDAMessage::from_le_bytes(
                        &read_buffer
                            [0 + message_offset..esda_interface::MESSAGE_SIZE + message_offset],
                    ) {
                        // If we got a valid message
                        Ok(message) => {
                            match message.id {
                                // Forward throttle commands to throttle driver via signalling channel
                                esda_interface::ESDAMessageID::SetTargetVelLeft => {
                                    throttle_command_signal.signal(
                                        esda_throttle::ThrottleCommand::SetThrottleLeft {
                                            new_throttle: message.data,
                                        },
                                    )
                                }
                                esda_interface::ESDAMessageID::SetTargetVelRight => {
                                    throttle_command_signal.signal(
                                        esda_throttle::ThrottleCommand::SetThrottleRight {
                                            new_throttle: message.data,
                                        },
                                    )
                                }
                                // If the E-Stop signal is received then pass the E-Stop signal to the throttle driver
                                esda_interface::ESDAMessageID::ESTOP => {
                                    println!("SERIAL_READER: Received E-Stop Signal, idling ESCs and ignoring all further throttle commands!");
                                    throttle_command_signal
                                        .signal(esda_throttle::ThrottleCommand::EngageEStop)
                                }
                                _ => {}
                            }
                        }
                        Err(invalid_data) => {
                            println!("SERIAL_READER<ERROR>: Got invalid message {invalid_data:?}")
                        }
                    }
                }

                offset += len;
            }
            // Otherwise log the error
            Err(e) => println!("SERIAL_READER<ERROR>: Serial RX Error: {:?}", e),
        }
    }
}
