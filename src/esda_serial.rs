// MCAV - Asterius MCU Firmware - esda_serial
//
// Authors: BMCG0011

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    gpio::Level, peripherals::UART1, uart::{UartRx, UartTx}, Async
};
use esp_println::println;

use crate::{
    esda_interface::{self}, esda_safety_light, esda_throttle
};

// // rx_fifo_full_threshold
pub(crate) const READ_BUF_SIZE: usize = 64;
// // EOT (CTRL-D)
pub(crate) const AT_CMD: u8 = 0x04;

pub const TX_PIN: u8 = 2;
pub const RX_PIN: u8 = 15;

// Buffer size for serial forwarding channel
pub const SERIAL_FORWARDING_BUFFER_SIZE: usize = 4;

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
            data: left_vel as u32,
        };
        let right = esda_interface::ESDAMessage {
            id: esda_interface::ESDAMessageID::CurrentVelRight,
            data: right_vel as u32,
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
    serial_forwarding_signal: &'static Channel<NoopRawMutex, esda_interface::ESDAMessage, {SERIAL_FORWARDING_BUFFER_SIZE}>,
) {
    use core::fmt::Write;
    loop {
        // Wait for us to be forwarded an ESDAMessage from the serial reader
        let message = serial_forwarding_signal.receive().await;
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
    throttle_command_channel: &'static Channel<NoopRawMutex, esda_throttle::ThrottleCommand, {esda_throttle::COMMAND_BUFFER_SIZE}>,
    safety_light_mode_signal: &'static Signal<NoopRawMutex, Level>
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut read_buffer: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;

    loop {
        let read_result = embedded_io_async::Read::read(&mut rx, &mut read_buffer[offset..]).await;
        println!("SERIAL_READER<DEBUG>: Read Result: {:?}", read_result);

        match read_result {
            Ok(len) => {
                offset += len;

                println!(
                    "SERIAL_READER<DEBUG>: Received ({len} bytes, data: {:?})",
                    &read_buffer[..offset]
                );

                // Ensure you only process complete messages
                let mut processed_bytes = 0;
                while offset - processed_bytes >= esda_interface::MESSAGE_SIZE {
                    let start = processed_bytes;
                    let end = start + esda_interface::MESSAGE_SIZE;

                    match esda_interface::ESDAMessage::from_le_bytes(&read_buffer[start..end]) {
                        Ok(message) => {
                            println!("MESSAGE RECEIVED: {:?}", message.data);
                            println!("MESSAGE ID <DEBUG> :{:?}", message.id);
                            match message.id {
                                
                                esda_interface::ESDAMessageID::SetTargetVelLeft => {
                                    let new_throttle = message.data as f32; // Store the new throttle value
                                    println!("Setting left throttle to: {}", new_throttle); // Print the new throttle value
                                    throttle_command_channel.send(
                                        esda_throttle::ThrottleCommand::SetThrottleLeft {
                                            new_throttle: message.data as f32,
                                        },
                                    ).await
                                }
                                esda_interface::ESDAMessageID::SetTargetVelRight => {
                                    let new_throttle = message.data as f32; // Store the new throttle value
                                    println!("Setting right throttle to: {}", new_throttle); // Print the new throttle value
                                    throttle_command_channel.send(
                                        esda_throttle::ThrottleCommand::SetThrottleRight {
                                            new_throttle: message.data as f32,
                                        },
                                    ).await
                                }
                                esda_interface::ESDAMessageID::ESTOP => {
                                    println!("SERIAL_READER: Received E-Stop Signal, idling ESCs and ignoring all further throttle commands!");
                                    throttle_command_channel
                                        .send(esda_throttle::ThrottleCommand::EngageEStop).await
                                },
                                esda_interface::ESDAMessageID::SetAutonomousMode => {
                                    match message.data {
                                        0 => {
                                            println!("WIRELESS_RECEIVER: Recieved signal to disengage autonomous mode!");
                                            // Turn safety light to solid
                                            safety_light_mode_signal.signal(Level::Low);
                                        },
                                        1 => {
                                            println!("WIRELESS_RECEIVER: Recieved signal to engage autonomous mode!");
                                            safety_light_mode_signal.signal(Level::High);
                                        },
                                        _ => {
                                            println!("WIRELESS_RECEIVER<WARN>: Recieved change to autonomous mode but the value is neither 0 nor 1");
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                        Err(invalid_data) => {
                            println!(
                                "SERIAL_READER<ERROR>: Got invalid message {invalid_data:?}"
                            );
                            break; // Stop processing if an invalid message is detected
                        }
                    }

                    processed_bytes += esda_interface::MESSAGE_SIZE;
                }

                // Shift remaining unprocessed bytes to the start of the buffer
                if processed_bytes > 0 {
                    read_buffer.copy_within(processed_bytes..offset, 0);
                    offset -= processed_bytes;
                }
            }
            Err(e) => println!("SERIAL_READER<ERROR>: Serial RX Error: {:?}", e),
        }
    }
}