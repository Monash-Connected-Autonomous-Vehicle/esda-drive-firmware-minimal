// MCAV - Asterius MCU Firmware - esda_wireless
//
// Authors: BMCG0011, Samuel Tri
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_println::{dbg, println};
use esp_wifi::esp_now::{EspNow, PeerInfo, BROADCAST_ADDRESS};

use crate::{esda_interface, esda_throttle};

/// Receiver task responsible for decoding and handling [ESDAMessages](crate::esda_interface::ESDAMessage) sent from the handheld controller over ESP-Now,
/// It is also responsible for manipulating and propagating them as need be and propagating them to the relevant tasks through the appropriate [Signals](embassy_sync::signal::Signal) and/or [Channels](embassy_sync::channel::Channel)
#[task]
pub async fn wireless_receiver(
    mut esp_now: EspNow<'static>,
    throttle_command_signal: &'static Signal<NoopRawMutex, esda_throttle::ThrottleCommand>,
    serial_forwarding_signal: &'static Signal<NoopRawMutex, esda_interface::ESDAMessage>,
) {
    loop {
        // Wait until we receive an espnow packet
        let received_packet = esp_now.receive_async().await;
        dbg!(
            "WIRELESS_RECEIVER<DEBUG>: Received packet {}",
            received_packet
        );
        let source_mac = received_packet.info.src_address;
        let dest_mac = received_packet.info.src_address;
        // If the packet was broadcast to all peers
        if dest_mac == BROADCAST_ADDRESS {
            // Check if we already know about this peer
            if esp_now.peer_exists(&source_mac) {
                // If so then add them to the known peers
                esp_now
                    .add_peer(PeerInfo {
                        peer_address: source_mac,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    })
                    .expect("WIRELESS_RECEIVER<FATAL>: Failed to add peer to known peers");
            }
        }

        // Handle the data sent from the esp
        let packet_data = received_packet.data;
        if received_packet.len % esda_interface::MESSAGE_SIZE as u8 != 0 {
            println!("WIRELESS_RECEIVER<ERROR>: Received packet not containing a whole number of messages {}", received_packet.len)
        }
        // Process all the messages in the packet
        for message_index in 0..(received_packet.len as usize) / esda_interface::MESSAGE_SIZE {
            match esda_interface::ESDAMessage::from_be_bytes(
                &packet_data[0 + message_index..esda_interface::MESSAGE_SIZE + message_index],
            ) {
                // If the message was valid
                Ok(message) => {
                    match message.id {
                        // Forward throttle commands to throttle driver via signalling channel
                        esda_interface::ESDAMessageID::SetTargetVelLeft => throttle_command_signal
                            .signal(esda_throttle::ThrottleCommand::SetThrottleLeft {
                                new_throttle: message.data,
                            }),
                        esda_interface::ESDAMessageID::SetTargetVelRight => throttle_command_signal
                            .signal(esda_throttle::ThrottleCommand::SetThrottleRight {
                                new_throttle: message.data,
                            }),
                        // Forward steering messages to serial (handled by ROS2)
                        esda_interface::ESDAMessageID::SteerAmount => {
                            dbg!("WIRELESS_RECEIVER<DEBUG>: Forwarding steering changes ({}) to ROS2", message);
                            serial_forwarding_signal.signal(message);
                        }
                        esda_interface::ESDAMessageID::ESTOP => {
                            println!("WIRELESS_RECEIVER: Received E-Stop Signal, idling ESCs and ignoring all further throttle commands!");
                            throttle_command_signal
                                .signal(esda_throttle::ThrottleCommand::EngageEStop)
                        }
                        esda_interface::ESDAMessageID::SetAutonomousMode => {}
                        _ => {}
                    }
                }
                Err(_) => todo!(),
            }
        }
    }
}
