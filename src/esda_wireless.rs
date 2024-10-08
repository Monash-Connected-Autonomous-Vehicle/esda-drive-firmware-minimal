// MCAV - Asterius MCU Firmware - esda_wireless
//
// Authors: BMCG0011, Samuel Tri
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use esp_println::{dbg, println};
use esp_wifi::esp_now::{EspNow, PeerInfo, BROADCAST_ADDRESS};

use crate::{esda_interface, esda_serial, esda_throttle};

/// Receiver task responsible for decoding and handling [ESDAMessages](crate::esda_interface::ESDAMessage) sent from the handheld controller over ESP-Now,
/// It is also responsible for manipulating and propagating them as need be and propagating them to the relevant tasks through the appropriate [Signals](embassy_sync::signal::Signal) and/or [Channels](embassy_sync::channel::Channel)
#[task]
pub async fn wireless_receiver(
    mut esp_now: EspNow<'static>,
    throttle_command_channel: &'static Channel<NoopRawMutex, esda_throttle::ThrottleCommand, {esda_throttle::COMMAND_BUFFER_SIZE}>,
    serial_forwarding_signal: &'static Channel<NoopRawMutex, esda_interface::ESDAMessage, {esda_serial::SERIAL_FORWARDING_BUFFER_SIZE}>,
) {
    loop {
        // Wait until we receive an espnow packet
        let received_packet = esp_now.receive_async().await;
        // println!(
        //     "WIRELESS_RECEIVER<DEBUG>: Received packet {}",
        //     received_packet
        // );
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

        // Ignore rogue broadcasts
        if source_mac != [0x7C,0x87,0xCE,0x2D,0x3C,0xE0] {
            continue
        }

        // Handle the data sent from the esp
        let packet_data = received_packet.data;
        if received_packet.len % esda_interface::MESSAGE_SIZE as u8 != 0 {
            println!("WIRELESS_RECEIVER<ERROR>: Received packet not containing a whole number of messages {}", received_packet.len)
        }

        // Process all the messages in the packet
        // for message_index in 0..(received_packet.len as usize) / esda_interface::MESSAGE_SIZE {
        //     match esda_interface::ESDAMessage::from_le_bytes(
        //         &packet_data[0 + message_index..esda_interface::MESSAGE_SIZE + message_index],
        //     ) {
            match esda_interface::ESDAMessage::from_le_bytes(&packet_data[0..=7]) {
                // If the message was valid
                Ok(message) => {
                    match message.id {
                        // Forward throttle commands to throttle driver via signalling channel
                        esda_interface::ESDAMessageID::SetTargetVelLeft => {
                            throttle_command_channel
                            .send(esda_throttle::ThrottleCommand::SetThrottleLeft {
                                new_throttle: message.data as f32,
                            }).await;
                            println!("WIRELESS_RECEIVER<DEBUG>: Forwarding Received change to left throttle: {:?}", message);
                        },
                        esda_interface::ESDAMessageID::SetTargetVelRight => {
                            let mut data: [u8; 8] = [0;8];
                            data.copy_from_slice(&packet_data[0..=7]);
                            println!("WIRELESS_RECEIVER: Received right throttle data {:?}\n{:b}", data, u64::from_le_bytes(data));
                            throttle_command_channel
                            .send(esda_throttle::ThrottleCommand::SetThrottleRight {
                                new_throttle: message.data as f32,
                            }).await;
                            println!("WIRELESS_RECEIVER<DEBUG>: Forwarding Received change to right throttle: {:?}", message);
                        },
                        // Forward steering messages to serial (handled by ROS2)
                        esda_interface::ESDAMessageID::SteerAmount => {
                            let mut data: [u8; 8] = [0;8];
                            data.copy_from_slice(&packet_data[0..=7]);
                            println!("WIRELESS_RECEIVER: Received data {:?}",  &packet_data[0..=7]);
                            println!("WIRELESS_RECEIVER<DEBUG>: Forwarding steering changes ({:?}) to ROS2", message);
                            
                            serial_forwarding_signal.send(message).await;
                        },
                        esda_interface::ESDAMessageID::ESTOP => {
                            println!("WIRELESS_RECEIVER: Received E-Stop Signal, idling ESCs and ignoring all further throttle commands!");
                            throttle_command_channel
                                .send(esda_throttle::ThrottleCommand::EngageEStop).await
                        },
                        esda_interface::ESDAMessageID::SetAutonomousMode => {},
                        _ => {},
                    }
                }
                Err(e) => { let mut data: [u8; 8] = [0;8];
                    data.copy_from_slice(&e[0..=7]);
                    println!("WIRELESS_RECEIVER: Received invalid data {:b}",  u64::from_le_bytes(data)) },
            }
        // }
    }
}