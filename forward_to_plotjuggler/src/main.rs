use std::error::Error;
use std::io::Read;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use serialport::SerialPort;
use serde::Serialize;

const MAGIC: u16 = 0xAA55;

#[derive(Serialize)]
struct Data {
    t: f64,
    d: Vec<u16>,
}

/// Reads a packet from the given serial port.
/// The packet format is:
/// - Header (3 bytes): 2-byte little-endian magic value and 1-byte payload length.
/// - Payload: exactly `payload_length` bytes.
fn read_packet(port: &mut dyn SerialPort) -> Option<Vec<u8>> {
    let mut header = [0u8; 3];
    if let Err(_e) = port.read_exact(&mut header) {
        // Timeout or error reading header.
        return None;
    }

    let magic = u16::from_le_bytes([header[0], header[1]]);
    let payload_length = header[2] as usize;

    if magic != MAGIC {
        println!("Invalid magic value, discarding packet");
        return None;
    }

    let mut payload = vec![0u8; payload_length];
    if let Err(e) = port.read_exact(&mut payload) {
        println!("Incomplete payload: {}", e);
        return None;
    }
    Some(payload)
}

fn main() -> Result<(), Box<dyn Error>> {
    // --- Configuration ---
    let serial_port_name = "COM5";
    let baud_rate = 115200;
    let udp_ip = "127.0.0.1";
    let udp_port = 9870;

    // number of boards sending all their ADC channels
    let num_boards = 1;
    let num_adc_values = num_boards * 4;

    // Expected packet length: 4 bytes for the timestamp + 2 bytes per ADC value.
    let expected_payload_length = 4 + 2 * num_adc_values;

    // --- Setup serial connection ---
    let mut port = serialport::new(serial_port_name, baud_rate)
        .timeout(Duration::from_secs(1))
        .open()
        .map_err(|e| {
            eprintln!("Error opening serial port: {}", e);
            e
        })?;
    println!("Opened serial port {} at {} baud.", serial_port_name, baud_rate);

    // --- Setup UDP socket ---
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let udp_target = format!("{}:{}", udp_ip, udp_port);
    println!("UDP socket created, sending to {}", udp_target);

    let mut packet_count = 0;
    let mut last_time = Instant::now();

    loop {
        if let Some(packet) = read_packet(&mut *port) {
            if packet.len() != expected_payload_length {
                println!(
                    "Payload length mismatch: got {} bytes, expected {}",
                    packet.len(),
                    expected_payload_length
                );
                continue;
            }

            // Unpack payload: first 4 bytes are the timestamp.
            if packet.len() < 4 {
                continue;
            }
            let timestamp = u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
            let timestamp_sec = (timestamp as f64) / 1e3;

            // Next bytes are ADC values (each 2 bytes, little-endian)
            let mut adc_values = Vec::with_capacity(num_adc_values);
            let mut offset = 4;
            for _ in 0..num_adc_values {
                if offset + 2 > packet.len() {
                    println!("Not enough bytes for ADC value");
                    break;
                }
                let adc = u16::from_le_bytes([packet[offset], packet[offset + 1]]);
                adc_values.push(adc);
                offset += 2;
            }
            if adc_values.len() != num_adc_values {
                println!(
                    "Unpacking error: expected {} ADC values, got {}",
                    num_adc_values,
                    adc_values.len()
                );
                continue;
            }

            // Create JSON structure for UDP transmission.
            let data = Data {
                t: timestamp_sec,
                d: adc_values,
            };
            let udp_message = match serde_json::to_string(&data) {
                Ok(msg) => msg,
                Err(e) => {
                    println!("JSON serialization error: {}", e);
                    continue;
                }
            };

            if let Err(e) = socket.send_to(udp_message.as_bytes(), &udp_target) {
                println!("UDP send error: {}", e);
            } else {
                packet_count += 1;
            }

            // Print packets per second every second.
            let current_time = Instant::now();
            if current_time.duration_since(last_time) >= Duration::from_secs(1) {
                println!("Packets per second: {}", packet_count);
                packet_count = 0;
                last_time = current_time;
            }
        }
    }
}
