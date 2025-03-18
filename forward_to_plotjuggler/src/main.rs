use std::error::Error;
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

    // Uncomment for debugging:
    // println!("magic: {:04x}, payload_length: {}", magic, payload_length);

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
            if packet.len() < 4 {
                println!("Payload too short: {} bytes", packet.len());
                continue;
            }

            // Unpack the 4-byte timestamp.
            let timestamp = u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
            let timestamp_sec = (timestamp as f64) / 1e3;

            // Calculate the number of ADC values based on the payload length.
            let adc_payload_len = packet.len() - 4;
            if adc_payload_len % 2 != 0 {
                println!("ADC payload length is not even: {}", adc_payload_len);
                continue;
            }
            let num_adc_values = adc_payload_len / 2;
            let mut adc_values = Vec::with_capacity(num_adc_values);
            let mut offset = 4;
            for _ in 0..num_adc_values {
                let adc = u16::from_le_bytes([packet[offset], packet[offset + 1]]);
                adc_values.push(adc);
                offset += 2;
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
