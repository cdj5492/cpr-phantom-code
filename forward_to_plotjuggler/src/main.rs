use std::error::Error;
use std::io::ErrorKind;
use std::net::UdpSocket;
use std::os::windows::process;
use std::thread;
use std::time::{Duration, Instant};

use serde::Serialize;
use serialport::{SerialPort, SerialPortType};

use Sensor::*;

const MAGIC: u16 = 0xAA55;
// Change this to the vendor ID you want to target.
const TARGET_VENDOR_ID: u16 = 0xf569;

enum Sensor {
    Flex(usize),
    Force(usize),
}

impl Sensor {
    fn apply_calibration(&self, x: i32) -> f32 {
        match self {
            Flex(id) => {
                let x = x as f32 - ZERO_FLEX_VALUES[*id - 1];
                // x * (x * 7.45e-6 - 0.0209) + 13.8
                x * (x * 7.45e-6 - 5.43e-3) + 0.174
            },
            Force(id) => {
                let x = x as f32 - ZERO_FORCE_VALUES[*id - 1];
                // TODO: re-do this
                x * (x * (x * 2.2e-5 + 0.0479) + 5.34) - 53.7
            },
        }
    }
}


// constant array of flat values for each flex sensor
const ZERO_FLEX_VALUES: [f32; 25] = [
    1162.0, // TODO: check this one
    1134.0,
    1168.0,
    1207.0,
    1175.0,
    1215.0,
    1153.0,
    1120.0,
    1205.0,
    1148.0,
    1203.0,
    1190.0,
    1080.0,
    1191.0,
    1175.0,
    1123.0,
    1146.0,
    1205.0,
    1145.0,
    1112.0,
    1175.0,
    1084.0,
    1026.0,
    1020.0,
    1122.0
];

// constant array of flat values for each force sensor
const ZERO_FORCE_VALUES: [f32; 7] = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
];

const CHANNEL_SENSOR_ID_MAP: [Sensor; 32] = [
    Flex(1), Flex(2), Flex(3), Flex(4), // board 0
    Flex(5), Flex(6), Flex(7), Flex(8), // board 1
    Flex(9), Flex(10), Flex(11), Flex(12), // board 2
    Flex(0), Force(0), Force(1), Force(2), // board 3
    Force(3), Force(4), Force(5), Force(6), // board 4
    Flex(13), Flex(14), Flex(15), Flex(16), // board 5
    Flex(17), Flex(18), Flex(19), Flex(20), // board 6
    Flex(21), Flex(22), Flex(23), Flex(24), // board 7
];  

#[derive(Serialize)]
struct Data {
    t: f64,
    d: Vec<f32>,
}

/// Reads a packet from the given serial port.
/// The packet format is:
/// - Header (3 bytes): 2-byte little-endian magic value and 1-byte payload length.
/// - Payload: exactly `payload_length` bytes.
///
/// Returns:
/// - Ok(Some(packet)) if a complete packet was read,
/// - Ok(None) if a timeout occurred or the packet was invalid,
/// - Err(e) if a non-recoverable error occurred (e.g. device disconnect).
fn read_packet(port: &mut dyn SerialPort) -> Result<Option<Vec<u8>>, Box<dyn Error>> {
    let mut header = [0u8; 3];
    match port.read_exact(&mut header) {
        Ok(()) => (),
        Err(e) => {
            // If it's a timeout, return None to allow looping.
            if e.kind() == ErrorKind::TimedOut {
                return Ok(None);
            } else {
                return Err(e.into());
            }
        }
    }

    let magic = u16::from_le_bytes([header[0], header[1]]);
    let payload_length = header[2] as usize;

    if magic != MAGIC {
        println!("Invalid magic value, discarding packet");
        return Ok(None);
    }

    let mut payload = vec![0u8; payload_length];
    match port.read_exact(&mut payload) {
        Ok(()) => Ok(Some(payload)),
        Err(e) => {
            if e.kind() == ErrorKind::TimedOut {
                Ok(None)
            } else {
                Err(e.into())
            }
        }
    }
}

/// Processes incoming data, running it through mapping functions before passing it on to plotjuggler
fn process_data(data: Vec<i16>) -> Vec<f32> {
    let ideal_flat = 1038.0;
    // let flex_map  = |x: f32| x * -0.0106 + 10.6;
    let flex_map = |x: f32| x * (x * 7.45e-6 - 0.0209) + 13.8;
    let force_map = |x: f32| x * (x * (x * 2.2e-5 + 0.0479) + 5.34) - 53.7;
    let adjust_zero_map = |ix: (usize, f32)| ix.1 - zero_points[ix.0] as f32 + ideal_flat;

    // Data comes in packets of 4 channels per board.
    // Pass board ids 3 and 4 through the force map, and the rest through the flex map.
    let mapped = data.iter()
        .map(|x| *x as f32)
        .enumerate()
        .map(adjust_zero_map)
        .collect::<Vec<f32>>();
    let mapped = mapped
        .chunks(4)
        .enumerate()
        .map(|(i, chunk)| {
            if i == 3 || i == 4 {
                chunk
                    .iter()
                    .map(|&x| force_map(x))
                    .collect::<Vec<f32>>()
            } else {
                chunk
                    .iter()
                    .map(|&x| flex_map(x))
                    .collect::<Vec<f32>>()
            }
        })
        .flatten()
        .collect::<Vec<f32>>();
    let mapped = data.iter().map(|x| *x as f32).collect::<Vec<f32>>();

    mapped
}

/// Attempts to open a serial port with the matching vendor ID.
/// Returns Some(port) if successful, or None if no matching port is found or open fails.
fn open_serial_port(baud_rate: u32) -> Option<Box<dyn SerialPort>> {
    let available_ports = serialport::available_ports().ok()?;
    let port_info = available_ports.into_iter().find(|p| match p.port_type {
        SerialPortType::UsbPort(ref usb_info) => usb_info.vid == TARGET_VENDOR_ID,
        _ => false,
    })?;
    let serial_port_name = port_info.port_name;
    println!("Using serial port: {}", serial_port_name);
    match serialport::new(&serial_port_name, baud_rate)
        .timeout(Duration::from_secs(1))
        .open()
    {
        Ok(port) => {
            println!(
                "Opened serial port {} at {} baud.",
                serial_port_name, baud_rate
            );
            Some(port)
        }
        Err(e) => {
            println!("Error opening serial port {}: {}", serial_port_name, e);
            None
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    // --- Configuration ---
    let baud_rate = 115200;
    let udp_ip = "127.0.0.1";
    let udp_port = 9870;

    // --- Setup UDP socket ---
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let udp_target = format!("{}:{}", udp_ip, udp_port);
    println!("UDP socket created, sending to {}", udp_target);

    let mut packet_count = 0;
    let mut last_time = Instant::now();

    let mut zeroes: Option<Vec<_>> = None;

    // Outer loop: always try to (re)connect
    loop {
        println!("Attempting to find and open the serial port...");
        let mut port = match open_serial_port(baud_rate) {
            Some(p) => p,
            None => {
                println!(
                    "No serial port found with vendor ID: 0x{:04x}. Retrying in 2 seconds...",
                    TARGET_VENDOR_ID
                );
                thread::sleep(Duration::from_secs(2));
                continue;
            }
        };

        // Inner loop: read from the open port.
        loop {
            match read_packet(&mut *port) {
                Ok(Some(packet)) => {
                    if packet.len() < 4 {
                        println!("Payload too short: {} bytes", packet.len());
                        continue;
                    }

                    // Unpack the 4-byte timestamp.
                    let timestamp =
                        u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
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
                        let adc = i16::from_le_bytes([packet[offset], packet[offset + 1]]);
                        adc_values.push(adc);
                        offset += 2;
                    }

                    // if it's none, set it to the first packet
                    if zeroes.is_none() {
                        zeroes = Some(adc_values.clone());
                        println!("Zeroes: {:?}", zeroes);
                    }

                    // Process the ADC values.
                    let adc_values = process_data(adc_values, zeroes.as_ref().unwrap());

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
                Ok(None) => {
                    // This branch typically occurs when a timeout happens or we read an invalid packet.
                    // We just continue reading.
                    continue;
                }
                Err(e) => {
                    // If a non-timeout error occurs (e.g. device disconnect), break out to reconnect.
                    println!("Serial port error: {}. Reconnecting...", e);
                    break;
                }
            }
        }

        println!("Lost connection to serial device. Retrying in 1 second...");
        thread::sleep(Duration::from_secs(1));
    }
}
