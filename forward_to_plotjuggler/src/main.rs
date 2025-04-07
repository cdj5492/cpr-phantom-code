use std::error::Error;
use std::io::ErrorKind;
use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant};

use serde::Serialize;
use serialport::{SerialPort, SerialPortType};

use Sensor::*;

const MAGIC: u16 = 0xAA55;
const TARGET_VENDOR_ID: u16 = 0xf569;


/// constant array of flat values for each flex sensor
const ZERO_FLEX_VALUES: [f32; 25] = [
    1162.0, // TODO: check this one (short one)
    1134.0, // 1
    1168.0, // 2
    1207.0, // 3
    1175.0, // 4
    1215.0, // 5
    1153.0, // 6
    1120.0, // 7
    1205.0, // 8
    1148.0, // 9
    1203.0, // 10
    1190.0, // 11
    1080.0, // 12
    1191.0, // 13
    1175.0, // 14
    1123.0, // 15
    1146.0, // 16
    1205.0, // 17
    1145.0, // 18
    1112.0, // 19
    1175.0, // 20
    970.0,  // 21
    1002.0, // 22
    1015.0, // 23
    1122.0, // 24
];

// const MULTIPLIER_FLEX_VALUES: [f32; 25] = [
//     1.0, // TODO: check this one (short one)
//     0.00736434108, // 1
//     0.00940594059, // 2
//     0.00616883116, // 3
//     0.00766129032, // 4
//     0.0076       , // 5
//     0.00565476190, // 6
//     0.00579268292, // 7
//     0.00452380952, // 8
//     0.00494791666, // 9
//     0.00896226415, // 10
//     0.00678571428, // 11
//     0.0095       , // 12
//     0.00714285714, // 13
//     0.00424107142, // 14
//     0.00489690721, // 15
//     0.00871559633, // 16
//     0.00833333333, // 17
//     0.01376811594, // 18
//     0.00650684931, // 19
//     0.00669014084, // 20
//     0.01079545455, // 21
//     0.0065,        // 22
//     0.00748,       // 23
//     0.00969387755, // 24
// ];

const MULTIPLIER_FLEX_VALUES: [f32; 25] = [
    1.0, // TODO: check this one (short one)                      
    0.008333333333, // 1
    0.01032608696,  // 2
    0.006597222222, // 3
    0.0095,         // 4
    0.01010638298,  // 5
    0.008050847458, // 6
    0.007089552239, // 7
    0.005277777778, // 8
    0.005107526882, // 9
    0.008962264151, // 10
    0.00753968254,  // 11
    0.009693877551, // 12
    0.00931372549,  // 13
    0.005337078652, // 14
    0.006506849315, // 15
    0.01130952381,  // 16
    0.0095,         // 17
    0.01696428571,  // 18
    0.008050847458, // 19
    0.008636363636, // 20
    0.01357142857,  // 21
    0.004702970297, // 22
    0.0059375,      // 23
    0.009895833333, // 24
];

/// Each force sensor has a different calibration curve.
/// Store those curves in this array.
const FORCE_CURVES: [fn(f32) -> f32; 7] = [
    |x| x * (x * (x * 1.62e-08 + 1.70e-06) + 0.0185) + 0.0974,   // 0
    |x| x * (x * (x * 3.09e-08 + -1.78e-05) + 0.0261) + 0.0762,   // 1
    |x| x * (x * (x * -1.88e-08 + 4.83e-05) + 0.0090) + 0.0051,   // 2
    |x| x * (x * (x * 3.00e-08 + -1.28e-05) + 0.0225) + 0.2349,   // 3
    |x| x * (x * (x * -6.63e-08 + 7.81e-05) + 0.0065) + 0.3268,   // 4
    |x| x * (x * (x * -5.02e-08 + 9.06e-05) + -0.0034) + 0.2722,   // 5
    |x| x * (x * (x * 2.19e-09 + 2.42e-05) + 0.0118) + 0.1831,   // 6
];
// const FORCE_CURVES: [fn(f32) -> f32; 7] = [
//     |x| x * (x * (x * 1.62e-8 + 1.70e-6) + 0.0185)  + 0.0974,   // 0
//     |x| x * (x * (x * 3.09e-8 - 1.78e-5) + 0.0261)  + 0.0762,   // 1
//     |x| x * (x * (x * 1.25e-7 - 4.70e-5) + 0.0123) - 0.1630,    // 2
//     |x| x * (x * (x * 8.96e-8 - 4.13e-5) + 0.0124) + 0.1460,    // 3
//     |x| x * (x * (x * -2.09e-10 + 4.83e-5) - 3.49e-3) + 0.3180, // 4
//     |x| x * (x * (x * 1.25e-7 - 3.13e-5) + 6.02e-3) + 0.0928,   // 5
//     |x| x * (x * (x * -1.27e-9 + 2.83e-5) + 0.0104) + 0.298,   // 6
// ];

/// Each channel on each board corresponds to a specific sensor, either a flex or a force sensor.
/// Store the type and id of each sensor and associated channel in this array.
/// WARNING: Does not enforce mutual exclusivity of flex and force sensors.
const CHANNEL_SENSOR_ID_MAP: [Sensor; 35] = [
    Potentiometer(0), Potentiometer(1), Potentiometer(2), // built-in channels
    Flex(1), Flex(2), Flex(3), Flex(4), // board 0
    Flex(5), Flex(6), Flex(7), Flex(8), // board 1
    Flex(9), Flex(10), Flex(11), Flex(12), // board 2
    Flex(0), Force(0), Force(1), Force(2), // board 3
    Force(3), Force(4), Force(5), Force(6), // board 4
    Flex(13), Flex(14), Flex(15), Flex(16), // board 5
    Flex(17), Flex(18), Flex(19), Flex(20), // board 6
    Flex(21), Flex(22), Flex(23), Flex(24), // board 7
];  

enum Sensor {
    /// (ID,)
    Flex(usize),
    /// (ID,)
    Force(usize),
    /// (ID,)
    Potentiometer(usize),
}

impl Sensor {
    fn apply_calibration(&self, x: i16) -> f32 {
        match self {
            Flex(id) => {
                // return x as f32;
                let x = ZERO_FLEX_VALUES[*id] - x as f32;
                let mult = MULTIPLIER_FLEX_VALUES[*id];
                x * mult
            },
            Force(id) => {
                // return x as f32;
                FORCE_CURVES[*id](x as f32)
            },
            Potentiometer(_id) => {
                // TODO: implement curves
                x as f32
            },
        }
    }
}


#[derive(Serialize)]
struct DataPacket {
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
    // Data comes in packets of 4 channels per board.
    let mapped = data
        .iter()
        .enumerate()
        .map(|(i, val)| CHANNEL_SENSOR_ID_MAP[i].apply_calibration(*val))
        .collect::<Vec<_>>();

    // uncomment this if you just want to see the raw data
    // let mapped = data.iter().map(|x| *x as f32).collect::<Vec<_>>();

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

                    // Process the ADC values.
                    let adc_values = process_data(adc_values);

                    // Create JSON structure for UDP transmission.
                    let data = DataPacket {
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
