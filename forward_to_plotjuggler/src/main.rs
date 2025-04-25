use std::error::Error;
use std::io::{ErrorKind, Write, stdout};
use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use crossterm::ExecutableCommand;
use crossterm::cursor::MoveToColumn;
use crossterm::terminal::{Clear, ClearType};
use serde::Serialize;
use serialport::{SerialPort, SerialPortType};

mod physical_params;
use physical_params::*;

mod rib;

const MAGIC: u16 = 0xAA55;
const TARGET_VENDOR_ID: u16 = 0xf569;

/// command line args
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// If included, send raw data (no calibration)
    #[arg(short, long)]
    raw: bool,
}

#[derive(Serialize)]
struct DataPacket {
    t: f64,
    sensor_data: Vec<f32>,
    rib1_x_values: Vec<f32>,
    rib1_y_values: Vec<f32>,
    rib2_x_values: Vec<f32>,
    rib2_y_values: Vec<f32>,
    rib3_x_values: Vec<f32>,
    rib3_y_values: Vec<f32>,
    rib4_x_values: Vec<f32>,
    rib4_y_values: Vec<f32>,
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
    // assert that the length of data is the same as the number of channels
    assert_eq!(data.len(), CHANNEL_SENSOR_ID_MAP.len());

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
    let cli = Args::parse();

    if cli.raw {
        println!("Raw mode enabled. No calibration will be applied.");
    } else {
        println!("Calibration mode enabled. Data will be calibrated.");
    }

    // --- Crossterm stdout setup ---
    let mut stdout = stdout();

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
                    let processed_values = if cli.raw {
                        adc_values.iter().map(|x| *x as f32).collect()
                    } else {
                        process_data(adc_values)
                    };

                    // let all_rib_x_values = vec![Vec::new(); RIBS.len()];
                    // let all_rib_y_values = vec![Vec::new(); RIBS.len()];

                    let (rib_x_values, rib_y_values): (Vec<Vec<f32>>, Vec<Vec<f32>>) = RIBS
                        .iter()
                        .map(|rib| {
                            let thetas = rib.extract_thetas(&processed_values);
                            let rib_points = rib.solve_rib(thetas);
                            let rib_x_values = rib_points.iter().map(|p| p.x).collect::<Vec<_>>();
                            let rib_y_values = rib_points.iter().map(|p| p.y).collect::<Vec<_>>();
                            (rib_x_values, rib_y_values)
                        })
                        .unzip(); // new favorite function!

                    // Create JSON structure for UDP transmission.
                    let data = DataPacket {
                        t: timestamp_sec,
                        sensor_data: processed_values,
                        rib1_x_values: rib_x_values[0].clone(),
                        rib1_y_values: rib_y_values[0].clone(),
                        rib2_x_values: rib_x_values[1].clone(),
                        rib2_y_values: rib_y_values[1].clone(),
                        rib3_x_values: rib_x_values[2].clone(),
                        rib3_y_values: rib_y_values[2].clone(),
                        rib4_x_values: rib_x_values[3].clone(),
                        rib4_y_values: rib_y_values[3].clone(),
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
                        // go back to start of current line
                        stdout.execute(MoveToColumn(0))?;
                        // stdout.execute(MoveToPreviousLine(1))?;
                        // clear line
                        stdout.execute(Clear(ClearType::CurrentLine))?;
                        print!("Packets per second: {}", packet_count);
                        stdout.flush()?;

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
