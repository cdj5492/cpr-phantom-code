use std::error::Error;
use std::io::{ErrorKind, Write, stdout};
use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use serde::Serialize;
use serialport::{SerialPort, SerialPortType};
use eframe::egui::{self, Pos2, Rect};
use egui::containers::Frame;
use egui::{emath, epaint, epaint::PathStroke};

mod physical_params;
use physical_params::*;

mod rib;

const MAGIC: u16 = 0xAA55;
const TARGET_VENDOR_ID: u16 = 0xf569;

// --- Configuration ---
const BAUD_RATE: u32 = 115200;
const UDP_IP: &str = "127.0.0.1";
const UDP_PORT: u16 = 9870;

/// command line args
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// If included, send raw data (no calibration)
    #[arg(short, long)]
    raw: bool,
}

/// eframe app
struct MyApp {
    packet_count: usize,
    packets_per_second: usize,
    port: Option<Box<dyn SerialPort>>,
    retry_timer: Instant,
    last_time: Instant,
    raw: bool,
    socket: UdpSocket,
    udp_target: String,
}

impl MyApp {
    fn new(raw: bool, socket: UdpSocket, udp_target: String) -> Self {
        Self {
            packet_count: 0,
            packets_per_second: 0,
            port: None,
            retry_timer: Instant::now(),
            last_time: Instant::now(),
            raw: raw,
            socket: socket,
            udp_target: udp_target,
        }
    }
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

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(port) = &mut self.port {
                match read_packet(port.as_mut()) {
                    Ok(Some(packet)) => {
                        if packet.len() < 4 {
                            println!("Payload too short: {} bytes", packet.len());
                            return;
                        }

                        // Unpack the 4-byte timestamp.
                        let timestamp =
                            u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
                        let timestamp_sec = (timestamp as f64) / 1e3;

                        // Calculate the number of ADC values based on the payload length.
                        let adc_payload_len = packet.len() - 4;
                        if adc_payload_len % 2 != 0 {
                            println!("ADC payload length is not even: {}", adc_payload_len);
                            return;
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
                        let processed_values = if self.raw {
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
                                return;
                            }
                        };

                        if let Err(e) = self.socket.send_to(udp_message.as_bytes(), &self.udp_target) {
                            println!("UDP send error: {}", e);
                            return;
                        } else {
                            self.packet_count += 1;
                        }

                        if self.last_time.elapsed() > Duration::from_secs(1) {
                            self.packets_per_second = self.packet_count;
                            self.packet_count = 0;
                            self.last_time = Instant::now();
                        }

                        // Print packets per second every second.
                        ui.label(format!("Packets per second: {}", self.packets_per_second));


                        // visualize the data in a canvas
                        Frame::canvas(ui.style()).show(ui, |ui| {
                            ui.ctx().request_repaint(); 
                            let desired_size = ui.available_size();
                            let (_id, rect) = ui.allocate_space(desired_size);
                            let to_screen = emath::RectTransform::from_to(Rect::from_x_y_ranges(0.0..=1.0, -1.0..=1.0), rect);

                            // draw a line
                            let mut shapes = vec![];
                            let thickness = 2.0;

                            let points = vec![
                                Pos2::new(0.0, 0.0),
                                Pos2::new(1.0, 1.0),
                            ];

                            shapes.push(epaint::Shape::line(
                                points,
                                PathStroke::new(thickness, epaint::Color32::WHITE)
                            ));

                            ui.painter().extend(shapes);
                        });

                    }
                    Ok(None) => {
                        // This branch typically occurs when a timeout happens or we read an invalid packet.
                        // We just continue reading.
                    }
                    Err(e) => {
                        // If a non-timeout error occurs (e.g. device disconnect), break out to reconnect.
                        println!("Serial port error: {}. Reconnecting...", e);
                        self.port = None;
                    }
                }
            } else if self.retry_timer.elapsed() >= Duration::from_secs(2) {
                if let Some(port) = open_serial_port(BAUD_RATE) {
                    println!("Serial port opened successfully on port {:?}", port.name());
                    self.port = Some(port);
                } else {
                    println!(
                        "No serial port found with vendor ID: 0x{:04x}. Retrying in 2 seconds...",
                        TARGET_VENDOR_ID
                    );
                    self.retry_timer = Instant::now();
                }
            } else {
                println!("Waiting for retry timer to expire... ({:?})", self.retry_timer.elapsed());
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    let cli = Args::parse();

    if cli.raw {
        println!("Raw mode enabled. No calibration will be applied.");
    } else {
        println!("Calibration mode enabled. Data will be calibrated.");
    }


    // --- Setup UDP socket ---
    let socket = UdpSocket::bind("0.0.0.0:0").expect("Failed to bind UDP socket");
    let udp_target = format!("{}:{}", UDP_IP, UDP_PORT);
    println!("UDP socket created, sending to {}", udp_target);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        vsync: false, // so we can send packets as fast as they come in
        ..Default::default()
    };
    eframe::run_native(
        "Forward to PlotJuggler",
        options,
        Box::new(|_cc| {
            Ok(Box::<MyApp>::new(MyApp::new(cli.raw, socket, udp_target)))
        }),
    )
}
