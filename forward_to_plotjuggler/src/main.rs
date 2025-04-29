use std::error::Error;
use std::io::ErrorKind;
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use eframe::egui::{self, Pos2};
use egui::containers::Frame;
use egui::{
    epaint,
    epaint::{Color32, PathStroke, Shape},
};
use serde::Serialize;
use serialport::{SerialPort, SerialPortType};

mod physical_params;
use physical_params::*;

mod rib;

const MAGIC: u16 = 0xAA55;
const TARGET_VENDOR_ID: u16 = 0xf569;

// --- Configuration ---
const BAUD_RATE: u32 = 115200;
const UDP_IP: &str = "127.0.0.1";
const UDP_PORT: u16 = 9870;

/// per-rib colors for the UI plot
const RIB_COLORS: [Color32; 4] = [Color32::RED, Color32::GREEN, Color32::BLUE, Color32::YELLOW];

/// command line args
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// If included, send raw data (no calibration)
    #[arg(short, long)]
    raw: bool,
}

/// JSON packet sent to PlotJuggler
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
    rib1_flex_values: Vec<f32>,
    rib2_flex_values: Vec<f32>,
    rib3_flex_values: Vec<f32>,
    rib4_flex_values: Vec<f32>,
    force_sensor_values: Vec<f32>,
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
        Err(e) if e.kind() == ErrorKind::TimedOut => return Ok(None),
        Err(e) => return Err(e.into()),
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
        Err(e) if e.kind() == ErrorKind::TimedOut => Ok(None),
        Err(e) => Err(e.into()),
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

type SharedRibData = Arc<Mutex<Option<Vec<Vec<(f32, f32)>>>>>;
type SharedPps = Arc<Mutex<usize>>;

/// high-speed worker running in its own thread (~1 kHz)
fn worker_loop(
    raw: bool,
    shared: SharedRibData,
    pps: SharedPps,
    udp_socket: UdpSocket,
    udp_target: String,
) {
    let mut port: Option<Box<dyn SerialPort>> = None;
    let mut last_time = Instant::now();
    let mut packet_count = 0usize;

    loop {
        // reconnect logic
        if port.is_none() {
            port = open_serial_port(BAUD_RATE);
            continue;
        }

        let prt = port.as_mut().unwrap();
        match read_packet(prt.as_mut()) {
            Ok(Some(packet)) => {
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
                    let adc = i16::from_le_bytes([packet[offset], packet[offset + 1]]);
                    adc_values.push(adc);
                    offset += 2;
                }

                // Process the ADC values.
                let processed_values = if raw {
                    adc_values.iter().map(|x| *x as f32).collect()
                } else {
                    process_data(adc_values)
                };

                let (rib_x_values, rib_y_values): (Vec<Vec<f32>>, Vec<Vec<f32>>) = RIBS
                    .iter()
                    .map(|rib| {
                        let thetas = rib.extract_thetas(&processed_values);
                        let rib_points = rib.solve_rib(thetas);
                        let rib_x_values = rib_points.iter().map(|p| p.x).collect::<Vec<_>>();
                        let rib_y_values = rib_points.iter().map(|p| p.y).collect::<Vec<_>>();
                        (rib_x_values, rib_y_values)
                    })
                    .unzip();

                // share data with UI
                {
                    let mut guard = shared.lock().unwrap();
                    let mut ribs = Vec::new();
                    for i in 0..RIBS.len() {
                        ribs.push(
                            rib_x_values[i]
                                .iter()
                                .zip(rib_y_values[i].iter())
                                .map(|(x, y)| (*x, *y))
                                .collect(),
                        );
                    }
                    *guard = Some(ribs);
                }

                // put together individual rib and force sensor values
                let rib1_flex_values = RIB0_SEGMENTS
                    .iter()
                    .map(|seg| processed_values[seg.channel as usize])
                    .collect::<Vec<_>>();
                let rib2_flex_values = RIB1_SEGMENTS
                    .iter()
                    .map(|seg| processed_values[seg.channel as usize])
                    .collect::<Vec<_>>();
                let rib3_flex_values = RIB2_SEGMENTS
                    .iter()
                    .map(|seg| processed_values[seg.channel as usize])
                    .collect::<Vec<_>>();
                let rib4_flex_values = RIB3_SEGMENTS
                    .iter()
                    .map(|seg| processed_values[seg.channel as usize])
                    .collect::<Vec<_>>();
                let force_sensor_values = processed_values[7..=13].to_vec();

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
                    rib1_flex_values,
                    rib2_flex_values,
                    rib3_flex_values,
                    rib4_flex_values,
                    force_sensor_values,
                };
                let udp_message = serde_json::to_string(&data).unwrap();
                let _ = udp_socket.send_to(udp_message.as_bytes(), &udp_target);

                packet_count += 1;
                if last_time.elapsed() > Duration::from_secs(1) {
                    *pps.lock().unwrap() = packet_count;
                    packet_count = 0;
                    last_time = Instant::now();
                }
            }
            Ok(None) => (), // timeout or invalid packet
            Err(e) => {
                println!("Serial port error: {}. Reconnecting...", e);
                port = None;
            }
        }
    }
}

/// eframe app
struct MyApp {
    shared: SharedRibData,
    pps: SharedPps,
}

impl MyApp {
    fn new(shared: SharedRibData, pps: SharedPps) -> Self {
        Self { shared, pps }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // continuous repaint
        ctx.request_repaint();

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.label(format!("Packets per second: {}", *self.pps.lock().unwrap()));

            // visualize the data in a canvas
            Frame::canvas(ui.style()).show(ui, |ui| {
                let ribs_opt = self.shared.lock().unwrap().clone();
                if ribs_opt.is_none() {
                    return;
                }
                let ribs = ribs_opt.unwrap();

                let desired_size = ui.available_size();
                let (_id, rect) = ui.allocate_space(desired_size);

                // compute bounds to center all ribs
                let mut all_x = Vec::<f32>::new();
                let mut all_y = Vec::<f32>::new();
                for rib in &ribs {
                    for (x, y) in rib {
                        all_x.push(*x);
                        all_y.push(*y);
                    }
                }
                let min_x = all_x.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                let max_x = all_x.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
                let min_y = all_y.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                let max_y = all_y.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));

                let center_x = (min_x + max_x) * 0.5;
                let center_y = (min_y + max_y) * 0.5;
                let scale = (rect.width().min(rect.height()))
                    / ((max_x - min_x).max(max_y - min_y) + 1e-3)
                    * 0.45;
                let screen_center = rect.center();

                let mut shapes = Vec::<epaint::Shape>::new();

                for (rib_idx, rib) in ribs.iter().enumerate() {
                    if rib.len() < 2 {
                        continue;
                    }
                    let pts: Vec<Pos2> = rib
                        .iter()
                        .map(|(x, y)| {
                            Pos2::new(
                                screen_center.x + (x - center_x) * scale,
                                screen_center.y - (y - center_y) * scale,
                            )
                        })
                        .collect();

                    // draw lines between points
                    shapes.push(Shape::line(
                        pts.clone(),
                        PathStroke::new(2.0, RIB_COLORS[rib_idx]),
                    ));
                    // draw points
                    for p in pts {
                        shapes.push(Shape::circle_filled(p, 3.0, RIB_COLORS[rib_idx]));
                    }
                }

                ui.painter().extend(shapes);
            });
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

    let shared: SharedRibData = Arc::new(Mutex::new(None));
    let pps: SharedPps = Arc::new(Mutex::new(0));

    // launch worker thread
    {
        let shared_clone = Arc::clone(&shared);
        let pps_clone = Arc::clone(&pps);
        let socket_clone = socket.try_clone().expect("udp clone");
        thread::spawn(move || {
            worker_loop(cli.raw, shared_clone, pps_clone, socket_clone, udp_target)
        });
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        vsync: false, // so we can send packets as fast as they come in
        ..Default::default()
    };
    eframe::run_native(
        "Forward to PlotJuggler",
        options,
        Box::new(|_cc| Ok(Box::<MyApp>::new(MyApp::new(shared, pps)))),
    )
}
