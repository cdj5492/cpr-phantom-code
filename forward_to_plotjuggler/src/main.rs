use std::error::Error;
use std::io::{ErrorKind, Write, stdout};
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use serde::Serialize;
use serialport::{SerialPort, SerialPortType};
use eframe::egui::{self, Pos2, Rect};
use egui::containers::Frame;
use egui::{emath, epaint, epaint::{PathStroke, Color32, Shape}};

mod physical_params;
use physical_params::*;

mod rib;

const MAGIC: u16 = 0xAA55;
const TARGET_VENDOR_ID: u16 = 0xf569;
const BAUD_RATE: u32 = 115200;
const UDP_IP: &str = "127.0.0.1";
const UDP_PORT: u16 = 9870;

const RIB_COLORS: [Color32; 4] = [
    Color32::RED,
    Color32::GREEN,
    Color32::BLUE,
    Color32::YELLOW,
];

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
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

type SharedRibData = Arc<Mutex<Option<Vec<Vec<(f32, f32)>>>>>;
type SharedCount    = Arc<Mutex<usize>>;

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
        return Ok(None);
    }

    let mut payload = vec![0u8; payload_length];
    match port.read_exact(&mut payload) {
        Ok(()) => Ok(Some(payload)),
        Err(e) if e.kind() == ErrorKind::TimedOut => Ok(None),
        Err(e) => Err(e.into()),
    }
}

fn process_data(data: Vec<i16>) -> Vec<f32> {
    assert_eq!(data.len(), CHANNEL_SENSOR_ID_MAP.len());
    data.iter()
        .enumerate()
        .map(|(i, v)| CHANNEL_SENSOR_ID_MAP[i].apply_calibration(*v))
        .collect()
}

fn open_serial_port(baud_rate: u32) -> Option<Box<dyn SerialPort>> {
    let ports = serialport::available_ports().ok()?;
    let pinfo = ports.into_iter().find(|p| matches!(p.port_type,
        SerialPortType::UsbPort(ref u) if u.vid == TARGET_VENDOR_ID))?;
    let name = pinfo.port_name;
    serialport::new(&name, baud_rate).timeout(Duration::from_secs(1)).open().ok()
}

fn worker_loop(raw: bool,
               shared: SharedRibData,
               pps: SharedCount,
               udp_socket: UdpSocket,
               udp_target: String) {
    let mut port: Option<Box<dyn SerialPort>> = None;
    let mut last_time = Instant::now();
    let mut cnt = 0usize;

    loop {
        if port.is_none() {
            port = open_serial_port(BAUD_RATE);
            continue;
        }
        let prt = port.as_mut().unwrap();
        match read_packet(prt.as_mut()) {
            Ok(Some(packet)) => {
                if packet.len() < 4 { continue; }
                let ts = u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]) as f64 / 1e3;
                let mut offs = 4;
                let mut raw_adc = Vec::with_capacity((packet.len() - 4) / 2);
                while offs + 1 < packet.len() {
                    raw_adc.push(i16::from_le_bytes([packet[offs], packet[offs + 1]]));
                    offs += 2;
                }
                let processed = if raw { raw_adc.iter().map(|v| *v as f32).collect() } else { process_data(raw_adc) };
                let (rib_x, rib_y):(Vec<Vec<f32>>, Vec<Vec<f32>>) = RIBS.iter().map(|r|{
                    let t = r.extract_thetas(&processed);
                    let pts = r.solve_rib(t);
                    (pts.iter().map(|p| p.x).collect(), pts.iter().map(|p| p.y).collect())
                }).unzip();

                {
                    let mut s = shared.lock().unwrap();
                    let mut ribs = Vec::new();
                    for i in 0..RIBS.len() {
                        ribs.push(rib_x[i].iter().zip(rib_y[i].iter()).map(|(x,y)|(*x,*y)).collect());
                    }
                    *s = Some(ribs);
                }

                let packet_json = serde_json::to_string(&DataPacket {
                    t: ts,
                    sensor_data: processed,
                    rib1_x_values: rib_x[0].clone(),
                    rib1_y_values: rib_y[0].clone(),
                    rib2_x_values: rib_x[1].clone(),
                    rib2_y_values: rib_y[1].clone(),
                    rib3_x_values: rib_x[2].clone(),
                    rib3_y_values: rib_y[2].clone(),
                    rib4_x_values: rib_x[3].clone(),
                    rib4_y_values: rib_y[3].clone(),
                }).unwrap();

                let _ = udp_socket.send_to(packet_json.as_bytes(), &udp_target);
                cnt += 1;
                if last_time.elapsed() >= Duration::from_secs(1) {
                    *pps.lock().unwrap() = cnt;
                    cnt = 0;
                    last_time = Instant::now();
                }
            }
            Ok(None) => (),
            Err(_)   => { port = None; }
        }
    }
}

struct MyApp {
    shared: SharedRibData,
    pps: SharedCount,
}

impl MyApp {
    fn new(shared: SharedRibData, pps: SharedCount) -> Self {
        Self { shared, pps }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.label(format!("Packets per second: {}", *self.pps.lock().unwrap()));

            let ribs_opt = self.shared.lock().unwrap().clone();
            if ribs_opt.is_none() { return; }
            let ribs = ribs_opt.unwrap();

            Frame::canvas(ui.style()).show(ui, |ui| {
                let available = ui.available_size();
                let (_id, rect) = ui.allocate_space(available);

                let mut all_x = Vec::new();
                let mut all_y = Vec::new();
                for r in &ribs {
                    for (x,y) in r {
                        all_x.push(*x);
                        all_y.push(*y);
                    }
                }
                if all_x.is_empty() { return; }
                let min_x = all_x.iter().fold(f32::INFINITY, |a,&b| a.min(b));
                let max_x = all_x.iter().fold(f32::NEG_INFINITY, |a,&b| a.max(b));
                let min_y = all_y.iter().fold(f32::INFINITY, |a,&b| a.min(b));
                let max_y = all_y.iter().fold(f32::NEG_INFINITY, |a,&b| a.max(b));
                let cx = (min_x + max_x) * 0.5;
                let cy = (min_y + max_y) * 0.5;
                let scale = (rect.width().min(rect.height()))
                    / ((max_x - min_x).max(max_y - min_y) + 1e-3) * 0.45;
                let screen_c = rect.center();

                let mut shapes = Vec::new();
                for (idx, rib) in ribs.iter().enumerate() {
                    if rib.len() < 2 { continue; }
                    let seg: Vec<Pos2> = rib.iter().map(|(x,y)|{
                        Pos2::new(
                            screen_c.x + (x - cx) * scale,
                            screen_c.y - (y - cy) * scale,
                        )
                    }).collect();
                    shapes.push(Shape::line(seg.clone(), PathStroke::new(2.0, RIB_COLORS[idx])));
                    for p in seg {
                        shapes.push(Shape::circle_filled(p, 3.0, RIB_COLORS[idx]));
                    }
                }
                ui.painter().extend(shapes);
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    let cli = Args::parse();

    let shared: SharedRibData = Arc::new(Mutex::new(None));
    let pps: SharedCount = Arc::new(Mutex::new(0));

    let udp_socket = UdpSocket::bind("0.0.0.0:0").expect("udp bind");
    let udp_target = format!("{}:{}", UDP_IP, UDP_PORT);

    {
        let shared_clone = Arc::clone(&shared);
        let pps_clone = Arc::clone(&pps);
        let socket_clone = udp_socket.try_clone().unwrap();
        thread::spawn(move || worker_loop(cli.raw, shared_clone, pps_clone, socket_clone, udp_target));
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        vsync: false,
        ..Default::default()
    };

    eframe::run_native(
        "Forward to PlotJuggler",
        options,
        Box::new(|_cc| Ok(Box::new(MyApp::new(shared, pps)))),
    )
}
