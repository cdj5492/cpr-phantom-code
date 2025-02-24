use std::convert::TryInto;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use eframe::egui;
use eframe::egui::{CentralPanel, ComboBox, Context, TextEdit};
use egui::widgets::plot::{Line, Plot, PlotPoints};
use rfd::FileDialog;
use serialport;

pub struct SerialPlotterApp {
    // Data buffer: each sample is (timestamp, [channel values])
    data_buffer: Vec<(f64, Vec<f64>)>,
    max_buffer_size: usize,
    max_channels: usize,
    x: f64, // used for simulated (random) data

    // For each channel, track whether the channel should be drawn and whether its value should be processed.
    channel_visibility: Vec<bool>,
    channel_processed: Vec<bool>,

    // Whether to use random data (test mode) instead of a serial port.
    use_random: bool,

    // For buffer-size editing.
    buffer_size_input: String,

    // Serial port UI state.
    serial_ports: Vec<String>,
    selected_serial_port: Option<String>,
    serial_connected: bool,

    // For reading from serial in a background thread.
    serial_receiver: Option<Receiver<Vec<u8>>>,
    serial_thread: Option<thread::JoinHandle<()>>,
    serial_thread_running: Option<Arc<AtomicBool>>,

    // A status message (or error) to show to the user.
    last_message: Option<String>,
}

impl SerialPlotterApp {
    pub fn new() -> Self {
        let ports = match serialport::available_ports() {
            Ok(ports) => ports.into_iter().map(|p| p.port_name).collect(),
            Err(_) => Vec::new(),
        };

        Self {
            data_buffer: Vec::new(),
            max_buffer_size: 500,
            max_channels: 0,
            x: 0.0,
            channel_visibility: Vec::new(),
            channel_processed: Vec::new(),
            use_random: false,
            buffer_size_input: String::new(),
            serial_ports: ports,
            selected_serial_port: None,
            serial_connected: false,
            serial_receiver: None,
            serial_thread: None,
            serial_thread_running: None,
            last_message: None,
        }
    }

    /// (Re)initialize the channel-related UI state. In our example, we’ll default to a given number of channels.
    fn initialize_channels(&mut self, n: usize) {
        self.max_channels = n;
        self.channel_visibility = vec![true; n];
        self.channel_processed = vec![false; n];
    }

    /// Process a channel’s raw value according to its “processing function.”
    /// (Channel 1: identity, Channel 2: square, Channel 3: cube, others: identity.)
    fn process_value(&self, channel: usize, value: f64) -> f64 {
        match channel {
            0 => value,
            1 => value * value,
            2 => value * value * value,
            _ => value,
        }
    }

    /// Write out CSV data.
    /// If `processed` is true then apply processing to each channel’s value.
    fn export_csv(&mut self, processed: bool) {
        if self.data_buffer.is_empty() {
            self.last_message = Some("No data to export.".to_string());
            return;
        }

        if let Some(path) = FileDialog::new().set_title("Save CSV").save_file() {
            let mut file = match File::create(&path) {
                Ok(f) => f,
                Err(e) => {
                    self.last_message = Some(format!("Failed to create file: {}", e));
                    return;
                }
            };

            // Write header line.
            let mut header = String::from("Timestamp");
            for i in 0..self.max_channels {
                header.push_str(&format!(",Channel {}", i + 1));
            }
            header.push('\n');
            if let Err(e) = file.write_all(header.as_bytes()) {
                self.last_message = Some(format!("Failed to write header: {}", e));
                return;
            }

            // Write each data row.
            for (timestamp, channels) in &self.data_buffer {
                let mut row = format!("{}", timestamp);
                for i in 0..self.max_channels {
                    let value = if i < channels.len() {
                        if processed {
                            self.process_value(i, channels[i])
                        } else {
                            channels[i]
                        }
                    } else {
                        f64::NAN
                    };
                    row.push_str(&format!(",{}", value));
                }
                row.push('\n');
                if let Err(e) = file.write_all(row.as_bytes()) {
                    self.last_message = Some(format!("Failed to write row: {}", e));
                    return;
                }
            }
            self.last_message =
                Some(format!("Data exported to {}", path.to_string_lossy().to_string()));
        }
    }

    fn clear_data(&mut self) {
        self.data_buffer.clear();
    }

    fn refresh_serial_ports(&mut self) {
        self.serial_ports = match serialport::available_ports() {
            Ok(ports) => ports.into_iter().map(|p| p.port_name).collect(),
            Err(e) => {
                self.last_message = Some(format!("Error listing ports: {}", e));
                Vec::new()
            }
        };
    }

    /// Connect to the selected serial port and spawn a thread to read lines.
    fn connect_serial(&mut self) {
        if self.serial_connected {
            return;
        }
        let port_name = match &self.selected_serial_port {
            Some(name) => name.clone(),
            None => {
                self.last_message = Some("No serial port selected.".to_string());
                return;
            }
        };
        let port_result = serialport::new(port_name.clone(), 9600)
            .timeout(Duration::from_millis(10))
            .open();
        match port_result {
            Ok(port) => {
                // Create a channel to receive serial data.
                let (tx, rx) = channel();
                self.serial_receiver = Some(rx);
                let running = Arc::new(AtomicBool::new(true));
                self.serial_thread_running = Some(running.clone());
                // Spawn a thread to continuously read from the serial port.
                let handle = thread::spawn(move || {
                    let mut reader = BufReader::new(port);
                    while running.load(Ordering::Relaxed) {
                        let mut buffer = Vec::new();
                        // Read until newline.
                        match reader.read_until(b'\n', &mut buffer) {
                            Ok(n) if n > 0 => {
                                // Send the raw line (including the newline) to the main thread.
                                let _ = tx.send(buffer);
                            }
                            _ => {
                                // Sleep briefly to avoid a busy loop.
                                thread::sleep(Duration::from_millis(10));
                            }
                        }
                    }
                });
                self.serial_thread = Some(handle);
                self.serial_connected = true;
                self.last_message = Some(format!("Connected to {}", port_name));
            }
            Err(e) => {
                self.last_message = Some(format!("Failed to connect: {}", e));
            }
        }
    }

    fn disconnect_serial(&mut self) {
        if self.serial_connected {
            if let Some(running) = self.serial_thread_running.take() {
                running.store(false, Ordering::Relaxed);
            }
            if let Some(handle) = self.serial_thread.take() {
                let _ = handle.join();
            }
            self.serial_connected = false;
            self.serial_receiver = None;
            self.last_message = Some("Disconnected.".to_string());
        }
    }

    /// Update the data buffer.
    /// In random mode, generate a new sample.
    /// Otherwise, poll the serial thread’s receiver for new lines.
    fn update_data(&mut self) {
        if self.use_random {
            if self.max_channels == 0 {
                self.initialize_channels(3);
            }
            self.x += 1.0;
            let timestamp = self.x;
            let mut channels = Vec::new();
            for i in 0..self.max_channels {
                // For demo purposes, generate a sine wave with a frequency that depends on the channel.
                let val =
                    ((std::f64::consts::PI / 180.0) * self.x / ((i + 1) as f64)).sin();
                channels.push(val);
            }
            self.data_buffer.push((timestamp, channels));
        } else if self.serial_connected {
            if let Some(rx) = &self.serial_receiver {
                // Collect all available lines first.
                let mut lines = Vec::new();
                while let Ok(line) = rx.try_recv() {
                    lines.push(line);
                }
                // Track if we need to update the channel count.
                let mut new_max_channels = self.max_channels;
                for line in lines {
                    if let Some(parsed) = parse_binary_line(&line) {
                        if parsed.len() < 1 {
                            continue;
                        }
                        let timestamp = parsed[0] / 1_000_000.0;
                        let channels = parsed[1..].to_vec();
                        if channels.len() > new_max_channels {
                            new_max_channels = channels.len();
                        }
                        self.data_buffer.push((timestamp, channels));
                    }
                }
                if new_max_channels > self.max_channels {
                    self.initialize_channels(new_max_channels);
                }
            }
        }
        // Trim the buffer if necessary.
        if self.data_buffer.len() > self.max_buffer_size {
            let excess = self.data_buffer.len() - self.max_buffer_size;
            self.data_buffer.drain(0..excess);
        }
    }
}

/// Parse a binary line (similar to the Python version).
/// We expect the first value to be 4 bytes (timestamp) and each subsequent value to be 2 bytes.
/// (For the 16-bit channel values, we convert them to i16.)
fn parse_binary_line(line: &[u8]) -> Option<Vec<f64>> {
    // Split on commas.
    let mut parts: Vec<&[u8]> = line.split(|&b| b == b',').collect();
    // Remove a trailing newline from the last part.
    if let Some(last) = parts.last_mut() {
        if let Some(stripped) = last.strip_suffix(b"\n") {
            *last = stripped;
        }
    }
    let mut result = Vec::new();
    for (i, part) in parts.iter().enumerate() {
        if i == 0 {
            if part.len() != 4 {
                return None;
            }
            let arr: [u8; 4] = (*part).try_into().ok()?;
            let val = u32::from_le_bytes(arr);
            result.push(val as f64);
        } else {
            if part.len() != 2 {
                return None;
            }
            let arr: [u8; 2] = (*part).try_into().ok()?;
            // Converting the raw u16 to an i16.
            let sval = i16::from_le_bytes(arr) as f64;
            result.push(sval);
        }
    }
    Some(result)
}

impl eframe::App for SerialPlotterApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        // Update our data (simulate the timer).
        self.update_data();

        CentralPanel::default().show(ctx, |ui| {
            ui.heading("CPR Phantom Interface");

            // Buffer size controls.
            ui.horizontal(|ui| {
                ui.label("Set Max Buffer Size:");
                ui.add(TextEdit::singleline(&mut self.buffer_size_input).desired_width(50.0));
                if ui.button("Update Buffer Size").clicked() {
                    if let Ok(new_size) = self.buffer_size_input.trim().parse::<usize>() {
                        if new_size > 0 {
                            self.max_buffer_size = new_size;
                            self.buffer_size_input.clear();
                        }
                    }
                }
            });

            // Serial port selection.
            ui.horizontal(|ui| {
                ui.label("Serial Port:");
                ComboBox::from_id_source("serial_port_combo")
                    .selected_text(
                        self.selected_serial_port
                            .clone()
                            .unwrap_or_else(|| "Select Port".to_string()),
                    )
                    .show_ui(ui, |ui| {
                        for port in &self.serial_ports {
                            ui.selectable_value(&mut self.selected_serial_port, Some(port.clone()), port);
                        }
                    });
                if ui.button("Refresh").clicked() {
                    self.refresh_serial_ports();
                }
                if !self.serial_connected {
                    if ui.button("Connect").clicked() {
                        self.connect_serial();
                    }
                } else if ui.button("Disconnect").clicked() {
                    self.disconnect_serial();
                }
            });

            // Random data (Test Mode) checkbox.
            ui.checkbox(&mut self.use_random, "Use Random Data (Test Mode)");

            // Channels UI.
            ui.group(|ui| {
                ui.label("Channels");
                if self.max_channels > 0 {
                    for i in 0..self.max_channels {
                        ui.horizontal(|ui| {
                            ui.checkbox(
                                &mut self.channel_visibility[i],
                                format!("Channel {}", i + 1),
                            );
                            ui.checkbox(&mut self.channel_processed[i], "Processed");
                        });
                    }
                }
            });

            // Export and Clear buttons.
            ui.horizontal(|ui| {
                if ui.button("Export Raw Data to CSV").clicked() {
                    self.export_csv(false);
                }
                if ui.button("Export Processed Data to CSV").clicked() {
                    self.export_csv(true);
                }
                if ui.button("Clear Data").clicked() {
                    self.clear_data();
                }
            });

            if let Some(ref msg) = self.last_message {
                ui.label(msg);
            }

            // Plot the data.
            egui::widgets::plot::Plot::new("data_plot").height(300.0).show(ui, |plot_ui| {
                for i in 0..self.max_channels {
                    if self.channel_visibility.get(i).copied().unwrap_or(false) {
                        // Build a list of points for this channel.
                        let points: PlotPoints = self
                            .data_buffer
                            .iter()
                            .map(|(t, channels)| {
                                let y = if i < channels.len() {
                                    if self.channel_processed[i] {
                                        self.process_value(i, channels[i])
                                    } else {
                                        channels[i]
                                    }
                                } else {
                                    f64::NAN
                                };
                                [*t, y]
                            })
                            .collect();
                        // Cycle through some colors.
                        let colors = [
                            egui::Color32::YELLOW,
                            egui::Color32::RED,
                            egui::Color32::GREEN,
                            egui::Color32::CYAN,
                            egui::Color32::MAGENTA,
                            egui::Color32::WHITE,
                            egui::Color32::BLUE,
                        ];
                        let color = colors[i % colors.len()];
                        let line = Line::new(points).color(color);
                        plot_ui.line(line);
                    }
                }
            });
        });
        // Request continuous repainting.
        ctx.request_repaint();
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "CPR Phantom Interface",
        options,
        Box::new(|_cc| Ok(Box::new(SerialPlotterApp::new()))),
    )
}
