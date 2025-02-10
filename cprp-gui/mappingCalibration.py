import sys
import math
import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QMessageBox
)
from PyQt5.QtCore import QTimer

class CalibrationPlotter(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration Plotter - Processing Function Creator")
        self.resize(1000, 800)
        
        # Data buffer: Each sample is a tuple (timestamp, raw_value, processed_value)
        self.data_buffer = []
        self.max_buffer_size = 1000
        
        # Serial port object (will be created when connected)
        self.serial_port = None
        
        # Build UI
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # --- Serial port controls ---
        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        serial_layout.addWidget(self.port_combo)
        self.refresh_ports()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        serial_layout.addWidget(self.connect_button)
        main_layout.addLayout(serial_layout)
        
        # --- Create three separate plot widgets ---
        self.raw_plot = pg.PlotWidget(title="Raw Data (Time Series)")
        self.processed_plot = pg.PlotWidget(title="Processed Data (Time Series)")
        self.mapping_plot = pg.PlotWidget(title="Mapping Function (Raw vs. Processed)")
        main_layout.addWidget(self.raw_plot)
        main_layout.addWidget(self.processed_plot)
        main_layout.addWidget(self.mapping_plot)
        
        # --- Control Buttons ---
        btn_layout = QHBoxLayout()
        self.clear_button = QPushButton("Clear Data")
        self.clear_button.clicked.connect(self.clear_data)
        btn_layout.addWidget(self.clear_button)
        main_layout.addLayout(btn_layout)
        
        # --- Plot curves and items ---
        # These curves allow us to update the data without re-creating plots.
        self.raw_curve = self.raw_plot.plot([], [], pen='y')
        self.processed_curve = self.processed_plot.plot([], [], pen='g')
        # In the mapping plot we'll show:
        #   - A scatter of all (raw, processed) calibration points.
        #   - The fitted mapping function (line).
        #   - A marker for the current mapping.
        self.mapping_scatter = self.mapping_plot.plot([], [], pen=None, symbol='o', symbolBrush='b')
        self.mapping_fit_curve = self.mapping_plot.plot([], [], pen=pg.mkPen('r', width=2))
        self.current_marker = self.mapping_plot.plot([], [], pen=None, symbol='x', symbolBrush='r', symbolSize=12)
        
        # --- Timer to update data ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)  # update every 50 ms
        
    def refresh_ports(self):
        """Populate the serial port combo box with available ports."""
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(port.device)
    
    def connect_serial(self):
        """Connect to the selected serial port."""
        port_name = self.port_combo.currentText()
        if not port_name:
            QMessageBox.warning(self, "No Port", "No serial port selected.")
            return
        try:
            self.serial_port = serial.Serial(port_name, 9600, timeout=1)
            QMessageBox.information(self, "Connected", f"Connected to {port_name}.")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Connection Error", f"Error opening serial port: {e}")
            self.serial_port = None
            
    def update_data(self):
        """
        Read a new data line from the serial port, update the data buffers and plots.
        Expected CSV format:
            timestamp, raw_value, processed_value
        """
        new_line = None
        if self.serial_port:
            if self.serial_port.in_waiting:
                try:
                    new_line = self.serial_port.readline().decode('utf-8').strip()
                except Exception as e:
                    print(f"Error reading serial data: {e}")
                    return
        # For testing you might uncomment a simulation block here.
        # if new_line is None:
        #     # Simulate data: use current time (in seconds) and a known mapping.
        #     timestamp = pg.ptime.time()
        #     raw_val = math.sin(timestamp)
        #     processed_val = raw_val**2 + 0.5  # for example, a non-linear mapping
        #     new_line = f"{timestamp},{raw_val},{processed_val}"
        
        if new_line:
            parts = new_line.split(',')
            if len(parts) < 3:
                print("Invalid data format. Expected at least 3 columns.")
                return
            try:
                timestamp = float(parts[0])
                raw_val = float(parts[1])
                processed_val = float(parts[2])
            except ValueError:
                print("Error parsing data:", new_line)
                return
            
            # Append the new calibration point
            self.data_buffer.append((timestamp, raw_val, processed_val))
            if len(self.data_buffer) > self.max_buffer_size:
                self.data_buffer = self.data_buffer[-self.max_buffer_size:]
            
            # Update the raw data time series plot
            ts = [pt[0] for pt in self.data_buffer]
            raw_vals = [pt[1] for pt in self.data_buffer]
            self.raw_curve.setData(ts, raw_vals)
            
            # Update the processed data time series plot
            proc_vals = [pt[2] for pt in self.data_buffer]
            self.processed_curve.setData(ts, proc_vals)
            
            # Update the mapping (raw vs. processed) scatter
            self.mapping_scatter.setData(raw_vals, proc_vals)
            
            # If we have enough calibration points, fit a mapping function.
            if len(self.data_buffer) >= 3:
                X = np.array(raw_vals)
                Y = np.array(proc_vals)
                # Fit a quadratic polynomial (degree 2)
                coeffs = np.polyfit(X, Y, 2)
                poly = np.poly1d(coeffs)
                # Create a fitted curve over the observed range of raw values.
                x_min, x_max = np.min(X), np.max(X)
                x_fit = np.linspace(x_min, x_max, 200)
                y_fit = poly(x_fit)
                self.mapping_fit_curve.setData(x_fit, y_fit)
                
                # Show the mapping for the current raw value using the fitted function.
                current_mapped = poly(raw_val)
                self.current_marker.setData([raw_val], [current_mapped])
            else:
                self.mapping_fit_curve.setData([], [])
                self.current_marker.setData([], [])
    
    def clear_data(self):
        """Clear the collected calibration data and reset all plots."""
        self.data_buffer = []
        self.raw_curve.setData([], [])
        self.processed_curve.setData([], [])
        self.mapping_scatter.setData([], [])
        self.mapping_fit_curve.setData([], [])
        self.current_marker.setData([], [])
        print("Data cleared.")
        
    def closeEvent(self, event):
        self.timer.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CalibrationPlotter()
    window.show()
    sys.exit(app.exec_())
