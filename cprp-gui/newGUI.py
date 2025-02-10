import sys
import random
import math
import csv
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLineEdit, QPushButton, QVBoxLayout, QWidget, QLabel,
    QHBoxLayout, QCheckBox, QComboBox, QFileDialog, QMessageBox, QGroupBox
)
from PyQt5.QtCore import QTimer

class SerialPlotter(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("CPR Phantom Interface")

        # Data buffer: each sample is a tuple (timestamp, [channel values])
        self.data_buffer = []
        self.max_buffer_size = 500

        # Keep track of the maximum number of channels encountered so far.
        self.max_channels = 0

        # For random mode, we use self.x as a simulated timestamp.
        self.x = 0

        # UI elements for each channel.
        self.channel_checkboxes = []            # Toggle channel visibility
        self.channel_processed_checkboxes = []    # Toggle raw vs processed data
        self.plot_curves = []                   # One plot curve per channel
        self.process_funcs = []                 # Processing function for each channel

        # Flag to choose random test data versus serial input.
        self.use_random = False
        self.serial_port = None

        # Build the UI.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Plot widget.
        self.plot_widget = pg.PlotWidget()
        main_layout.addWidget(self.plot_widget)

        # Buffer size controls.
        buffer_layout = QHBoxLayout()
        self.buffer_input_label = QLabel("Set Max Buffer Size:")
        buffer_layout.addWidget(self.buffer_input_label)
        self.buffer_input = QLineEdit()
        self.buffer_input.setPlaceholderText(str(self.max_buffer_size))
        buffer_layout.addWidget(self.buffer_input)
        self.set_buffer_button = QPushButton("Update Buffer Size")
        buffer_layout.addWidget(self.set_buffer_button)
        self.set_buffer_button.clicked.connect(self.update_buffer_size)
        main_layout.addLayout(buffer_layout)

        # Serial port selection.
        serial_layout = QHBoxLayout()
        self.serial_port_label = QLabel("Serial Port:")
        serial_layout.addWidget(self.serial_port_label)
        self.port_combo = QComboBox()
        serial_layout.addWidget(self.port_combo)
        self.refresh_ports()
        self.connect_button = QPushButton("Connect")
        serial_layout.addWidget(self.connect_button)
        self.connect_button.clicked.connect(self.connect_serial)
        main_layout.addLayout(serial_layout)

        # Random data (Test Mode) checkbox.
        self.random_checkbox = QCheckBox("Use Random Data (Test Mode)")
        self.random_checkbox.stateChanged.connect(self.toggle_random_mode)
        main_layout.addWidget(self.random_checkbox)

        # Channels UI: these will be dynamically created when the first sample arrives.
        self.channels_group = QGroupBox("Channels")
        self.channels_layout = QVBoxLayout()
        self.channels_group.setLayout(self.channels_layout)
        main_layout.addWidget(self.channels_group)

        # Export and Clear buttons.
        buttons_layout = QHBoxLayout()
        self.export_raw_button = QPushButton("Export Raw Data to CSV")
        buttons_layout.addWidget(self.export_raw_button)
        self.export_raw_button.clicked.connect(self.export_raw_data)
        self.export_processed_button = QPushButton("Export Processed Data to CSV")
        buttons_layout.addWidget(self.export_processed_button)
        self.export_processed_button.clicked.connect(self.export_processed_data)
        self.clear_button = QPushButton("Clear Data")
        buttons_layout.addWidget(self.clear_button)
        self.clear_button.clicked.connect(self.clear_data)
        main_layout.addLayout(buttons_layout)

        # Timer to periodically update the plot.
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)

    def refresh_ports(self):
        """Populate the serial port combo box with available ports."""
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(port.device)

    def connect_serial(self):
        """Attempt to open the selected serial port."""
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

    def toggle_random_mode(self, state):
        """Toggle between random test data and real serial data."""
        self.use_random = (state == 2)  # Qt.Checked is 2

    def update_buffer_size(self):
        """Update the maximum size of the data buffer."""
        try:
            new_size = int(self.buffer_input.text())
            if new_size > 0:
                self.max_buffer_size = new_size
                self.buffer_input.clear()
                self.buffer_input.setPlaceholderText(str(self.max_buffer_size))
                print(f"Buffer size updated to: {self.max_buffer_size}")
            else:
                print("Buffer size must be a positive integer.")
        except ValueError:
            print("Please enter a valid integer for buffer size.")

    def initialize_channels(self, n):
        """
        (Re)initialize the channels UI, plot curves, and processing functions
        for n channels.
        """
        # Clear any existing channel UI elements.
        for i in reversed(range(self.channels_layout.count())):
            item = self.channels_layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget:
                    widget.deleteLater()
                else:
                    # In case the item is a layout.
                    while item.count():
                        child = item.takeAt(0)
                        if child.widget():
                            child.widget().deleteLater()
        self.channel_checkboxes = []
        self.channel_processed_checkboxes = []
        self.plot_curves = []
        self.process_funcs = []
        
        # Define a list of colors.
        colors = ['y', 'r', 'g', 'c', 'm', 'w', 'b']
        for i in range(n):
            channel_layout = QHBoxLayout()
            # Checkbox for channel visibility.
            ch_checkbox = QCheckBox(f"Channel {i+1}")
            ch_checkbox.setChecked(True)
            self.channel_checkboxes.append(ch_checkbox)
            channel_layout.addWidget(ch_checkbox)
            # Checkbox for toggling processed/raw display.
            processed_checkbox = QCheckBox("Processed")
            processed_checkbox.setChecked(False)
            self.channel_processed_checkboxes.append(processed_checkbox)
            channel_layout.addWidget(processed_checkbox)
            self.channels_layout.addLayout(channel_layout)
            
            # Create a plot curve for this channel.
            color = colors[i % len(colors)]
            curve = self.plot_widget.plot(pen=color, name=f"Channel {i+1}")
            self.plot_curves.append(curve)
            
            # Assign a processing function.
            # For demo: Channel 1 = identity, Channel 2 = square, Channel 3 = cube, others = identity.
            if i == 0:
                self.process_funcs.append(lambda x: x)
            elif i == 1:
                self.process_funcs.append(lambda x: x**2)
            elif i == 2:
                self.process_funcs.append(lambda x: x**3)
            else:
                self.process_funcs.append(lambda x: x)

    def update_plot(self):
        """Read the next line (from serial or random mode), parse it, and update the plot.
        
        Assumes that the first CSV column is the timestamp (x-axis) and the remaining columns
        are channel values.
        """
        new_line = None

        if self.use_random:
            # If in test mode and no channels have been initialized, assume a default of 3.
            if self.max_channels == 0:
                self.max_channels = 3
                self.initialize_channels(self.max_channels)
            self.x += 1
            sample_timestamp = self.x
            # Use the provided random generation formula.
            values = [math.sin((math.pi/180.0) * self.x / (i + 1)) for i in range(self.max_channels)]
            new_line = ",".join([str(sample_timestamp)] + [f"{v:.4f}" for v in values])
        else:
            if self.serial_port is None:
                return
            if self.serial_port.in_waiting:
                try:
                    new_line = self.serial_port.readline().decode('utf-8').strip()
                except Exception as e:
                    print(f"Error reading serial data: {e}")
                    return

        if new_line:
            parts = new_line.split(',')
            try:
                # First column is the timestamp.
                timestamp = float(parts[0])
                # The rest are the channel values.
                channel_values = [float(x) for x in parts[1:]]
            except ValueError:
                print(f"Invalid data received: {new_line}")
                return

            # Update maximum channel count if the new sample has more channels.
            if len(channel_values) > self.max_channels:
                self.max_channels = len(channel_values)
                self.initialize_channels(self.max_channels)
            # Append the new sample. (If a sample has fewer channels than max_channels,
            # missing values will be handled when plotting.)
            self.data_buffer.append((timestamp, channel_values))
            if len(self.data_buffer) > self.max_buffer_size:
                self.data_buffer = self.data_buffer[-self.max_buffer_size:]
            
            # Update plots for each channel.
            for i, curve in enumerate(self.plot_curves):
                xs = []
                ys = []
                for sample in self.data_buffer:
                    t = sample[0]
                    # If the sample doesn’t have a value for channel i, use NaN.
                    if i < len(sample[1]):
                        raw_val = sample[1][i]
                    else:
                        raw_val = float('nan')
                    # Use the processed value if the "Processed" checkbox is checked.
                    if self.channel_processed_checkboxes[i].isChecked():
                        y_val = self.process_funcs[i](raw_val)
                    else:
                        y_val = raw_val
                    xs.append(t)
                    ys.append(y_val)
                if self.channel_checkboxes[i].isChecked():
                    curve.setData(xs, ys)
                else:
                    curve.setData([], [])

    def export_raw_data(self):
        """Export all raw data (even data that is not currently shown) to a CSV file."""
        if not self.data_buffer:
            print("No data to export.")
            return
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Raw Data", "", "CSV Files (*.csv)", options=options)
        if file_path:
            try:
                with open(file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    # Header: Timestamp, Channel 1, Channel 2, ...
                    header = ["Timestamp"] + [f"Channel {i+1}" for i in range(self.max_channels)]
                    writer.writerow(header)
                    for row in self.data_buffer:
                        row_data = [row[0]]
                        for i in range(self.max_channels):
                            if i < len(row[1]):
                                row_data.append(row[1][i])
                            else:
                                row_data.append("")
                        writer.writerow(row_data)
                print(f"Raw data exported to {file_path}")
            except Exception as e:
                print(f"Failed to export raw data: {e}")

    def export_processed_data(self):
        """Export all processed data (raw data passed through the processing functions) to CSV."""
        if not self.data_buffer:
            print("No data to export.")
            return
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Processed Data", "", "CSV Files (*.csv)", options=options)
        if file_path:
            try:
                with open(file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    header = ["Timestamp"] + [f"Channel {i+1}" for i in range(self.max_channels)]
                    writer.writerow(header)
                    for row in self.data_buffer:
                        row_data = [row[0]]
                        for i in range(self.max_channels):
                            if i < len(row[1]):
                                raw_val = row[1][i]
                                processed_val = self.process_funcs[i](raw_val)
                                row_data.append(processed_val)
                            else:
                                row_data.append("")
                        writer.writerow(row_data)
                print(f"Processed data exported to {file_path}")
            except Exception as e:
                print(f"Failed to export processed data: {e}")

    def clear_data(self):
        """Clear all stored data and reset the plots."""
        self.data_buffer = []
        for curve in self.plot_curves:
            curve.setData([], [])
        print("Data cleared.")

    def closeEvent(self, event):
        self.timer.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    plotter = SerialPlotter()
    plotter.show()
    sys.exit(app.exec_())
