import sys
import serial
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer

class SerialPlotter(QMainWindow):
    def __init__(self, port='COM8', baudrate=9600, parent=None):
        super().__init__(parent)
        self.setWindowTitle('CPR Phantom Interface')
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Plot widget
        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)
        self.plot_data = self.plot_widget.plot(pen='y')
        self.data_buffer = []
        self.max_buffer_size = 500

        # Input for buffer size
        self.buffer_input_label = QLabel('Set Max Buffer Size:')
        layout.addWidget(self.buffer_input_label)
        self.buffer_input = QLineEdit()
        self.buffer_input.setPlaceholderText(str(self.max_buffer_size))
        layout.addWidget(self.buffer_input)

        # Button to set buffer size
        self.set_buffer_button = QPushButton('Update Buffer Size')
        layout.addWidget(self.set_buffer_button)
        self.set_buffer_button.clicked.connect(self.update_buffer_size)

        # Initialize serial port
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1)
        self.x = 0

    def update_buffer_size(self):
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

    def update_plot(self):
        if self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                y_str = line
                y = float(y_str)
                self.x += 1
                self.data_buffer.append((self.x, y))
                # truncate the buffer
                self.data_buffer = self.data_buffer[-self.max_buffer_size:]
                x_data, y_data = zip(*self.data_buffer)
                self.plot_data.setData(x_data, y_data)
            except ValueError:
                print(f"Invalid data received: {line}")

    def closeEvent(self, event):
        self.timer.stop()
        self.serial_port.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    plotter = SerialPlotter()
    plotter.show()
    sys.exit(app.exec_())