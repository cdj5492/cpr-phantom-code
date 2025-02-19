import serial
import socket
import sys
import json
import time

def main():
    # --- Configuration ---
    serial_port = 'COM11'  # Update to your serial port
    baud_rate = 115200     # Set the baud rate as needed
    udp_ip = "127.0.0.1"   # IP address where PlotJuggler is listening
    udp_port = 9870        # UDP port for PlotJuggler

    # --- Setup serial connection ---
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Opened serial port {serial_port} at {baud_rate} baud.")
    except Exception as e:
        print("Error opening serial port:", e)
        sys.exit(1)

    # --- Setup UDP socket ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"UDP socket created, sending to {udp_ip}:{udp_port}")

    packet_count = 0
    last_time = time.time()

    try:
        while True:
            # Read one line from the serial port
            raw_line = ser.readline()
            if not raw_line:
                continue  # No data available, skip this iteration

            try:
                # Decode and strip newline characters
                line = raw_line.decode('utf-8').strip()
            except UnicodeDecodeError as e:
                print("Decoding error:", e)
                continue

            # Split the comma-separated values
            parts = line.split(',')
            if len(parts) < 2:
                continue  # Expect at least a timestamp and one datapoint

            try:
                # Convert timestamp from microseconds to seconds
                timestamp_us = float(parts[0])
                timestamp = timestamp_us / 1e6
                # Convert remaining fields to floats for datapoints
                data_points = [float(val) for val in parts[1:]]
            except ValueError as e:
                print("Conversion error:", e, "in line:", line)
                continue

            # Create a minimal JSON structure
            data_json = {"t": timestamp, "d": data_points}
            udp_message = json.dumps(data_json)

            try:
                sock.sendto(udp_message.encode('utf-8'), (udp_ip, udp_port))
                packet_count += 1
            except Exception as e:
                print("UDP send error:", e)

            # Print packets per second every second
            current_time = time.time()
            if current_time - last_time >= 1.0:
                print(f"Packets per second: {packet_count}")
                packet_count = 0
                last_time = current_time

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
