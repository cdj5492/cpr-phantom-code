import serial
import sys
import time
import pylsl
import argparse

def main(serial_port='COM11', baud_rate=115200, stream_name="SerialData", stream_type="Sensor"):
    # --- Setup serial connection ---
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Opened serial port {serial_port} at {baud_rate} baud.")
    except Exception as e:
        print("Error opening serial port:", e)
        sys.exit(1)

    # Read one line to determine the number of channels
    # This assumes the data format is consistent
    while True:
        raw_line = ser.readline()
        if raw_line:
            try:
                line = raw_line.decode('utf-8').strip()
                parts = line.split(',')
                if len(parts) >= 2:  # At least timestamp and one data point
                    n_channels = len(parts) - 1  # Subtract 1 for the timestamp
                    break
            except:
                continue
        time.sleep(0.1)

    # Reset serial position
    ser.reset_input_buffer()

    # Create channel names (generic naming)
    channel_names = [f"Channel_{i}" for i in range(n_channels)]

    # --- Setup LSL stream ---
    # Create a new stream info
    info = pylsl.StreamInfo(
        stream_name,      # Stream name
        stream_type,      # Stream type
        n_channels,       # Number of channels
        0,                # Irregular sampling rate (we'll use timestamps from the data)
        "float32",        # Data format
        "serial_sensor"   # Stream ID
    )

    # Add metadata
    info.desc().append_child_value("manufacturer", "SerialSensor")
    chns = info.desc().append_child("channels")
    for i, label in enumerate(channel_names):
        ch = chns.append_child("channel")
        ch.append_child_value("label", label)
        ch.append_child_value("unit", "unknown")
        ch.append_child_value("type", "Sensor")

    # Create an outlet
    outlet = pylsl.StreamOutlet(info, 32, 360)
    print(f"LSL stream '{stream_name}' created with {n_channels} channels")

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
                
                # If number of data points doesn't match the expected channel count, pad or truncate
                if len(data_points) != n_channels:
                    if len(data_points) < n_channels:
                        data_points.extend([0.0] * (n_channels - len(data_points)))
                    else:
                        data_points = data_points[:n_channels]
                        
            except ValueError as e:
                print("Conversion error:", e, "in line:", line)
                continue
                
            # Send the sample to LSL with the timestamp
            outlet.push_sample(data_points, timestamp)
            packet_count += 1
            
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
    parser = argparse.ArgumentParser(description='Serial to LSL Bridge')
    parser.add_argument('--port', default='COM11', help='Serial port (default: COM11)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--name', default='SerialData', help='LSL stream name (default: SerialData)')
    parser.add_argument('--type', default='Sensor', help='LSL stream type (default: Sensor)')
    args = parser.parse_args()
    
    main(serial_port=args.port, baud_rate=args.baud, stream_name=args.name, stream_type=args.type)