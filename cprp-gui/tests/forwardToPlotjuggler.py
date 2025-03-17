import serial
import socket
import sys
import json
import time
import struct

MAGIC = 0xAA55

def read_packet(ser):
    # Read the fixed header: 2 bytes magic + 2 bytes length
    header = ser.read(3)
    if len(header) < 3:
        return None
    magic, payload_length = struct.unpack('<HB', header)
    if magic != MAGIC:
        print("Invalid magic value, discarding packet")
        return None
    # Read the payload using the length from the header
    payload = ser.read(payload_length)
    if len(payload) < payload_length:
        print("Incomplete payload")
        return None
    return payload

def main():
    # --- Configuration ---
    serial_port = 'COM5'   # Serial port where the device is connected
    baud_rate = 115200     # Baud rate as needed
    udp_ip = "127.0.0.1"   # IP address where PlotJuggler is listening
    udp_port = 9870        # UDP port for PlotJuggler

    # number of boards sending all their ADC channels
    NUM_BOARDS = 1
    NUM_ADC_VALUES = NUM_BOARDS * 4

    # Expected packet length: 4 bytes for the timestamp + 2 bytes per ADC value.
    expected_payload_length = 4 + 2 * NUM_ADC_VALUES

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
            packet = read_packet(ser)
            if packet is None:
                continue
            if len(packet) != expected_payload_length:
                print(f"Payload length mismatch: got {len(packet)} bytes, expected {expected_payload_length}")
                continue

            try:
                # Unpack payload: first 4 bytes are timestamp, then ADC values
                timestamp, = struct.unpack('<I', packet[:4])
                adc_values = list(struct.unpack(f'<{NUM_ADC_VALUES}H', packet[4:]))
                timestamp_sec = timestamp / 1e3  # Convert ms to seconds if needed
            except struct.error as e:
                print("Unpacking error:", e)
                continue

            # Create JSON structure for UDP transmission
            data_json = {"t": timestamp_sec, "d": adc_values}
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