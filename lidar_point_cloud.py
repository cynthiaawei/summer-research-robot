import serial
import time
import csv


def find_serial_port():
    """Try different serial port names"""
    ports = ['/dev/serial10', '/dev/ttyS0', '/dev/ttyAMA0']
    for port in ports:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.5)
            print(f"Successfully opened {port}")
            return ser
        except serial.SerialException as e:
            print(f"Failed to open {port}: {e}")
    return None

def read_tfluna(ser):
    """Read TF-Luna data with improved error handling"""
    try:
        # Clear any existing data in buffer
        ser.flushInput()
        
        # 1) Find the two-byte header 0x59 0x59
        header_count = 0
        max_attempts = 100  # Prevent infinite loop
        
        while header_count < max_attempts:
            b1 = ser.read(1)
            if not b1:
                header_count += 1
                continue
                
            if b1 != b'\x59': 
                header_count += 1
                continue
                
            b2 = ser.read(1)
            if not b2:
                header_count += 1
                continue
                
            if b2 == b'\x59':
                break
            header_count += 1
        
        if header_count >= max_attempts:
            return None, "Header timeout"
        
        # 2) Read the next 7 bytes
        packet = ser.read(7)
        if len(packet) < 7:
            return None, f"Incomplete packet: got {len(packet)} bytes"
        
        # 3) Verify checksum
        data = b'\x59\x59' + packet
        calculated_checksum = sum(data[0:8]) & 0xFF
        received_checksum = data[8]
        
        if calculated_checksum != received_checksum:
            return None, f"Checksum mismatch: calc={calculated_checksum}, recv={received_checksum}"
        
        # 4) Parse data
        dist = packet[0] | (packet[1] << 8)
        strength = packet[2] | (packet[3] << 8)
        raw_temp = packet[4] | (packet[5] << 8)
        temp_c = raw_temp / 8.0 - 256
        
        return (dist, strength, temp_c), None
        
    except Exception as e:
        return None, f"Exception: {e}"

def main():
    # Try to open serial port
    ser = find_serial_port()
    while True:
        out = input("Should I start Recording: ")
        if out == 'y':
            break
    
    if not ser:
        print("ERROR: Could not open any serial port!")
        return
    
    print("TF-Luna LiDAR Reader - Press Ctrl+C to stop")
    print("-" * 50)
    
    success_count = 0
    error_count = 0
    with open('file.csv', 'w', newline='') as f:
        csvwriter = csv.writer(f)
        try:
            while True:
                result, error = read_tfluna(ser)
                
                if result:
                    dist, strength, temp = result
                    success_count += 1
                    print(f"✓ Distance: {dist:4d} cm | Strength: {strength:4d} | Temp: {temp:5.1f}°C | Success: {success_count}")
                    csvwriter.writerow([dist, strength, temp])
                    # Check for reasonable values
                    if dist == 0 or dist > 8000:
                        print(f"  ⚠ Warning: Distance {dist}cm seems unusual")
                    if strength == 0 or strength > 65000:
                        print(f"  ⚠ Warning: Strength {strength} seems unusual")
                    
                else:
                    error_count += 1
                    print(f"✗ Error #{error_count}: {error}")
                    
                    # Too many consecutive errors might indicate a problem
                    if error_count > 10 and success_count == 0:
                        print("Too many errors with no successful reads.")
                        print("Check wiring and power to TF-Luna sensor.")
                        break
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\nStopped. Success: {success_count}, Errors: {error_count}")
        
        finally:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
