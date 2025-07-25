#!/usr/bin/env python3

import serial
import time
import sys

def find_serial_port():
    """Try the serial ports that actually exist on your system"""
    ports = [
        # This links to ttyAMA10 
        '/dev/ttyAMA10',   # Direct access  
        '/dev/ttyAMA1',    # Alternative UART
    ]
    
    for port in ports:
        try:
            print(f"Trying {port}...", end=" ")
            ser = serial.Serial(port, baudrate=115200, timeout=0.5)
            print(f"✓ Successfully opened!")
            return ser, port
        except serial.SerialException as e:
            print(f"✗ Failed: {e}")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    return None, None

def read_tfluna(ser):
    """Read TF-Luna data with improved error handling"""
    try:
        # Clear any existing data in buffer
        ser.flushInput()
        
        # 1) Find the two-byte header 0x59 0x59
        header_count = 0
        max_attempts = 100
        
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
            return None, f"Checksum mismatch: calc={calculated_checksum:02x}, recv={received_checksum:02x}"
        
        # 4) Parse data
        dist = packet[0] | (packet[1] << 8)
        strength = packet[2] | (packet[3] << 8)
        raw_temp = packet[4] | (packet[5] << 8)
        temp_c = raw_temp / 8.0 - 256
        
        return (dist, strength, temp_c), None
        
    except Exception as e:
        return None, f"Exception: {e}"

def debug_raw_data(ser, port_name):
    """Look for any data coming through"""
    print(f"\nDebugging raw data on {port_name}")
    print("Listening for 5 seconds...")
    
    start_time = time.time()
    total_bytes = 0
    
    while time.time() - start_time < 5.0:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            print(f"Got {len(data)} bytes: {[hex(b) for b in data[:20]]}")
            
            # Look for TF Luna header
            for i in range(len(data) - 1):
                if data[i] == 0x59 and data[i + 1] == 0x59:
                    print(f"Found TF Luna header at position {i}!")
        
        time.sleep(0.1)
    
    print(f"Total bytes received: {total_bytes}")
    return total_bytes > 0

def main():
    print("TF-Luna Debug Script")
    print("===================")
    
    # Try to open a working serial port
    ser, port_name = find_serial_port()
    if not ser:
        print("\n❌ Could not open any serial port!")
        print("Available devices found:")
        import os
        for device in ['/dev/serial0', '/dev/ttyAMA1', '/dev/ttyAMA10']:
            exists = "✓" if os.path.exists(device) else "✗"
            print(f"  {exists} {device}")
        return
    
    print(f"\n🎉 Using serial port: {port_name}")
    
    # First, try to get structured TF Luna data
    print(f"\nTesting TF-Luna protocol...")
    success_count = 0
    error_count = 0
    
    for attempt in range(10):
        result, error = read_tfluna(ser)
        
        if result:
            dist, strength, temp = result
            success_count += 1
            print(f"✓ Distance: {dist:4d} mm | Strength: {strength:4d} | Temp: {temp:5.1f}°C")
        else:
            error_count += 1
            print(f"✗ Error: {error}")
        
        time.sleep(0.2)
    
    if success_count == 0:
        print(f"\n⚠️  No valid TF Luna packets found. Trying raw data debug...")
        debug_raw_data(ser, port_name)
    else:
        print(f"\n✅ Success! Got {success_count}/10 valid readings")
    
    ser.close()
    print("Serial port closed.")

if __name__ == "__main__":
    main()
