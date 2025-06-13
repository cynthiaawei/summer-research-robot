import serial
import time
import json
import csv
from datetime import datetime

# Serial setup for TF-Luna
ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.5)

def read_tfluna():
    """Read TF-Luna data packet"""
    while True:
        b1 = ser.read(1)
        if not b1 or b1 != b'\x59': 
            continue
        b2 = ser.read(1)
        if b2 == b'\x59':
            break
    
    packet = ser.read(7)
    if len(packet) < 7:
        return None
    
    data = b'\x59\x59' + packet
    
    if (sum(data[0:8]) & 0xFF) != data[8]:
        return None
    
    distance = packet[0] | (packet[1] << 8)
    strength = packet[2] | (packet[3] << 8)
    raw_temp = packet[4] | (packet[5] << 8)
    temp_c = raw_temp / 8.0 - 256
    
    return distance, strength, temp_c

class DataStorage:
    def __init__(self):
        self.data_points = []
        self.recording = False
        
    def add_point(self, distance, strength, temp, angle=0):
        """Add a data point with timestamp"""
        timestamp = time.time()
        point = {
            'timestamp': timestamp,
            'distance_mm': distance,
            'distance_m': distance / 1000.0,
            'strength': strength,
            'temperature': temp,
            'angle': angle
        }
        self.data_points.append(point)
        
    def save_to_json(self, filename=None):
        """Save data to JSON file"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"lidar_data_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.data_points, f, indent=2)
        
        print(f"Saved {len(self.data_points)} points to {filename}")
        return filename
    
    def save_to_csv(self, filename=None):
        """Save data to CSV file"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"lidar_data_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Header
            writer.writerow(['timestamp', 'distance_mm', 'distance_m', 'strength', 'temperature', 'angle'])
            
            # Data
            for point in self.data_points:
                writer.writerow([
                    point['timestamp'],
                    point['distance_mm'],
                    point['distance_m'],
                    point['strength'],
                    point['temperature'],
                    point['angle']
                ])
        
        print(f"Saved {len(self.data_points)} points to {filename}")
        return filename
    
    def clear_data(self):
        """Clear all stored data"""
        self.data_points = []
        print("Data cleared")

# Create storage instance
storage = DataStorage()

print("TF-Luna Data Collection System")
print("Commands:")
print("  y - Start/Stop recording")
print("  s - Save data to files")
print("  c - Clear data")
print("  q - Quit")
print("-" * 40)

try:
    while True:
        # Check for user input (non-blocking)
        import select
        import sys
        
        if select.select([sys.stdin], [], [], 0)[0]:
            command = input().strip().lower()
            
            if command == 'y':
                storage.recording = not storage.recording
                if storage.recording:
                    print("🔴 RECORDING STARTED")
                else:
                    print("⏹️  RECORDING STOPPED")
                    
            elif command == 's':
                if storage.data_points:
                    json_file = storage.save_to_json()
                    csv_file = storage.save_to_csv()
                    print(f"Data saved to {json_file} and {csv_file}")
                else:
                    print("No data to save")
                    
            elif command == 'c':
                storage.clear_data()
                
            elif command == 'q':
                break
        
        # Read sensor data
        result = read_tfluna()
        if result:
            distance, strength, temp = result
            
            # Display current reading
            status = "🔴 REC" if storage.recording else "⏸️  "
            print(f"{status} | Distance: {distance:4d}mm | Strength: {strength:4d} | Temp: {temp:5.1f}°C | Points: {len(storage.data_points)}")
            
            # Store data if recording
            if storage.recording:
                storage.add_point(distance, strength, temp)
        
        time.sleep(0.2)  # 5 readings per second
        
except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    # Auto-save data if any exists
    if storage.data_points:
        print(f"Auto-saving {len(storage.data_points)} data points...")
        storage.save_to_json()
        storage.save_to_csv()
    
    ser.close()
    print("Serial port closed")
