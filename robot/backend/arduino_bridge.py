# # arduino_bridge.py - Run this on Windows
# import serial
# import serial.tools.list_ports
# import socket
# import threading
# import time

# def arduino_bridge():
#     # Arduino COM port - CHANGE THIS to your port!
#     arduino_port = 'COM12'  # Your Arduino port
    
#     print(f"🔍 Looking for Arduino on {arduino_port}...")
    
#     # Connect to Arduino
#     try:
#         arduino = serial.Serial(arduino_port, 115200, timeout=1)
#         time.sleep(2)
#         print(f"✅ Connected to Arduino on {arduino_port}")
#     except Exception as e:
#         print(f"❌ Failed to connect: {e}")
#         print("Available ports:")
#         for port in serial.tools.list_ports.comports():
#             print(f"  {port.device} - {port.description}")
#         return
    
#     # Start TCP server for WSL2 - USING PORT 8888
#     try:
#         server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         server.bind(('0.0.0.0', 8888))  # Using port 8888 instead of 9999
#         server.listen(1)
#         print("🌐 TCP Server started successfully")
#         print("🌐 Waiting for WSL2 connection on port 8888...")
#     except Exception as e:
#         print(f"❌ Failed to start server: {e}")
#         arduino.close()
#         return
    
#     try:
#         while True:
#             client, addr = server.accept()
#             print(f"✅ WSL2 connected from {addr}")
            
#             try:
#                 while True:
#                     # Forward Arduino data to WSL2
#                     if arduino.in_waiting > 0:
#                         data = arduino.readline()
#                         if data:
#                             decoded = data.decode().strip()
#                             print(f"Arduino: {decoded}")
#                             client.send(data)
                    
#                     # Forward WSL2 commands to Arduino
#                     client.settimeout(0.01)
#                     try:
#                         command = client.recv(1024)
#                         if command:
#                             arduino.write(command)
#                             print(f"WSL2 command: {command.decode().strip()}")
#                     except socket.timeout:
#                         pass
#                     except:
#                         break
                        
#             except Exception as e:
#                 print(f"❌ Client error: {e}")
#             finally:
#                 client.close()
#                 print("📱 WSL2 disconnected")
                
#     except KeyboardInterrupt:
#         print("\n🛑 Stopping bridge...")
#     finally:
#         arduino.close()
#         server.close()

# if __name__ == "__main__":
#     arduino_bridge()
# arduino_bridge.py - Run this on Windows
import serial
import socket
import threading
import time

class ArduinoBridge:
    def __init__(self):
        self.arduino = None
        self.server_socket = None
        self.client_socket = None
        self.running = True
        
        # Arduino connection
        self.arduino_port = "COM3"  # Adjust as needed
        self.arduino_baud = 115200
        
        # Network settings
        self.tcp_port = 8888
        
    def connect_arduino(self):
        """Connect to Arduino"""
        try:
            print(f"🔌 Connecting to Arduino on {self.arduino_port}...")
            self.arduino = serial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print("✅ Arduino connected!")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def setup_server(self):
        """Setup TCP server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.tcp_port))
            self.server_socket.listen(1)
            print(f"🌐 Bridge server listening on port {self.tcp_port}")
            return True
        except Exception as e:
            print(f"❌ Server setup failed: {e}")
            return False
    
    def handle_client_commands(self):
        """Handle commands from ROS node"""
        while self.running and self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8', errors='ignore')
                if data:
                    commands = data.strip().split('\n')
                    for command in commands:
                        command = command.strip()
                        if command == 'PING':
                            print("🏓 Received PING, sending PONG")
                            self.client_socket.send(b'PONG\n')
                        elif command == 'RESET':
                            print("🔄 Received RESET command, resetting Arduino...")
                            self.reset_arduino()
                            
            except Exception as e:
                print(f"❌ Client command error: {e}")
                break
    
    def reset_arduino(self):
        """Send reset command to Arduino and confirm"""
        try:
            if self.arduino and self.arduino.is_open:
                # Send reset command
                self.arduino.write(b'RESET\n')
                self.arduino.flush()
                print("📤 Sent RESET to Arduino")
                
                # Wait for Arduino acknowledgment
                start_time = time.time()
                while time.time() - start_time < 3:  # 3 second timeout
                    if self.arduino.in_waiting > 0:
                        response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                        if 'RESET_OK' in response or 'Position reset' in response:
                            print("✅ Arduino confirmed reset")
                            if self.client_socket:
                                self.client_socket.send(b'RESET_OK\n')
                            return True
                
                # If no confirmation, still tell ROS we tried
                print("⚠️ Arduino didn't confirm reset, but command sent")
                if self.client_socket:
                    self.client_socket.send(b'RESET_OK\n')
                    
        except Exception as e:
            print(f"❌ Reset error: {e}")
    
    def bridge_data(self):
        """Forward Arduino data to ROS"""
        while self.running:
            try:
                if self.arduino and self.arduino.is_open and self.client_socket:
                    if self.arduino.in_waiting > 0:
                        line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # Forward to ROS node
                            self.client_socket.send(f"{line}\n".encode('utf-8'))
                            
                            # Log ODOM messages
                            if line.startswith('ODOM:'):
                                print(f"📊 {line}")
                                
                time.sleep(0.01)  # Small delay
                
            except Exception as e:
                print(f"❌ Bridge error: {e}")
                break
    
    def run(self):
        """Main bridge loop"""
        if not self.connect_arduino():
            return
            
        if not self.setup_server():
            return
            
        print("🤖 Arduino Bridge running - waiting for ROS connection...")
        
        try:
            while self.running:
                # Wait for client connection
                self.client_socket, addr = self.server_socket.accept()
                print(f"✅ ROS node connected from {addr}")
                
                # Start command handler thread
                cmd_thread = threading.Thread(target=self.handle_client_commands, daemon=True)
                cmd_thread.start()
                
                # Bridge data
                self.bridge_data()
                
                print("❌ Client disconnected")
                if self.client_socket:
                    self.client_socket.close()
                    
        except KeyboardInterrupt:
            print("\n🛑 Bridge stopping...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        self.running = False
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("🧹 Bridge cleaned up")

if __name__ == "__main__":
    bridge = ArduinoBridge()
    bridge.run()