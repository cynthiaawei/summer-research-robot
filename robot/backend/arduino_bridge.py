# arduino_bridge.py - Run this on Windows
import serial
import serial.tools.list_ports
import socket
import threading
import time

def arduino_bridge():
    # Arduino COM port - CHANGE THIS to your port!
    arduino_port = 'COM12'  # Your Arduino port
    
    print(f"🔍 Looking for Arduino on {arduino_port}...")
    
    # Connect to Arduino
    try:
        arduino = serial.Serial(arduino_port, 115200, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to Arduino on {arduino_port}")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        print("Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device} - {port.description}")
        return
    
    # Start TCP server for WSL2 - USING PORT 8888
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', 8888))  # Using port 8888 instead of 9999
        server.listen(1)
        print("🌐 TCP Server started successfully")
        print("🌐 Waiting for WSL2 connection on port 8888...")
    except Exception as e:
        print(f"❌ Failed to start server: {e}")
        arduino.close()
        return
    
    try:
        while True:
            client, addr = server.accept()
            print(f"✅ WSL2 connected from {addr}")
            
            try:
                while True:
                    # Forward Arduino data to WSL2
                    if arduino.in_waiting > 0:
                        data = arduino.readline()
                        if data:
                            decoded = data.decode().strip()
                            print(f"Arduino: {decoded}")
                            client.send(data)
                    
                    # Forward WSL2 commands to Arduino
                    client.settimeout(0.01)
                    try:
                        command = client.recv(1024)
                        if command:
                            arduino.write(command)
                            print(f"WSL2 command: {command.decode().strip()}")
                    except socket.timeout:
                        pass
                    except:
                        break
                        
            except Exception as e:
                print(f"❌ Client error: {e}")
            finally:
                client.close()
                print("📱 WSL2 disconnected")
                
    except KeyboardInterrupt:
        print("\n🛑 Stopping bridge...")
    finally:
        arduino.close()
        server.close()

if __name__ == "__main__":
    arduino_bridge()