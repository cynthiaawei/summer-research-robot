# arduino_bridge.py - Run this on Windows
import serial
import socket
import threading
import time

def arduino_bridge():
    # Find Arduino COM port
    import serial.tools.list_ports
    arduino_port = None
    
    print("🔍 Looking for Arduino...")
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description or 'USB Serial' in port.description:
            arduino_port = port.device
            break
    
    if not arduino_port:
        print("❌ Arduino not found!")
        print("Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device} - {port.description}")
        return
    
    print(f"✅ Found Arduino on {arduino_port}")
    
    # Connect to Arduino
    try:
        arduino = serial.Serial(arduino_port, 115200, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to Arduino on {arduino_port}")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Start TCP server for WSL2
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', 9999))
    server.listen(1)
    print("🌐 Waiting for WSL2 connection on port 9999...")
    
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
                            print(f"Arduino: {data.decode().strip()}")
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