#!/usr/bin/env python3
# simple_bt_receiver.py - No service advertising, just raw RFCOMM
import json
import threading
import time

# Import movement functions
try:
    from robot_movement import move as RM_move, stop as RM_stop, emergency_stop as RM_estop
    MOVE, STOP, ESTOP = RM_move, RM_stop, RM_estop
    print("✅ Imported robot_movement successfully")
except Exception:
    try:
        from enhanced_robot_movement import move as RM_move, stop as RM_stop, emergency_stop as RM_estop
        MOVE, STOP, ESTOP = RM_move, RM_stop, RM_estop
        print("✅ Imported enhanced_robot_movement successfully")
    except Exception as e:
        print(f"❌ Could not import movement module: {e}")
        exit(1)

from bluetooth import BluetoothSocket, RFCOMM

def handle_client(client_sock, addr):
    print(f"📱 Client connected from {addr}")
    buffer = b""
    
    try:
        while True:
            data = client_sock.recv(1024)
            if not data:
                break
                
            buffer += data
            
            # Process complete JSON lines
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                message = line.decode('utf-8').strip()
                
                if message:
                    try:
                        cmd = json.loads(message)
                        process_command(cmd)
                    except json.JSONDecodeError as e:
                        print(f"❌ Invalid JSON: {e}")
                        
    except Exception as e:
        print(f"❌ Client error: {e}")
    finally:
        client_sock.close()
        print(f"📱 Client {addr} disconnected")

def process_command(cmd):
    cmd_type = cmd.get("type", "")
    
    if cmd_type == "emergency_stop":
        print("🛑 Emergency stop!")
        try:
            ESTOP()
        except:
            STOP()
        return
    
    if cmd_type == "plan_steps":
        steps = cmd.get("steps", [])
        print(f"📋 Received plan with {len(steps)} steps")
        
        for i, step in enumerate(steps):
            direction = step.get("direction", "").lower()
            duration_ms = int(step.get("duration_ms", 0))
            speed_pct = int(step.get("speed_percent", 18))
            
            print(f"  Step {i+1}: {direction} for {duration_ms}ms at {speed_pct}%")
            
            if direction == "stop":
                STOP()
            elif direction in ["forward", "backward", "turnleft", "turnright"]:
                try:
                    result = MOVE(direction, speed=speed_pct, duration_ms=duration_ms)
                    if not result:
                        print(f"❌ Movement failed for step {i+1}")
                        STOP()
                        break
                except Exception as e:
                    print(f"❌ Movement error: {e}")
                    STOP()
                    break
            else:
                print(f"❌ Unknown direction: {direction}")
        
        print("✅ Plan execution complete")

def main():
    # Create RFCOMM socket on a fixed channel
    server_sock = BluetoothSocket(RFCOMM)
    
    try:
        # Bind to any available port
        server_sock.bind(("", 1))  # Try channel 1 first
        server_sock.listen(1)
        
        port = server_sock.getsockname()[1]
        print(f"🔵 Bluetooth server listening on RFCOMM channel {port}")
        print(f"📡 Pi MAC address: Use 'bluetoothctl show' to find it")
        print(f"🔗 On laptop, connect to this Pi and use RFCOMM channel {port}")
        print("⏳ Waiting for connections...")
        
        while True:
            client_sock, client_addr = server_sock.accept()
            
            # Handle each client in a separate thread
            client_thread = threading.Thread(
                target=handle_client, 
                args=(client_sock, client_addr),
                daemon=True
            )
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down server...")
    except Exception as e:
        print(f"❌ Server error: {e}")
    finally:
        try:
            server_sock.close()
        except:
            pass

if __name__ == "__main__":
    main()