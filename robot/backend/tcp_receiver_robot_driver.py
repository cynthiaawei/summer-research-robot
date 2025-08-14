#!/usr/bin/env python3
# tcp_receiver.py - TCP server that uses your default robot speeds
import socket
import threading
import json
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

def handle_client(conn, addr):
    print(f"📱 Client connected from {addr}")
    buffer = b""
    
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
                
            buffer += data
            
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
        conn.close()
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
            
            print(f"  Step {i+1}: {direction} for {duration_ms}ms (using default speed)")
            
            if direction == "stop":
                STOP()
            elif direction in ["forward", "backward", "turnleft", "turnright"]:
                try:
                    # Use your default speed - no speed parameter passed
                    result = MOVE(direction, duration_ms=duration_ms)
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
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind(('0.0.0.0', 9999))
        server.listen(1)
        
        # Get Pi's IP address
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        
        print(f"🌐 TCP server listening on port 9999")
        print(f"📡 Pi IP: {local_ip}")
        print(f"🔗 On laptop, use: --host {local_ip}")
        print("🏎️  Using robot's default speeds")
        print("⏳ Waiting for connections...")
        
        while True:
            conn, addr = server.accept()
            client_thread = threading.Thread(
                target=handle_client, 
                args=(conn, addr),
                daemon=True
            )
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down server...")
    except Exception as e:
        print(f"❌ Server error: {e}")
    finally:
        server.close()

if __name__ == "__main__":
    main()