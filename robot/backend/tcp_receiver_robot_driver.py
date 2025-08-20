#!/usr/bin/env python3
# tcp_receiver.py - TCP server that uses your default robot speeds

# recieve data through TCP using socket library: allows comms over BSD socket interface
# so comms can go through "all modern Unix systems"; windows, macos, etc.
# basically, we are making a network server with our laptop subscribbling and pi as server

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

# @param: conn (socket.socket obj) = object representing the client connection, bidirectional com channel
#                                    can be used to send, recieve data and can be closed etc
#         addr (address) = client address that was connected and doing things

def handle_client(conn, addr):
    print(f"📱 Client connected from {addr}")
    buffer = b"" # GET THAT DATA!!
    
    try:
        while True:
            data = conn.recv(1024) # recieve data from conn, max 1024 bytes read at once
            if not data:
                break
                
            buffer += data # add data to the buffer
            
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1) # split in the buffer based on enter and only once to get one upcoming line
                message = line.decode('utf-8').strip() # mfing utf-8 encoding, also remove trailing things like spaces and enters
                
                if message:
                    try:
                        cmd = json.loads(message) # deserialize the data which is in json format, into python dict
                        process_command(cmd)      # run the movement commands on the robot
                    except json.JSONDecodeError as e: # EXCEPTIONNN:: THE MESSAGE CANNOT BE DECODED!!
                        print(f"❌ Invalid JSON: {e}")

    except Exception as e: # CLIENT CNANOT CONNECT  
        print(f"❌ Client error: {e}") 
    finally:
        conn.close() # close the connection
        print(f"📱 Client {addr} disconnected")

# IT IS TIME TO PROCESS THE MOVEMENT COMMAND BASED ON THE TEXT MSG RECIEVED which is in a python dict
def process_command(cmd):
    cmd_type = cmd.get("type", "") # key: type, then grab the associated value. 2nd param is value to return if key dne
    
    if cmd_type == "emergency_stop": # handle emergency stop
        print("🛑 Emergency stop!")
        try:
            ESTOP()
        except:
            STOP()
        return
    
    if cmd_type == "plan_steps": # plan sent over by tcp 
        steps = cmd.get("steps", []) # steps (array) default value is empty
        print(f"📋 Received plan with {len(steps)} steps") # print to the holy terminal
        
        # going one by one through the list steps, make the robot move
        for i, step in enumerate(steps):
            direction = step.get("direction", "").lower()   # go through the dict again :/
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