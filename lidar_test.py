# import serial
# import time


# def setup_serial():
#     ser= serial.Serial(
#         port='/dev/serial0',
#         baudrate=115200,
#         bytesize=serial.EIGHTBITS,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         timeout=1
#     )

#     print(f" Serial Connection Opened: { ser.port}")
#     return ser

# def read_tfluna_packet(ser):
#     if ser.in_waiting >= 9: #in_waiting tells hwo many bytes are currently in the serial receive buffer
#         data= ser.read(9)

#         if len(data) == 9:
#             packet= list(data)

#             if packet[0]== 0x59 and packet[1] == 0x59:
#                 distance = packet[2] + (packet[3]<<8)
#                 print(distance)
#                 return distance
#     return None

# def main():
#     ser = setup_serial()
#     if not ser: 
#         return
    
#     try:
#         while True:
#             read_tfluna_packet(ser)
#             time.sleep(0.05)
#     except KeyboardInterrupt:
#         pass

#     finally:
#         if ser:
#             ser.close()

# if __name__ == "__main__":
#     main()
import serial
import time

def setup_serial():
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    print(f"Serial Connection Opened: {ser.port}")
    return ser

def send_trigger_command(ser):
    """Send trigger command to TF Luna"""
    # TF Luna trigger command: 5A 04 04 5F
    trigger_cmd = bytes([0x5A, 0x04, 0x04, 0x5F])
    
    print("Sending trigger command...")
    ser.write(trigger_cmd)
    time.sleep(0.1)
    
    # Check for response
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(f"Response: {[hex(b) for b in data]}")
        return True
    else:
        print("No response to trigger command")
        return False

def set_continuous_mode(ser):
    """Set TF Luna to continuous measurement mode"""
    # Command to set continuous mode: 5A 06 03 01 00 63
    continuous_cmd = bytes([0x5A, 0x06, 0x03, 0x01, 0x00, 0x63])
    
    print("Setting continuous mode...")
    ser.write(continuous_cmd)
    time.sleep(0.5)  # Wait longer for mode change
    
    # Check for response
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(f"Mode change response: {[hex(b) for b in data]}")
        return True
    else:
        print("No response to mode change command")
        return False

def main():
    ser = setup_serial()
    if not ser:
        return
    
    try:
        print("Trying to wake up TF Luna...")
        
        # First, try setting continuous mode
        set_continuous_mode(ser)
        
        # Wait and see if we get continuous data
        print("Waiting for continuous data...")
        for i in range(50):  # Wait 5 seconds
            if ser.in_waiting >= 9:
                data = ser.read(9)
                if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                    distance = data[2] + (data[3] << 8)
                    print(f"Success! Distance: {distance} cm")
                    break
                else:
                    print(f"Got data but wrong format: {[hex(b) for b in data]}")
            time.sleep(0.1)
        else:
            print("No continuous data received")
            
            # Try trigger mode
            print("Trying trigger mode...")
            for i in range(10):
                send_trigger_command(ser)
                time.sleep(0.1)
                
                if ser.in_waiting >= 9:
                    data = ser.read(9)
                    if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                        distance = data[2] + (data[3] << 8)
                        print(f"Trigger success! Distance: {distance} cm")
                        break
                    else:
                        print(f"Trigger response: {[hex(b) for b in data]}")
            else:
                print("Trigger mode also failed")
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if ser:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
