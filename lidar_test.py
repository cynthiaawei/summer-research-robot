# import serial
# import time

# # open the Pi’s UART port at TF-Luna’s default baudrate
# ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.5)

# def read_tfluna():
#     # 1) find the two-byte header 0x59 0x59
#     while True:
#         b1 = ser.read(1)
#         if not b1 or b1 != b'\x59': 
#             continue
#         b2 = ser.read(1)
#         if b2 == b'\x59':
#             break

#     # 2) read the next 7 bytes (distL, distH, strL, strH, tempL, tempH, checksum)
#     packet = ser.read(7)
#     if len(packet) < 7:
#         return None

#     data = b'\x59\x59' + packet
#     # 3) checksum: low 8 bits of sum of bytes 0–7
#     if (sum(data[0:8]) & 0xFF) != data[8]:
#         return None

#     dist  = packet[0] | (packet[1] << 8)       # mm
#     strength = packet[2] | (packet[3] << 8)
#     raw_temp = packet[4] | (packet[5] << 8)
#     temp_c = raw_temp/8.0 - 256                # °C, per datasheet

#     return dist, strength, temp_c

# try:
#     while True:
#         result = read_tfluna()
#         if result:
#             dist, strength, temp = result
#             print(f"Distance: {dist} mm  |  Strength: {strength}  |  Temp: {temp:.1f} °C")
#         else:
#             print("Bad frame, retrying…")
#         time.sleep(0.1)

# except KeyboardInterrupt:
#     pass
# 

import serial
import time


def setup_serial():
    ser= serial.Serial(
        port='/dev/serial0',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    print(f" Serial Connection Opened: { ser.port}")
    return ser

def read_tfluna_packet(ser):
    if ser.in_waiting >= 9: #in_waiting tells hwo many bytes are currently in the serial receive buffer
        data= ser.read(9)

        if len(data) == 9:
            packet= list(data)

            if packet[0]== 0x59 and packet[1] == 0x59:
                distance = packet[2] + (packet[3]<<8)
                print(distance)
                return distance
    return None

def main():
    ser = setup_serial()
    if not ser: 
        return
    
    try:
        while True:
            read_tfluna_packet(ser)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    finally:
        if ser:
            ser.close()

if __name__ == "__main__":
    main()
