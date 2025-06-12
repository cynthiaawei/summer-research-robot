import serial
import time

# open the Pi’s UART port at TF-Luna’s default baudrate
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.5)

def read_tfluna():
    # 1) find the two-byte header 0x59 0x59
    while True:
        b1 = ser.read(1)
        if not b1 or b1 != b'\x59': 
            continue
        b2 = ser.read(1)
        if b2 == b'\x59':
            break

    # 2) read the next 7 bytes (distL, distH, strL, strH, tempL, tempH, checksum)
    packet = ser.read(7)
    if len(packet) < 7:
        return None

    data = b'\x59\x59' + packet
    # 3) checksum: low 8 bits of sum of bytes 0–7
    if (sum(data[0:8]) & 0xFF) != data[8]:
        return None

    dist  = packet[0] | (packet[1] << 8)       # mm
    strength = packet[2] | (packet[3] << 8)
    raw_temp = packet[4] | (packet[5] << 8)
    temp_c = raw_temp/8.0 - 256                # °C, per datasheet

    return dist, strength, temp_c

try:
    while True:
        result = read_tfluna()
        if result:
            dist, strength, temp = result
            print(f"Distance: {dist} mm  |  Strength: {strength}  |  Temp: {temp:.1f} °C")
        else:
            print("Bad frame, retrying…")
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    ser.close()
