
import RPi.GPIO as GPIO
import serial
import time
import threading

# === Motor Pin Definitions ===
Motor1_Speed = 38
Motor1_Dir = 40
Motor2_Speed = 32
Motor2_Dir = 36
Motor3_Speed = 16
Motor3_Dir = 26

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Motor1_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)

# Set PWM frequencies
freq = 1000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# === Global State ===
gCurSpeed1 = 0
gCurSpeed2 = 0
gCurSpeed3 = 0
gSliderSpeed = 25  # From main code
motor3_compensate = 15  # From main code
movement_lock = threading.Lock()

def find_serial_port():
    """Try different serial port names."""
    ports = ['/dev/serial10', '/dev/ttyS0', '/dev/ttyAMA0']
    for port in ports:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.5)
            return ser
        except serial.SerialException:
            pass
    return None

def read_tfluna(ser):
    """Read TF-Luna data with error handling."""
    try:
        ser.flushInput()
        header_count = 0
        max_attempts = 100
        while header_count < max_attempts:
            b1 = ser.read(1)
            if not b1 or b1 != b'\x59':
                header_count += 1
                continue
            b2 = ser.read(1)
            if not b2 or b2 != b'\x59':
                header_count += 1
                continue
            break
        if header_count >= max_attempts:
            return None
        
        packet = ser.read(7)
        if len(packet) < 7:
            return None
        
        data = b'\x59\x59' + packet
        calculated_checksum = sum(data[0:8]) & 0xFF
        received_checksum = packet[6]
        if calculated_checksum != received_checksum:
            return None
        
        dist = packet[0] | (packet[1] << 8)
        strength = packet[2] | (packet[3] << 8)
        raw_temp = packet[4] | (packet[5] << 8)
        temp_c = raw_temp / 8.0 - 256
        return dist, strength, temp_c
    except Exception:
        return None

def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    """Smoothly change motor speeds."""
    with movement_lock:
        steps = max(abs(newSpeed1 - curSpeed1), abs(newSpeed2 - curSpeed2), abs(newSpeed3 - curSpeed3))
        if steps > 0:
            for step in range(steps + 1):
                progress = step / steps if steps > 0 else 1
                speed1 = int(curSpeed1 + (newSpeed1 - curSpeed1) * progress)
                speed2 = int(curSpeed2 + (newSpeed2 - curSpeed2) * progress)
                speed3 = int(curSpeed3 + (newSpeed3 - curSpeed3) * progress)
                
                speed1 = max(0, min(100, speed1))
                speed2 = max(0, min(100, speed2))
                speed3 = max(0, min(100, speed3))
                
                Motor1_pwm.ChangeDutyCycle(speed1)
                Motor2_pwm.ChangeDutyCycle(speed2)
                Motor3_pwm.ChangeDutyCycle(speed3)
                
                time.sleep(0.01)
        
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1 = newSpeed1
        gCurSpeed2 = newSpeed2
        gCurSpeed3 = newSpeed3

def stopNoTime():
    """Immediate stop without timing."""
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(0)
        Motor3_pwm.ChangeDutyCycle(0)
        
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = 0
        gCurSpeed3 = 0

def immediateStop():
    """Immediate stop with state reset."""
    stopNoTime()

def move_robot(direction, speed=None):
    """Unified movement function for rotation."""
    if speed is None:
        speed = gSliderSpeed
    
    direction_configs = {
        "turnRight": {
            "dirs": [GPIO.LOW, GPIO.HIGH, GPIO.HIGH],
            "speeds": [speed, speed, speed + motor3_compensate]
        }
    }
    
    if direction not in direction_configs:
        return False
    
    config = direction_configs[direction]
    
    GPIO.output(Motor1_Dir, config["dirs"][0])
    GPIO.output(Motor2_Dir, config["dirs"][1])
    GPIO.output(Motor3_Dir, config["dirs"][2])
    
    changeSpeedSmooth(gCurSpeed1, config["speeds"][0], 
                     gCurSpeed2, config["speeds"][1], 
                     gCurSpeed3, config["speeds"][2])
    
    return True

def turnRight(speed=None):
    """Start right turn."""
    return move_robot("turnRight", speed)

def measure_rotation_with_lidar(tolerance=5, max_duration=30):
    """Measure rotation time by detecting the same LiDAR distance twice."""
    ser = find_serial_port()
    if not ser:
        print("ERROR: Could not open any serial port!")
        return None, None
    
    print("Starting rotation. Hold your hand at a fixed distance (e.g., 50 cm) from the LiDAR.")
    print("Robot will stop when the same distance is detected again.")
    
    first_distance = None
    start_time = None
    
    # Start rotation
    turnRight()  # Uses gSliderSpeed=25
    
    try:
        while time.time() - (start_time or time.time()) < max_duration:
            result = read_tfluna(ser)
            if result is None:
                time.sleep(0.1)
                continue
            
            dist, _, _ = result
            
            # Validate distance
            if dist <= 0 or dist > 8000:
                time.sleep(0.1)
                continue
            
            # First detection
            if first_distance is None and dist >= 10:
                first_distance = dist
                start_time = time.time()
            
            # Second detection
            elif first_distance is not None and abs(dist - first_distance) <= tolerance:
                immediateStop()
                rotation_time = time.time() - start_time
                angular_speed = 360 / rotation_time if rotation_time > 0 else 0
                print(f"Final Rotation Time: {rotation_time:.2f} seconds")
                print(f"Final Angular Speed: {angular_speed:.2f} degrees/second")
                return rotation_time, angular_speed
            
            time.sleep(0.1)  # Match TF-Luna's ~10Hz rate
        
        print(f"Max duration ({max_duration}s) reached. No matching distance found.")
        return None, None
    
    except KeyboardInterrupt:
        return None, None
    finally:
        immediateStop()
        ser.close()

if __name__ == "__main__":
    try:
        rotation_time, angular_speed = measure_rotation_with_lidar(tolerance=5)
        if rotation_time:
            print(f"Final Results: Rotation Time = {rotation_time:.2f} seconds, Angular Speed = {angular_speed:.2f} degrees/second")
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        immediateStop()
        Motor1_pwm.stop()
        Motor2_pwm.stop()
        Motor3_pwm.stop()
        GPIO.cleanup()
        print("Cleanup complete.")
