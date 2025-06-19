import RPi.GPIO as GPIO
import time

# === Ultrasonic Sensor Pins ===
Echo1 = 31
Echo2 = 29
Echo3 = 22
Trig1 = 11
Trig2 = 13
Trig3 = 15

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Echo1, GPIO.IN)
GPIO.setup(Echo2, GPIO.IN)
GPIO.setup(Echo3, GPIO.IN)
GPIO.setup(Trig1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Trig2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Trig3, GPIO.OUT, initial=GPIO.LOW)

def get_distance(trig_pin, echo_pin, timeout=0.5):
    """
    Get distance from ultrasonic sensor with timeout protection
    Returns -1 if measurement fails
    """
    try:
        # Ensure trigger is low initially
        GPIO.output(trig_pin, False)
        time.sleep(0.000002)  # 2us settle time
        
        # Send trigger pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)  # 10us pulse
        GPIO.output(trig_pin, False)
        
        # Wait for echo start with timeout
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 0:
            if time.time() - timeout_start > timeout:
                return -1
        
        # Record when echo goes HIGH
        pulse_start = time.time()
        
        # Wait for echo end with timeout
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            if time.time() - timeout_start > timeout:
                return -1
        
        # Record when echo goes LOW
        pulse_end = time.time()
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = round(distance, 2)
        
        return distance
        
    except Exception as e:
        return -1

try:
    print("Starting ultrasonic sensor monitoring...")
    print("Press Ctrl+C to stop...")
    
    while True:
        # Measure distance for each sensor
        dist1 = get_distance(Trig1, Echo1)
        dist2 = get_distance(Trig2, Echo2)
        dist3 = get_distance(Trig3, Echo3)
        
        # Print distances
        print(f"Sensor 1: {dist1 if dist1 > 0 else 'FAIL'} cm")
        print(f"Sensor 2: {dist2 if dist2 > 0 else 'FAIL'} cm")
        print(f"Sensor 3: {dist3 if dist3 > 0 else 'FAIL'} cm")
        print("---")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    GPIO.cleanup()
