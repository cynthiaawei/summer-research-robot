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

def get_distance(trig_pin, echo_pin, timeout=1.0):
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
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                print(f"Timeout waiting for echo start on pin {echo_pin}")
                return -1
        
        # Wait for echo end with timeout
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                print(f"Timeout waiting for echo end on pin {echo_pin}")
                return -1
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = round(distance, 2)
        
        # Validate reasonable distance (HC-SR04 range is typically 2-400cm)
        if distance < 2 or distance > 400:
            print(f"Invalid distance reading: {distance}cm on pin {echo_pin}")
            return -1
            
        return distance
        
    except Exception as e:
        print(f"Error reading sensor on pins {trig_pin}/{echo_pin}: {e}")
        return -1

def test_individual_sensor(trig_pin, echo_pin, sensor_name):
    """Test individual sensor to help debug"""
    print(f"Testing {sensor_name} (Trig: {trig_pin}, Echo: {echo_pin})")
    
    # Check initial pin states
    echo_state = GPIO.input(echo_pin)
    print(f"Initial echo pin state: {echo_state}")
    
    for i in range(3):
        distance = get_distance(trig_pin, echo_pin)
        if distance > 0:
            print(f"  Attempt {i+1}: {distance} cm")
        else:
            print(f"  Attempt {i+1}: Failed")
        time.sleep(0.1)
    print()

try:
    print("Starting ultrasonic sensor diagnostics...")
    print("Initial pin state check:")
    print(f"Echo1 ({Echo1}): {GPIO.input(Echo1)}")
    print(f"Echo2 ({Echo2}): {GPIO.input(Echo2)}")
    print(f"Echo3 ({Echo3}): {GPIO.input(Echo3)}")
    print()
    
    # Test each sensor individually first
    print("Testing individual sensors:")
    test_individual_sensor(Trig1, Echo1, "Sensor 1")
    test_individual_sensor(Trig2, Echo2, "Sensor 2")
    test_individual_sensor(Trig3, Echo3, "Sensor 3")
    
    print("Starting continuous monitoring...")
    print("Place objects in front of sensors...")
    
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
        
        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("\nTest stopped by user")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up")
