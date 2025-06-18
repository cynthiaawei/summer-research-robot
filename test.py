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

def get_distance(trig_pin, echo_pin):
    # Send trigger pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  # 10us pulse
    GPIO.output(trig_pin, False)

    # Wait for echo start
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
    # Wait for echo end
    pulse_end = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    # Calculate distance (speed of sound = 34300 cm/s, divided by 2 for round trip)
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # 34300 / 2
    distance = round(distance, 2)
    return distance

try:
    print("Starting ultrasonic sensor test. Place objects in front of sensors...")
    while True:
        # Measure distance for each sensor
        dist1 = get_distance(Trig1, Echo1)
        dist2 = get_distance(Trig2, Echo2)
        dist3 = get_distance(Trig3, Echo3)

        # Print distances
        print(f"Sensor 1 (Echo1/Trig1) Distance: {dist1} cm")
        print(f"Sensor 2 (Echo2/Trig2) Distance: {dist2} cm")
        print(f"Sensor 3 (Echo3/Trig3) Distance: {dist3} cm")
        print("---")
        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("\nTest stopped by user")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up")
