import RPi.GPIO as GPIO
import time

# BOARD pin assignments
TRIG1 = 11    # trigger pin sensor 1
ECHO1 = 31    # echo pin sensor 1
TRIG2 = 13    # trigger pin sensor 2
ECHO2 = 29    # echo pin sensor 2
TRIG3 = 15    # trigger pin sensor 3
ECHO3 = 22    # echo pin sensor 3


THRESH_CM = 30

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(TRIG3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)

GPIO.output(TRIG1, False)
GPIO.output(TRIG2, False)
GPIO.output(TRIG3, False)
time.sleep(0.5)  # let sensors settle

def get_distance_cm(trig_pin, echo_pin):
    # send 10 µs pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    # wait for echo start
    start = time.time()
    while GPIO.input(echo_pin) == 0:
        start = time.time()
    
    # wait for echo end
    end = start
    while GPIO.input(echo_pin) == 1:
        end = time.time()
    
    elapsed = end - start
    return elapsed * 17150  # convert to cm

try:
    while True:
        dist1 = get_distance_cm(TRIG1, ECHO1)
        dist2 = get_distance_cm(TRIG2, ECHO2)
        dist3 = get_distance_cm(TRIG3, ECHO3)
        
        # Check if any sensor detects object within threshold
        if dist1 < THRESH_CM or dist2 < THRESH_CM or dist3 < THRESH_CM:
            state = 'h'
        else:
            state = 'l'
        
        print(f"Sensor 1: {dist1:.1f} cm")
        print(f"Sensor 2: {dist2:.1f} cm")
        print(f"Sensor 3: {dist3:.1f} cm")
        print(f"LED: {state}")
        print("---")
        
        time.sleep(0.5)

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
