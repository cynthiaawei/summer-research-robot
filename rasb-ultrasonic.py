import RPi.GPIO as GPIO
import time

# BCM pin assignments
TRIG = 37    # trigger pin
ECHO = 31    # echo pin
LED = 6
# TRIG2 = 35    # trigger pin
# ECHO2 = 29    # echo pin
# TRIG3 = 33    # trigger pin
# ECHO3 = 27    # echo pin

THRESH_CM = 30

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(LED, GPIO.OUT)
GPIO.output(TRIG, False)
time.sleep(0.5)  # let sensor settle

def get_distance_cm():
    # send 10 µs pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # wait for echo start
    start = time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    # wait for echo end
    end = start
    while GPIO.input(ECHO) == 1:
        end = time.time()

    elapsed = end - start
    return elapsed * 17150  # convert to cm

try:
    while True:
        dist = get_distance_cm()
        if dist < THRESH_CM:
            GPIO.output(LED, True)
            state = 'h'
        else:
            GPIO.output(LED, False)
            state = 'l'
        print(f"Distance: {dist:.1f} cm → {state}")
        time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
