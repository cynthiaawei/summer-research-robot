import RPi.GPIO as GPIO
import time

# adjust this to the GPIO pin you're using
MOTOR_PIN = 18    # BCM numbering

# --- setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

# create a 1 kHz PWM instance on MOTOR_PIN
pwm = GPIO.PWM(MOTOR_PIN, 1000)

# start at ≈39% duty (100/255×100)
pwm.start((100/255) * 100)

# run motor for 0.5 s
time.sleep(0.5)

# stop motor
pwm.ChangeDutyCycle(0)
pwm.stop()

# cleanup GPIO state
GPIO.cleanup()
