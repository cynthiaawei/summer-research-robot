import RPi.GPIO as GPIO
from time import sleep

MOTOR_PIN = 24  # BCM pin number

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

# 1 kHz PWM
pwm = GPIO.PWM(MOTOR_PIN, 1000)
# duty-cycle = (100/255)*100 ≈ 39%
pwm.start(39.2)

sleep(0.5)   # motor on for 0.5 s
pwm.stop()

GPIO.cleanup()
