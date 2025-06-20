import RPi.GPIO as GPIO
import time

MOTOR_PIN = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

pwm = GPIO.PWM(MOTOR_PIN, 1000)
pwm.start(100)   # 100% duty = pin held HIGH
time.sleep(1)
pwm.stop()
GPIO.cleanup()
