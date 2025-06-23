import RPi.GPIO as GPIO
import time

MOTOR_PIN = 18     # stays the same
IN1_PIN   = 35
IN2_PIN   = 37

GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN,  GPIO.OUT)
GPIO.setup(IN2_PIN,  GPIO.OUT)

pwm = GPIO.PWM(MOTOR_PIN, 1000)   # 1 kHz
pwm.start(0)                      # motor initially off

def set_speed(freq_hz):
    pwm.ChangeFrequency(freq_hz)
    pwm.ChangeDutyCycle(100)      # full speed

def right(sec):
    GPIO.output(IN1_PIN, False)
    GPIO.output(IN2_PIN, True)
    time.sleep(sec)

def left(sec):
    GPIO.output(IN1_PIN, True)
    GPIO.output(IN2_PIN, False)
    time.sleep(sec)

seconds = 3
time.sleep(seconds)

print("right")
set_speed(1000)
right(seconds)

time.sleep(seconds - 2)

print("left")
left(seconds)

time.sleep(seconds - 2)
time.sleep(seconds)

pwm.stop()
GPIO.cleanup()
