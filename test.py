import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# === MotorShield 1 ===
Motor1_PWM = 38  # GPIO20
Motor1_DIR = 40  # GPIO21

Motor2_PWM = 32  # GPIO12 (PWM0)
Motor2_DIR = 36  # GPIO16

# === MotorShield 2 ===
Motor3_PWM = 26  # GPIO7
Motor3_DIR = 28  # GPIO1 (⚠️ ID_SD - only use if I2C is disabled)

# === Setup pins
GPIO.setup(Motor1_DIR, GPIO.OUT)
GPIO.setup(Motor1_PWM, GPIO.OUT)

GPIO.setup(Motor2_DIR, GPIO.OUT)
GPIO.setup(Motor2_PWM, GPIO.OUT)

GPIO.setup(Motor3_DIR, GPIO.OUT)
GPIO.setup(Motor3_PWM, GPIO.OUT)

# === Setup PWM objects
freq = 1000
pwm1 = GPIO.PWM(Motor1_PWM, freq)
pwm2 = GPIO.PWM(Motor2_PWM, freq)
pwm3 = GPIO.PWM(Motor3_PWM, freq)

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

try:
    print("Testing Motor 1 (Shield 1 - Pin 38/40)")
    GPIO.output(Motor1_DIR, GPIO.HIGH)
    pwm1.ChangeDutyCycle(50)
    time.sleep(2)
    pwm1.ChangeDutyCycle(0)
    time.sleep(1)

    print("Testing Motor 2 (Shield 1 - Pin 32/36)")
    GPIO.output(Motor2_DIR, GPIO.HIGH)
    pwm2.ChangeDutyCycle(50)
    time.sleep(2)
    pwm2.ChangeDutyCycle(0)
    time.sleep(1)

    print("Testing Motor 3 (Shield 2 - Pin 26/28)")
    GPIO.output(Motor3_DIR, GPIO.HIGH)
    pwm3.ChangeDutyCycle(50)
    time.sleep(2)
    pwm3.
