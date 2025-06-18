import RPi.GPIO as GPIO
import time

# === GPIO Board Mode ===
GPIO.setmode(GPIO.BOARD)

# === Motor PWM Pins ===
Motor1_Speed = 38  # Motor 1 - PWM
Motor2_Speed = 32  # Motor 2 - PWM
Motor3_Speed = 16  # Motor 3 - PWM

# === Setup Pins ===
GPIO.setup(Motor1_Speed, GPIO.OUT)
GPIO.setup(Motor2_Speed, GPIO.OUT)
GPIO.setup(Motor3_Speed, GPIO.OUT)

# === Set PWM frequency (1kHz is standard for DC motors) ===
freq = 1000
motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
motor3_pwm = GPIO.PWM(Motor3_Speed, freq)

# === Start with 0% duty cycle ===
motor1_pwm.start(0)
motor2_pwm.start(0)
motor3_pwm.start(0)

try:
    print("Testing Motor 1 (PWM 38)...")
    motor1_pwm.ChangeDutyCycle(50)
    time.sleep(2)
    motor1_pwm.ChangeDutyCycle(0)
    time.sleep(1)

    print("Testing Motor 2 (PWM 32)...")
    motor2_pwm.ChangeDutyCycle(50)
    time.sleep(2)
    motor2_pwm.ChangeDutyCycle(0)
    time.sleep(1)

    print("Testing Motor 3 (PWM 16)...")
    motor3_pwm.ChangeDutyCycle(50)
    time.sleep(2)
    motor3_pwm.ChangeDutyCycle(0)
    time.sleep(1)

except KeyboardInterrupt:
    print("Test interrupted.")

finally:
    print("Cleaning up GPIO...")
    motor1_pwm.stop()
    motor2_pwm.stop()
    motor3_pwm.stop()
    GPIO.cleanup()
