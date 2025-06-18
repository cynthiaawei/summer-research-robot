import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# === Pin Definitions (confirmed by you) ===
Motor1_Speed = 38  # GPIO20
Motor1_Dir   = 40  # GPIO21

Motor2_Speed = 32  # GPIO12 (PWM0)
Motor2_Dir   = 36  # GPIO16

Motor3_Speed = 16  # GPIO23
Motor3_Dir   = 26  # GPIO7

# === GPIO Setup ===
GPIO.setup(Motor1_Speed, GPIO.OUT)
GPIO.setup(Motor1_Dir, GPIO.OUT)

GPIO.setup(Motor2_Speed, GPIO.OUT)
GPIO.setup(Motor2_Dir, GPIO.OUT)

GPIO.setup(Motor3_Speed, GPIO.OUT)
GPIO.setup(Motor3_Dir, GPIO.OUT)

# === PWM Setup (1kHz) ===
freq = 1000
pwm1 = GPIO.PWM(Motor1_Speed, freq)
pwm2 = GPIO.PWM(Motor2_Speed, freq)
pwm3 = GPIO.PWM(Motor3_Speed, freq)

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

try:
    # === Motor 1 Test ===
    print("▶️ Testing Motor 1 (Pin 38/40)")
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    pwm1.ChangeDutyCycle(50)
    time.sleep(2)
    pwm1.ChangeDutyCycle(0)
    time.sleep(1)

    # === Motor 2 Test ===
    print("▶️ Testing Motor 2 (Pin 32/36)")
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    pwm2.ChangeDutyCycle(50)
    time.sleep(2)
    pwm2.ChangeDutyCycle(0)
    time.sleep(1)

    # === Motor 3 Test ===
    print("▶️ Testing Motor 3 (Pin 16/26)")
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    pwm3.ChangeDutyCycle(50)
    time.sleep(2)
    pwm3.ChangeDutyCycle(0)
    time.sleep(1)

except KeyboardInterrupt:
    print("❌ Interrupted by user.")

finally:
    print("🧼 Cleaning up GPIO...")
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    GPIO.cleanup()
