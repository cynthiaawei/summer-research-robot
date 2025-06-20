import RPi.GPIO as GPIO
import time

MOTOR_PIN = 18    # BCM numbering

GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

pwm = GPIO.PWM(MOTOR_PIN, 1000)               # 1 kHz PWM
pwm.start((100/255) * 100)                    # ~39% duty
time.sleep(0.5)                               # run for 0.5 s
pwm.ChangeDutyCycle(0)                        # stop PWM
pwm.stop()                                    # stop cleanly
del pwm                                       # force __del__ now

GPIO.cleanup()
