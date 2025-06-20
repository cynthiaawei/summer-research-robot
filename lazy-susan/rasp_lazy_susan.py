import RPi.GPIO as GPIO
from time import sleep

LS_PIN = 24  # BCM pin number

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LS_PIN, GPIO.OUT)

    pwm = GPIO.PWM(LS_PIN, 1000)  # 1 kHz PWM
    pwm.start(39.2)               # ≈ analogWrite(pin,100)
    sleep(0.5)                    # run 0.5 s
    pwm.stop()                    # stop PWM

    # delete it now so its __del__ runs here, while
    # GPIO is still active and the stop() call succeeds
    del pwm

    GPIO.cleanup()

if __name__ == "__main__":
    main()
