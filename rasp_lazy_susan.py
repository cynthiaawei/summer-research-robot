from gpiozero import PWMOutputDevice
from time import sleep

# Use a pin that supports PWMOutputDevice (software PWM) — 
# e.g. BCM 18. Change to your wiring.
MOTOR_PIN = 18  

def run_motor():
    motor = PWMOutputDevice(MOTOR_PIN, frequency=1000)  # 1 kHz PWM
    motor.value = 100/255    # Map Arduino’s 0–255 range to 0.0–1.0
    sleep(0.5)               # 500 ms on
    motor.value = 0          # stop
    motor.close()

if __name__ == "__main__":
    run_motor()
