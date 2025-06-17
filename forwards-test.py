import RPi.GPIO as GPIO
import time

# === Motor Pin Definitions ===
Motor1_Speed = 38  # PWM 1
Motor1_Dir = 40   # Dir 1
Motor2_Speed = 32  # PWM 2
Motor2_Dir = 36    # Dir 2
Motor3_Speed = 16  # PWM 3
Motor3_Dir = 26    # Dir 3

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Motor1_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)

# Set PWM frequencies
freq = 5000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# === Global State ===
gCurSpeed1 = 0
gCurSpeed2 = 0
gCurSpeed3 = 0
gSliderSpeed = 25  # Max 85
curDir1 = GPIO.HIGH
curDir2 = GPIO.HIGH
curDir3 = GPIO.HIGH
motor3_compensate = 15
permStop = True
interruptRequested = False
spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
dir_list = [Motor1_Dir, Motor2_Dir, Motor3_Dir]
#removed BLE code
commandCharacter = "" #replaces commandCharacter

#-- HELPER: Ramp motor speeds smoothly --
def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
  global interruptRequested, gCurSpeed1, gCurSpeed2, gCurSpeed3
  i = curSpeed1
  j = curSpeed2
  k = curSpeed3

  while ((i != newSpeed1 or j != newSpeed2 or k != newSpeed3) and (not interruptRequested)):
    # Check for interruption during ramping

    if (i < newSpeed1):
        i += 1
    elif (i > newSpeed1): 
        i -= 1

    if (j < newSpeed2):
       j += 1
    elif (j > newSpeed2):
       j -= 1

    if (k < newSpeed3):
       k += 1
    elif (k > newSpeed3):
       k -= 1

    Motor1_pwm.ChangeDutyCycle(i)
    Motor2_pwm.ChangeDutyCycle(j)
    Motor3_pwm.ChangeDutyCycle(k)

    time.sleep(5/1000) # Reduced from 10ms for faster response

    if not interruptRequested:
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = newSpeed1, newSpeed2, newSpeed3



#=== ORIGINAL TIMED MOVEMENT FUNCTIONS FOR SPEECH/TEXT CONTROL ===#
def goForwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = False
    triggered2 = False
    triggered3 = False

    # Set individual motor directions for forward
    GPIO.output(Motor1_Dir, GPIO.HIGH)    # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.HIGH)    # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)     # Motor 3 forward

    changeSpeedSmooth(gCurSpeed1, 0,
                      gCurSpeed2, speed,
                      gCurSpeed3, speed + motor3_compensate)

    if (interruptRequested): return

    start = time.time()
    while (time.time() - start < time_ms/1000): 
        if (commandCharacter):
            print("Movement interrupted by new command")
            break
        
        if(interruptHandler()): break 

        time.sleep(10/1000)

def goBackwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = False
    triggered2 = False
    triggered3 = False

    # Set individual motor directions for backward
    GPIO.output(Motor1_Dir, GPIO.HIGH)    # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.LOW)     # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.HIGH)    # Motor 3 backward

    changeSpeedSmooth(gCurSpeed1, 0,
                      gCurSpeed2, speed,
                      gCurSpeed3, speed + motor3_compensate)

    if (interruptRequested): return

    start = time.time()
    while (time.time() - start < time_ms/1000): 
        if (commandCharacter):
            print("Movement interrupted by new command")
            break
        
        if(interruptHandler()): break 

        time.sleep(10/1000)

def stopMotors(time_ms):
  global triggered1, triggered2, triggered3, interruptRequested
  # Ramp down smoothly to zero
  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, 0,
                    gCurSpeed3, 0)

  if (time_ms >= 0): 
    start = time.time()
    while (time.time()- start < time_ms/1000): 
      if (commandCharacter):
        print("Stop interrupted by new command")
        break
      time.sleep(10/1000)
    
  else:
    permStop = True
  


def turnRight(speed, time_ms):
  global triggered1, triggered2, triggered3, interruptRequested
  triggered1 = False
  triggered2 = False
  triggered3 = False


  GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH)) # replaces below code
  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return
 
  start = time.time()
  while (time.time()- start < time_ms/1000): 
    if (commandCharacter):
      print("Turn right interrupted by new command")
      break
    if interruptHandler():
      break
    time.sleep(10/1000)
  


def turnLeft(speed, time_ms):
  global triggered1, triggered2, triggered3, interruptRequested
  triggered1 = False
  triggered2 = False
  triggered3 = False

  GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms/1000): 
    if (commandCharacter):
      print("Turn left interrupted by new command")
      break
    if interruptHandler():
      break
    time.sleep(10/1000)
  


def moveRight(speed, time_ms):
  global triggered1, triggered2, triggered3, interruptRequested
  triggered1 = False
  triggered2 = False
  triggered3 = False

  GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, 0,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms/1000): 
    if (commandCharacter):
      print("Move right interrupted by new command")
      break
    if interruptHandler():
      break
    time.sleep(10/1000)
  


def moveLeft(speed, time_ms):
  global triggered1, triggered2, triggered3, interruptRequested
  triggered1 = False
  triggered2 = False
  triggered3 = False

  GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, speed,
                    gCurSpeed3, 0)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms/1000): 
    if (commandCharacter):
      print("Move left interrupted by new command")
      break
    if interruptHandler():
      break
    time.sleep(10/1000)



async def main():
    
    try:
        goForwards(25, 2000)
        goBackwards(25, 2000)
        turnLeft(25, 2000)
        turnRight(25, 2000)
        moveLeft(25, 2000)
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
        time.sleep(10000)
    finally:
        print("Cleaning up GPIO...")
        Motor1_pwm.stop()
        Motor2_pwm.stop()
        Motor3_pwm.stop()
        GPIO.cleanup()
