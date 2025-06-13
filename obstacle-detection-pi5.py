import RPi.GPIO as GPIO
import time

#--Motor pin definitions--
Motor1_Speed = 38 #PWM 1
Motor1_Dir   = 40 #Dir 1
Motor2_Speed = 32 #PWM 2
Motor2_Dir   = 36 #Dir 2
Motor3_Speed = 28 #PWM 3
Motor3_Dir   = 26 #Dir 3

#Set PWM frequencies
freq = 5000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor2_Speed, freq)

#--Ultrasonic Sensor--
Echo1 = 31
Echo2 = 29
Echo3 = 27
Trig1 = 33
Trig2 = 35
Trig3 = 37
#interrupts?
triggered1 = False
triggered2 = False
triggered3 = False

def onSignal1():
    triggered1 = True

def onSignal2():
    triggered2 = True

def onSignal3():
    triggered3 = True


#--global state--
gCurSpeed1 = 0
gCurSpeed2 = 0
gCurSpeed3 = 0
gSliderSpeed = 25 #set speed of motors, max 85

curDir1 = GPIO.HIGH # HIGH = FWD
curDir2 = GPIO.HIGH # HIGH = FWD
curDir3 = GPIO.HIGH # HIGH = FWD

motor3_compensate = 15 #probably need to change
permStop = True
interruptRequested = False

# tidy up
spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
dir_list = [curDir1, curDir2, curDir3]

#removed BLE code
commandCharacter = "" #replaces commandCharacter

#-- HELPER: Ramp motor speeds smoothly --
def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
  i = curSpeed1
  j = curSpeed2
  k = curSpeed3
  interruptRequested = False

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

  if (not interruptRequested):
    gCurSpeed1 = newSpeed1
    gCurSpeed2 = newSpeed2
    gCurSpeed3 = newSpeed3

def stopNoTime():
  GPIO.out(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.HIGH))
  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)

def interruptHandler():
  if (triggered1):
    triggered1 = False
        #print("Interrupt from sensor 1!")
    stopNoTime()
    return True
  
  if (triggered2):
    triggered2 = False
        # print("Interrupt from sensor 2!")
    stopNoTime()
    return True
  
  if (triggered3):
    triggered3 = False
        #print("Interrupt from sensor 3!")
    stopNoTime()
    return True
  
  return False


#=== IMMEDIATE MOVEMENT FUNCTIONS FOR KEYBOARD CONTROL ===#
def startForward():
  print("Starting forward movement")
  GPIO.out(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.HIGH))

  # Set speeds immediately without ramping for responsiveness
  Motor1_pwm.changeDutyCycle(0)
  Motor2_pwm.changeDutyCycle(gSliderSpeed)
  Motor3_pwm.changeDutyCycle(gSliderSpeed + motor3_compensate)

  gCurSpeed1 = 0
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate

def startBackward():
  print("Starting backward movement")
  GPIO.out(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH))

  Motor1_pwm.ChangeDutyCycle(0)
  Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

  gCurSpeed1 = 0
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate


def startTurnLeft():
  print("Starting left turn")
  GPIO.out(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))

  Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

  gCurSpeed1 = gSliderSpeed
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate


def startTurnRight():
  print("Starting right turn")

  GPIO.OUT(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH))

  Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

  gCurSpeed1 = gSliderSpeed
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate

def immediateStop():
  print("Immediate stop")

  Motor1_pwm.ChangeDutyCycle(0)
  Motor2_pwm.ChangeDutyCycle(0)
  Motor3_pwm.ChangeDutyCycle(0) 

  gCurSpeed1 = 0
  gCurSpeed2 = 0
  gCurSpeed3 = 0


#=== ORIGINAL TIMED MOVEMENT FUNCTIONS FOR SPEECH/TEXT CONTROL ===#
def goForwards(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False

  # Serial.print("forwards ")
  # Serial.println(time_ms)

  GPIO.OUT(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW))

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return # Exit early if interrupted

  start = time.time()
  while (time.time() - start < time_ms): 
    if (commandCharacter):
      print("Movement interrupted by new command")
      break
    
    if(interruptHandler()): break 

    time.sleep(10/1000) # Small delay to prevent excessive polling


def goBackwards(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False

  GPIO.OUT(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH))
  # Serial.print("backwards ")
  # Serial.println(time_ms)

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return

  start = time.time()
  while (time.time() - start < time_ms): 
    if (commandCharacter):
      print("Movement interrupted by new command")
      break
    
    if(interruptHandler()): break 

    time.sleep(10/1000) # Small delay to prevent excessive polling  


def stopMotors(time_ms):
  # Serial.print("stop ")
  # Serial.println(time_ms)

  # Ramp down smoothly to zero
  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, 0,
                    gCurSpeed3, 0)

  if (time_ms >= 0): 
    start = time.time()
    while (time.time()- start < time_ms): 
      if (commandCharacter):
        print("Stop interrupted by new command")
        break
      time.sleep(10/1000)
    
  else:
    permStop = True
  


def turnRight(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False
  # Serial.print("turnRight ")
  # Serial.println(time_ms)

  GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH)) # replaces below code
  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return
 
  start = time.time()
  while (time.time()- start < time_ms): 
    if (commandCharacter):
      print("Turn right interrupted by new command")
      break
    time.sleep(10/1000)
  


def turnLeft(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False
  # Serial.print("turnLeft ")
  # Serial.println(time_ms)

  GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms): 
    if (commandCharacter):
      print("Turn left interrupted by new command")
      break
    time.sleep(10/1000)
  


def moveRight(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False
  # Serial.print("moveRight ")
  # Serial.println(time_ms)

  GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, 0,
                    gCurSpeed3, speed + motor3_compensate)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms): 
    if (commandCharacter):
      print("Move right interrupted by new command")
      break
    time.sleep(10/1000)
  


def moveLeft(speed, time_ms):
  triggered1 = False
  triggered2 = False
  triggered3 = False
  # Serial.print("moveLeft ")
  # Serial.println(time_ms)

  GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW)) # replaces below code

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, speed,
                    gCurSpeed3, 0)

  if (interruptRequested): return

  start = time.time()
  while (time.time()- start < time_ms): 
    if (commandCharacter):
      print("Move left interrupted by new command")
      break
    time.sleep(10/1000)
    
#   if(interruptHandler()): 
#     break
  time.sleep(10/1000)


#=== STRING SPLITTING HELPERS ===#
isEnd = False

# def nextWord(String &input) 
#   input.trim()
#   int index = input.indexOf(' ')
#   if (index == -1) 
#     isEnd = true
#     word = input
#     input = ""
#     return word
  
#   String word = input.substring(0, index)
#   input = input.substring(index + 1)
#   return word


# void splitString(String input, String words[]) 
#   int i = 0
#   isEnd = false
#   while (!isEnd && i < 20) 
#     words[i++] = nextWord(input)
  


#=== COMMAND PROCESSING FUNCTIONS ===#
def processImmediateCommand(command):
  # Serial.print("Immediate command: ")
  # Serial.println(command)

  if (command == "forward"): 
    startForward()
  elif(command == "backward"):
    startBackward()
  elif (command == "left"):
    startTurnLeft()
  elif (command == "right"): 
    startTurnRight()
  elif (command == "stop"):
    immediateStop()
    # else 
    # Serial.print("Unknown immediate command: ")
    # Serial.println(command)
  


def processCommand(command, time_ms):
  # Serial.print("Processing: ")
  # Serial.print(command)
  # Serial.print(" for ")
  # Serial.print(time_ms)
  # Serial.println("ms")

  if (command == "forward"):
    goForwards(gSliderSpeed, time_ms)
  elif (command == "backward"):
    goBackwards(gSliderSpeed, time_ms)
  elif (command == "turnRight"):
    turnRight(gSliderSpeed, time_ms)
  elif (command == "turnLeft"):
    turnLeft(gSliderSpeed, time_ms)
  elif (command == "moveRight"):
    moveRight(gSliderSpeed, time_ms)
  elif(command == "moveLeft"):
    moveLeft(gSliderSpeed, time_ms)
  elif(command == "stop"):
    stopMotors(time_ms)
  else:
    print("Unknown timed command: ")
    print(command)


#Initialize code
    # Initialize serial for debugging
    #Serial.begin(9600)
    #while (!Serial && millis() < 5000) # Wait up to 5 seconds for Serial
    #Serial.println("=== Robot BLE Controller Starting ===")
  
  # Configure motor pins
GPIO.setmode(GPIO.BOARD) #using board numbering system
GPIO.setup(Motor1_Speed, GPIO.OUT, initial = 0)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial = 0)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial = 0)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial = GPIO.LOW)

# Initialize motors to stopped state
GPIO.output(spd_list, GPIO.LOW)
GPIO.output(dir_list, (curDir1, curDir2, curDir3))
Motor1_pwm(0)
Motor2_pwm(0)
Motor3_pwm(0)

# Interrupts
    #   GPIO.setup(interruptPin1, INPUT_PULLUP)
    #   GPIO.setup(interruptPin2, INPUT_PULLUP)
    #   GPIO.setup(interruptPin3, INPUT_PULLUP)

    #   attachInterrupt(digitalPinToInterrupt(interruptPin1), onSignal1, RISING)
    #   attachInterrupt(digitalPinToInterrupt(interruptPin2), onSignal2, RISING)
    #   attachInterrupt(digitalPinToInterrupt(interruptPin3), onSignal3, RISING)

# Initialize BLE
    #   if (!BLE.begin()) 
    #     Serial.println("ERROR: Starting BLE failed!")
    #     while (1)

    #   BLE.setLocalName("RobotBLE")
    #   robotService.addCharacteristic(commandChar)
    #   BLE.addService(robotService)
    #   commandChar.writeValue("idle")
    #   BLE.setAdvertisedService(robotService)
    #   BLE.setDeviceName("RobotBLE")
    #   BLE.setAdvertisingInterval(100) # 100 ms interval
    #   BLE.setConnectable(true)

    #   if (!BLE.advertise()) 
    #     Serial.println("ERROR: Failed to start advertising!")
    #     while (1)
    
    #   Serial.println("BLE Robot Ready! Advertising as RobotBLE")




while(True):
  # Poll BLE stack
  # BLE.poll()

  # Check if a central has connected
  # BLEDevice central = BLE.central()
#   if (central):
#     Serial.print("Connected to central: ")
#     Serial.println(central.address())

    # While connected, handle incoming writes
    # while (central.connected()) 
    if (commandCharacter):
        # Read the BLE data into bleBuffer
        len = commandCharacter.valueLength()
        if (len > 127) len = 127 # Prevent overfFalse
        commandCharacter.readValue((uint8_t*)bleBuffer, len)
        bleBuffer[len] = '\0'

        print("Command received: ")
        print(commandCharacter + "\n")

        # Convert to String and split into words
        incoming = String(bleBuffer)
        incoming.trim()
        static String words[20]
        for (int i = 0 i < 20 i++) 
          words[i] = ""
        
        splitString(incoming, words)

        # If only a single token, treat as immediate command
        if (words[0] != "" && words[1] == "") 
          processImmediateCommand(words[0])
        
        else 
          # Otherwise, parse pairs: <command> <time_ms>
          for (int i = 0 i + 1 < 20 i += 2) 
            if (words[i] == "" || words[i + 1] == "") break
            String subCmd = words[i]
            int duration = words[i + 1].toInt()
            interruptRequested = false
            processCommand(subCmd, duration)
            if (interruptRequested) 
              Serial.println("Previous command interrupted by new BLE data")
              break
            
          
        

        bleBuffer[0] = '\0'         # Clear the buffer
        permStop = false           # AlFalse movement
      

      # If permStop is set externally, ensure motors are off
      if (permStop) 
        analogWrite(Motor1_Speed, 0)
        analogWrite(Motor2_Speed, 0)
        analogWrite(Motor3_Speed, 0)
      
    #   delay(10) # Avoid hammering BLE stack too quickly
    

    # Disconnected from central
    # Serial.println("Disconnected from central.")
    # bleBuffer[0] = '\0'
    # permStop = true
    # interruptRequested = false
  
GPIO.cleanup()


