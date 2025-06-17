import asyncio
import re
import threading
import keyboard
import subprocess
import time
import platform
import speech_recognition as sr
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import sys
import os
import RPi.GPIO as GPIO

# === Motor Pin Definitions ===
Motor1_Speed = 38  # PWM 1
Motor1_Dir = 40   # Dir 1
Motor2_Speed = 32  # PWM 2
Motor2_Dir = 36    # Dir 2
Motor3_Speed = 16  # PWM 3
Motor3_Dir = 26    # Dir 3

# === Ultrasonic Sensor ===
Echo1 = 31
Echo2 = 29
Echo3 = 22
Trig1 = 11
Trig2 = 13
Trig3 = 15
triggered1 = False
triggered2 = False
triggered3 = False

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Echo1, GPIO.IN)
GPIO.setup(Echo2, GPIO.IN)
GPIO.setup(Echo3, GPIO.IN)
GPIO.setup(Trig1, GPIO.OUT)
GPIO.setup(Trig2, GPIO.OUT)
GPIO.setup(Trig3, GPIO.OUT)
GPIO.setup(Motor1_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)

# Set PWM frequencies
freq = 5000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# Interrupt handlers
def onSignal1(channel):
    global triggered1
    triggered1 = True

def onSignal2(channel):
    global triggered2
    triggered2 = True

def onSignal3(channel):
    global triggered3
    triggered3 = True

GPIO.add_event_detect(Echo1, GPIO.RISING, callback=onSignal1)
GPIO.add_event_detect(Echo2, GPIO.RISING, callback=onSignal2)
GPIO.add_event_detect(Echo3, GPIO.RISING, callback=onSignal3)

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

def stopNoTime():
  GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.HIGH))
  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)

def interruptHandler():
  global triggered1,triggered2,triggered3
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
    # Set individual motor directions
    GPIO.output(Motor1_Dir, GPIO.HIGH)    # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.HIGH)    # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)     # Motor 3 forward

    # Set speeds - Motor 1 stopped, Motors 2&3 moving
    Motor1_pwm.ChangeDutyCycle(0)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
    Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = 0
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def startBackward():
    print("Starting backward movement")
    # Set individual motor directions
    GPIO.output(Motor1_Dir, GPIO.HIGH)    # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.LOW)     # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.HIGH)    # Motor 3 backward

    # Set speeds - Motor 1 stopped, Motors 2&3 moving
    Motor1_pwm.ChangeDutyCycle(0)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
    Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = 0
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def startTurnLeft():
  print("Starting left turn")
  GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))

  Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

  global gCurSpeed1, gCurSpeed2, gCurSpeed3
  gCurSpeed1 = gSliderSpeed
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate


def startTurnRight():
  print("Starting right turn")

  GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH))

  Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
  Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)

  global gCurSpeed1, gCurSpeed2, gCurSpeed3
  gCurSpeed1 = gSliderSpeed
  gCurSpeed2 = gSliderSpeed
  gCurSpeed3 = gSliderSpeed + motor3_compensate

def immediateStop():
  print("Immediate stop")

  Motor1_pwm.ChangeDutyCycle(0)
  Motor2_pwm.ChangeDutyCycle(0)
  Motor3_pwm.ChangeDutyCycle(0)

  global gCurSpeed1, gCurSpeed2, gCurSpeed3
  gCurSpeed1 = 0
  gCurSpeed2 = 0
  gCurSpeed3 = 0


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

#=== COMMAND PROCESSING FUNCTIONS ===#
def processImmediateCommand(command):


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
  


def processCommand(command, time_ms):

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

def speak(text):
    """Cross-platform TTS implementation"""
    system = platform.system().lower()
    try:
        if system == "windows":
            ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_command], capture_output=True, check=True)
        elif system == "darwin":
            subprocess.run(["say", text], check=True)
        elif system == "linux":
            try:
                subprocess.run(["espeak", text], check=True)
            except (subprocess.CalledProcessError, FileNotFoundError):
                try:
                    subprocess.run(["echo", text, "|", "festival", "--tts"], shell=True, check=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    subprocess.run(["spd-say", text], check=True)
        else:
            print(f"🔊 TTS: {text}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            if system == "windows":
                vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
                subprocess.run(["wscript", "/nologo", "-"], input=vbs_script, text=True, capture_output=True)
            elif system == "darwin":
                applescript = f'say "{text}"'
                subprocess.run(["osascript", "-e", applescript], check=True)
            else:
                raise subprocess.CalledProcessError(1, "TTS failed")
        except:
            print(f"🔊 TTS: {text}")

def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        audio = r.listen(source)
        try:
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("❌ Didn't catch that.")
        except sr.RequestError:
            print("❌ API error.")
        return None

template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse"],
    "stop": ["stop", "halt", "stand still"],
    "turnLeft": ["turn left"],
    "turnRight": ["turn right"],
    "moveLeft": ["move left"],
    "moveRight": ["move right"],
}
time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours": r"(\d+)\s*hours?"
}

def get_direction(user_input):
    for direction, phrases in directions.items():
        if any(phrase in user_input.lower() for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(text):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match:
            value = int(match.group(1))
            return value * 1000 if unit == "seconds" else value * 60000 if unit == "minutes" else value * 3600000
    return None

def keyboard_control_continuous():
    """Continuous keyboard control mode with immediate response"""
    global current_key, last_command, keyboard_mode_active, exit_keyboard_mode
    
    print("🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Forward, ↓=Backward, ←=Left, →=Right, SPACE=Stop, E=Exit")
    
    last_key_state = {
        "up": False,
        "down": False, 
        "left": False,
        "right": False,
        "space": False
    }
    
    # Mapping keys to immediate movement commands (no duration)
    key_commands = {
        "up": "forward",
        "down": "backward", 
        "left": "left",
        "right": "right"
    }
    
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            # Check for exit key
            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode...")
                # Send stop command before exiting
                processImmediateCommand("stop\n")
                exit_keyboard_mode = True
                break
            
            # Check each key state
            current_key_states = {
                "up": keyboard.is_pressed("up"),
                "down": keyboard.is_pressed("down"),
                "left": keyboard.is_pressed("left"), 
                "right": keyboard.is_pressed("right"),
                "space": keyboard.is_pressed("space")
            }
            
            # Handle key press events (key just pressed)
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state[key]:
                    # Key just pressed - start movement
                    if key == "space":
                        processImmediateCommand("stop\n")
                        print("🛑 STOP pressed")
                    else:
                        command = key_commands[key]
                        processImmediateCommand(f"{command}\n")
                        print(f"▶️ {key.upper()} pressed → {command}")
            
            # Handle key release events (key just released)
            for key, is_pressed in current_key_states.items():
                if not is_pressed and last_key_state[key]:
                    # Key just released - stop movement (except for space key)
                    if key != "space":
                        processImmediateCommand("stop\n")
                        print(f"⏹️ {key.upper()} released → stop")
            
            # Update previous key states
            last_key_state = current_key_states.copy()
            
            time.sleep(0.02)  # 50Hz polling rate for responsive control
            
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}")
            # Continue running even if there's an error
            time.sleep(0.1)
async def process_user_input(user_input, context):
    """Process user input for movement commands. Returns True if movement command was processed."""
    long_instruction = ""
    contain_instructions = False

    instructions = [instr.strip() for instr in user_input.split("then")]
    for instruction in instructions:
        instr_lower = instruction.lower()
        direction = get_direction(instr_lower)
        time_in_ms = convert_to_milliseconds(instr_lower)

        # Override for explicit "turn left" / "turn right"
        if 'turn left' in instr_lower:
            direction = 'turnLeft'
            time_in_ms = 5000
        elif 'turn right' in instr_lower:
            direction = 'turnRight'
            time_in_ms = 5000

        if direction and (time_in_ms is not None):
            long_instruction += f"{direction} {time_in_ms} "
            contain_instructions = True
        elif direction == "stop":
            long_instruction += "stop -1"
            contain_instructions = True
    if contain_instructions:
        print("Bot:", long_instruction.strip())
        # Split instructions into pairs and process each
        words = long_instruction.strip().split()
        for i in range(0, len(words), 2):
            if i + 1 < len(words):
                command = words[i]
                duration = int(words[i + 1])
                commandCharacter = command  # Set for interrupt checking
                processCommand(command, duration)
        return True
    return False
async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode
    context = ""
    print("Welcome to the AI Chatbot! Type 'exit' to quit.")
    
    while True:
        mode = input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()

        # -----------------------------------
        # 1) SPEECH MODE
        # -----------------------------------
        if mode == 's':
            while True:
                user_input = listen()
                if not user_input:
                    continue
                    
                if user_input.lower() == "exit":
                    return  # Exit completely
                
                # Process the input (movement or AI)
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    # AI response
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                
                # Ask if they want to continue in speech mode or switch
                continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break

        # -----------------------------------
        # 2) TEXT MODE  
        # -----------------------------------
        elif mode == 't':
            while True:
                user_input = input("You: ")
                
                if user_input.lower() == "exit":
                    return  # Exit completely
                
                # Process the input (movement or AI)
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    # AI response
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                
                # Ask if they want to continue in text mode or switch
                continue_mode = input("Continue text mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break

        # -----------------------------------
        # 3) KEYBOARD MODE (Continuous)
        # -----------------------------------
        elif mode == 'k':
            keyboard_mode_active = True
            exit_keyboard_mode = False
            
            # Start keyboard control in a separate thread
            keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
            keyboard_thread.start()
            
            # Wait for exit signal
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            
            keyboard_mode_active = False
            print("🔄 Returning to mode selection...")
            
        else:
            print("❌ Invalid mode. Please choose 's', 't', or 'k'.")
            continue
async def main():
    global main_loop
    main_loop = asyncio.get_running_loop()
    try:
        await handle_conversation()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        print("Cleaning up GPIO...")
        Motor1_pwm.stop()
        Motor2_pwm.stop()
        Motor3_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
