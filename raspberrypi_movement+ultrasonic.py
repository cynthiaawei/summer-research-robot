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
import RPi.GPIO as GPIO

# === Motor Pin Definitions (updated from test code) ===
Motor1_Speed = 38  # PWM 1 (pin 20, GPIO 20)
Motor1_Dir = 40    # Dir 1 (pin 38, GPIO 21)
Motor2_Speed = 32  # PWM 2 (pin 12, GPIO 12)
Motor2_Dir = 36    # Dir 2 (pin 31, GPIO 16)
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
GPIO.setup(Trig1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Trig2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Trig3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor1_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)

# Set PWM frequencies (adjusted from 5000 to 1000Hz for stability)
freq = 1000
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
motor3_compensate = 15
permStop = True
interruptRequested = False
spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
dir_list = [Motor1_Dir, Motor2_Dir, Motor3_Dir]
commandCharacter = ""
movement_lock = threading.Lock()

# === Obstacle Detection Variables ===
obstacle_detection_active = False
obstacle_detected = False
obstacle_detection_thread = None
OBSTACLE_THRESHOLD = 30.0  # 30 cm threshold

def get_distance(trig_pin, echo_pin, timeout=0.1):
    """
    Get distance from ultrasonic sensor with timeout protection
    Returns -1 if measurement fails
    """
    try:
        # Ensure trigger is low initially
        GPIO.output(trig_pin, False)
        time.sleep(0.000002)  # 2us settle time
        
        # Send trigger pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)  # 10us pulse
        GPIO.output(trig_pin, False)
        
        # Wait for echo start with timeout
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 0:
            if time.time() - timeout_start > timeout:
                return -1
        
        # Record when echo goes HIGH
        pulse_start = time.time()
        
        # Wait for echo end with timeout
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            if time.time() - timeout_start > timeout:
                return -1
        
        # Record when echo goes LOW
        pulse_end = time.time()
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = round(distance, 2)
        
        return distance
        
    except Exception as e:
        return -1

def obstacle_detection_loop():
    """
    Continuous obstacle detection that runs in a separate thread
    Only active when robot is moving
    """
    global obstacle_detection_active, obstacle_detected
    
    while True:
        if obstacle_detection_active:
            try:
                # Measure distance for each sensor
                dist1 = get_distance(Trig1, Echo1)
                dist2 = get_distance(Trig2, Echo2)
                dist3 = get_distance(Trig3, Echo3)
                
                # Check if any sensor detects obstacle within threshold
                distances = [dist1, dist2, dist3]
                valid_distances = [d for d in distances if d > 0]
                
                if valid_distances:
                    min_distance = min(valid_distances)
                    if min_distance < OBSTACLE_THRESHOLD:
                        print(f"🚨 OBSTACLE DETECTED! Distance: {min_distance}cm")
                        obstacle_detected = True
                        stopNoTime()  # Emergency stop
                        
                time.sleep(0.05)  # Check every 50ms when active
            except Exception as e:
                print(f"Obstacle detection error: {e}")
                time.sleep(0.1)
        else:
            time.sleep(0.1)  # Slower polling when inactive

def start_obstacle_detection():
    """Start obstacle detection monitoring"""
    global obstacle_detection_active, obstacle_detected
    obstacle_detection_active = True
    obstacle_detected = False
    print("🛡️ Obstacle detection ACTIVATED")

def stop_obstacle_detection():
    """Stop obstacle detection monitoring"""
    global obstacle_detection_active
    obstacle_detection_active = False
    print("🛡️ Obstacle detection DEACTIVATED")

def is_robot_moving():
    """Check if robot is currently moving"""
    return gCurSpeed1 > 0 or gCurSpeed2 > 0 or gCurSpeed3 > 0

# === Helper: Ramp motor speeds smoothly ===
def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    global interruptRequested, gCurSpeed1, gCurSpeed2, gCurSpeed3, obstacle_detected
    with movement_lock:
        i = curSpeed1
        j = curSpeed2
        k = curSpeed3
        
        # Start obstacle detection if moving
        if newSpeed1 > 0 or newSpeed2 > 0 or newSpeed3 > 0:
            start_obstacle_detection()
        
        while (i != newSpeed1 or j != newSpeed2 or k != newSpeed3) and not interruptRequested and not obstacle_detected:
            if i < newSpeed1:
                i += 1
            elif i > newSpeed1:
                i -= 1
            if j < newSpeed2:
                j += 1
            elif j > newSpeed2:
                j -= 1
            if k < newSpeed3:
                k += 1
            elif k > newSpeed3:
                k -= 1
            i = max(0, min(100, i))
            j = max(0, min(100, j))
            k = max(0, min(100, k))
            Motor1_pwm.ChangeDutyCycle(i)
            Motor2_pwm.ChangeDutyCycle(j)
            Motor3_pwm.ChangeDutyCycle(k)
            time.sleep(0.005)
            
        if not interruptRequested and not obstacle_detected:
            gCurSpeed1, gCurSpeed2, gCurSpeed3 = newSpeed1, newSpeed2, newSpeed3
            
        # Stop obstacle detection if not moving
        if newSpeed1 == 0 and newSpeed2 == 0 and newSpeed3 == 0:
            stop_obstacle_detection()

def stopNoTime():
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(0)
        Motor3_pwm.ChangeDutyCycle(0)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, 0, 0
        stop_obstacle_detection()

def interruptHandler():
    global triggered1, triggered2, triggered3
    if triggered1:
        triggered1 = False
        print("Interrupt from sensor 1!")
        stopNoTime()
        return True
    if triggered2:
        triggered2 = False
        print("Interrupt from sensor 2!")
        stopNoTime()
        return True
    if triggered3:
        triggered3 = False
        print("Interrupt from sensor 3!")
        stopNoTime()
        return True
    return False

# === Immediate Movement Functions ===
def startForward():
    print("Starting forward movement")
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 forward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startBackward():
    print("Starting backward movement")
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.LOW)   # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.HIGH)  # Motor 3 backward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnLeft():
    print("Starting left turn")
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 forward
    GPIO.output(Motor2_Dir, GPIO.LOW)   # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 backward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnRight():
    print("Starting right turn")
    GPIO.output(Motor1_Dir, GPIO.LOW)   # Motor 1 backward
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.HIGH)  # Motor 3 forward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveLeft():
    print("Starting left strafe")
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 forward
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 forward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveRight():
    print("Starting right strafe")
    GPIO.output(Motor1_Dir, GPIO.LOW)   # Motor 1 backward
    GPIO.output(Motor2_Dir, GPIO.LOW)   # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.HIGH)  # Motor 3 backward (opposite direction)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

def immediateStop():
    print("Immediate stop")
    stopNoTime()

# === Timed Movement Functions ===
def goForwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 forward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def goBackwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 stopped
    GPIO.output(Motor2_Dir, GPIO.LOW)   # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.HIGH)  # Motor 3 backward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def stopMotors(time_ms):
    global interruptRequested
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
    if time_ms >= 0:
        start = time.time()
        while time.time() - start < time_ms / 1000:
            if commandCharacter:
                print("Stop interrupted by new command")
                break
            time.sleep(0.01)
    else:
        global permStop
        permStop = True

def turnRight(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.LOW)   # Motor 1 backward
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.HIGH)  # Motor 3 forward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            print("Turn right interrupted by new command")
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def turnLeft(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 forward
    GPIO.output(Motor2_Dir, GPIO.LOW)   # Motor 2 backward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 backward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            print("Turn left interrupted by new command")
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def moveRight(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.LOW)   # Motor 1 backward
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 forward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, 0, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            print("Move right interrupted by new command")
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def moveLeft(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested, obstacle_detected
    triggered1 = triggered2 = triggered3 = False
    obstacle_detected = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)  # Motor 1 forward
    GPIO.output(Motor2_Dir, GPIO.HIGH)  # Motor 2 forward
    GPIO.output(Motor3_Dir, GPIO.LOW)   # Motor 3 forward (opposite direction)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, 0)
    if interruptRequested or obstacle_detected:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

# === Command Processing Functions ===
def processImmediateCommand(command):
    command = command.strip().lower()
    if command == "forward":
        startForward()
    elif command == "backward":
        startBackward()
    elif command == "left":
        startTurnLeft()
    elif command == "right":
        startTurnRight()
    elif command == "moveleft":
        startMoveLeft()
    elif command == "moveright":
        startMoveRight()
    elif command == "stop":
        immediateStop()

def processCommand(command, time_ms):
    if command == "forward":
        goForwards(gSliderSpeed, time_ms)
    elif command == "backward":
        goBackwards(gSliderSpeed, time_ms)
    elif command == "turnRight":
        turnRight(gSliderSpeed, time_ms)
    elif command == "turnLeft":
        turnLeft(gSliderSpeed, time_ms)
    elif command == "moveRight":
        moveRight(gSliderSpeed, time_ms)
    elif command == "moveLeft":
        moveLeft(gSliderSpeed, time_ms)
    elif command == "stop":
        stopMotors(time_ms)
    else:
        print("Unknown timed command:", command)

def speak(text):
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
    "moveLeft": ["move left", "strafe left"],
    "moveRight": ["move right", "strafe right"],
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
    global keyboard_mode_active, exit_keyboard_mode
    print("🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Forward, ↓=Backward, ←=Turn Left, →=Turn Right")
    print("          A=Strafe Left, D=Strafe Right, SPACE=Stop, E=Exit")
    last_key_state = {"up": False, "down": False, "left": False, "right": False, "a": False, "d": False, "space": False}
    key_commands = {"up": "forward", "down": "backward", "left": "left", "right": "right", "a": "moveleft", "d": "moveright"}
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode...")
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
            current_key_states = {"up": keyboard.is_pressed("up"), "down": keyboard.is_pressed("down"), "left": keyboard.is_pressed("left"), "right": keyboard.is_pressed("right"), "a": keyboard.is_pressed("a"), "d": keyboard.is_pressed("d"), "space": keyboard.is_pressed("space")}
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state[key]:
                    if key == "space":
                        processImmediateCommand("stop")
                        print("🛑 STOP pressed")
                    else:
                        command = key_commands[key]
                        processImmediateCommand(command)
                        print(f"▶️ {key.upper()} pressed → {command}")
            for key, is_pressed in current_key_states.items():
                if not is_pressed and last_key_state[key] and key != "space":
                    processImmediateCommand("stop")
                    print(f"⏹️ {key.upper()} released → stop")
            last_key_state = current_key_states.copy()
            time.sleep(0.02)
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}")
            time.sleep(0.1)

async def process_user_input(user_input, context):
    long_instruction = ""
    contain_instructions = False
    instructions = [instr.strip() for instr in user_input.split("then")]
    
    for instruction in instructions:
        instr_lower = instruction.lower()
        direction = get_direction(instr_lower)
        time_in_ms = convert_to_milliseconds(instr_lower)
        
        if 'turn left' in instr_lower:
            direction = 'turnLeft'
            time_in_ms = time_in_ms or 2000
        elif 'turn right' in instr_lower:
            direction = 'turnRight'
            time_in_ms = time_in_ms or 2000
        elif 'move left' in instr_lower or 'strafe left' in instr_lower:
            direction = 'moveLeft'
            time_in_ms = time_in_ms or 2000
        elif 'move right' in instr_lower or 'strafe right' in instr_lower:
            direction = 'moveRight'
            time_in_ms = time_in_ms or 2000
        
        if direction and (time_in_ms is not None):
            long_instruction += f"{direction} {time_in_ms} "
            contain_instructions = True
        elif direction == "stop":
            long_instruction += "stop -1"
            contain_instructions = True
    
    if contain_instructions:
        print("Bot:", long_instruction.strip())
        words = long_instruction.strip().split()
        for i in range(0, len(words), 2):
            if i + 1 < len(words):
                command = words[i]
                duration = int(words[i + 1])
                global commandCharacter, obstacle_detected
                print(f"Executing: {command} for {duration}ms")
                commandCharacter = command  # Set for the current command
                obstacle_detected = False  # Reset obstacle flag
                processCommand(command, duration)
                commandCharacter = ""  # Clear after completion
                await asyncio.sleep(duration / 1000)  # Wait for the command to finish
        return True
    return False

keyboard_mode_active = False
exit_keyboard_mode = False

async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode
    context = ""
    print("Welcome to the AI Chatbot! Type 'exit' to quit.")
    while True:
        try:
            mode = input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()
            if mode == 's':
                while True:
                    user_input = listen()
                    if not user_input:
                        continue
                    if user_input.lower() == "exit":
                        return
                    if await process_user_input(user_input, context):
                        context += f"\nUser: {user_input}\nAI: [Movement Command]"
                    else:
                        if chain:
                            try:
                                result = chain.invoke({"context": context, "question": user_input})
                                print("Bot:", result)
                                speak(str(result))
                                context += f"\nUser: {user_input}\nAI: {result}"
                            except Exception as e:
                                print(f"LLM error: {e}")
                                response = "I'm having trouble processing that request."
                                print("Bot:", response)
                                speak(response)
                        else:
                            response = "LLM not available. Please use movement commands."
                            print("Bot:", response)
                            speak(response)
                    continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
                    if continue_mode == 'n':
                        break
            elif mode == 't':
                while True:
                    user_input = input("You: ")
                    if user_input.lower() == "exit":
                        return
                    if await process_user_input(user_input, context):
                        context += f"\nUser: {user_input}\nAI: [Movement Command]"
                    else:
                        if chain:
                            try:
                                result = chain.invoke({"context": context, "question": user_input})
                                print("Bot:", result)
                                speak(str(result))
                                context += f"\nUser: {user_input}\nAI: {result}"
                            except Exception as e:
                                print(f"LLM error: {e}")
                                response = "I'm having trouble processing that request."
                                print("Bot:", response)
                                speak(response)
                        else:
                            response = "LLM not available. Please use movement commands."
                            print("Bot:", response)
                            speak(response)
                    continue_mode = input("Continue text mode? (y/n): ").strip().lower()
                    if continue_mode == 'n':
                        break
            elif mode == 'k':
                keyboard_mode_active = True
                exit_keyboard_mode = False
                keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
                keyboard_thread.start()
                while keyboard_mode_active and not exit_keyboard_mode:
                    await asyncio.sleep(0.1)
                keyboard_mode_active = False
                print("🔄 Returning to mode selection...")
            else:
                print("❌ Invalid mode. Please choose 's', 't', or 'k'.")
                continue
        except KeyboardInterrupt:
            print("\n👋 Shutting down...")
            return
        except Exception as e:
            print(f"Error in conversation handler: {e}")
            continue

async def main():
    try:
        await handle_conversation()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    except Exception as e:
        print(f"Main error: {e}")
    finally:
        print("Cleaning up...")
        try:
            Motor1_pwm.stop()
            Motor2_pwm.stop()
            Motor3_pwm.stop()
            GPIO.cleanup()
        except Exception as e:
            print(f"Cleanup error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"Program error: {e}")
        try:
            GPIO.cleanup()
        except:
            pass
