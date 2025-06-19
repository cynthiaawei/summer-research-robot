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

# ==============================================================================
# === PIN DEFINITIONS & HARDWARE SETUP =========================================
# ==============================================================================

# --- Motor Pin Definitions ---
Motor1_Speed = 38
Motor1_Dir = 40
Motor2_Speed = 32
Motor2_Dir = 36
Motor3_Speed = 16
Motor3_Dir = 26

# --- Ultrasonic Sensor Pins & Assumed Placement ---
# IMPORTANT: Adjust these comments if your sensor placement is different.
Trig1, Echo1 = 11, 31  # Sensor 1: Front-facing
Trig2, Echo2 = 13, 29  # Sensor 2: Right-facing
Trig3, Echo3 = 15, 22  # Sensor 3: Left-facing

# --- GPIO Setup ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Motor Pins
GPIO.setup([Motor1_Speed, Motor1_Dir, Motor2_Speed, Motor2_Dir, Motor3_Speed, Motor3_Dir], GPIO.OUT, initial=GPIO.LOW)

# Sensor Pins
GPIO.setup([Echo1, Echo2, Echo3], GPIO.IN)
GPIO.setup([Trig1, Trig2, Trig3], GPIO.OUT, initial=GPIO.LOW)

# --- PWM Initialization ---
freq = 1000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# ==============================================================================
# === GLOBAL STATE & CONFIGURATION =============================================
# ==============================================================================

gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, 0, 0
gSliderSpeed = 35  # Base speed for movements (0-85 is a safe range)
motor3_compensate = 15 # Optional compensation for a weaker motor
commandCharacter = ""
movement_lock = threading.Lock() # Ensures motor commands don't conflict
OBSTACLE_DISTANCE_THRESHOLD = 30.0 # Stop if an object is closer than 30 cm

# ==============================================================================
# === SENSOR & OBSTACLE AVOIDANCE ==============================================
# ==============================================================================

def get_distance(trig_pin, echo_pin, timeout=0.05):
    """Measures distance from a single ultrasonic sensor."""
    try:
        GPIO.output(trig_pin, False)
        time.sleep(0.000002)
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        timeout_start = time.time()
        while GPIO.input(echo_pin) == 0:
            if time.time() - timeout_start > timeout: return -1
        pulse_start_time = time.time()

        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            if time.time() - timeout_start > timeout: return -1
        pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        return round(pulse_duration * 17150, 2)
    except RuntimeError:
        return -1

def check_for_obstacles(direction="any"):
    """
    Checks sensors based on movement direction and stops the robot if needed.
    This function is ONLY called from within active movement loops.
    Returns True if an obstacle was detected, False otherwise.
    """
    dist1 = get_distance(Trig1, Echo1) # Front
    dist2 = get_distance(Trig2, Echo2) # Right
    dist3 = get_distance(Trig3, Echo3) # Left

    # Always check the front sensor for any forward-related movement
    if direction in ["forward", "turnLeft", "turnRight", "any"]:
        if 0 < dist1 < OBSTACLE_DISTANCE_THRESHOLD:
            print(f"⚠️ OBSTACLE! Front sensor detected object at {dist1:.1f} cm. Stopping.")
            immediateStop()
            return True

    # Check side sensors for strafing or turning
    if direction in ["moveRight", "turnRight", "any"]:
        if 0 < dist2 < OBSTACLE_DISTANCE_THRESHOLD:
            print(f"⚠️ OBSTACLE! Right sensor detected object at {dist2:.1f} cm. Stopping.")
            immediateStop()
            return True

    if direction in ["moveLeft", "turnLeft", "any"]:
         if 0 < dist3 < OBSTACLE_DISTANCE_THRESHOLD:
            print(f"⚠️ OBSTACLE! Left sensor detected object at {dist3:.1f} cm. Stopping.")
            immediateStop()
            return True

    return False

# ==============================================================================
# === CORE MOVEMENT FUNCTIONS ==================================================
# ==============================================================================

def immediateStop():
    """Stops all motors instantly without ramping."""
    with movement_lock:
        if any(s != 0 for s in [gCurSpeed1, gCurSpeed2, gCurSpeed3]):
            print("🛑 Immediate Stop Triggered!")
            Motor1_pwm.ChangeDutyCycle(0)
            Motor2_pwm.ChangeDutyCycle(0)
            Motor3_pwm.ChangeDutyCycle(0)
            global gCurSpeed1, gCurSpeed2, gCurSpeed3
            gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, 0, 0

def changeSpeedSmooth(newSpeed1, newSpeed2, newSpeed3):
    """Ramps motor speeds up or down smoothly."""
    with movement_lock:
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        i, j, k = gCurSpeed1, gCurSpeed2, gCurSpeed3
        while i != newSpeed1 or j != newSpeed2 or k != newSpeed3:
            if i < newSpeed1: i += 1
            elif i > newSpeed1: i -= 1
            if j < newSpeed2: j += 1
            elif j > newSpeed2: j -= 1
            if k < newSpeed3: k += 1
            elif k > newSpeed3: k -= 1
            Motor1_pwm.ChangeDutyCycle(max(0, min(100, i)))
            Motor2_pwm.ChangeDutyCycle(max(0, min(100, j)))
            Motor3_pwm.ChangeDutyCycle(max(0, min(100, k)))
            time.sleep(0.005)
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = newSpeed1, newSpeed2, newSpeed3

# --- TIMED MOVEMENT (for LLM commands) ---

def goForwards(speed, time_ms):
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(0, speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if check_for_obstacles(direction="forward"): break
        if commandCharacter: break
        time.sleep(0.05)

def goBackwards(speed, time_ms):
    # WARNING: No obstacle check on reverse unless a rear sensor is installed.
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(0, speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter: break
        time.sleep(0.01)

def turnRight(speed, time_ms):
    GPIO.output(Motor1_Dir, GPIO.LOW); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(speed, speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if check_for_obstacles(direction="turnRight"): break
        if commandCharacter: break
        time.sleep(0.05)

def turnLeft(speed, time_ms):
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(speed, speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if check_for_obstacles(direction="turnLeft"): break
        if commandCharacter: break
        time.sleep(0.05)

def moveRight(speed, time_ms):
    GPIO.output(Motor1_Dir, GPIO.LOW); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(int(speed * 1.5), speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if check_for_obstacles(direction="moveRight"): break
        if commandCharacter: break
        time.sleep(0.05)

def moveLeft(speed, time_ms):
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(int(speed * 1.5), speed, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if check_for_obstacles(direction="moveLeft"): break
        if commandCharacter: break
        time.sleep(0.05)

def stopMotors(speed, time_ms): # 'speed' argument is unused but keeps signature consistent
    changeSpeedSmooth(0, 0, 0)
    if time_ms > 0: time.sleep(time_ms / 1000)

# --- IMMEDIATE MOVEMENT (for Keyboard control) ---

def startForward():
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startBackward():
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnLeft():
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnRight():
    GPIO.output(Motor1_Dir, GPIO.LOW); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveLeft():
    GPIO.output(Motor1_Dir, GPIO.HIGH); GPIO.output(Motor2_Dir, GPIO.HIGH); GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5)); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveRight():
    GPIO.output(Motor1_Dir, GPIO.LOW); GPIO.output(Motor2_Dir, GPIO.LOW); GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5)); Motor2_pwm.ChangeDutyCycle(gSliderSpeed); Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3; gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

# ==============================================================================
# === COMMAND PROCESSING & AI INTEGRATION ======================================
# ==============================================================================

def processImmediateCommand(command):
    command_map = {
        "forward": startForward, "backward": startBackward,
        "left": startTurnLeft, "right": startTurnRight,
        "moveleft": startMoveLeft, "moveright": startMoveRight,
        "stop": immediateStop
    }
    action = command_map.get(command.strip().lower())
    if action: action()

def processCommand(command, time_ms):
    command_map = {
        "forward": goForwards, "backward": goBackwards, "turnRight": turnRight,
        "turnLeft": turnLeft, "moveRight": moveRight, "moveLeft": moveLeft,
        "stop": stopMotors
    }
    action = command_map.get(command)
    if action: action(gSliderSpeed, time_ms)
    else: print(f"Unknown timed command: {command}")

def speak(text):
    system = platform.system().lower()
    try:
        if system == "linux":
            subprocess.run(["espeak", text], check=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        elif system == "darwin":
            subprocess.run(["say", text], check=True)
        elif system == "windows":
            ps_command = f'Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_command], check=True, capture_output=True)
        else: print(f"🔊 TTS: {text}")
    except (subprocess.CalledProcessError, FileNotFoundError): print(f"🔊 TTS (fallback): {text}")

def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        r.pause_threshold = 1
        r.adjust_for_ambient_noise(source, duration=1)
        audio = r.listen(source)
        try:
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            speak("I'm sorry, I didn't catch that.")
        except sr.RequestError:
            speak("Sorry, my speech service is down.")
        return None

template = "Answer conversationally.\nHistory: {context}\nQuestion: {question}\nAnswer:"
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse"],
    "stop": ["stop", "halt", "stand still"],
    "turnLeft": ["turn left"], "turnRight": ["turn right"],
    "moveLeft": ["move left", "strafe left"], "moveRight": ["move right", "strafe right"],
}
time_patterns = {"seconds": r"(\d+)\s*seconds?", "minutes": r"(\d+)\s*minutes?"}

def get_direction(user_input):
    for direction, phrases in directions.items():
        if any(phrase in user_input.lower() for phrase in phrases): return direction
    return None

def convert_to_milliseconds(text):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match: return int(match.group(1)) * (1000 if unit == "seconds" else 60000)
    return None

async def process_user_input(user_input, context):
    instructions = [instr.strip() for instr in user_input.split("then")]
    command_sequence, has_movement = [], False
    for instruction in instructions:
        direction = get_direction(instruction)
        time_ms = convert_to_milliseconds(instruction)
        if direction in ["turnLeft", "turnRight"] and time_ms is None: time_ms = 1500
        if direction and time_ms is not None:
            command_sequence.append((direction, time_ms)); has_movement = True
        elif direction == "stop":
            command_sequence.append(("stop", 0)); has_movement = True
    if has_movement:
        print(f"🤖 Executing sequence: {command_sequence}")
        for command, duration in command_sequence:
            global commandCharacter; commandCharacter = command
            processCommand(command, duration)
            commandCharacter = ""
            if all(s == 0 for s in [gCurSpeed1, gCurSpeed2, gCurSpeed3]) and command != "stop":
                print("Sequence interrupted by obstacle."); break
        return True
    return False

# ==============================================================================
# === MAIN APPLICATION LOOP & USER INTERACTION =================================
# ==============================================================================

def keyboard_control_continuous():
    global keyboard_mode_active, exit_keyboard_mode
    print("\n🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Fwd, ↓=Bwd, ←=TurnL, →=TurnR, [A]=StrafeL, [D]=StrafeR, SPACE=Stop, E=Exit")
    
    last_key_state, key_commands = {}, {"up": "forward", "down": "backward", "left": "left", "right": "right", "a": "moveleft", "d": "moveright"}
    
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            if any(s > 0 for s in [gCurSpeed1, gCurSpeed2, gCurSpeed3]):
                if check_for_obstacles(direction="any"):
                    time.sleep(0.1); continue

            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode..."); immediateStop(); exit_keyboard_mode = True; break
            
            current_key_states = {key: keyboard.is_pressed(key) for key in list(key_commands.keys()) + ["space"]}
            
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state.get(key, False):
                    processImmediateCommand("space" if key == "space" else key_commands[key])
            
            movement_key_pressed = any(current_key_states[key] for key in key_commands)
            if not movement_key_pressed and any(last_key_state.get(key, False) for key in key_commands):
                processImmediateCommand("stop")

            last_key_state = current_key_states
            time.sleep(0.02)
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}"); immediateStop(); time.sleep(0.1)

async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode
    context = ""
    print("="*50 + "\n🤖 AI Omni-Wheel Robot Assistant Initialized 🤖\n" + "="*50)
    while True:
        mode = input("\nChoose mode: (s)peech, (t)ype, (k)eyboard, or (q)uit? ").strip().lower()
        if mode in ['s', 'speech']:
            user_input = listen()
            if not user_input: continue
            if not await process_user_input(user_input, context):
                result = chain.invoke({"context": context, "question": user_input}); print(f"Bot: {result}"); speak(str(result)); context += f"\nUser: {user_input}\nAI: {result}"
        elif mode in ['t', 'type']:
            user_input = input("You: ")
            if not user_input: continue
            if not await process_user_input(user_input, context):
                result = chain.invoke({"context": context, "question": user_input}); print(f"Bot: {result}"); speak(str(result)); context += f"\nUser: {user_input}\nAI: {result}"
        elif mode in ['k', 'keyboard']:
            keyboard_mode_active = True; exit_keyboard_mode = False
            keyboard_control_continuous()
            keyboard_mode_active = False; print("🔄 Returning to mode selection...")
        elif mode in ['q', 'quit', 'exit']: break
        else: print("❌ Invalid mode.")

async def main():
    try:
        await handle_conversation()
    except (KeyboardInterrupt, SystemExit):
        print("\n👋 User interrupted. Shutting down...")
    finally:
        print("Final cleanup: Stopping motors and releasing GPIO pins...")
        immediateStop(); time.sleep(0.1)
        GPIO.cleanup(); print("✅ Cleanup complete. Goodbye!")

if __name__ == "__main__":
    asyncio.run(main())
