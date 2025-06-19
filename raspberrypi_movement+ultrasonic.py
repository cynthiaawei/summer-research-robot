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

# === Motor Pin Definitions ===
Motor1_Speed = 38  # PWM 1 (pin 20, GPIO 20)
Motor1_Dir = 40    # Dir 1 (pin 38, GPIO 21)
Motor2_Speed = 32  # PWM 2 (pin 12, GPIO 12)
Motor2_Dir = 36    # Dir 2 (pin 31, GPIO 16)
Motor3_Speed = 16  # PWM 3
Motor3_Dir = 26    # Dir 3

# === Ultrasonic Sensor Pins ===
Trig1, Echo1 = 11, 31  # Sensor 1: Front-facing
Trig2, Echo2 = 13, 29  # Sensor 2: Right-facing
Trig3, Echo3 = 15, 22  # Sensor 3: Left-facing

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Motor Pins
GPIO.setup([Motor1_Speed, Motor1_Dir, Motor2_Speed, Motor2_Dir, Motor3_Speed, Motor3_Dir], GPIO.OUT, initial=GPIO.LOW)

# Sensor Pins
GPIO.setup([Echo1, Echo2, Echo3], GPIO.IN)
GPIO.setup([Trig1, Trig2, Trig3], GPIO.OUT, initial=GPIO.LOW)

# === PWM Initialization ===
freq = 1000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# === Global State ===
gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, 0, 0
gSliderSpeed = 25  # Max 85
motor3_compensate = 15
commandCharacter = ""
movement_lock = threading.Lock()
OBSTACLE_DISTANCE_THRESHOLD = 30.0  # Stop if object closer than 30 cm
emergency_stop = False
keyboard_mode_active = False
exit_keyboard_mode = False
sensor_thread = None

# === Sensor Monitoring ===
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
            if time.time() - timeout_start > timeout:
                return -1
        pulse_start = time.time()

        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            if time.time() - timeout_start > timeout:
                return -1
        pulse_end = time.time()

        distance = round((pulse_end - pulse_start) * 17150, 2)
        if distance < 2 or distance > 400:
            return -1
        return distance
    except Exception as e:
        print(f"Sensor error: {e}")
        return -1

def sensor_monitoring():
    """Thread function to monitor sensors only when robot is in motion."""
    global emergency_stop
    while not emergency_stop:
        if any(s > 0 for s in [gCurSpeed1, gCurSpeed2, gCurSpeed3]):
            with movement_lock:
                dist1 = get_distance(Trig1, Echo1)  # Front
                dist2 = get_distance(Trig2, Echo2)  # Right
                dist3 = get_distance(Trig3, Echo3)  # Left

                # Check relevant sensors based on current command
                if commandCharacter in ["forward", "turnLeft", "turnRight", ""]:
                    if 0 < dist1 < OBSTACLE_DISTANCE_THRESHOLD:
                        print(f"⚠️ Front obstacle at {dist1:.1f} cm! Stopping.")
                        immediateStop()
                        continue
                if commandCharacter in ["moveRight", "turnRight", ""]:
                    if 0 < dist2 < OBSTACLE_DISTANCE_THRESHOLD:
                        print(f"⚠️ Right obstacle at {dist2:.1f} cm! Stopping.")
                        immediateStop()
                        continue
                if commandCharacter in ["moveLeft", "turnLeft", ""]:
                    if 0 < dist3 < OBSTACLE_DISTANCE_THRESHOLD:
                        print(f"⚠️ Left obstacle at {dist3:.1f} cm! Stopping.")
                        immediateStop()
                        continue
        time.sleep(0.05)  # Sensor polling interval

# === Motor Control Functions ===
def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    global gCurSpeed1, gCurSpeed2, gCurSpeed3, emergency_stop
    with movement_lock:
        i, j, k = curSpeed1, curSpeed2, curSpeed3
        while (i != newSpeed1 or j != newSpeed2 or k != newSpeed3) and not emergency_stop:
            if i < newSpeed1: i += 1
            elif i > newSpeed1: i -= 1
            if j < newSpeed2: j += 1
            elif j > newSpeed2: j -= 1
            if k < newSpeed3: k += 1
            elif k > newSpeed3: k -= 1
            i = max(0, min(100, i))
            j = max(0, min(100, j))
            k = max(0, min(100, k))
            try:
                Motor1_pwm.ChangeDutyCycle(i)
                Motor2_pwm.ChangeDutyCycle(j)
                Motor3_pwm.ChangeDutyCycle(k)
            except Exception as e:
                print(f"Error changing motor speed: {e}")
                break
            time.sleep(0.005)
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = i, j, k

def immediateStop():
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    with movement_lock:
        print("🛑 Immediate Stop")
        try:
            Motor1_pwm.ChangeDutyCycle(0)
            Motor2_pwm.ChangeDutyCycle(0)
            Motor3_pwm.ChangeDutyCycle(0)
            gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, 0, 0
        except Exception as e:
            print(f"Error stopping motors: {e}")

# === Immediate Movement Functions ===
def startForward():
    print("Starting forward movement")
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startBackward():
    print("Starting backward movement")
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = 0, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnLeft():
    print("Starting left turn")
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startTurnRight():
    print("Starting right turn")
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = gSliderSpeed, gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveLeft():
    print("Starting left strafe")
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

def startMoveRight():
    print("Starting right strafe")
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = int(gSliderSpeed * 1.5), gSliderSpeed, gSliderSpeed + motor3_compensate

# === Timed Movement Functions ===
def goForwards(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        time.sleep(0.01)

def goBackwards(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        time.sleep(0.01)

def turnRight(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Turn right interrupted by new command")
            break
        time.sleep(0.01)

def turnLeft(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Turn left interrupted by new command")
            break
        time.sleep(0.01)

def moveRight(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Move right interrupted by new command")
            break
        time.sleep(0.01)

def moveLeft(speed, time_ms):
    global commandCharacter, emergency_stop
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    start = time.time()
    while time.time() - start < time_ms / 1000 and not emergency_stop:
        if commandCharacter:
            print("Move left interrupted by new command")
            break
        time.sleep(0.01)

def stopMotors(time_ms):
    global emergency_stop
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
    if time_ms > 0:
        start = time.time()
        while time.time() - start < time_ms / 1000 and not emergency_stop:
            if commandCharacter:
                print("Stop interrupted by new command")
                break
            time.sleep(0.01)

# === Command Processing ===
def processImmediateCommand(command):
    command = command.strip().lower()
    command_map = {
        "forward": startForward,
        "backward": startBackward,
        "left": startTurnLeft,
        "right": startTurnRight,
        "moveleft": startMoveLeft,
        "moveright": startMoveRight,
        "stop": immediateStop
    }
    action = command_map.get(command)
    if action:
        action()
    else:
        print(f"Unknown immediate command: {command}")

def processCommand(command, time_ms):
    command_map = {
        "forward": goForwards,
        "backward": goBackwards,
        "turnRight": turnRight,
        "turnLeft": turnLeft,
        "moveRight": moveRight,
        "moveLeft": moveLeft,
        "stop": stopMotors
    }
    action = command_map.get(command)
    if action:
        action(gSliderSpeed, time_ms)
    else:
        print(f"Unknown timed command: {command}")

# === Speech and AI Integration ===
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
        else:
            print(f"🔊 TTS: {text}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"🔊 TTS (fallback): {text}")

def listen():
    try:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("🎙️ Listening...")
            r.pause_threshold = 1
            r.adjust_for_ambient_noise(source, duration=1)
            audio = r.listen(source, timeout=10, phrase_time_limit=5)
            text = r.recognize_google(audio, language='en-US')
            print(f"🗣️ You said: {text}")
            return text.lower()
    except sr.WaitTimeoutError:
        speak("I didn't hear anything. Please try again.")
    except sr.UnknownValueError:
        speak("I'm sorry, I didn't catch that.")
    except sr.RequestError as e:
        speak("Sorry, my speech service is down.")
        print(f"Speech recognition error: {e}")
    return None

try:
    template = "Answer conversationally.\nHistory: {context}\nQuestion: {question}\nAnswer:"
    model = OllamaLLM(model="llama3")
    prompt = ChatPromptTemplate.from_template(template)
    chain = prompt | model
    AI_AVAILABLE = True
except Exception as e:
    print(f"⚠️ AI model initialization failed: {e}")
    AI_AVAILABLE = False

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse", "back up"],
    "stop": ["stop", "halt", "stand still", "freeze"],
    "turnLeft": ["turn left", "rotate left"],
    "turnRight": ["turn right", "rotate right"],
    "moveLeft": ["move left", "strafe left", "slide left"],
    "moveRight": ["move right", "strafe right", "slide right"],
}

time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?"
}

def get_direction(user_input):
    user_lower = user_input.lower()
    for direction, phrases in directions.items():
        if any(phrase in user_lower for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(text):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match:
            return int(match.group(1)) * (1000 if unit == "seconds" else 60000)
    return None

async def process_user_input(user_input, context):
    global commandCharacter
    if not user_input:
        return False
    instructions = [instr.strip() for instr in user_input.split("then")]
    command_sequence, has_movement = [], False
    for instruction in instructions:
        direction = get_direction(instruction)
        time_ms = convert_to_milliseconds(instruction)
        if direction in ["turnLeft", "turnRight"] and time_ms is None:
            time_ms = 1500
        if direction and time_ms is not None:
            command_sequence.append((direction, time_ms))
            has_movement = True
        elif direction == "stop":
            command_sequence.append(("stop", 0))
            has_movement = True
    if has_movement:
        print(f"🤖 Executing sequence: {command_sequence}")
        for command, duration in command_sequence:
            commandCharacter = command
            processCommand(command, duration)
            commandCharacter = ""
            if all(s == 0 for s in [gCurSpeed1, gCurSpeed2, gCurSpeed3]) and command != "stop":
                print("Sequence interrupted by obstacle or stop command.")
                break
            await asyncio.sleep(duration / 1000)
        return True
    return False

def keyboard_control_continuous():
    global keyboard_mode_active, exit_keyboard_mode, emergency_stop
    print("\n🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Fwd, ↓=Bwd, ←=TurnL, →=TurnR, [A]=StrafeL, [D]=StrafeR, SPACE=Stop, E=Exit")
    last_key_state = {}
    key_commands = {
        "up": "forward",
        "down": "backward",
        "left": "left",
        "right": "right",
        "a": "moveleft",
        "d": "moveright"
    }
    while keyboard_mode_active and not exit_keyboard_mode and not emergency_stop:
        try:
            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode...")
                immediateStop()
                exit_keyboard_mode = True
                break
            current_key_states = {key: keyboard.is_pressed(key) for key in list(key_commands.keys()) + ["space"]}
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state.get(key, False):
                    if key == "space":
                        processImmediateCommand("stop")
                    else:
                        processImmediateCommand(key_commands[key])
            movement_key_pressed = any(current_key_states[key] for key in key_commands)
            if not movement_key_pressed and any(last_key_state.get(key, False) for key in key_commands):
                processImmediateCommand("stop")
            last_key_state = current_key_states
            time.sleep(0.02)
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}")
            immediateStop()
            time.sleep(0.1)

async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode, emergency_stop, sensor_thread
    context = ""
    print("="*50 + "\n🤖 AI Omni-Wheel Robot Assistant Initialized 🤖\n" + "="*50)
    if not AI_AVAILABLE:
        print("⚠️ AI functionality disabled - only movement commands available")
    sensor_thread = threading.Thread(target=sensor_monitoring, daemon=True)
    sensor_thread.start()
    while not emergency_stop:
        try:
            mode = input("\nChoose mode: (s)peech, (t)ype, (k)eyboard, or (q)uit? ").strip().lower()
            if mode in ['s', 'speech']:
                user_input = listen()
                if not user_input:
                    continue
                if not await process_user_input(user_input, context):
                    if AI_AVAILABLE:
                        try:
                            result = chain.invoke({"context": context, "question": user_input})
                            print(f"Bot: {result}")
                            speak(str(result))
                            context += f"\nUser: {user_input}\nAI: {result}"
                        except Exception as e:
                            print(f"AI error: {e}")
                            speak("Sorry, I had trouble processing that.")
                    else:
                        speak("I can only understand movement commands right now.")
            elif mode in ['t', 'type']:
                user_input = input("You: ").strip()
                if not user_input:
                    continue
                if not await process_user_input(user_input, context):
                    if AI_AVAILABLE:
                        try:
                            result = chain.invoke({"context": context, "question": user_input})
                            print(f"Bot: {result}")
                            speak(str(result))
                            context += f"\nUser: {user_input}\nAI: {result}"
                        except Exception as e:
                            print(f"AI error: {e}")
                            print("Sorry, I had trouble processing that.")
                    else:
                        print("I can only understand movement commands right now.")
            elif mode in ['k', 'keyboard']:
                keyboard_mode_active = True
                exit_keyboard_mode = False
                keyboard_control_continuous()
                keyboard_mode_active = False
                print("🔄 Returning to mode selection...")
            elif mode in ['q', 'quit', 'exit']:
                break
            else:
                print("❌ Invalid mode. Please choose 's', 't', 'k', or 'q'.")
        except KeyboardInterrupt:
            print("\n⚠️ Keyboard interrupt detected!")
            emergency_stop = True
            break
        except Exception as e:
            print(f"⚠️ Unexpected error: {e}")
            time.sleep(1)

async def main():
    global emergency_stop
    try:
        print("🚀 Starting robot control system...")
        await handle_conversation()
    except (KeyboardInterrupt, SystemExit):
        print("\n👋 User interrupted. Shutting down...")
        emergency_stop = True
    finally:
        print("🧹 Final cleanup: Stopping motors and releasing GPIO pins...")
        immediateStop()
        time.sleep(0.2)
        try:
            Motor1_pwm.stop()
            Motor2_pwm.stop()
            Motor3_pwm.stop()
            GPIO.cleanup()
            print("✅ Cleanup complete. Goodbye!")
        except:
            print("⚠️ GPIO cleanup had issues, but system is shutting down.")

if __name__ == "__main__":
    asyncio.run(main())
