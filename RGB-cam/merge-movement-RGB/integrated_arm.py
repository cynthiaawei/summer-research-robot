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
import logging
import face_helper as FR

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# === Motor Pin Definitions ===
Motor1_Speed = 38
Motor1_Dir = 40
Motor2_Speed = 32
Motor2_Dir = 36
Motor3_Speed = 16
Motor3_Dir = 26

# === Ultrasonic Sensor ===
Echo1 = 31
Echo2 = 29
Echo3 = 22
Trig1 = 11
Trig2 = 13
Trig3 = 15

# === Arm Specifications ===
Digital0 = 3
Digital1 = 5
Digital2 = 7

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
GPIO.setup(Digital0, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Digital1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Digital2, GPIO.OUT, initial=GPIO.LOW)

# Set PWM frequencies
freq = 1000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# === Global State ===
outGest = "string"
gCurSpeed1 = 0
gCurSpeed2 = 0
gCurSpeed3 = 0
gSliderSpeed = 25
motor3_compensate = 15
permStop = False
interruptRequested = False
commandCharacter = ""
movement_lock = threading.Lock()
keyboard_mode_active = False
exit_keyboard_mode = False
return_to_mode_selection = False
obstacle_detection_active = False
obstacle_detected = False
obstacle_detection_thread = None
OBSTACLE_THRESHOLD = 30.0

# Fixed interrupt handling - using proper threading events
interrupt_event = threading.Event()
obstacle_event = threading.Event()

def sendGesture(outGest):
    """send out a 3-bit signal through pins 3, 5, 7 using digital logic"""
    if(outGest == "waving" or outGest == "Waving"):
        GPIO.output(Digital2, False)
        GPIO.output(Digital1, False)
        GPIO.output(Digital0, True)
    elif(outGest == "shaking hands" or outGest == "Shaking Hands"):
        GPIO.output(Digital2, False)
        GPIO.output(Digital1, True)
        GPIO.output(Digital0, False)
    else:
        GPIO.output(Digital2, False)
        GPIO.output(Digital1, False)
        GPIO.output(Digital0, False)

def get_distance(trig_pin, echo_pin, timeout=0.5):
    """Improved distance measurement with better error handling"""
    try:
        # Ensure clean state
        GPIO.output(trig_pin, False)
        time.sleep(0.000002)
        
        # Send trigger pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        # Wait for echo start
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 0:
            if time.time() - timeout_start > timeout:
                return -1
        pulse_start = time.time()
        
        # Wait for echo end
        timeout_start = time.time()
        while GPIO.input(echo_pin) == 1:
            if time.time() - timeout_start > timeout:
                return -1
        pulse_end = time.time()
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound calculation
        distance = round(distance, 2)
        
        # Validate distance range
        if distance < 2 or distance > 400:
            return -1
            
        return distance
    except Exception as e:
        return -1

def obstacle_detection_loop():
    """Improved obstacle detection with better timing"""
    global obstacle_detection_active, obstacle_detected, return_to_mode_selection
    
    while True:
        try:
            if obstacle_detection_active and not obstacle_detected:
                # Only check when robot is moving
                if any([gCurSpeed1 > 0, gCurSpeed2 > 0, gCurSpeed3 > 0]):
                    distances = []
                    for trig, echo in [(Trig1, Echo1), (Trig2, Echo2), (Trig3, Echo3)]:
                        dist = get_distance(trig, echo, timeout=0.3)
                        if dist > 0:
                            distances.append(dist)
                    
                    if distances:
                        min_distance = min(distances)
                        logger.debug(f"Min distance: {min_distance}cm")
                        
                        if min_distance < OBSTACLE_THRESHOLD:
                            logger.warning(f"Obstacle detected at {min_distance}cm!")
                            obstacle_detected = True
                            obstacle_event.set()
                            emergency_stop()
                            
                    time.sleep(0.05)  # Increased interval for stability
                else:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
                
        except Exception as e:
            logger.error(f"Error in obstacle detection: {e}")
            time.sleep(0.1)

def emergency_stop():
    """Improved emergency stop with proper cleanup"""
    global obstacle_detected, return_to_mode_selection
        
    with movement_lock:
        try:
            Motor1_pwm.ChangeDutyCycle(0)
            Motor2_pwm.ChangeDutyCycle(0)
            Motor3_pwm.ChangeDutyCycle(0)
            
            global gCurSpeed1, gCurSpeed2, gCurSpeed3
            gCurSpeed1 = 0
            gCurSpeed2 = 0
            gCurSpeed3 = 0
            
            stop_obstacle_detection()
            return_to_mode_selection = True
            
        except Exception as e:
            logger.error(f"Error during emergency stop: {e}")

def start_obstacle_detection():
    """Start obstacle detection with proper state management"""
    global obstacle_detection_active, obstacle_detected
    obstacle_detection_active = True
    obstacle_detected = False
    obstacle_event.clear()

def stop_obstacle_detection():
    """Stop obstacle detection"""
    global obstacle_detection_active
    obstacle_detection_active = False

def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    """Improved smooth speed changes with better interrupt handling"""
    global gCurSpeed1, gCurSpeed2, gCurSpeed3, obstacle_detected
    
    with movement_lock:
        if obstacle_detected:
            return
            
        # Start obstacle detection if moving
        if any([newSpeed1 > 0, newSpeed2 > 0, newSpeed3 > 0]):
            start_obstacle_detection()
        
        # Smooth speed transition
        steps = max(abs(newSpeed1 - curSpeed1), abs(newSpeed2 - curSpeed2), abs(newSpeed3 - curSpeed3))
        if steps > 0:
            for step in range(steps + 1):
                if obstacle_detected or interrupt_event.is_set():
                    break
                    
                progress = step / steps if steps > 0 else 1
                speed1 = int(curSpeed1 + (newSpeed1 - curSpeed1) * progress)
                speed2 = int(curSpeed2 + (newSpeed2 - curSpeed2) * progress)
                speed3 = int(curSpeed3 + (newSpeed3 - curSpeed3) * progress)
                
                # Clamp speeds
                speed1 = max(0, min(100, speed1))
                speed2 = max(0, min(100, speed2))
                speed3 = max(0, min(100, speed3))
                
                Motor1_pwm.ChangeDutyCycle(speed1)
                Motor2_pwm.ChangeDutyCycle(speed2)
                Motor3_pwm.ChangeDutyCycle(speed3)
                
                time.sleep(0.01)  # Smoother transitions
        
        # Update global speeds if successful
        if not obstacle_detected and not interrupt_event.is_set():
            gCurSpeed1 = newSpeed1
            gCurSpeed2 = newSpeed2
            gCurSpeed3 = newSpeed3
        else:
            # Emergency stop if interrupted
            Motor1_pwm.ChangeDutyCycle(0)
            Motor2_pwm.ChangeDutyCycle(0)
            Motor3_pwm.ChangeDutyCycle(0)
            gCurSpeed1 = 0
            gCurSpeed2 = 0
            gCurSpeed3 = 0
        
        # Stop detection if not moving
        if all([newSpeed1 == 0, newSpeed2 == 0, newSpeed3 == 0]):
            stop_obstacle_detection()

def stopNoTime():
    """Immediate stop without timing"""
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(0)
        Motor3_pwm.ChangeDutyCycle(0)
        
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = 0
        gCurSpeed3 = 0
        
        stop_obstacle_detection()

def move_robot(direction, speed=None, duration_ms=None):
    """Unified movement function with proper direction control"""
    global obstacle_detected, return_to_mode_selection
    
    if obstacle_detected:
        return False
        
    if speed is None:
        speed = gSliderSpeed
        
    # Reset interrupt states
    obstacle_detected = False
    return_to_mode_selection = False
    interrupt_event.clear()
    
    # Direction configurations for omni-wheel robot
    # Assuming 120-degree wheel arrangement
    direction_configs = {
        "forward": {
            "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW],
            "speeds": [0, speed, speed + motor3_compensate]
        },
        "backward": {
            "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.HIGH],
            "speeds": [0, speed, speed + motor3_compensate]
        },
        "turnLeft": {
            "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.LOW],
            "speeds": [speed, speed, speed + motor3_compensate]
        },
        "turnRight": {
            "dirs": [GPIO.LOW, GPIO.HIGH, GPIO.HIGH],
            "speeds": [speed, speed, speed + motor3_compensate]
        },
        "moveLeft": {
            "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW],
            "speeds": [int(speed * 1.5), speed, 0]
        },
        "moveRight": {
            "dirs": [GPIO.LOW, GPIO.LOW, GPIO.HIGH],
            "speeds": [int(speed * 1.5), speed, speed + motor3_compensate]
        }
    }
    
    if direction not in direction_configs:
        logger.error(f"Unknown direction: {direction}")
        return False
        
    config = direction_configs[direction]
    
    # Set motor directions
    GPIO.output(Motor1_Dir, config["dirs"][0])
    GPIO.output(Motor2_Dir, config["dirs"][1])
    GPIO.output(Motor3_Dir, config["dirs"][2])
    
    # Apply speeds
    changeSpeedSmooth(gCurSpeed1, config["speeds"][0], 
                     gCurSpeed2, config["speeds"][1], 
                     gCurSpeed3, config["speeds"][2])
    
    # Handle duration
    if duration_ms is not None and duration_ms > 0:
        start_time = time.time()
        while time.time() - start_time < duration_ms / 1000:
            time.sleep(0.01)
        
        # Stop after duration
        if not obstacle_detected:
            changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
    
    return True

# Simplified movement functions using unified move_robot
def goForwards(speed, time_ms):
    return move_robot("forward", speed, time_ms)

def goBackwards(speed, time_ms):
    return move_robot("backward", speed, time_ms)

def turnLeft(speed, time_ms):
    return move_robot("turnLeft", speed, time_ms)

def turnRight(speed, time_ms):
    return move_robot("turnRight", speed, time_ms)

def moveLeft(speed, time_ms):
    return move_robot("moveLeft", speed, time_ms)

def moveRight(speed, time_ms):
    return move_robot("moveRight", speed, time_ms)

def startForward():
    move_robot("forward")

def startBackward():
    move_robot("backward")

def startTurnLeft():
    move_robot("turnLeft")

def startTurnRight():
    move_robot("turnRight")

def startMoveLeft():
    move_robot("moveLeft")

def startMoveRight():
    move_robot("moveRight")

def immediateStop():
    """Immediate stop with state reset"""
    global obstacle_detected, return_to_mode_selection
    obstacle_detected = False
    return_to_mode_selection = False
    interrupt_event.clear()
    stopNoTime()

def stopMotors(time_ms):
    """Stop motors with optional delay"""
    global obstacle_detected, return_to_mode_selection, permStop

    obstacle_detected = False
    return_to_mode_selection = False
    interrupt_event.clear()
    

    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)

    if time_ms >= 0:
        start_time = time.time()
        while time.time() - start_time < time_ms / 1000:
            if commandCharacter or return_to_mode_selection:
                break
            time.sleep(0.01)
    else:
        permStop = True

def processImmediateCommand(command):
    """Process immediate movement commands"""
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
    
    if command in command_map:
        command_map[command]()
        logger.debug(f"Executed immediate command: {command}")
    else:
        logger.warning(f"Unknown immediate command: {command}")

def processCommand(command, time_ms):
    """Process timed movement commands"""
    global obstacle_detected
    
    if obstacle_detected and command != "stop":
        logger.warning("Command blocked - obstacle detected")
        return
        
    command_map = {
        "forward": lambda: goForwards(gSliderSpeed, time_ms),
        "backward": lambda: goBackwards(gSliderSpeed, time_ms),
        "turnRight": lambda: turnRight(gSliderSpeed, time_ms),
        "turnLeft": lambda: turnLeft(gSliderSpeed, time_ms),
        "moveRight": lambda: moveRight(gSliderSpeed, time_ms),
        "moveLeft": lambda: moveLeft(gSliderSpeed, time_ms),
        "stop": lambda: stopMotors(time_ms)
    }
    
    if command in command_map:
        command_map[command]()
        logger.debug(f"Executed timed command: {command} for {time_ms}ms")
    else:
        logger.warning(f"Unknown timed command: {command}")

def speak(text):
    """Text-to-speech with improved error handling"""
    system = platform.system().lower()
    try:
        if system == "windows":
            ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_command], 
                         capture_output=True, check=True, timeout=10)
        elif system == "darwin":
            subprocess.run(["say", text], check=True, timeout=10)
        elif system == "linux":
            # Try multiple TTS options
            tts_commands = [
                ["espeak", text],
                ["spd-say", text],
                ["festival", "--tts", "--stdin"]
            ]
            
            for cmd in tts_commands:
                try:
                    if cmd[0] == "festival":
                        subprocess.run(cmd, input=text, text=True, 
                                     check=True, timeout=10)
                    else:
                        subprocess.run(cmd, check=True, timeout=10)
                    break
                except (subprocess.CalledProcessError, FileNotFoundError):
                    continue
            else:
                print(f"🔊 {text}")
        else:
            print(f"🔊 {text}")
    except Exception as e:
        logger.error(f"TTS error: {e}")
        print(f"🔊 {text}")

def listen():
    """Speech recognition with improved error handling"""
    r = sr.Recognizer()
    r.energy_threshold = 4000
    r.dynamic_energy_threshold = True
    r.pause_threshold = 0.8
    
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        try:
            r.adjust_for_ambient_noise(source, duration=1)
            audio = r.listen(source, timeout=5, phrase_time_limit=10)
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("❌ Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"❌ Speech recognition error: {e}")
            return None
        except sr.WaitTimeoutError:
            print("❌ Listening timeout")
            return None
        except Exception as e:
            logger.error(f"Unexpected error in speech recognition: {e}")
            return None

# Initialize AI model with better error handling
try:
    template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
    model = OllamaLLM(model="llama3")
    prompt = ChatPromptTemplate.from_template(template)
    chain = prompt | model
    logger.info("AI model initialized successfully")
except Exception as e:
    logger.error(f"Error initializing AI model: {e}")
    model = None
    chain = None

# Movement direction mappings
directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
    "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
    "stop": ["stop", "halt", "stand still", "brake", "freeze"],
    "turnLeft": ["turn left", "rotate left", "spin left"],
    "turnRight": ["turn right", "rotate right", "spin right"],
    "moveLeft": ["move left", "strafe left", "slide left", "sidestep left"],
    "moveRight": ["move right", "strafe right", "slide right", "sidestep right"]
}

time_patterns = {
    "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
    "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
    "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
}

def get_direction(user_input):
    """Extract movement direction from user input"""
    user_input = user_input.lower()
    for direction, phrases in directions.items():
        if any(phrase in user_input for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(text):
    """Convert time expressions to milliseconds"""
    text = text.lower()
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match:
            value = float(match.group(1))
            if unit == "seconds":
                return int(value * 1000)
            elif unit == "minutes":
                return int(value * 60000)
            elif unit == "hours":
                return int(value * 3600000)
    return None

def keyboard_control_continuous():
    """Improved keyboard control with better state management"""
    global keyboard_mode_active, exit_keyboard_mode, return_to_mode_selection
    
    print("🎮 Continuous keyboard mode activated!")
    print("Controls:")
    print("  ↑ = Forward    ↓ = Backward")
    print("  ← = Turn Left  → = Turn Right")
    print("  A = Move Left  D = Move Right")
    print("  SPACE = Stop   E = Exit")
    
    active_movements = set()
    
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            # Check for exit
            if keyboard.is_pressed("e"):
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
                
            if return_to_mode_selection:
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
            
            # Current key states
            current_keys = {
                "up": keyboard.is_pressed("up"),
                "down": keyboard.is_pressed("down"),
                "left": keyboard.is_pressed("left"),
                "right": keyboard.is_pressed("right"),
                "a": keyboard.is_pressed("a"),
                "d": keyboard.is_pressed("d"),
                "space": keyboard.is_pressed("space")
            }
            
            # Handle stop key
            if current_keys["space"]:
                if active_movements:
                    processImmediateCommand("stop")
                    active_movements.clear()
            else:
                # Handle movement keys
                key_to_command = {
                    "up": "forward",
                    "down": "backward",
                    "left": "left",
                    "right": "right",
                    "a": "moveleft",
                    "d": "moveright"
                }
                
                current_active = set()
                for key, is_pressed in current_keys.items():
                    if key != "space" and is_pressed:
                        current_active.add(key)
                
                # Start new movements
                for key in current_active - active_movements:
                    processImmediateCommand(key_to_command[key])
                
                # Stop released movements
                if active_movements - current_active:
                    processImmediateCommand("stop")
                
                active_movements = current_active
            
            time.sleep(0.05)  # Reduced CPU usage
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            logger.error(f"Keyboard control error: {e}")
            immediateStop()
            time.sleep(0.1)
    
    # Cleanup
    if active_movements:
        processImmediateCommand("stop")

async def process_user_input(user_input, context):
    """Process user input for movement commands"""
    global return_to_mode_selection, obstacle_detected, commandCharacter
    
    if not user_input:
        return False
    
    # Split commands by "then" for sequential execution
    instructions = [instr.strip() for instr in user_input.split("then")]
    command_sequence = []
    has_movement = False
    
    # Reset states
    obstacle_detected = False
    return_to_mode_selection = False
    
    # Parse instructions
    for instruction in instructions:
        direction = get_direction(instruction)
        time_ms = convert_to_milliseconds(instruction)
        
        # Default turn time
        if direction in ["turnLeft", "turnRight"] and time_ms is None:
            time_ms = 1500
        
        if direction and time_ms is not None:
            command_sequence.append((direction, time_ms))
            has_movement = True
        elif direction == "stop":
            command_sequence.append(("stop", 0))
            has_movement = True
    
    # Execute command sequence
    if has_movement:
        logger.info(f"Executing command sequence: {command_sequence}")
        
        for command, duration in command_sequence:
            commandCharacter = command
            processCommand(command, duration)
            commandCharacter = ""
            
            # Check for interrupts
            if (permStop):
                break
            if(obstacle_detected):
                break
            if(return_to_mode_selection):
                break
            if(interrupt_event.is_set()):
                break
            
            # Wait for command completion
            if duration > 0:
                await asyncio.sleep(duration / 1000)
        
        # Reset states after completion
        return_to_mode_selection = False
        obstacle_detected = False
        return True
    
    return False

async def handle_conversation():
    """Main conversation handler with improved mode management"""
    global keyboard_mode_active, exit_keyboard_mode, return_to_mode_selection, obstacle_detected
    
    context = ""
    print("=" * 60)
    print("🤖 AI Omni-Wheel Robot Assistant v2.0 🤖")
    print("=" * 60)
    print("Features:")
    print("  • Voice control with speech recognition")
    print("  • Text-based commands")
    print("  • Real-time keyboard control")
    print("  • Obstacle detection and avoidance")
    print("  • AI-powered conversation")
    print("=" * 60)
    
    while True:
        # Reset states at mode selection
        if return_to_mode_selection:
            return_to_mode_selection = False
            obstacle_detected = False
            interrupt_event.clear()
            immediateStop()
            print("\n⚠️ Returned to mode selection due to interrupt")
        
        print("\n🎯 Mode Selection:")
        mode = input("Choose: (s)peech, (t)ype, (k)eyboard, or (q)uit? ").strip().lower()
        
        if mode in ['s', 'speech']:
            print("\n🎤 Speech Mode Activated")
            print("Say 'exit' to return to mode selection")
            
            name = FR.findMatch("s")
            result = f"Hi {name}, what would you like to ask today?"
            print(result)
            speak(str(result))
          
            while True:
                user_input = listen()
                
                if user_input == "exit":
                    break
                
                if not user_input:
                    continue
                
                # Try to process as movement command first
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command Executed]"
                    speak("Command executed")
                else:
                    # Process as conversation
                    if chain is None:
                        response = "AI model not available. Please check Ollama installation."
                        print(f"Bot: {response}")
                        speak(response)
                    else:
                        try:
                            result = chain.invoke({"context": context, "question": user_input})
                            print(f"Bot: {result}")
                            speak(str(result))
                            context += f"\nUser: {user_input}\nAI: {result}"
                        except Exception as e:
                            logger.error(f"AI model error: {e}")
                            response = "Sorry, I encountered an error processing your request."
                            print(f"Bot: {response}")
                            speak(response)
                
                # Check for interrupts
                if return_to_mode_selection:
                    break
                new_name = FR.findMatch("s")
                if new_name != name: break
                # Ask to continue
                continue_mode = input("\nContinue speech mode? (y/n): ").strip().lower()
                new_name = FR.findMatch("s")
                if new_name != name: break
                if continue_mode != 'y':
                    break
        
        elif mode in ['t', 'type']:
            print("\n⌨️ Text Mode Activated")
            print("Type 'exit' to return to mode selection")
            name = FR.findMatch("t")
            print(f"Hi {name}, what would you like to ask today?")
            while True:
                user_input = input("\nYou: ").strip()
                
                if user_input.lower() == "exit":
                    break
                
                if not user_input:
                    continue
                
                # Try to process as movement command first
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command Executed]"
                    print("✅ Movement command executed")
                else:
                    # Process as conversation
                    if chain is None:
                        response = "AI model not available. Please check Ollama installation."
                        print(f"Bot: {response}")
                    else:
                        try:
                            result = chain.invoke({"context": context, "question": user_input})
                            print(f"Bot: {result}")
                            context += f"\nUser: {user_input}\nAI: {result}"
                        except Exception as e:
                            logger.error(f"AI model error: {e}")
                            print("Bot: Sorry, I encountered an error processing your request.")
                
                # Check for interrupts
                if return_to_mode_selection:
                    break
                new_name = FR.findMatch("t")
                if new_name != name: break
                # Ask to continue
                continue_mode = input("\nContinue text mode? (y/n): ").strip().lower()
                new_name = FR.findMatch("t")
                if new_name != name: break
                if continue_mode != 'y':
                    break
        
        elif mode in ['k', 'keyboard']:
            print("\n🎮 Keyboard Mode Activated")
            keyboard_mode_active = True
            exit_keyboard_mode = False
            
            # Start keyboard control in separate thread
            keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
            keyboard_thread.start()
            
            # Wait for keyboard mode to end
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            
            keyboard_mode_active = False
            print("🎮 Keyboard mode ended")
        
        elif mode in ['q', 'quit']:
            print("👋 Goodbye!")
            break
        
        else:
            print("❌ Invalid choice. Please select 's', 't', 'k', or 'q'.")
async def main():
    global obstacle_detection_thread
    try:
        obstacle_detection_thread = threading.Thread(target=obstacle_detection_loop, daemon=True)
        obstacle_detection_thread.start()
        await handle_conversation()
    except (KeyboardInterrupt, SystemExit):
        print("\n👋 User interrupted. Shutting down...")
    finally:
        print("Final cleanup: Stopping motors and releasing GPIO pins...")
        immediateStop()
        time.sleep(0.1)
        GPIO.remove_event_detect(Echo1)
        GPIO.remove_event_detect(Echo2)
        GPIO.remove_event_detect(Echo3)
        Motor1_pwm.stop()
        Motor2_pwm.stop()
        Motor3_pwm.stop()
        GPIO.cleanup()
        print("✅ Cleanup complete. Goodbye!")
if __name__ == "__main__":
    asyncio.run(main())
