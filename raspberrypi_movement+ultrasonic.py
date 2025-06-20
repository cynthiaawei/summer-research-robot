# import asyncio
# import re
# import threading
# import keyboard
# import subprocess
# import time
# import platform
# import speech_recognition as sr
# from langchain_ollama import OllamaLLM
# from langchain_core.prompts import ChatPromptTemplate
# import RPi.GPIO as GPIO

# # === Motor Pin Definitions ===
# Motor1_Speed = 38
# Motor1_Dir = 40
# Motor2_Speed = 32
# Motor2_Dir = 36
# Motor3_Speed = 16
# Motor3_Dir = 26

# # === Ultrasonic Sensor ===
# Echo1 = 31
# Echo2 = 29
# Echo3 = 22
# Trig1 = 11
# Trig2 = 13
# Trig3 = 15
# triggered1 = False
# triggered2 = False
# triggered3 = False

# # === GPIO Setup ===
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(Echo1, GPIO.IN)
# GPIO.setup(Echo2, GPIO.IN)
# GPIO.setup(Echo3, GPIO.IN)
# GPIO.setup(Trig1, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Trig2, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Trig3, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor1_Speed, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor2_Speed, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor3_Speed, GPIO.OUT, initial=GPIO.LOW)
# GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)

# # Set PWM frequencies
# freq = 1000
# Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
# Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
# Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
# Motor1_pwm.start(0)
# Motor2_pwm.start(0)
# Motor3_pwm.start(0)

# # Interrupt handlers
# def onSignal1(channel):
#     global triggered1
#     triggered1 = True

# def onSignal2(channel):
#     global triggered2
#     triggered2 = True

# def onSignal3(channel):
#     global triggered3
#     triggered3 = True

# GPIO.add_event_detect(Echo1, GPIO.RISING, callback=onSignal1)
# GPIO.add_event_detect(Echo2, GPIO.RISING, callback=onSignal2)
# GPIO.add_event_detect(Echo3, GPIO.RISING, callback=onSignal3)

# # === Global State ===
# gCurSpeed1 = 0
# gCurSpeed2 = 0
# gCurSpeed3 = 0
# gSliderSpeed = 25
# motor3_compensate = 15
# permStop = True
# interruptRequested = False
# spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
# dir_list = [Motor1_Dir, Motor2_Dir, Motor3_Dir]
# commandCharacter = ""
# movement_lock = threading.Lock()
# keyboard_mode_active = False
# exit_keyboard_mode = False
# return_to_mode_selection = False
# obstacle_detection_active = False
# obstacle_detected = False
# obstacle_detection_thread = None
# OBSTACLE_THRESHOLD = 30.0

# def get_distance(trig_pin, echo_pin, timeout=0.5):
#     try:
#         GPIO.output(trig_pin, False)
#         time.sleep(0.000002)
#         GPIO.output(trig_pin, True)
#         time.sleep(0.00001)
#         GPIO.output(trig_pin, False)
#         timeout_start = time.time()
#         while GPIO.input(echo_pin) == 0:
#             if time.time() - timeout_start > timeout:
#                 return -1
#         pulse_start = time.time()
#         timeout_start = time.time()
#         while GPIO.input(echo_pin) == 1:
#             if time.time() - timeout_start > timeout:
#                 return -1
#         pulse_end = time.time()
#         pulse_duration = pulse_end - pulse_start
#         distance = pulse_duration * 17150
#         distance = round(distance, 2)
#         if distance < 2 or distance > 400:
#             return -1
#         return distance
#     except Exception:
#         return -1

# def obstacle_detection_loop():
#     global obstacle_detection_active
#     global obstacle_detected
#     global return_to_mode_selection
#     while True:
#         if obstacle_detection_active and not obstacle_detected:
#             try:
#                 if gCurSpeed1 > 0 or gCurSpeed2 > 0 or gCurSpeed3 > 0:
#                     dist1 = get_distance(Trig1, Echo1)
#                     dist2 = get_distance(Trig2, Echo2)
#                     dist3 = get_distance(Trig3, Echo3)
#                     distances = [dist1, dist2, dist3]
#                     valid_distances = [d for d in distances if d > 0]
#                     if valid_distances:
#                         min_distance = min(valid_distances)
#                         if min_distance < OBSTACLE_THRESHOLD:
#                             obstacle_detected = True
#                             return_to_mode_selection = True
#                             emergency_stop()
#                             break
#                 time.sleep(0.02)
#             except Exception:
#                 time.sleep(0.1)
#         else:
#             time.sleep(0.1)

# def emergency_stop():
#     global return_to_mode_selection
#     print("Stopped due to obstacle detected")
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(0)
#         Motor2_pwm.ChangeDutyCycle(0)
#         Motor3_pwm.ChangeDutyCycle(0)
#         global gCurSpeed1, gCurSpeed2, gCurSpeed3
#         gCurSpeed1 = 0
#         gCurSpeed2 = 0
#         gCurSpeed3 = 0
#         stop_obstacle_detection()

# def start_obstacle_detection():
#     global obstacle_detection_active
#     global obstacle_detected
#     obstacle_detection_active = True
#     obstacle_detected = False

# def stop_obstacle_detection():
#     global obstacle_detection_active
#     obstacle_detection_active = False

# def is_robot_moving():
#     return gCurSpeed1 > 0 or gCurSpeed2 > 0 or gCurSpeed3 > 0

# def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
#     global interruptRequested
#     global gCurSpeed1
#     global gCurSpeed2
#     global gCurSpeed3
#     global obstacle_detected
#     global return_to_mode_selection
#     with movement_lock:
#         if obstacle_detected:
#             return
#         i = curSpeed1
#         j = curSpeed2
#         k = curSpeed3
#         if newSpeed1 > 0 or newSpeed2 > 0 or newSpeed3 > 0:
#             start_obstacle_detection()
#         while (i != newSpeed1 or j != newSpeed2 or k != newSpeed3) and not interruptRequested and not obstacle_detected:
#             if i < newSpeed1:
#                 i += 1
#             elif i > newSpeed1:
#                 i -= 1
#             if j < newSpeed2:
#                 j += 1
#             elif j > newSpeed2:
#                 j -= 1
#             if k < newSpeed3:
#                 k += 1
#             elif k > newSpeed3:
#                 k -= 1
#             i = max(0, min(100, i))
#             j = max(0, min(100, j))
#             k = max(0, min(100, k))
#             Motor1_pwm.ChangeDutyCycle(i)
#             Motor2_pwm.ChangeDutyCycle(j)
#             Motor3_pwm.ChangeDutyCycle(k)
#             time.sleep(0.005)
#         if not interruptRequested and not obstacle_detected:
#             gCurSpeed1 = newSpeed1
#             gCurSpeed2 = newSpeed2
#             gCurSpeed3 = newSpeed3
#         else:
#             Motor1_pwm.ChangeDutyCycle(0)
#             Motor2_pwm.ChangeDutyCycle(0)
#             Motor3_pwm.ChangeDutyCycle(0)
#             gCurSpeed1 = 0
#             gCurSpeed2 = 0
#             gCurSpeed3 = 0
#         if newSpeed1 == 0 and newSpeed2 == 0 and newSpeed3 == 0:
#             stop_obstacle_detection()

# def stopNoTime():
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(0)
#         Motor2_pwm.ChangeDutyCycle(0)
#         Motor3_pwm.ChangeDutyCycle(0)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = 0
#         gCurSpeed2 = 0
#         gCurSpeed3 = 0
#         stop_obstacle_detection()

# def interruptHandler():
#     global triggered1
#     global triggered2
#     global triggered3
#     global return_to_mode_selection
#     if triggered1:
#         triggered1 = False
#         return_to_mode_selection = True
#         emergency_stop()
#         return True
#     if triggered2:
#         triggered2 = False
#         return_to_mode_selection = True
#         emergency_stop()
#         return True
#     if triggered3:
#         triggered3 = False
#         return_to_mode_selection = True
#         emergency_stop()
#         return True
#     return False

# def startForward():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(0)
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = 0
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def startBackward():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.LOW)
#     GPIO.output(Motor3_Dir, GPIO.HIGH)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(0)
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = 0
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def startTurnLeft():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.LOW)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = gSliderSpeed
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def startTurnRight():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.LOW)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.HIGH)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = gSliderSpeed
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def startMoveLeft():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = int(gSliderSpeed * 1.5)
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def startMoveRight():
#     global obstacle_detected
#     if obstacle_detected:
#         return
#     GPIO.output(Motor1_Dir, GPIO.LOW)
#     GPIO.output(Motor2_Dir, GPIO.LOW)
#     GPIO.output(Motor3_Dir, GPIO.HIGH)
#     start_obstacle_detection()
#     with movement_lock:
#         Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
#         Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
#         Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
#         global gCurSpeed1
#         global gCurSpeed2
#         global gCurSpeed3
#         gCurSpeed1 = int(gSliderSpeed * 1.5)
#         gCurSpeed2 = gSliderSpeed
#         gCurSpeed3 = gSliderSpeed + motor3_compensate

# def immediateStop():
#     global obstacle_detected
#     global return_to_mode_selection
#     obstacle_detected = False
#     return_to_mode_selection = False
#     stopNoTime()

# def goForwards(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def goBackwards(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.LOW)
#     GPIO.output(Motor3_Dir, GPIO.HIGH)
#     changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def stopMotors(time_ms):
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     obstacle_detected = False
#     return_to_mode_selection = False
#     changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
#     if time_ms >= 0:
#         start = time.time()
#         while time.time() - start < time_ms / 1000:
#             if commandCharacter or return_to_mode_selection:
#                 break
#             time.sleep(0.01)
#     else:
#         global permStop
#         permStop = True

# def turnRight(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.LOW)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.HIGH)
#     changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def turnLeft(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.LOW)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def moveRight(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.LOW)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, 0, gCurSpeed3, speed + motor3_compensate)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def moveLeft(speed, time_ms):
#     global triggered1
#     global triggered2
#     global triggered3
#     global interruptRequested
#     global obstacle_detected
#     global return_to_mode_selection
#     triggered1 = False
#     triggered2 = False
#     triggered3 = False
#     obstacle_detected = False
#     return_to_mode_selection = False
#     GPIO.output(Motor1_Dir, GPIO.HIGH)
#     GPIO.output(Motor2_Dir, GPIO.HIGH)
#     GPIO.output(Motor3_Dir, GPIO.LOW)
#     changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, 0)
#     if interruptRequested or obstacle_detected or return_to_mode_selection:
#         return
#     start = time.time()
#     while time.time() - start < time_ms / 1000:
#         if commandCharacter or return_to_mode_selection:
#             break
#         if interruptHandler() or obstacle_detected:
#             break
#         time.sleep(0.01)

# def processImmediateCommand(command):
#     command = command.strip().lower()
#     if command == "forward":
#         startForward()
#     elif command == "backward":
#         startBackward()
#     elif command == "left":
#         startTurnLeft()
#     elif command == "right":
#         startTurnRight()
#     elif command == "moveleft":
#         startMoveLeft()
#     elif command == "moveright":
#         startMoveRight()
#     elif command == "stop":
#         immediateStop()

# def processCommand(command, time_ms):
#     global obstacle_detected
#     if obstacle_detected and command != "stop":
#         return
#     if command == "forward":
#         goForwards(gSliderSpeed, time_ms)
#     elif command == "backward":
#         goBackwards(gSliderSpeed, time_ms)
#     elif command == "turnRight":
#         turnRight(gSliderSpeed, time_ms)
#     elif command == "turnLeft":
#         turnLeft(gSliderSpeed, time_ms)
#     elif command == "moveRight":
#         moveRight(gSliderSpeed, time_ms)
#     elif command == "moveLeft":
#         moveLeft(gSliderSpeed, time_ms)
#     elif command == "stop":
#         stopMotors(time_ms)

# def speak(text):
#     system = platform.system().lower()
#     try:
#         if system == "windows":
#             ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
#             subprocess.run(["powershell", "-Command", ps_command], capture_output=True, check=True)
#         elif system == "darwin":
#             subprocess.run(["say", text], check=True)
#         elif system == "linux":
#             try:
#                 subprocess.run(["espeak", text], check=True)
#             except (subprocess.CalledProcessError, FileNotFoundError):
#                 try:
#                     subprocess.run(f"echo '{text}' | festival --tts", shell=True, check=True)
#                 except (subprocess.CalledProcessError, FileNotFoundError):
#                     subprocess.run(["spd-say", text], check=True)
#         else:
#             print(f"🔊 {text}")
#     except (subprocess.CalledProcessError, FileNotFoundError):
#         try:
#             if system == "windows":
#                 vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
#                 subprocess.run(["cscript", "//nologo", "-"], input=vbs_script, text=True, capture_output=True)
#             elif system == "darwin":
#                 applescript = f'say "{text}"'
#                 subprocess.run(["osascript", "-e", applescript], check=True)
#             else:
#                 print(f"🔊 {text}")
#         except Exception:
#             print(f"🔊 {text}")

# def listen():
#     r = sr.Recognizer()
#     with sr.Microphone() as source:
#         print("🎙️ Listening...")
#         r.pause_threshold = 1
#         r.adjust_for_ambient_noise(source, duration=1)
#         try:
#             audio = r.listen(source, timeout=5)
#             text = r.recognize_google(audio)
#             print(f"🗣️ You said: {text}")
#             return text.lower()
#         except sr.UnknownValueError:
#             print("❌ Didn't catch that.")
#             return None
#         except sr.RequestError:
#             print("❌ API error.")
#             return None
#         except sr.WaitTimeoutError:
#             print("❌ Timed out waiting for speech.")
#             return None

# template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
# try:
#     model = OllamaLLM(model="llama3")
#     prompt = ChatPromptTemplate.from_template(template)
#     chain = prompt | model
# except Exception:
#     print("Error initializing OllamaLLM")
#     model = None
#     chain = None

# directions = {
#     "forward": ["go forward", "move forward", "move ahead", "advance"],
#     "backward": ["go backward", "move backward", "reverse"],
#     "stop": ["stop", "halt", "stand still"],
#     "turnLeft": ["turn left"],
#     "turnRight": ["turn right"],
#     "moveLeft": ["move left", "strafe left"],
#     "moveRight": ["move right", "strafe right"]
# }
# time_patterns = {
#     "seconds": r"(\d+)\s*seconds?",
#     "minutes": r"(\d+)\s*minutes?",
#     "hours": r"(\d+)\s*hours?"
# }

# def get_direction(user_input):
#     for direction, phrases in directions.items():
#         if any(phrase in user_input.lower() for phrase in phrases):
#             return direction
#     return None

# def convert_to_milliseconds(text):
#     for unit, pattern in time_patterns.items():
#         match = re.search(pattern, text)
#         if match:
#             value = int(match.group(1))
#             return value * 1000 if unit == "seconds" else value * 60000 if unit == "minutes" else value * 3600000
#     return None

# def keyboard_control_continuous():
#     global keyboard_mode_active
#     global exit_keyboard_mode
#     global return_to_mode_selection
#     print("🎮 Continuous keyboard mode activated!")
#     print("Controls: ↑=Forward, ↓=Backward, ←=Turn Left, →=Turn Right")
#     print("          A=Strafe Left, D=Strafe Right, SPACE=Stop, E=Exit")
#     last_key_state = {
#         "up": False,
#         "down": False,
#         "left": False,
#         "right": False,
#         "a": False,
#         "d": False,
#         "space": False
#     }
#     key_commands = {
#         "up": "forward",
#         "down": "backward",
#         "left": "left",
#         "right": "right",
#         "a": "moveleft",
#         "d": "moveright"
#     }
#     while keyboard_mode_active and not exit_keyboard_mode:
#         try:
#             if keyboard.is_pressed("e"):
#                 processImmediateCommand("stop")
#                 exit_keyboard_mode = True
#                 break
#             if return_to_mode_selection:
#                 processImmediateCommand("stop")
#                 exit_keyboard_mode = True
#                 break
#             current_key_states = {
#                 "up": keyboard.is_pressed("up"),
#                 "down": keyboard.is_pressed("down"),
#                 "left": keyboard.is_pressed("left"),
#                 "right": keyboard.is_pressed("right"),
#                 "a": keyboard.is_pressed("a"),
#                 "d": keyboard.is_pressed("d"),
#                 "space": keyboard.is_pressed("space")
#             }
#             for key, is_pressed in current_key_states.items():
#                 if is_pressed and not last_key_state[key]:
#                     if key == "space":
#                         processImmediateCommand("stop")
#                     else:
#                         processImmediateCommand(key_commands[key])
#             for key, is_pressed in current_key_states.items():
#                 if not is_pressed and last_key_state[key] and key != "space":
#                     processImmediateCommand("stop")
#             last_key_state = current_key_states.copy()
#             time.sleep(0.02)
#         except Exception:
#             immediateStop()
#             time.sleep(0.1)

# async def process_user_input(user_input, context):
#     global return_to_mode_selection
#     if not user_input:
#         return False
#     instructions = [instr.strip() for instr in user_input.split("then")]
#     command_sequence = []
#     has_movement = False
#     for instruction in instructions:
#         direction = get_direction(instruction)
#         time_ms = convert_to_milliseconds(instruction)
#         if direction in ["turnLeft", "turnRight"] and time_ms is None:
#             time_ms = 1500
#         if direction and time_ms is not None:
#             command_sequence.append((direction, time_ms))
#             has_movement = True
#         elif direction == "stop":
#             command_sequence.append(("stop", 0))
#             has_movement = True
#     if has_movement:
#         for command, duration in command_sequence:
#             global commandCharacter
#             commandCharacter = command
#             processCommand(command, duration)
#             commandCharacter = ""
#             if permStop or obstacle_detected or return_to_mode_selection:
#                 break
#             await asyncio.sleep(duration / 1000)
#         return_to_mode_selection = False
#         return True
#     return False

# async def handle_conversation():
#     global keyboard_mode_active
#     global exit_keyboard_mode
#     global return_to_mode_selection
#     context = ""
#     print("="*50 + "\n🤖 AI Omni-Wheel Robot Assistant Initialized 🤖\n" + "="*50)
#     while True:
#         if return_to_mode_selection:
#             return_to_mode_selection = False
#         mode = input("\nChoose mode: (s)peech, (t)ype, (k)eyboard, or (q)uit? ").strip().lower()
#         if mode in ['s', 'speech']:
#             while True:
#                 user_input = listen()
#                 if user_input == "exit":
#                     return
#                 if not user_input:
#                     continue
#                 if await process_user_input(user_input, context):
#                     context += f"\nUser: {user_input}\nAI: [Movement Command]"
#                 else:
#                     if chain is None:
#                         print("Bot: AI model not available.")
#                         speak("AI model not available.")
#                     else:
#                         result = chain.invoke({"context": context, "question": user_input})
#                         print(f"Bot: {result}")
#                         speak(str(result))
#                         context += f"\nUser: {user_input}\nAI: {result}"
#                 if return_to_mode_selection:
#                     break
#                 continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
#                 if continue_mode == 'n':
#                     break
#         elif mode in ['t', 'type']:
#             while True:
#                 user_input = input("You: ")
#                 if user_input.lower() == "exit":
#                     return
#                 if not user_input:
#                     continue
#                 if await process_user_input(user_input, context):
#                     context += f"\nUser: {user_input}\nAI: [Movement Command]"
#                 else:
#                     if chain is None:
#                         print("Bot: AI model not available.")
#                         speak("AI model not available.")
#                     else:
#                         result = chain.invoke({"context": context, "question": user_input})
#                         print(f"Bot: {result}")
#                         speak(str(result))
#                         context += f"\nUser: {user_input}\nAI: {result}"
#                 if return_to_mode_selection:
#                     break
#                 continue_mode = input("Continue text mode? (y/n): ").strip().lower()
#                 if continue_mode == 'n':
#                     break
#         elif mode in ['k', 'keyboard']:
#             keyboard_mode_active = True
#             exit_keyboard_mode = False
#             keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
#             keyboard_thread.start()
#             while keyboard_mode_active and not exit_keyboard_mode:
#                 await asyncio.sleep(0.1)
#             keyboard_mode_active = False
#         elif mode in ['q', 'quit']:
#             break
#         else:
#             print("❌ Invalid mode. Please choose 's', 't', 'k', or 'q'.")

# async def main():
#     global obstacle_detection_thread
#     try:
#         obstacle_detection_thread = threading.Thread(target=obstacle_detection_loop, daemon=True)
#         obstacle_detection_thread.start()
#         await handle_conversation()
#     except (KeyboardInterrupt, SystemExit):
#         print("\n👋 User interrupted. Shutting down...")
#     finally:
#         print("Final cleanup: Stopping motors and releasing GPIO pins...")
#         immediateStop()
#         time.sleep(0.1)
#         GPIO.remove_event_detect(Echo1)
#         GPIO.remove_event_detect(Echo2)
#         GPIO.remove_event_detect(Echo3)
#         Motor1_pwm.stop()
#         Motor2_pwm.stop()
#         Motor3_pwm.stop()
#         GPIO.cleanup()
#         print("✅ Cleanup complete. Goodbye!")

# if __name__ == "__main__":
#     asyncio.run(main())
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

# Set PWM frequencies
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
gSliderSpeed = 25
motor3_compensate = 15
permStop = True
interruptRequested = False
spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
dir_list = [Motor1_Dir, Motor2_Dir, Motor3_Dir]
commandCharacter = ""
movement_lock = threading.Lock()
keyboard_mode_active = False
exit_keyboard_mode = False
return_to_mode_selection = False
obstacle_detection_active = False
obstacle_detected = False
obstacle_detection_thread = None
OBSTACLE_THRESHOLD = 30.0

def get_distance(trig_pin, echo_pin, timeout=0.5):
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
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        if distance < 2 or distance > 400:
            return -1
        return distance
    except Exception:
        return -1

def obstacle_detection_loop():
    global obstacle_detection_active
    global obstacle_detected
    global return_to_mode_selection
    while True:
        if obstacle_detection_active and not obstacle_detected:
            try:
                if gCurSpeed1 > 0 or gCurSpeed2 > 0 or gCurSpeed3 > 0:
                    dist1 = get_distance(Trig1, Echo1)
                    dist2 = get_distance(Trig2, Echo2)
                    dist3 = get_distance(Trig3, Echo3)
                    distances = [dist1, dist2, dist3]
                    valid_distances = [d for d in distances if d > 0]
                    if valid_distances:
                        min_distance = min(valid_distances)
                        if min_distance < OBSTACLE_THRESHOLD:
                            obstacle_detected = True
                            return_to_mode_selection = True
                            emergency_stop()
                            # Removed break to keep thread running
                time.sleep(0.02)
            except Exception:
                time.sleep(0.1)
        else:
            time.sleep(0.1)

def emergency_stop():
    global return_to_mode_selection
    global obstacle_detected
    print("Stopped due to obstacle detected.")
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(0)
        Motor3_pwm.ChangeDutyCycle(0)
        global gCurSpeed1, gCurSpeed2, gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = 0
        gCurSpeed3 = 0
        stop_obstacle_detection()
        obstacle_detected = False  # Reset obstacle_detected

def start_obstacle_detection():
    global obstacle_detection_active
    global obstacle_detected
    obstacle_detection_active = True
    obstacle_detected = False

def stop_obstacle_detection():
    global obstacle_detection_active
    obstacle_detection_active = False

def is_robot_moving():
    return gCurSpeed1 > 0 or gCurSpeed2 > 0 or gCurSpeed3 > 0

def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    global interruptRequested
    global gCurSpeed1
    global gCurSpeed2
    global gCurSpeed3
    global obstacle_detected
    global return_to_mode_selection
    with movement_lock:
        if obstacle_detected:
            return
        i = curSpeed1
        j = curSpeed2
        k = curSpeed3
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
            gCurSpeed1 = newSpeed1
            gCurSpeed2 = newSpeed2
            gCurSpeed3 = newSpeed3
        else:
            Motor1_pwm.ChangeDutyCycle(0)
            Motor2_pwm.ChangeDutyCycle(0)
            Motor3_pwm.ChangeDutyCycle(0)
            gCurSpeed1 = 0
            gCurSpeed2 = 0
            gCurSpeed3 = 0
        if newSpeed1 == 0 and newSpeed2 == 0 and newSpeed3 == 0:
            stop_obstacle_detection()

def stopNoTime():
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(0)
        Motor3_pwm.ChangeDutyCycle(0)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = 0
        gCurSpeed3 = 0
        stop_obstacle_detection()

def interruptHandler():
    global triggered1
    global triggered2
    global triggered3
    global return_to_mode_selection
    if triggered1:
        triggered1 = False
        return_to_mode_selection = True
        emergency_stop()
        return True
    if triggered2:
        triggered2 = False
        return_to_mode_selection = True
        emergency_stop()
        return True
    if triggered3:
        triggered3 = False
        return_to_mode_selection = True
        emergency_stop()
        return True
    return False

def startForward():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def startBackward():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(0)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = 0
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def startTurnLeft():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = gSliderSpeed
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def startTurnRight():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = gSliderSpeed
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def startMoveLeft():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = int(gSliderSpeed * 1.5)
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def startMoveRight():
    global obstacle_detected
    if obstacle_detected:
        return
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    start_obstacle_detection()
    with movement_lock:
        Motor1_pwm.ChangeDutyCycle(int(gSliderSpeed * 1.5))
        Motor2_pwm.ChangeDutyCycle(gSliderSpeed)
        Motor3_pwm.ChangeDutyCycle(gSliderSpeed + motor3_compensate)
        global gCurSpeed1
        global gCurSpeed2
        global gCurSpeed3
        gCurSpeed1 = int(gSliderSpeed * 1.5)
        gCurSpeed2 = gSliderSpeed
        gCurSpeed3 = gSliderSpeed + motor3_compensate

def immediateStop():
    global obstacle_detected
    global return_to_mode_selection
    obstacle_detected = False
    return_to_mode_selection = False
    stopNoTime()

def goForwards(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def goBackwards(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def stopMotors(time_ms):
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    obstacle_detected = False
    return_to_mode_selection = False
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
    if time_ms >= 0:
        start = time.time()
        while time.time() - start < time_ms / 1000:
            if commandCharacter or return_to_mode_selection:
                break
            time.sleep(0.01)
    else:
        global permStop
        permStop = True

def turnRight(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.HIGH)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def turnLeft(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.LOW)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def moveRight(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.LOW)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, 0, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

def moveLeft(speed, time_ms):
    global triggered1
    global triggered2
    global triggered3
    global interruptRequested
    global obstacle_detected
    global return_to_mode_selection
    triggered1 = False
    triggered2 = False
    triggered3 = False
    obstacle_detected = False
    return_to_mode_selection = False
    GPIO.output(Motor1_Dir, GPIO.HIGH)
    GPIO.output(Motor2_Dir, GPIO.HIGH)
    GPIO.output(Motor3_Dir, GPIO.LOW)
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, 0)
    if interruptRequested or obstacle_detected or return_to_mode_selection:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        if commandCharacter or return_to_mode_selection:
            break
        if interruptHandler() or obstacle_detected:
            break
        time.sleep(0.01)

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
    global obstacle_detected
    if obstacle_detected and command != "stop":
        return
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
                    subprocess.run(f"echo '{text}' | festival --tts", shell=True, check=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    subprocess.run(["spd-say", text], check=True)
        else:
            print(f"🔊 {text}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            if system == "windows":
                vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
                subprocess.run(["cscript", "//nologo", "-"], input=vbs_script, text=True, capture_output=True)
            elif system == "darwin":
                applescript = f'say "{text}"'
                subprocess.run(["osascript", "-e", applescript], check=True)
            else:
                print(f"🔊 {text}")
        except Exception:
            print(f"🔊 {text}")

def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        r.pause_threshold = 1
        r.adjust_for_ambient_noise(source, duration=1)
        try:
            audio = r.listen(source, timeout=5)
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("❌ Didn't catch that.")
            return None
        except sr.RequestError:
            print("❌ API error.")
            return None
        except sr.WaitTimeoutError:
            print("❌ Timed out waiting for speech.")
            return None

template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
try:
    model = OllamaLLM(model="llama3")
    prompt = ChatPromptTemplate.from_template(template)
    chain = prompt | model
except Exception:
    print("Error initializing OllamaLLM")
    model = None
    chain = None

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse"],
    "stop": ["stop", "halt", "stand still"],
    "turnLeft": ["turn left"],
    "turnRight": ["turn right"],
    "moveLeft": ["move left", "strafe left"],
    "moveRight": ["move right", "strafe right"]
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
    global keyboard_mode_active
    global exit_keyboard_mode
    global return_to_mode_selection
    print("🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Forward, ↓=Backward, ←=Turn Left, →=Turn Right")
    print("          A=Strafe Left, D=Strafe Right, SPACE=Stop, E=Exit")
    last_key_state = {
        "up": False,
        "down": False,
        "left": False,
        "right": False,
        "a": False,
        "d": False,
        "space": False
    }
    key_commands = {
        "up": "forward",
        "down": "backward",
        "left": "left",
        "right": "right",
        "a": "moveleft",
        "d": "moveright"
    }
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            if keyboard.is_pressed("e"):
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
            if return_to_mode_selection:
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
            current_key_states = {
                "up": keyboard.is_pressed("up"),
                "down": keyboard.is_pressed("down"),
                "left": keyboard.is_pressed("left"),
                "right": keyboard.is_pressed("right"),
                "a": keyboard.is_pressed("a"),
                "d": keyboard.is_pressed("d"),
                "space": keyboard.is_pressed("space")
            }
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state[key]:
                    if key == "space":
                        processImmediateCommand("stop")
                    else:
                        processImmediateCommand(key_commands[key])
            for key, is_pressed in current_key_states.items():
                if not is_pressed and last_key_state[key] and key != "space":
                    processImmediateCommand("stop")
            last_key_state = current_key_states.copy()
            time.sleep(0.02)
        except Exception:
            immediateStop()
            time.sleep(0.1)

async def process_user_input(user_input, context):
    global return_to_mode_selection
    global obstacle_detected
    if not user_input:
        return False
    instructions = [instr.strip() for instr in user_input.split("then")]
    command_sequence = []
    has_movement = False
    obstacle_detected = False  # Reset before processing
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
        for command, duration in command_sequence:
            global commandCharacter
            commandCharacter = command
            processCommand(command, duration)
            commandCharacter = ""
            if permStop or obstacle_detected or return_to_mode_selection:
                break
            await asyncio.sleep(duration / 1000)
        return_to_mode_selection = False
        return True
    return False

async def handle_conversation():
    global keyboard_mode_active
    global exit_keyboard_mode
    global return_to_mode_selection
    global obstacle_detected
    context = ""
    print("="*50 + "\n🤖 AI Omni-Wheel Robot Assistant Initialized 🤖\n" + "="*50)
    while True:
        if return_to_mode_selection:
            return_to_mode_selection = False
            obstacle_detected = False  # Reset before mode selection
        mode = input("\nChoose mode: (s)peech, (t)ype, (k)eyboard, or (q)uit? ").strip().lower()
        if mode in ['s', 'speech']:
            while True:
                user_input = listen()
                if user_input == "exit":
                    return
                if not user_input:
                    continue
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    if chain is None:
                        print("Bot: AI model not available.")
                        speak("AI model not available.")
                    else:
                        result = chain.invoke({"context": context, "question": user_input})
                        print(f"Bot: {result}")
                        speak(str(result))
                        context += f"\nUser: {user_input}\nAI: {result}"
                if return_to_mode_selection:
                    break
                continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break
        elif mode in ['t', 'type']:
            while True:
                user_input = input("You: ")
                if user_input.lower() == "exit":
                    return
                if not user_input:
                    continue
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    if chain is None:
                        print("Bot: AI model not available.")
                        speak("AI model not available.")
                    else:
                        result = chain.invoke({"context": context, "question": user_input})
                        print(f"Bot: {result}")
                        speak(str(result))
                        context += f"\nUser: {user_input}\nAI: {result}"
                if return_to_mode_selection:
                    break
                continue_mode = input("Continue text mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break
        elif mode in ['k', 'keyboard']:
            keyboard_mode_active = True
            exit_keyboard_mode = False
            keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
            keyboard_thread.start()
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            keyboard_mode_active = False
        elif mode in ['q', 'quit']:
            break
        else:
            print("❌ Invalid mode. Please choose 's', 't', 'k', or 'q'.")

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
        
