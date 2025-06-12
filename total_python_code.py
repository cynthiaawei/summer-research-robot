import asyncio
import re
import threading
import keyboard
import subprocess
import time
import platform
import speech_recognition as sr
import RPi.GPIO as GPIO
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate

# ====== MOTOR & SENSOR PINS (BCM numbering) ======
# adjust these to your wiring!
MOTOR1_PWM_PIN = 18   # PWM for front motor
MOTOR1_DIR_PIN = 23
MOTOR2_PWM_PIN = 19   # PWM for back‐left motor
MOTOR2_DIR_PIN = 24
MOTOR3_PWM_PIN = 13   # PWM for back‐right motor
MOTOR3_DIR_PIN = 25

# ultrasonic sensors: one TRIG, one ECHO each
ULTRA_TRIG_PINS = [16, 20, 21]
ULTRA_ECHO_PINS = [12, 26, 27]

# ====== GLOBAL STATE ======
gCurSpeed1 = gCurSpeed2 = gCurSpeed3 = 0
gSliderSpeed = 25
motor3_compensate = 15

# for ultrasonic timing:
echo_start = [0.0, 0.0, 0.0]          # timestamp when echo rose
triggered = [False, False, False]    # flags if <5cm detected

# ====== GPIO SETUP ======
GPIO.setmode(GPIO.BCM)
# motors
for pwm_pin, dir_pin in [
    (MOTOR1_PWM_PIN, MOTOR1_DIR_PIN),
    (MOTOR2_PWM_PIN, MOTOR2_DIR_PIN),
    (MOTOR3_PWM_PIN, MOTOR3_DIR_PIN),
]:
    GPIO.setup(dir_pin, GPIO.OUT)
    GPIO.setup(pwm_pin, GPIO.OUT)

# create PWM objects @1kHz
pwm1 = GPIO.PWM(MOTOR1_PWM_PIN, 1000)
pwm2 = GPIO.PWM(MOTOR2_PWM_PIN, 1000)
pwm3 = GPIO.PWM(MOTOR3_PWM_PIN, 1000)
pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

# ultrasonic
for trig in ULTRA_TRIG_PINS:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, GPIO.LOW)

for echo in ULTRA_ECHO_PINS:
    GPIO.setup(echo, GPIO.IN)

# echo callback
def echo_callback(channel):
    idx = ULTRA_ECHO_PINS.index(channel)
    if GPIO.input(channel):
        # rising
        echo_start[idx] = time.time()
    else:
        # falling
        duration = time.time() - echo_start[idx]
        dist_cm = (duration * 34300) / 2
        if dist_cm < 5.0:
            triggered[idx] = True

for echo in ULTRA_ECHO_PINS:
    GPIO.add_event_detect(echo, GPIO.BOTH, callback=echo_callback, bouncetime=1)

# ====== MOTOR CONTROL HELPERS ======
def set_motor(pwm_obj, dir_pin, speed, forward=True):
    GPIO.output(dir_pin, GPIO.HIGH if forward else GPIO.LOW)
    pwm_obj.ChangeDutyCycle(speed)

def immediate_stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    pwm3.ChangeDutyCycle(0)

def go_forwards(speed, time_ms):
    # check interrupts every 10ms
    set_motor(pwm1, MOTOR1_DIR_PIN, 0, True)
    set_motor(pwm2, MOTOR2_DIR_PIN, speed, True)
    set_motor(pwm3, MOTOR3_DIR_PIN, speed + motor3_compensate, False)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

def go_backwards(speed, time_ms):
    set_motor(pwm1, MOTOR1_DIR_PIN, 0, True)
    set_motor(pwm2, MOTOR2_DIR_PIN, speed, False)
    set_motor(pwm3, MOTOR3_DIR_PIN, speed + motor3_compensate, True)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

def turn_left(speed, time_ms):
    set_motor(pwm1, MOTOR1_DIR_PIN, speed, True)
    set_motor(pwm2, MOTOR2_DIR_PIN, speed, False)
    set_motor(pwm3, MOTOR3_DIR_PIN, speed + motor3_compensate, False)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

def turn_right(speed, time_ms):
    set_motor(pwm1, MOTOR1_DIR_PIN, speed, False)
    set_motor(pwm2, MOTOR2_DIR_PIN, speed, True)
    set_motor(pwm3, MOTOR3_DIR_PIN, speed + motor3_compensate, True)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

def move_left(speed, time_ms):
    set_motor(pwm1, MOTOR1_DIR_PIN, speed*1.5, True)
    set_motor(pwm2, MOTOR2_DIR_PIN, speed, True)
    set_motor(pwm3, MOTOR3_DIR_PIN, 0, True)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

def move_right(speed, time_ms):
    set_motor(pwm1, MOTOR1_DIR_PIN, speed*1.5, False)
    set_motor(pwm2, MOTOR2_DIR_PIN, 0, True)
    set_motor(pwm3, MOTOR3_DIR_PIN, speed + motor3_compensate, True)
    start = time.time()
    while (time.time() - start)*1000 < time_ms:
        if any(triggered):
            immediate_stop()
            break
        time.sleep(0.01)

# ====== TTS & LISTEN ======
def speak(text):
    system = platform.system().lower()
    try:
        if system=="windows":
            ps = f'Add-Type -AssemblyName System.Speech; ' \
                 f'$s=New-Object System.Speech.Synthesis.SpeechSynthesizer; ' \
                 f'$s.Speak("{text}")'
            subprocess.run(["powershell","-Command",ps], check=True)
        elif system=="darwin":
            subprocess.run(["say",text], check=True)
        else:
            subprocess.run(["espeak",text], check=True)
    except:
        print("🔊", text)

def listen():
    r=sr.Recognizer()
    with sr.Microphone() as src:
        print("🎙️ Listening...")
        audio=r.listen(src)
    try:
        txt=r.recognize_google(audio)
        print("🗣️ You said:",txt)
        return txt.lower()
    except:
        return None

# ====== AI SETUP ======
template = """Answer the question below.
Here is the conversation history: {context}
Question: {question}
Answer:"""
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

directions = {
    "forward": ["go forward","move forward","move ahead","advance"],
    "backward": ["go backward","move backward","reverse"],
    "stop": ["stop","halt","stand still"],
    "turnLeft": ["turn left"],
    "turnRight": ["turn right"],
    "moveLeft": ["move left"],
    "moveRight": ["move right"],
}
time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours":   r"(\d+)\s*hours?"
}

def get_direction(inp):
    for d,phr in directions.items():
        if any(p in inp for p in phr):
            return d
    return None

def convert_to_milliseconds(txt):
    for unit,pat in time_patterns.items():
        m=re.search(pat,txt)
        if m:
            val=int(m.group(1))
            if unit=="seconds": return val*1000
            if unit=="minutes": return val*60000
            if unit=="hours":   return val*3600000
    return None

# ====== PROCESS USER INPUT ======
async def process_user_input(user_input, context):
    long_instr=""
    got=False
    for chunk in user_input.split("then"):
        l=chunk.strip().lower()
        dirn=get_direction(l)
        tms=convert_to_milliseconds(l)
        if "turn left" in l:
            dirn, tms = "turnLeft", 5000
        if "turn right" in l:
            dirn, tms = "turnRight", 5000
        if dirn and tms is not None:
            long_instr += f"{dirn} {tms} "
            got=True
        elif dirn=="stop":
            long_instr += "stop -1 "
            got=True

    if got:
        print("Parsed move:", long_instr.strip())
        parts=long_instr.strip().split()
        for cmd, ms in zip(parts[0::2], map(int,parts[1::2])):
            {
              "forward":  go_forwards,
              "backward": go_backwards,
              "turnLeft": turn_left,
              "turnRight":turn_right,
              "moveLeft": move_left,
              "moveRight":move_right,
              "stop":     lambda s,d: immediate_stop()
            }[cmd](gSliderSpeed, ms)
        return True
    return False

# ====== CONTINUOUS KEYBOARD MODE ======
keyboard_mode_active=False
exit_keyboard_mode=False

def keyboard_control_continuous():
    global keyboard_mode_active, exit_keyboard_mode
    print("🎮 Keyboard mode: ↑↓←→ + SPACE to stop, E to exit")
    last_state={k:False for k in ("up","down","left","right","space")}
    mapping={
      "up":    startForward,
      "down":  startBackward,
      "left":  startTurnLeft,
      "right": startTurnRight
    }
    while keyboard_mode_active and not exit_keyboard_mode:
        if keyboard.is_pressed("e"):
            immediate_stop()
            exit_keyboard_mode=True
            break
        curr={k:keyboard.is_pressed(k) for k in last_state}
        for k,pressed in curr.items():
            if pressed and not last_state[k]:
                if k=="space":
                    immediate_stop()
                else:
                    mapping[k]()
        for k,pressed in curr.items():
            if not pressed and last_state[k] and k!="space":
                immediate_stop()
        last_state=curr
        time.sleep(0.02)

# ====== CONVERSATION LOOP ======
async def handle_conversation():
    context=""
    print("Welcome! Type exit to quit.")
    while True:
        mode=input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()
        if mode=="s":
            while True:
                ui=listen()
                if not ui: continue
                if ui=="exit": return
                if await process_user_input(ui,context):
                    context+=f"\nUser:{ui}\nAI:[move]"
                else:
                    res=chain.invoke({"context":context,"question":ui})
                    print("Bot:",res)
                    speak(res)
                    context+=f"\nUser:{ui}\nAI:{res}"
                if input("Continue speech? (y/n): ").strip().lower()!="y":
                    break
        elif mode=="t":
            while True:
                ui=input("You: ")
                if ui=="exit": return
                if await process_user_input(ui,context):
                    context+=f"\nUser:{ui}\nAI:[move]"
                else:
                    res=chain.invoke({"context":context,"question":ui})
                    print("Bot:",res)
                    speak(res)
                    context+=f"\nUser:{ui}\nAI:{res}"
                if input("Continue text? (y/n): ").strip().lower()!="y":
                    break
        elif mode=="k":
            global keyboard_mode_active, exit_keyboard_mode
            keyboard_mode_active=True
            exit_keyboard_mode=False
            t=threading.Thread(target=keyboard_control_continuous,daemon=True)
            t.start()
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            keyboard_mode_active=False
        else:
            print("Invalid mode.")

async def main():
    try:
        await handle_conversation()
    finally:
        GPIO.cleanup()

if __name__=="__main__":
    asyncio.run(main())
