import speech_recognition as sr
import pyttsx3
import json
import re
import time
import serial
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate

# === TEXT TO SPEECH ===
speaker = pyttsx3.init()

def speak(text):
    speaker.say(text)
    speaker.runAndWait()

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
            return None
        except sr.RequestError:
            print("❌ Speech recognition error.")
            return None

# === OLLAMA SETUP ===
template = """
Answer the question below.

Here is the conversation history: {context}

Question: {question}

Answer:
"""
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

# === DIRECTION/TIME MATCHING ===
directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance", "go ahead"],
    "backward": ["go backward", "move backward", "move back", "reverse", "go back"],
    "right": ["turn right", "go right", "turn to the right"],
    "left": ["turn left", "go left", "turn to the left"],
    "moveleft": ["strafe left", "move left", "side step left"],
    "moveright": ["strafe right", "move right", "side step right"],
    "stop": ["stop", "halt", "wait", "stand still", "pause"]
}

time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours": r"(\d+)\s*hours?"
}

def get_direction(user_input):
    for direction, phrases in directions.items():
        if any(phrase in user_input for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(text):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match:
            value = int(match.group(1))
            if unit == "seconds": return value * 1000
            if unit == "minutes": return value * 60000
            if unit == "hours": return value * 3600000
    return 2000  # Default duration

# === SERIAL SETUP ===
try:
    serial_port = serial.Serial('COM9', 9600, timeout=1)  # Replace COM4 with your Arduino port
    time.sleep(2)
except Exception as e:
    print(f"❌ Could not connect to Arduino: {e}")
    serial_port = None

# === MAIN LOOP ===
context = ""
listening_enabled = False

while True:
    if not listening_enabled:
        input("🔁 Press Enter to wake Bumblebee...")
        print("🕯️ Say 'Hey Bumblebee' to activate.")

    speech = listen()
    if not speech:
        continue

    if not listening_enabled:
        if "hey bumblebee" in speech:
            speak("I'm listening.")
            listening_enabled = True
        else:
            print("⏸️ Waiting for wake word.")
        continue

    if "bye bumblebee" in speech:
        speak("Goodbye. Say 'Hey Bumblebee' to wake me up again.")
        listening_enabled = False
        continue

    # === PARSE COMMAND ===
    handled = False
    instructions = [s.strip() for s in speech.split(",")]
    for instr in instructions:
        direction = get_direction(instr)
        duration = convert_to_milliseconds(instr)

        if direction:
            print(f"✅ Command: {direction}, Time: {duration}ms")
            speak(f"{direction} for {duration // 1000} seconds")
            if serial_port:
                serial_port.write(f"{direction},{duration}\n".encode())
            handled = True

    # === FALLBACK TO CHATBOT ===
    if not handled:
        response = chain.invoke({"context": context, "question": speech})
        print("💬 Chatbot:", response)
        speak(response)
        context += f"\nUser: {speech}\nAI: {response}"
