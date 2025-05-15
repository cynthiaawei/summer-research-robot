import speech_recognition as sr
import pyttsx3
import json
import re
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate

# === SPEECH & TEXT-TO-SPEECH SETUP ===
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

# === LLaMA MODEL SETUP ===
template = """
Answer the question below.

Here is the conversation history: {context}

Question: {question}

Answer:
"""

model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

# === DIRECTION & TIME LOGIC ===
directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance", "go ahead"],
    "backward": ["go backward", "move backward", "move back", "reverse", "go back"],
    "right": ["turn right", "go right", "turn to the right"],
    "left": ["turn left", "go left", "turn to the left"],
    "stop": ["stop", "halt", "stop moving", "cease movement", "stand still", "wait", "idle"]
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

def convert_to_milliseconds(time_input):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, time_input.lower())
        if match:
            value = int(match.group(1))
            if unit == "seconds":
                return value * 1000
            elif unit == "minutes":
                return value * 60000
            elif unit == "hours":
                return value * 3600000
    return None

# === MAIN LOOP ===
context = ""
listening_enabled = False

while True:
    if not listening_enabled:
        input("🔁 Press Enter to wake Bumblebee...")
        print("🕯️ Waiting for 'Hey Bumblebee'...")

    speech = listen()
    if not speech:
        continue

    if not listening_enabled:
        if "hey bumblebee" in speech:
            print("✅ Wake word detected. Entering listening mode.")
            speak("I'm listening.")
            listening_enabled = True
        else:
            print("⏸️ Not listening. Say 'Hey Bumblebee' to activate.")
        continue

    if "bye bumblebee" in speech:
        print("👋 Bumblebee going to sleep.")
        speak("Goodbye. Say 'Hey Bumblebee' to wake me up again.")
        listening_enabled = False
        continue

    # === PARSE INSTRUCTION ===
    instructions = [instr.strip() for instr in speech.split(",")]
    handled = False

    for instruction in instructions:
        direction = get_direction(instruction)
        time_in_ms = convert_to_milliseconds(instruction)

        if direction and time_in_ms is not None:
            print(f"✅ Direction: {direction}")
            print(f"🕒 Duration: {time_in_ms} milliseconds")
            speak(f"Moving {direction} for {time_in_ms // 1000} seconds.")
            handled = True
        elif direction:
            print(f"✅ Direction: {direction}")
            speak(f"Moving {direction}.")
            handled = True
        elif time_in_ms is not None:
            print(f"🕒 Duration: {time_in_ms} milliseconds")
            speak(f"Duration is {time_in_ms // 1000} seconds.")
            handled = True

    if not handled:
        result = chain.invoke({"context": context, "question": speech})
        print("💬 Chatbot:", result)
        speak(result)
        context += f"\nUser: {speech}\nAI: {result}"
