import speech_recognition as sr
import pyttsx3
import ollama
import json

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

def ask_llama(user_input):
    system_prompt = """
You are a robot assistant named Bumblebee.
If the user gives a command like "move forward for 5 seconds", reply ONLY with:
{"action": "forward", "duration": 5}
Use only this JSON format for robot commands.
If it's not a command, respond like a normal assistant.
"""

    response = ollama.chat(
        model='llama3',
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_input}
        ]
    )
    return response['message']['content']

# === MAIN LOOP ===
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

    # Now fully awake — handle any command or conversation
    response = ask_llama(speech)

    try:
        command = json.loads(response)
        print(f"✅ Robot command detected: {command}")
        speak("Command received.")
        # TODO: Send to robot here
    except json.JSONDecodeError:
        print(f"💬 Chatbot reply: {response}")
        speak(response)
