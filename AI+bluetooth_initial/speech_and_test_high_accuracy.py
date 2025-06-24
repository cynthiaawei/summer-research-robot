import re
import speech_recognition as sr
import pyttsx3
# AI Code Setup
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
engine = pyttsx3.init()
engine.say("Testing text to speech")
engine.runAndWait()
# # Uncomment this later when Arduino is connected
# import serial
# import time
# arduino = serial.Serial('COM4', 9600, timeout=1)
# time.sleep(2)

# def send_data_to_arduino(data):
#     print("Sending to Arduino: ", data)
#     arduino.write(data.encode())  # Send data to Arduino as bytes
#     time.sleep(1)  # Wait for Arduino to process the data
#     if arduino.in_waiting > 0:
#         response = arduino.readline().decode('utf-8').strip()
#         print("Arduino response: ", response)

#TEXT TO SPEECH
speaker= pyttsx3.init()
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

# AI Setup
template = """
Answer the question below.

Here is the conversation history: {context}

Question: {question}

Answer:
"""

model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

# Direction mapping
directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance", "go ahead"],
    "backward": ["go backward", "move backward", "move back", "reverse", "go back"],
    "stop": ["stop", "halt", "stop moving", "cease movement", "stand still", "wait", "idle"],
    "turnLeft": ["turn left", "turn to the left"],
    "turnRight": ["turn right", "turn to the right"],
    "moveLeft": ["move left", "shift left", "go left"],
    "moveRight": ["move right", "shift right", "go right"],
}

time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours": r"(\d+)\s*hours?"
}

turnLeft_seconds = 5
turnRight_seconds = 5

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

def handle_conversation():

    context = ""
    print("Welcome to the AI Chatbot, Type 'exit' to quit.")
    print("You can speat or type")
    while True:
        mode=input("Choose whaty mode you want to use (s for speak/ t for type)")
        if mode=='s':
            user_input=listen()
            if not user_input:
                continue
        if mode=='t':
            user_input = input("You: ")
        if user_input.lower() == "exit":
            break
        long_instruction = ""
        instructions = [instr.strip() for instr in user_input.split("then")]
        contain_instructions = False

        for i, instruction in enumerate(instructions):
            direction = get_direction(instruction)
            time_in_ms = convert_to_milliseconds(instruction)

            if 'turn left' in instruction.lower():
                time_in_ms = turnLeft_seconds * 1000
                direction = 'turnLeft'
            elif 'turn right' in instruction.lower():
                time_in_ms = turnRight_seconds * 1000
                direction = 'turnRight'

            if direction and time_in_ms is not None:
                long_instruction += f"{direction} {time_in_ms} "
                contain_instructions = True
            elif direction == "stop" and i == len(instructions) - 1:
                long_instruction += f"{direction} "

        if contain_instructions:
            print("Bot: ", long_instruction)
            # send_data_to_arduino(long_instruction)
        else:
            result = chain.invoke({"context": context, "question": user_input})
            print("Bot: ", result)
            if result:  # Only speak if AI actually returned something
                    text = str(result)
                    speak(text)

        context += f"\nUser: {user_input}\nAI: {result if not contain_instructions else long_instruction}"

if __name__ == "__main__":
    handle_conversation()
