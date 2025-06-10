import re

# AI Code Setup
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate

# PYSERIAL SET UP
import serial
import time

arduino = serial.Serial('/dev/tty.usbmodem1101', 9600, timeout=1)
time.sleep(2)  # Wait for serial connection

def send_data_to_arduino(data):
    print("Sending to Arduino: ", data)
    arduino.write(data.encode())  # Send data to Arduino as bytes
    time.sleep(1)  # Wait for Arduino to process the data

    # Read the response from Arduino
    if arduino.in_waiting > 0:
        response = arduino.readline().decode('utf-8').strip()
        print("Arduino response: ", response)

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

# Time patterns to extract seconds, minutes, and hours
time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours": r"(\d+)\s*hours?"
}

# Set initial time for turnLeft and turnRight in seconds
turnLeft_seconds = 5  # Set default turnLeft time (seconds)
turnRight_seconds = 5  # Set default turnRight time (seconds)

def get_direction(user_input):
    """Check user input for direction-related commands."""
    for direction, phrases in directions.items():
        if any(phrase in user_input.lower() for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(time_input):
    """Convert time to milliseconds."""
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, time_input.lower())
        if match:
            value = int(match.group(1))
            if unit == "seconds":
                return value * 1000  # Convert seconds to milliseconds
            elif unit == "minutes":
                return value * 60000  # Convert minutes to milliseconds
            elif unit == "hours":
                return value * 3600000  # Convert hours to milliseconds
    return None

def handle_conversation():
    """Handles the conversation with the user."""
    context = ""
    print("Welcome to the AI Chatbot, Type 'exit' to quit.")
    while True:
        user_input = input("You: ")
        if user_input.lower() == "exit":
            break
        long_instruction = ""
        # Split the input into separate instructions (e.g., "go forward for 9 minutes, turn left")
        instructions = [instr.strip() for instr in user_input.split("then")]

        contain_instructions = False

        for i, instruction in enumerate(instructions):
            direction = get_direction(instruction)
            time_in_ms = convert_to_milliseconds(instruction)

            # Check for 'turnLeft' and 'turnRight' and use associated time
            if 'turn left' in instruction.lower():
                time_in_ms = turnLeft_seconds * 1000  # Convert turnLeft time to milliseconds
                direction = 'turnLeft'
            elif 'turn right' in instruction.lower():
                time_in_ms = turnRight_seconds * 1000  # Convert turnRight time to milliseconds
                direction = 'turnRight'

            if direction and time_in_ms is not None: # Direction and time are both found, output both
                long_instruction += f"{direction} {time_in_ms} "
                contain_instructions = True
            elif direction == "stop" and i == len(instructions) - 1:
                long_instruction += f"{direction} -1"
        if(contain_instructions):
            print("Bot: ", long_instruction)
            send_data_to_arduino(long_instruction)
        else:
            # If no direction or time is found, use AI to respond
            result = chain.invoke({"context": context, "question": user_input})
            print("Bot: ", result)

        # Update context to remember conversation history
        context += f"\nUser: {user_input}\nAI: {result if not direction and time_in_ms is None else (str(time_in_ms) + ' milliseconds' if time_in_ms else direction)}"

if __name__ == "__main__":
    handle_conversation()
