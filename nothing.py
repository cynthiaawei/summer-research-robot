from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import re

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
    # Check user input for direction-related commands
    for direction, phrases in directions.items():
        if any(phrase in user_input.lower() for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(time_input):
    # Try to match the input with time patterns
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
    context = ""
    print("Welcome to the AI Chatbot, Type 'exit' to quit.")
    while True:
        user_input = input("You: ")
        if user_input.lower() == "exit":
            break
        
        # Split the input into separate instructions
        instructions = [instr.strip() for instr in user_input.split(",")]

        for instruction in instructions:
            direction = get_direction(instruction)
            time_in_ms = convert_to_milliseconds(instruction)

            if direction and time_in_ms is not None:
                print(f"Bot: {direction}")
                print(f"Bot: {time_in_ms} milliseconds")  # Output the time in milliseconds
            elif direction:
                print(f"Bot: {direction}")  # Output direction if time is not mentioned
            elif time_in_ms is not None:
                print(f"Bot: {time_in_ms} milliseconds")  # Output time in milliseconds if direction is not mentioned
            else:
                # Use the chain to generate a response for other types of questions
                result = chain.invoke({"context": context, "question": user_input})
                print("Bot: ", result)
        
        # Update context
        context += f"\nUser: {user_input}\nAI: {result if not direction and time_in_ms is None else (str(time_in_ms) + ' milliseconds' if time_in_ms else direction)}"


if __name__ == "__main__":
    handle_conversation()
