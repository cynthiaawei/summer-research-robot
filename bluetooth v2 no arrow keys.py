import asyncio
import re
import threading
import keyboard
import subprocess
import time
import speech_recognition as sr
from bleak import BleakClient, BleakScanner
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import sys
import os

CHARACTERISTIC_UUID = "19B10011-E8F2-537E-4F6C-D104768A1214"
ble_client = None
ble_connected = False
current_key = None
last_command = ""
main_loop = None

def speak(text):
    """Use Windows built-in TTS to avoid COM conflicts"""
    try:
        # Method 1: Use Windows Speech API via PowerShell (most reliable)
        ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
        subprocess.run(["powershell", "-Command", ps_command], 
                      capture_output=True, check=True)
    except:
        try:
            # Method 2: Use Windows SAPI via wscript (fallback)
            vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
            subprocess.run(["wscript", "/nologo", "-"], 
                          input=vbs_script, text=True, capture_output=True)
        except:
            # Method 3: Print as fallback
            print(f"🔊 TTS: {text}")

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
        except sr.RequestError:
            print("❌ API error.")
        return None

# === AI ===
template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse"],
    "stop": ["stop", "halt", "stand still"],
    "turnLeft": ["turn left"],
    "turnRight": ["turn right"],
    "moveLeft": ["move left"],
    "moveRight": ["move right"],
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

async def send_ble_command(command):
    global ble_client
    try:
        if ble_client and ble_client.is_connected:
            print(f"📤 Sending BLE: {command.strip()}")
            await ble_client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
        else:
            print("⚠️ BLE not connected!")
    except Exception as e:
        print(f"❌ BLE write error: {e}")

def safe_send_ble_command(command):
    """Thread-safe way to send BLE commands"""
    global main_loop
    if main_loop and not main_loop.is_closed():
        asyncio.run_coroutine_threadsafe(send_ble_command(command), main_loop)

def keyboard_control():
    global current_key, last_command
    while True:
        new_command = None
        if keyboard.is_pressed("up") and current_key != "forward":
            new_command = "forward"
        elif keyboard.is_pressed("down") and current_key != "backward":
            new_command = "backward"
        elif keyboard.is_pressed("left") and current_key != "left":
            new_command = "left"
        elif keyboard.is_pressed("right") and current_key != "right":
            new_command = "right"
        elif not any([keyboard.is_pressed(k) for k in ["up", "down", "left", "right"]]) and current_key is not None:
            new_command = "stop"

        if new_command:
            safe_send_ble_command(new_command + "\n")
            current_key = new_command if new_command != "stop" else None
            last_command = new_command

        time.sleep(0.05)

async def handle_conversation():
    context = ""
    print("Welcome to the AI Chatbot, Type 'exit' to quit.")
    while True:
        mode = input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()

        # -----------------------------------
        # 1) SPEECH MODE
        # -----------------------------------
        if mode == 's':
            user_input = listen()
            if not user_input:
                continue

        # -----------------------------------
        # 2) TEXT MODE
        # -----------------------------------
        elif mode == 't':
            user_input = input("You: ")

        # -----------------------------------
        # 3) KEYBOARD / ARROW‐KEY MODE (one command per “round”)
        # -----------------------------------
        elif mode == 'k':
            print("🔹 Arrow‐key mode: press ↑, ↓, ← or → (or press 'q' to cancel).")
            cmd = None

            # Wait for exactly one arrow key (or 'q' to cancel)
            while True:
                if keyboard.is_pressed("up"):
                    cmd = "forward 5000 stop -1"
                    break
                elif keyboard.is_pressed("down"):
                    cmd = "backward 5000 stop -1"
                    break
                elif keyboard.is_pressed("left"):
                    cmd = "left 5000 stop -1"
                    break
                elif keyboard.is_pressed("right"):
                    cmd = "right 5000 stop -1"
                    break
                elif keyboard.is_pressed("space"):
                    cmd = "stop -1"
                    break
                elif keyboard.is_pressed("q"):
                    # User pressed 'q' to quit arrow mode without sending a command.
                    cmd = None
                    break

                time.sleep(0.05)

            if cmd:
                # Send exactly one BLE command, then go back to asking mode
                safe_send_ble_command(cmd + "\n")
                print(f"→ Sent: {cmd}")
            else:
                print("→ Arrow‐key mode cancelled.")

            # Immediately loop back to the top and ask for (s)/(t)/(k) again
            continue

        else:
            # Unrecognized mode; re‐ask
            continue

        # -----------------------------------
        # If we reach here, `user_input` is set (either speech or text).
        # We check if they typed “exit” to break out completely:
        # -----------------------------------
        if user_input.lower() == "exit":
            break

        # -----------------------------------
        # 4) CHECK FOR “MOVE” INSTRUCTIONS INSIDE user_input
        # -----------------------------------
        long_instruction = ""
        contain_instructions = False

        instructions = [instr.strip() for instr in user_input.split("then")]
        for instruction in instructions:
            instr_lower = instruction.lower()
            direction = get_direction(instr_lower)
            time_in_ms = convert_to_milliseconds(instr_lower)

            # Override for explicit “turn left” / “turn right”
            if 'turn left' in instr_lower:
                direction = 'turnLeft'
                time_in_ms = 5000
            elif 'turn right' in instr_lower:
                direction = 'turnRight'
                time_in_ms = 5000

            if direction and (time_in_ms is not None):
                long_instruction += f"{direction} {time_in_ms} "
                contain_instructions = True
            elif direction == "stop":
                long_instruction += "stop -1"
                contain_instructions = True

        # -----------------------------------
        # 5) SEND MOVEMENT COMMAND OR FALL BACK TO AI
        # -----------------------------------
        if contain_instructions:
            print("Bot:", long_instruction.strip())
            await send_ble_command(long_instruction.strip() + "\n")
            # After sending, loop back to ask mode again
            context += f"\nUser: {user_input}\nAI: {long_instruction.strip()}"
            continue

        # -----------------------------------
        # 6) NO MOVE KEYWORDS ⇒ HAND OFF TO LLM
        # -----------------------------------
        result = chain.invoke({"context": context, "question": user_input})
        print("Bot:", result)
        speak(str(result))

        context += f"\nUser: {user_input}\nAI: {result}"

    # End of while True

async def setup_ble():
    global ble_client, ble_connected
    max_retries = 3
    retry_count = 0

    while not ble_connected and retry_count < max_retries:
        try:
            print("🔍 Scanning for BLE devices...")
            # Scan with timeout
            devices = await asyncio.wait_for(BleakScanner.discover(timeout=10.0), timeout=15.0)
            
            # Filter for Arduino devices (you can customize this)
            arduino_devices = []
            for d in devices:
                if d.name and ("arduino" in d.name.lower() or "nano" in d.name.lower()):
                    arduino_devices.append(d)
            
            if not arduino_devices:
                print("No Arduino devices found. Showing all devices:")
                arduino_devices = [d for d in devices if d.name]  # Only named devices
            
            if not arduino_devices:
                print("No named BLE devices found!")
                retry_count += 1
                continue
                
            for i, d in enumerate(arduino_devices):
                print(f"{i}: {d.name or 'Unknown'} - {d.address}")
            
            try:
                index = int(input(f"Select device (0-{len(arduino_devices)-1}): "))
                if index < 0 or index >= len(arduino_devices):
                    raise ValueError("Invalid index")
                    
                selected_device = arduino_devices[index]
                print(f"Connecting to {selected_device.name} ({selected_device.address})...")
                
                # Connect with timeout
                client = BleakClient(selected_device.address)
                await asyncio.wait_for(client.connect(), timeout=10.0)
                
                # Verify connection and services
                if client.is_connected:
                    print(f"✅ Connected to {selected_device.name}!")
                    
                    # Check if our characteristic exists
                    services = await client.get_services()
                    char_found = False
                    for service in services:
                        for char in service.characteristics:
                            if char.uuid.upper() == CHARACTERISTIC_UUID.upper():
                                char_found = True
                                break
                        if char_found:
                            break
                    
                    if char_found:
                        print("✅ Required characteristic found!")
                        ble_client = client
                        ble_connected = True
                    else:
                        print(f"❌ Characteristic {CHARACTERISTIC_UUID} not found!")
                        await client.disconnect()
                        retry_count += 1
                else:
                    print("❌ Failed to establish connection")
                    retry_count += 1
                    
            except (ValueError, IndexError):
                print("❌ Invalid selection")
                retry_count += 1
            except asyncio.TimeoutError:
                print("❌ Connection timeout")
                retry_count += 1
            except Exception as e:
                print(f"❌ Connection error: {e}")
                retry_count += 1
                
        except asyncio.TimeoutError:
            print("❌ Device scanning timeout")
            retry_count += 1
        except Exception as e:
            print(f"⚠️ Scanning error: {e}")
            retry_count += 1
        
        if not ble_connected:
            print(f"Retrying... ({retry_count}/{max_retries})")
            await asyncio.sleep(2)
    
    if not ble_connected:
        print("❌ Failed to connect after all retries")
        return False
    
    return True

async def cleanup():
    global ble_client
    if ble_client and ble_client.is_connected:
        print("🔌 Disconnecting BLE...")
        try:
            await ble_client.disconnect()
        except Exception as e:
            print(f"Error during disconnect: {e}")

async def main():
    global main_loop
    main_loop = asyncio.get_running_loop()
    
    # Setup BLE connection
    if not await setup_ble():
        print("Cannot proceed without BLE connection")
        return
    
    # Start keyboard control in separate thread
    keyboard_thread = threading.Thread(target=keyboard_control, daemon=True)
    keyboard_thread.start()
    
    try:
        # Handle conversation in main loop
        await handle_conversation()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        await cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
