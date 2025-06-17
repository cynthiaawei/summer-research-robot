import asyncio
import re
import threading
import keyboard
import subprocess
import time
import platform
import speech_recognition as sr
from bleak import BleakClient, BleakScanner
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import sys
import simple_facerec as FR

CHARACTERISTIC_UUID = "19B10011-E8F2-537E-4F6C-D104768A1214"
ble_client = None
ble_connected = False
current_key = None
last_command = ""
main_loop = None
keyboard_mode_active = False
exit_keyboard_mode = False

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

def keyboard_control_continuous():
    """Continuous keyboard control mode with immediate response"""
    global current_key, last_command, keyboard_mode_active, exit_keyboard_mode
    
    print("🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Forward, ↓=Backward, ←=Left, →=Right, SPACE=Stop, E=Exit")
    
    last_key_state = {
        "up": False,
        "down": False, 
        "left": False,
        "right": False,
        "space": False
    }
    
    # Mapping keys to immediate movement commands (no duration)
    key_commands = {
        "up": "forward",
        "down": "backward", 
        "left": "left",
        "right": "right"
    }
    
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            # Check for exit key
            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode...")
                # Send stop command before exiting
                safe_send_ble_command("stop\n")
                exit_keyboard_mode = True
                break
            
            # Check each key state
            current_key_states = {
                "up": keyboard.is_pressed("up"),
                "down": keyboard.is_pressed("down"),
                "left": keyboard.is_pressed("left"), 
                "right": keyboard.is_pressed("right"),
                "space": keyboard.is_pressed("space")
            }
            
            # Handle key press events (key just pressed)
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state[key]:
                    # Key just pressed - start movement
                    if key == "space":
                        safe_send_ble_command("stop\n")
                        print("🛑 STOP pressed")
                    else:
                        command = key_commands[key]
                        safe_send_ble_command(f"{command}\n")
                        print(f"▶️ {key.upper()} pressed → {command}")
            
            # Handle key release events (key just released)
            for key, is_pressed in current_key_states.items():
                if not is_pressed and last_key_state[key]:
                    # Key just released - stop movement (except for space key)
                    if key != "space":
                        safe_send_ble_command("stop\n")
                        print(f"⏹️ {key.upper()} released → stop")
            
            # Update previous key states
            last_key_state = current_key_states.copy()
            
            time.sleep(0.02)  # 50Hz polling rate for responsive control
            
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}")
            # Continue running even if there's an error
            time.sleep(0.1)

async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode
    context = ""
    print("Welcome to the Robot! Type 'exit' to quit.")
    
    while True:
        mode = input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()

        # -----------------------------------
        # 1) SPEECH MODE
        # -----------------------------------
        if mode == 's':
            name = FR.findMatch("s")
            result = f"Hi {name}, what would you like to ask today?"
            print(result)
            FR.speak(str(result))
            
            while True:
                user_input = FR.listen()
                if not user_input:
                    continue
                    
                if user_input.lower() == "exit":
                    return  # Exit completely
                
                # Process the input (movement or AI)
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    # AI response
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    FR.speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                new_name = FR.findMatch("s")
                if new_name != name: break
                # Ask if they want to continue in speech mode or switch
                continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
                new_name = FR.findMatch("s")
                if new_name != name: break
                if continue_mode == 'n':
                    break

        # -----------------------------------
        # 2) TEXT MODE  
        # -----------------------------------
        elif mode == 't':
            name = FR.findMatch("t")
            print(f"Hi {name}, what would you like to ask today?")
            while True:
                user_input = input("You: ")
                
                if user_input.lower() == "exit":
                    return  # Exit completely
                
                # Process the input (movement or AI)
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    # AI response
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    FR.speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                
                # Ask if they want to continue in text mode or switch
                new_name = FR.findMatch("t")
                if new_name != name: break
                continue_mode = input("Continue text mode? (y/n): ").strip().lower()
                new_name = FR.findMatch("t")
                if new_name != name: break
                if continue_mode == 'n':
                    break

        # -----------------------------------
        # 3) KEYBOARD MODE (Continuous)
        # -----------------------------------
        elif mode == 'k':
            keyboard_mode_active = True
            exit_keyboard_mode = False
            
            # Start keyboard control in a separate thread
            keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
            keyboard_thread.start()
            
            # Wait for exit signal
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            
            keyboard_mode_active = False
            print("🔄 Returning to mode selection...")
            
        else:
            print("❌ Invalid mode. Please choose 's', 't', or 'k'.")
            continue

async def process_user_input(user_input, context):
    """Process user input for movement commands. Returns True if movement command was processed."""
    long_instruction = ""
    contain_instructions = False

    instructions = [instr.strip() for instr in user_input.split("then")]
    for instruction in instructions:
        instr_lower = instruction.lower()
        direction = get_direction(instr_lower)
        time_in_ms = convert_to_milliseconds(instr_lower)

        # Override for explicit "turn left" / "turn right"
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

    if contain_instructions:
        print("Bot:", long_instruction.strip())
        await send_ble_command(long_instruction.strip() + "\n")
        return True
    
    return False

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
                if d.name and ("arduino" in d.name.lower() or "nano" in d.name.lower() or "robot" in d.name.lower()):
                    arduino_devices.append(d)
            
            if not arduino_devices:
                print("No Arduino/Robot devices found. Showing all devices:")
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
    global ble_client, keyboard_mode_active
    keyboard_mode_active = False
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
    # if not await setup_ble():
    #     print("Cannot proceed without BLE connection")
    #     return
    
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
