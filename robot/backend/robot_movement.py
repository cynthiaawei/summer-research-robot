import asyncio
import re
import threading
import time
import logging
from typing import Dict, Optional

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠️ GPIO not available - running in simulation mode")

try:
    from langchain_ollama import OllamaLLM
    from langchain_core.prompts import ChatPromptTemplate
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False
    print("⚠️ AI model not available - install langchain-ollama for AI features")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotController:
    def __init__(self):
        """Initialize the robot controller"""
        self.setup_gpio()
        self.setup_ai()
        self.reset_state()
        
    def setup_gpio(self):
        """Setup GPIO pins and PWM"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            return
            
        # Motor Pin Definitions
        self.Motor1_Speed = 38
        self.Motor1_Dir = 40
        self.Motor2_Speed = 32
        self.Motor2_Dir = 36
        self.Motor3_Speed = 16
        self.Motor3_Dir = 26
        
        # Ultrasonic Sensor Pins
        self.Echo1 = 31
        self.Echo2 = 29
        self.Echo3 = 22
        self.Trig1 = 11
        self.Trig2 = 13
        self.Trig3 = 15
        
        try:
            # GPIO Setup
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.Echo1, GPIO.IN)
            GPIO.setup(self.Echo2, GPIO.IN)
            GPIO.setup(self.Echo3, GPIO.IN)
            GPIO.setup(self.Trig1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Trig2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Trig3, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor1_Speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor1_Dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor2_Speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor2_Dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor3_Speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.Motor3_Dir, GPIO.OUT, initial=GPIO.LOW)
            
            # Set PWM frequencies
            freq = 1000
            self.Motor1_pwm = GPIO.PWM(self.Motor1_Speed, freq)
            self.Motor2_pwm = GPIO.PWM(self.Motor2_Speed, freq)
            self.Motor3_pwm = GPIO.PWM(self.Motor3_Speed, freq)
            self.Motor1_pwm.start(0)
            self.Motor2_pwm.start(0)
            self.Motor3_pwm.start(0)
            
            logger.info("GPIO initialized successfully")
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            
    def setup_ai(self):
        """Setup AI model"""
        if not AI_AVAILABLE:
            logger.warning("AI model not available")
            self.chain = None
            return
            
        try:
            template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
            model = OllamaLLM(model="llama3")
            prompt = ChatPromptTemplate.from_template(template)
            self.chain = prompt | model
            logger.info("AI model initialized successfully")
        except Exception as e:
            logger.error(f"AI model initialization failed: {e}")
            self.chain = None
            
    def reset_state(self):
        """Reset robot state"""
        self.gCurSpeed1 = 0
        self.gCurSpeed2 = 0
        self.gCurSpeed3 = 0
        self.gSliderSpeed = 25
        self.motor3_compensate = 15
        self.obstacle_detected = False
        self.obstacle_detection_active = False
        self.OBSTACLE_THRESHOLD = 30.0
        self.movement_lock = threading.Lock()
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.command_character = ""
        self.perm_stop = False
        self.return_to_mode_selection = False
        
        # Start obstacle detection thread
        self.obstacle_thread = threading.Thread(target=self.obstacle_detection_loop, daemon=True)
        self.obstacle_thread.start()
        
    def get_status(self) -> str:
        """Get current robot status"""
        if self.obstacle_detected:
            return "obstacle_detected"
        elif any([self.gCurSpeed1 > 0, self.gCurSpeed2 > 0, self.gCurSpeed3 > 0]):
            return "moving"
        else:
            return "idle"
            
    def get_current_speeds(self) -> Dict[str, int]:
        """Get current motor speeds"""
        return {
            "motor1": self.gCurSpeed1,
            "motor2": self.gCurSpeed2,
            "motor3": self.gCurSpeed3
        }
        
    def get_distance(self, trig_pin, echo_pin, timeout=0.5) -> float:
        """Get distance from ultrasonic sensor"""
        if not GPIO_AVAILABLE:
            return 100.0  # Simulation mode - no obstacles
            
        try:
            # Ensure clean state
            GPIO.output(trig_pin, False)
            time.sleep(0.000002)
            
            # Send trigger pulse
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            # Wait for echo start
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 0:
                if time.time() - timeout_start > timeout:
                    return -1
            pulse_start = time.time()
            
            # Wait for echo end
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() - timeout_start > timeout:
                    return -1
            pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound calculation
            distance = round(distance, 2)
            
            # Validate distance range
            if distance < 2 or distance > 400:
                return -1
                
            return distance
        except Exception as e:
            logger.error(f"Distance measurement error: {e}")
            return -1
            
    def obstacle_detection_loop(self):
        """Obstacle detection loop"""
        while True:
            try:
                if self.obstacle_detection_active and not self.obstacle_detected:
                    if any([self.gCurSpeed1 > 0, self.gCurSpeed2 > 0, self.gCurSpeed3 > 0]):
                        if GPIO_AVAILABLE:
                            distances = []
                            for trig, echo in [(self.Trig1, self.Echo1), (self.Trig2, self.Echo2), (self.Trig3, self.Echo3)]:
                                dist = self.get_distance(trig, echo, timeout=0.3)
                                if dist > 0:
                                    distances.append(dist)
                            
                            if distances:
                                min_distance = min(distances)
                                if min_distance < self.OBSTACLE_THRESHOLD:
                                    logger.warning(f"Obstacle detected at {min_distance}cm!")
                                    self.obstacle_detected = True
                                    self.obstacle_event.set()
                                    self.emergency_stop()
                        
                        time.sleep(0.05)
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Error in obstacle detection: {e}")
                time.sleep(0.1)
                
    def emergency_stop(self):
        """Emergency stop"""
        with self.movement_lock:
            try:
                if GPIO_AVAILABLE:
                    self.Motor1_pwm.ChangeDutyCycle(0)
                    self.Motor2_pwm.ChangeDutyCycle(0)
                    self.Motor3_pwm.ChangeDutyCycle(0)
                
                self.gCurSpeed1 = 0
                self.gCurSpeed2 = 0
                self.gCurSpeed3 = 0
                self.obstacle_detection_active = False
                self.perm_stop = True
                self.return_to_mode_selection = True
                
                logger.info("Emergency stop executed")
            except Exception as e:
                logger.error(f"Error during emergency stop: {e}")
                
    def change_speed_smooth(self, new_speed1, new_speed2, new_speed3):
        """Change motor speeds smoothly"""
        if not GPIO_AVAILABLE:
            self.gCurSpeed1 = new_speed1
            self.gCurSpeed2 = new_speed2
            self.gCurSpeed3 = new_speed3
            return
            
        with self.movement_lock:
            if self.obstacle_detected or self.interrupt_event.is_set():
                return
                
            if any([new_speed1 > 0, new_speed2 > 0, new_speed3 > 0]):
                self.obstacle_detection_active = True
            
            steps = max(abs(new_speed1 - self.gCurSpeed1), 
                       abs(new_speed2 - self.gCurSpeed2), 
                       abs(new_speed3 - self.gCurSpeed3))
            
            if steps > 0:
                for step in range(steps + 1):
                    if self.obstacle_detected or self.interrupt_event.is_set():
                        break
                        
                    progress = step / steps if steps > 0 else 1
                    speed1 = int(self.gCurSpeed1 + (new_speed1 - self.gCurSpeed1) * progress)
                    speed2 = int(self.gCurSpeed2 + (new_speed2 - self.gCurSpeed2) * progress)
                    speed3 = int(self.gCurSpeed3 + (new_speed3 - self.gCurSpeed3) * progress)
                    
                    speed1 = max(0, min(100, speed1))
                    speed2 = max(0, min(100, speed2))
                    speed3 = max(0, min(100, speed3))
                    
                    if GPIO_AVAILABLE:
                        self.Motor1_pwm.ChangeDutyCycle(speed1)
                        self.Motor2_pwm.ChangeDutyCycle(speed2)
                        self.Motor3_pwm.ChangeDutyCycle(speed3)
                    
                    time.sleep(0.01)
            
            if not self.obstacle_detected and not self.interrupt_event.is_set():
                self.gCurSpeed1 = new_speed1
                self.gCurSpeed2 = new_speed2
                self.gCurSpeed3 = new_speed3
            else:
                if GPIO_AVAILABLE:
                    self.Motor1_pwm.ChangeDutyCycle(0)
                    self.Motor2_pwm.ChangeDutyCycle(0)
                    self.Motor3_pwm.ChangeDutyCycle(0)
                self.gCurSpeed1 = 0
                self.gCurSpeed2 = 0
                self.gCurSpeed3 = 0
                
            if all([new_speed1 == 0, new_speed2 == 0, new_speed3 == 0]):
                self.obstacle_detection_active = False
                
    def move_robot(self, direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
        """Move robot in specified direction"""
        if self.obstacle_detected:
            return False
            
        if speed is None:
            speed = self.gSliderSpeed
            
        self.obstacle_detected = False
        self.interrupt_event.clear()
        
        direction_configs = {
            "forward": {
                "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW] if GPIO_AVAILABLE else [1, 1, 0],
                "speeds": [0, speed, speed + self.motor3_compensate]
            },
            "backward": {
                "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.HIGH] if GPIO_AVAILABLE else [1, 0, 1],
                "speeds": [0, speed, speed + self.motor3_compensate]
            },
            "turnLeft": {
                "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.LOW] if GPIO_AVAILABLE else [1, 0, 0],
                "speeds": [speed, speed, speed + self.motor3_compensate]
            },
            "turnRight": {
                "dirs": [GPIO.LOW, GPIO.HIGH, GPIO.HIGH] if GPIO_AVAILABLE else [0, 1, 1],
                "speeds": [speed, speed, speed + self.motor3_compensate]
            },
            "moveLeft": {
                "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW] if GPIO_AVAILABLE else [1, 1, 0],
                "speeds": [int(speed * 1.5), speed, 0]
            },
            "moveRight": {
                "dirs": [GPIO.LOW, GPIO.LOW, GPIO.HIGH] if GPIO_AVAILABLE else [0, 0, 1],
                "speeds": [int(speed * 1.5), speed, speed + self.motor3_compensate]
            },
            "stop": {
                "dirs": [GPIO.LOW, GPIO.LOW, GPIO.LOW] if GPIO_AVAILABLE else [0, 0, 0],
                "speeds": [0, 0, 0]
            }
        }
        
        if direction not in direction_configs:
            logger.error(f"Unknown direction: {direction}")
            return False
            
        config = direction_configs[direction]
        
        if GPIO_AVAILABLE:
            GPIO.output(self.Motor1_Dir, config["dirs"][0])
            GPIO.output(self.Motor2_Dir, config["dirs"][1])
            GPIO.output(self.Motor3_Dir, config["dirs"][2])
        
        self.change_speed_smooth(config["speeds"][0], config["speeds"][1], config["speeds"][2])
        
        if duration_ms is not None and duration_ms > 0:
            start_time = time.time()
            while time.time() - start_time < duration_ms / 1000:
                if self.obstacle_detected or self.interrupt_event.is_set():
                    break
                time.sleep(0.01)
            if not self.obstacle_detected and not self.interrupt_event.is_set():
                self.change_speed_smooth(0, 0, 0)
        
        return True
        
    async def execute_command(self, command: str, duration: Optional[int] = None) -> bool:
        """Execute a robot command"""
        try:
            self.command_character = command
            command_map = {
                "forward": "forward",
                "backward": "backward",
                "turnLeft": "turnLeft",
                "turnRight": "turnRight",
                "moveLeft": "moveLeft",
                "moveRight": "moveRight",
                "stop": "stop"
            }
            
            robot_direction = command_map.get(command.lower())
            if robot_direction:
                success = self.move_robot(robot_direction, duration_ms=duration)
                self.command_character = ""
                logger.info(f"Command executed successfully: {command}")
                return success
            else:
                logger.warning(f"Unknown command: {command}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing command: {e}")
            self.command_character = ""
            return False
            
    def cleanup(self):
        """Cleanup GPIO resources"""
        if GPIO_AVAILABLE:
            try:
                self.Motor1_pwm.stop()
                self.Motor2_pwm.stop()
                self.Motor3_pwm.stop()
                GPIO.cleanup()
                logger.info("GPIO cleanup completed")
            except Exception as e:
                logger.error(f"Error during GPIO cleanup: {e}")

# Global robot controller instance
robot_controller = None

def initialize_robot() -> RobotController:
    """Initialize the global robot controller"""
    global robot_controller
    if robot_controller is None:
        robot_controller = RobotController()
    return robot_controller

def process_immediate_command(command: str):
    """Process immediate movement commands"""
    global robot_controller
    if robot_controller is None:
        robot_controller = initialize_robot()
    
    command = command.strip().lower()
    robot_controller.move_robot(command)

async def process_user_input(user_input: str, context: str = "") -> bool:
    """Process user input for movement commands"""
    global robot_controller
    if robot_controller is None:
        robot_controller = initialize_robot()
    
    if not user_input:
        return False
    
    directions = {
        "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
        "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
        "stop": ["stop", "halt", "stand still", "brake", "freeze"],
        "turnLeft": ["turn left", "rotate left", "spin left"],
        "turnRight": ["turn right", "rotate right", "spin right"],
        "moveLeft": ["move left", "strafe left", "slide left", "sidestep left"],
        "moveRight": ["move right", "strafe right", "slide right", "sidestep right"]
    }
    
    time_patterns = {
        "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
        "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
        "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
    }
    
    def get_direction(text: str) -> Optional[str]:
        """Extract movement direction from text"""
        text = text.lower()
        for direction, phrases in directions.items():
            if any(phrase in text for phrase in phrases):
                return direction
        return None
    
    def convert_to_milliseconds(text: str) -> Optional[int]:
        """Convert time expressions to milliseconds"""
        text = text.lower()
        for unit, pattern in time_patterns.items():
            match = re.search(pattern, text)
            if match:
                value = float(match.group(1))
                if unit == "seconds":
                    return int(value * 1000)
                elif unit == "minutes":
                    return int(value * 60000)
                elif unit == "hours":
                    return int(value * 3600000)
        return None
    
    instructions = [instr.strip() for instr in user_input.split("then")]
    command_sequence = []
    has_movement = False
    
    robot_controller.return_to_mode_selection = False
    robot_controller.obstacle_detected = False
    
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
        logger.info(f"Executing command sequence: {command_sequence}")
        for command, duration in command_sequence:
            robot_controller.command_character = command
            success = await robot_controller.execute_command(command, duration)
            robot_controller.command_character = ""
            
            if not success or robot_controller.perm_stop or robot_controller.obstacle_detected or robot_controller.return_to_mode_selection or robot_controller.interrupt_event.is_set():
                break
                
            if duration > 0:
                await asyncio.sleep(duration / 1000)
        
        robot_controller.return_to_mode_selection = False
        robot_controller.obstacle_detected = False
        return True
    
    # Handle non-movement commands with AI if available
    if robot_controller.chain:
        try:
            result = robot_controller.chain.invoke({"context": context, "question": user_input})
            logger.info(f"AI response: {result}")
            return False  # AI response handled, no movement executed
        except Exception as e:
            logger.error(f"AI model error: {e}")
            return False
    
    return False