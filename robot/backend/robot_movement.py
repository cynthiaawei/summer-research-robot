# robot_movement.py - Fixed version
import asyncio
import re
import threading
import time
import logging
from typing import Dict, Optional, List, Tuple, Union
from dataclasses import dataclass
from enum import Enum
import json

# Optional imports with graceful fallbacks
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

try:
    from langchain_ollama import OllamaLLM
    from langchain_core.prompts import ChatPromptTemplate
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotStatus(Enum):
    """Robot status enumeration"""
    IDLE = "idle"
    MOVING = "moving"
    OBSTACLE_DETECTED = "obstacle_detected"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"

class MovementDirection(Enum):
    """Movement direction enumeration"""
    FORWARD = "forward"
    BACKWARD = "backward"
    TURN_LEFT = "turnleft"
    TURN_RIGHT = "turnright"
    MOVE_LEFT = "moveleft"
    MOVE_RIGHT = "moveright"
    STOP = "stop"

@dataclass
class RobotConfig:
    """Robot configuration dataclass"""
    # Motor pins
    motor1_speed: int = 38
    motor1_dir: int = 40
    motor2_speed: int = 32
    motor2_dir: int = 36
    motor3_speed: int = 16
    motor3_dir: int = 26
    
    # Ultrasonic sensor pins
    echo1: int = 31
    echo2: int = 29
    echo3: int = 22
    trig1: int = 11
    trig2: int = 13
    trig3: int = 15
    
    # Motor settings
    pwm_frequency: int = 1000
    default_speed: int = 25
    motor3_compensate: int = 15
    
    # Obstacle detection
    obstacle_threshold: float = 30.0
    sensor_timeout: float = 0.5
    
    # AI model
    ai_model: str = "llama3"

@dataclass
class RobotState:
    """Robot state dataclass"""
    motor1_speed: int = 0
    motor2_speed: int = 0
    motor3_speed: int = 0
    status: RobotStatus = RobotStatus.IDLE
    obstacle_detected: bool = False
    last_distances: List[float] = None
    last_command: str = ""
    uptime: float = 0.0
    
    def __post_init__(self):
        if self.last_distances is None:
            self.last_distances = []

class RobotController:
    """Main robot controller class for backend integration"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """Initialize robot controller with configuration"""
        self.config = config or RobotConfig()
        self.state = RobotState()
        self.start_time = time.time()
        
        # Threading controls
        self.movement_lock = threading.Lock()
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Initialize components
        self._setup_gpio()
        self._setup_ai()
        self._start_background_tasks()
        
        logger.info("Robot controller initialized successfully")
    
    def _setup_gpio(self):
        """Setup GPIO pins and PWM"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            self.gpio_initialized = False
            return
        
        try:
            # GPIO Setup
            GPIO.setmode(GPIO.BOARD)
            
            # Setup ultrasonic sensors
            GPIO.setup(self.config.echo1, GPIO.IN)
            GPIO.setup(self.config.echo2, GPIO.IN)
            GPIO.setup(self.config.echo3, GPIO.IN)
            GPIO.setup(self.config.trig1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig3, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup motors
            motor_pins = [
                (self.config.motor1_speed, self.config.motor1_dir),
                (self.config.motor2_speed, self.config.motor2_dir),
                (self.config.motor3_speed, self.config.motor3_dir)
            ]
            
            for speed_pin, dir_pin in motor_pins:
                GPIO.setup(speed_pin, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup PWM
            self.motor1_pwm = GPIO.PWM(self.config.motor1_speed, self.config.pwm_frequency)
            self.motor2_pwm = GPIO.PWM(self.config.motor2_speed, self.config.pwm_frequency)
            self.motor3_pwm = GPIO.PWM(self.config.motor3_speed, self.config.pwm_frequency)
            
            for pwm in [self.motor1_pwm, self.motor2_pwm, self.motor3_pwm]:
                pwm.start(0)
            
            self.gpio_initialized = True
            logger.info("GPIO initialized successfully")
            
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            self.gpio_initialized = False
    
    def _setup_ai(self):
        """Setup AI model for conversation"""
        if not AI_AVAILABLE:
            logger.warning("AI not available - conversational features disabled")
            self.ai_chain = None
            return
        
        try:
            template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
            model = OllamaLLM(model=self.config.ai_model)
            prompt = ChatPromptTemplate.from_template(template)
            self.ai_chain = prompt | model
            logger.info("AI model initialized successfully")
        except Exception as e:
            logger.error(f"AI initialization failed: {e}")
            self.ai_chain = None
    
    def _start_background_tasks(self):
        """Start background monitoring tasks"""
        self.obstacle_thread = threading.Thread(target=self._obstacle_detection_loop, daemon=True)
        self.obstacle_thread.start()
        
        self.status_thread = threading.Thread(target=self._status_update_loop, daemon=True)
        self.status_thread.start()
    
    def _obstacle_detection_loop(self):
        """Background obstacle detection loop"""
        while not self.shutdown_event.is_set():
            try:
                if self._is_moving() and not self.state.obstacle_detected:
                    distances = self._get_all_distances()
                    
                    if distances:
                        min_distance = min(distances)
                        self.state.last_distances = distances
                        
                        if min_distance < self.config.obstacle_threshold:
                            logger.warning(f"Obstacle detected at {min_distance}cm!")
                            self.state.obstacle_detected = True
                            self.state.status = RobotStatus.OBSTACLE_DETECTED
                            self.obstacle_event.set()
                            self.emergency_stop()
                    
                    time.sleep(0.05)
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Error in obstacle detection: {e}")
                time.sleep(0.1)
    
    def _status_update_loop(self):
        """Background status update loop"""
        while not self.shutdown_event.is_set():
            try:
                self.state.uptime = time.time() - self.start_time
                
                # Update status based on current state
                if self.state.obstacle_detected:
                    self.state.status = RobotStatus.OBSTACLE_DETECTED
                elif self._is_moving():
                    self.state.status = RobotStatus.MOVING
                else:
                    self.state.status = RobotStatus.IDLE
                
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in status update: {e}")
                time.sleep(0.1)
    
    def _is_moving(self) -> bool:
        """Check if robot is currently moving"""
        return any([
            self.state.motor1_speed > 0,
            self.state.motor2_speed > 0,
            self.state.motor3_speed > 0
        ])
    
    def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
        """Get distance from a single ultrasonic sensor"""
        if not self.gpio_initialized:
            return 100.0  # Simulation mode
        
        try:
            # Clean state
            GPIO.output(trig_pin, False)
            time.sleep(0.000002)
            
            # Send trigger pulse
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            # Wait for echo
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 0:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return -1
            pulse_start = time.time()
            
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return -1
            pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
            
            return distance if 2 <= distance <= 400 else -1
            
        except Exception as e:
            logger.error(f"Distance measurement error: {e}")
            return -1
    
    def _get_all_distances(self) -> List[float]:
        """Get distances from all sensors"""
        sensor_configs = [
            (self.config.trig1, self.config.echo1),
            (self.config.trig2, self.config.echo2),
            (self.config.trig3, self.config.echo3)
        ]
        
        distances = []
        for trig, echo in sensor_configs:
            dist = self._get_distance(trig, echo)
            if dist > 0:
                distances.append(dist)
        
        return distances
    
    def _change_speeds_smooth(self, new_speeds: Tuple[int, int, int]):
        """Smoothly change motor speeds"""
        if not self.gpio_initialized:
            self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
            return
        
        with self.movement_lock:
            if self.state.obstacle_detected or self.interrupt_event.is_set():
                return
            
            current_speeds = (self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed)
            steps = max(abs(new_speeds[i] - current_speeds[i]) for i in range(3))
            
            if steps > 0:
                for step in range(steps + 1):
                    if self.state.obstacle_detected or self.interrupt_event.is_set():
                        break
                    
                    progress = step / steps if steps > 0 else 1
                    speeds = [
                        int(current_speeds[i] + (new_speeds[i] - current_speeds[i]) * progress)
                        for i in range(3)
                    ]
                    
                    # Clamp speeds
                    speeds = [max(0, min(100, speed)) for speed in speeds]
                    
                    # Apply PWM
                    self.motor1_pwm.ChangeDutyCycle(speeds[0])
                    self.motor2_pwm.ChangeDutyCycle(speeds[1])
                    self.motor3_pwm.ChangeDutyCycle(speeds[2])
                    
                    time.sleep(0.01)
            
            # Update state
            if not self.state.obstacle_detected and not self.interrupt_event.is_set():
                self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
            else:
                self._stop_motors_immediate()
    
    def _stop_motors_immediate(self):
        """Immediately stop all motors"""
        if self.gpio_initialized:
            self.motor1_pwm.ChangeDutyCycle(0)
            self.motor2_pwm.ChangeDutyCycle(0)
            self.motor3_pwm.ChangeDutyCycle(0)
        
        self.state.motor1_speed = 0
        self.state.motor2_speed = 0
        self.state.motor3_speed = 0
    
    # Public API methods
    
    def get_status(self) -> Dict:
        """Get current robot status"""
        return {
            "status": self.state.status.value,
            "message": "Robot operational" if not self.state.obstacle_detected else "Obstacle detected",
            "obstacle_detected": self.state.obstacle_detected,
            "current_speeds": {
                "motor1": self.state.motor1_speed,
                "motor2": self.state.motor2_speed,
                "motor3": self.state.motor3_speed
            },
            "last_distances": self.state.last_distances,
            "last_command": self.state.last_command,
            "uptime": self.state.uptime,
            "gpio_available": self.gpio_initialized
        }
    
    def get_sensor_data(self) -> Dict:
        """Get current sensor readings"""
        distances = self._get_all_distances()
        return {
            "distances": distances,
            "min_distance": min(distances) if distances else None,
            "obstacle_threshold": self.config.obstacle_threshold,
            "obstacle_detected": self.state.obstacle_detected
        }
    
    def move(self, direction: Union[str, MovementDirection], speed: Optional[int] = None, 
             duration_ms: Optional[int] = None) -> bool:
        """Move robot in specified direction"""
        # Map frontend directions to backend directions
        direction_map = {
            "up": "forward",
            "down": "backward", 
            "left": "turnleft",
            "right": "turnright",
            "forward": "forward",
            "backward": "backward",
            "turnleft": "turnleft",
            "turnright": "turnright",
            "moveleft": "moveleft",
            "moveright": "moveright",
            "stop": "stop"
        }
        
        if isinstance(direction, str):
            # Map the direction if needed
            mapped_direction = direction_map.get(direction.lower(), direction.lower())
            try:
                direction = MovementDirection(mapped_direction)
            except ValueError:
                logger.error(f"Invalid direction: {direction}")
                return False
        
        if self.state.obstacle_detected and direction != MovementDirection.STOP:
            logger.warning("Movement blocked - obstacle detected")
            return False
        
        speed = speed or self.config.default_speed
        self.state.last_command = direction.value
        
        # Reset states
        self.state.obstacle_detected = False
        self.interrupt_event.clear()
        
        # Direction configurations
        direction_configs = {
            MovementDirection.FORWARD: {
                "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW] if self.gpio_initialized else [1, 1, 0],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.BACKWARD: {
                "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.HIGH] if self.gpio_initialized else [1, 0, 1],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.TURN_LEFT: {
                "dirs": [GPIO.HIGH, GPIO.LOW, GPIO.LOW] if self.gpio_initialized else [1, 0, 0],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.TURN_RIGHT: {
                "dirs": [GPIO.LOW, GPIO.HIGH, GPIO.HIGH] if self.gpio_initialized else [0, 1, 1],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.MOVE_LEFT: {
                "dirs": [GPIO.HIGH, GPIO.HIGH, GPIO.LOW] if self.gpio_initialized else [1, 1, 0],
                "speeds": [int(speed * 1.5), speed, 0]
            },
            MovementDirection.MOVE_RIGHT: {
                "dirs": [GPIO.LOW, GPIO.LOW, GPIO.HIGH] if self.gpio_initialized else [0, 0, 1],
                "speeds": [int(speed * 1.5), speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.STOP: {
                "dirs": [GPIO.LOW, GPIO.LOW, GPIO.LOW] if self.gpio_initialized else [0, 0, 0],
                "speeds": [0, 0, 0]
            }
        }
        
        config = direction_configs[direction]
        
        # Set motor directions
        if self.gpio_initialized:
            GPIO.output(self.config.motor1_dir, config["dirs"][0])
            GPIO.output(self.config.motor2_dir, config["dirs"][1])
            GPIO.output(self.config.motor3_dir, config["dirs"][2])
        
        # Apply speeds
        self._change_speeds_smooth(tuple(config["speeds"]))
        
        # Handle duration
        if duration_ms is not None and duration_ms > 0:
            time.sleep(duration_ms / 1000)
            if not self.state.obstacle_detected:
                self._change_speeds_smooth((0, 0, 0))
        
        return True
    
    def stop(self) -> bool:
        """Stop the robot"""
        return self.move(MovementDirection.STOP)
    
    def emergency_stop(self) -> bool:
        """Emergency stop with state reset"""
        logger.warning("Emergency stop activated")
        self.state.status = RobotStatus.EMERGENCY_STOP
        self._stop_motors_immediate()
        return True
    
    def reset_obstacle_detection(self) -> bool:
        """Reset obstacle detection state"""
        self.state.obstacle_detected = False
        self.obstacle_event.clear()
        logger.info("Obstacle detection reset")
        return True
    
    def set_speed(self, speed: int) -> bool:
        """Set default movement speed"""
        if 0 <= speed <= 100:
            self.config.default_speed = speed
            logger.info(f"Default speed set to {speed}")
            return True
        return False
    
    def parse_command(self, command: str) -> Optional[Tuple[str, Optional[int]]]:
        """Parse natural language command to movement direction and duration"""
        command = command.lower().strip()
        
        directions = {
            "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
            "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
            "stop": ["stop", "halt", "stand still", "brake", "freeze"],
            "turnleft": ["turn left", "rotate left", "spin left"],
            "turnright": ["turn right", "rotate right", "spin right"],
            "moveleft": ["move left", "strafe left", "slide left", "sidestep left"],
            "moveright": ["move right", "strafe right", "slide right", "sidestep right"]
        }
        
        time_patterns = {
            "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
            "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
            "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
        }
        
        # Find direction
        direction = None
        for dir_key, phrases in directions.items():
            if any(phrase in command for phrase in phrases):
                direction = dir_key
                break
        
        # Find duration
        duration_ms = None
        for unit, pattern in time_patterns.items():
            match = re.search(pattern, command)
            if match:
                value = float(match.group(1))
                if unit == "seconds":
                    duration_ms = int(value * 1000)
                elif unit == "minutes":
                    duration_ms = int(value * 60000)
                elif unit == "hours":
                    duration_ms = int(value * 3600000)
                break
        
        return (direction, duration_ms) if direction else None
    
    async def execute_command_sequence(self, commands: List[str]) -> bool:
        """Execute a sequence of commands"""
        for command in commands:
            parsed = self.parse_command(command)
            if parsed:
                direction, duration = parsed
                if not self.move(direction, duration_ms=duration):
                    return False
                
                if duration:
                    await asyncio.sleep(duration / 1000)
                    
                if self.state.obstacle_detected:
                    break
        
        return True
    
    async def chat(self, message: str, context: str = "") -> str:
        """Chat with AI model"""
        if not self.ai_chain:
            return "AI model not available. Please check Ollama installation."
        
        try:
            result = self.ai_chain.invoke({"context": context, "question": message})
            return str(result)
        except Exception as e:
            logger.error(f"AI chat error: {e}")
            return "Error processing message with AI model."
    
    def get_config(self) -> Dict:
        """Get current configuration"""
        return {
            "motor_pins": {
                "motor1": {"speed": self.config.motor1_speed, "dir": self.config.motor1_dir},
                "motor2": {"speed": self.config.motor2_speed, "dir": self.config.motor2_dir},
                "motor3": {"speed": self.config.motor3_speed, "dir": self.config.motor3_dir}
            },
            "sensor_pins": {
                "sensor1": {"trig": self.config.trig1, "echo": self.config.echo1},
                "sensor2": {"trig": self.config.trig2, "echo": self.config.echo2},
                "sensor3": {"trig": self.config.trig3, "echo": self.config.echo3}
            },
            "settings": {
                "default_speed": self.config.default_speed,
                "obstacle_threshold": self.config.obstacle_threshold,
                "pwm_frequency": self.config.pwm_frequency
            }
        }
    
    def shutdown(self):
        """Shutdown robot controller"""
        logger.info("Shutting down robot controller...")
        
        # Signal shutdown
        self.shutdown_event.set()
        
        # Stop all motors
        self._stop_motors_immediate()
        
        # Cleanup GPIO
        if self.gpio_initialized:
            try:
                self.motor1_pwm.stop()
                self.motor2_pwm.stop()
                self.motor3_pwm.stop()
                GPIO.cleanup()
                logger.info("GPIO cleanup completed")
            except Exception as e:
                logger.error(f"GPIO cleanup error: {e}")
        
        logger.info("Robot controller shutdown complete")


# Global robot controller instance
_robot_controller = None

def get_robot_controller():
    """Get or create the global robot controller instance"""
    global _robot_controller
    if _robot_controller is None:
        _robot_controller = RobotController()
    return _robot_controller

# Public API functions for FastAPI backend compatibility
def get_status() -> Dict:
    """Get current robot status - called by FastAPI"""
    return get_robot_controller().get_status()

def move(direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
    """Move robot in specified direction - called by FastAPI"""
    return get_robot_controller().move(direction, speed, duration_ms)

def stop() -> bool:
    """Stop the robot - called by FastAPI"""
    return get_robot_controller().stop()

def parse_command(command: str) -> Optional[Tuple[str, Optional[int]]]:
    """Parse natural language command - called by FastAPI"""
    return get_robot_controller().parse_command(command)

async def chat(message: str, context: str = "") -> str:
    """Chat with AI model - called by FastAPI"""
    return await get_robot_controller().chat(message, context)

def emergency_stop() -> bool:
    """Emergency stop - called by FastAPI"""
    return get_robot_controller().emergency_stop()

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection - called by FastAPI"""
    return get_robot_controller().reset_obstacle_detection()

# Factory function for easy initialization
def create_robot(config: Optional[RobotConfig] = None) -> RobotController:
    """Create a robot controller instance"""
    return RobotController(config)