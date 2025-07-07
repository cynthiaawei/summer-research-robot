# robot_movement.py - Complete fixed version with proper threading, error handling, and state management
import asyncio
import re
import threading
import time
import logging
from typing import Dict, Optional, List, Tuple, Union
from dataclasses import dataclass, field
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

# Constants
SOUND_SPEED_CM_PER_SEC = 34300  # Speed of sound in cm/s
DISTANCE_CALCULATION_FACTOR = SOUND_SPEED_CM_PER_SEC / 2  # Divide by 2 for round trip
MIN_VALID_DISTANCE_CM = 2
MAX_VALID_DISTANCE_CM = 400
DEFAULT_SENSOR_ERROR_DISTANCE = -1
OBSTACLE_CHECK_INTERVAL = 0.1  # seconds
STATUS_UPDATE_INTERVAL = 0.1   # seconds
SMOOTH_SPEED_STEP_DELAY = 0.01  # seconds

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
    """Robot configuration dataclass with proper motor compensation"""
    # Motor pins - matching your old code exactly
    motor1_speed: int = 38  # Motor1_Speed
    motor1_dir: int = 40    # Motor1_Dir
    motor2_speed: int = 32  # Motor2_Speed
    motor2_dir: int = 36    # Motor2_Dir
    motor3_speed: int = 16  # Motor3_Speed
    motor3_dir: int = 26    # Motor3_Dir
    
    # Ultrasonic sensor pins - matching your old code exactly
    echo1: int = 31  # Echo1
    echo2: int = 29  # Echo2
    echo3: int = 22  # Echo3
    trig1: int = 11  # Trig1
    trig2: int = 13  # Trig2
    trig3: int = 15  # Trig3
    
    # Motor settings - matching your old code exactly
    pwm_frequency: int = 1000  # freq = 1000
    default_speed: int = 25    # gSliderSpeed = 25
    motor3_compensate: int = 3 # motor3_compensate = 15
    
    # Obstacle detection - matching your old code
    obstacle_threshold: float = 30.0  # OBSTACLE_THRESHOLD = 30.0
    sensor_timeout: float = 0.5
    obstacle_clear_threshold: float = 50.0  # Distance to clear obstacle status
    
    # AI model
    ai_model: str = "llama3"

@dataclass
class RobotState:
    """Robot state dataclass with proper initialization"""
    motor1_speed: int = 0  # gCurSpeed1
    motor2_speed: int = 0  # gCurSpeed2
    motor3_speed: int = 0  # gCurSpeed3
    status: RobotStatus = RobotStatus.IDLE
    obstacle_detected: bool = False
    last_distances: List[float] = field(default_factory=list)
    sensor_distances: Dict[str, float] = field(default_factory=lambda: {"front": 0.0, "left": 0.0, "right": 0.0})
    obstacle_sensor: str = ""  # Which sensor detected the obstacle
    obstacle_distance: float = 0.0  # Distance of detected obstacle
    last_command: str = ""
    uptime: float = 0.0

class RobotController:
    """Main robot controller class for backend integration with fixed threading and error handling"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """Initialize robot controller with configuration"""
        self.config = config or RobotConfig()
        self.state = RobotState()
        self.start_time = time.time()
        
        # Threading controls with proper initialization
        self.movement_lock = threading.RLock()  # Use RLock to allow recursive locking
        self.state_lock = threading.RLock()     # Separate lock for state updates
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Thread references for proper cleanup
        self.obstacle_thread = None
        self.status_thread = None
        
        # PWM object references for cleanup
        self.motor1_pwm = None
        self.motor2_pwm = None
        self.motor3_pwm = None
        
        # Define direction constants that work in both modes
        self.HIGH = GPIO.HIGH if GPIO_AVAILABLE else 1
        self.LOW = GPIO.LOW if GPIO_AVAILABLE else 0
        
        # Initialize components
        self.gpio_initialized = False
        self._setup_gpio()
        self._setup_ai()
        self._start_background_tasks()
        
        logger.info("Robot controller initialized successfully")
    
    def _setup_gpio(self):
        """Setup GPIO pins and PWM with proper error handling - matching old code structure"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            self.gpio_initialized = False
            return
        
        pwm_objects = []
        try:
            # GPIO Setup - exactly like old code
            GPIO.setmode(GPIO.BOARD)
            
            # Setup ultrasonic sensors - exactly like old code
            GPIO.setup(self.config.echo1, GPIO.IN)
            GPIO.setup(self.config.echo2, GPIO.IN)
            GPIO.setup(self.config.echo3, GPIO.IN)
            GPIO.setup(self.config.trig1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig3, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup motors - exactly like old code
            GPIO.setup(self.config.motor1_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor1_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_dir, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup PWM with proper cleanup tracking - exactly like old code
            self.motor1_pwm = GPIO.PWM(self.config.motor1_speed, self.config.pwm_frequency)
            pwm_objects.append(self.motor1_pwm)
            
            self.motor2_pwm = GPIO.PWM(self.config.motor2_speed, self.config.pwm_frequency)
            pwm_objects.append(self.motor2_pwm)
            
            self.motor3_pwm = GPIO.PWM(self.config.motor3_speed, self.config.pwm_frequency)
            pwm_objects.append(self.motor3_pwm)
            
            # Start all PWM - exactly like old code
            self.motor1_pwm.start(0)
            self.motor2_pwm.start(0)
            self.motor3_pwm.start(0)
            
            self.gpio_initialized = True
            logger.info("GPIO initialized successfully")
            
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            # Cleanup any partially initialized PWM
            for pwm in pwm_objects:
                try:
                    if hasattr(pwm, 'stop'):
                        pwm.stop()
                except Exception as cleanup_error:
                    logger.error(f"PWM cleanup error: {cleanup_error}")
            
            # Reset PWM references
            self.motor1_pwm = None
            self.motor2_pwm = None
            self.motor3_pwm = None
            self.gpio_initialized = False
    
    def _setup_ai(self):
        """Setup AI model for conversation - matching old code AI setup"""
        if not AI_AVAILABLE:
            logger.warning("AI not available - conversational features disabled")
            self.ai_chain = None
            return
        
        try:
            # Exactly like old code AI setup
            template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
            model = OllamaLLM(model=self.config.ai_model)
            prompt = ChatPromptTemplate.from_template(template)
            self.ai_chain = prompt | model
            logger.info("AI model initialized successfully")
        except Exception as e:
            logger.error(f"AI initialization failed: {e}")
            self.ai_chain = None
    
    def _start_background_tasks(self):
        """Start background monitoring tasks with proper thread management"""
        try:
            self.obstacle_thread = threading.Thread(
                target=self._obstacle_detection_loop, 
                daemon=True, 
                name="ObstacleDetection"
            )
            self.obstacle_thread.start()
            
            self.status_thread = threading.Thread(
                target=self._status_update_loop, 
                daemon=True, 
                name="StatusUpdate"
            )
            self.status_thread.start()
            
            logger.info("Background tasks started successfully")
        except Exception as e:
            logger.error(f"Failed to start background tasks: {e}")
    
    def _update_obstacle_state(self, detected: bool, sensor: str = "", distance: float = 0.0):
        """Atomically update all obstacle-related state variables"""
        with self.state_lock:
            self.state.obstacle_detected = detected
            self.state.obstacle_sensor = sensor
            self.state.obstacle_distance = distance
            
            if detected:
                self.state.status = RobotStatus.OBSTACLE_DETECTED
                self.obstacle_event.set()
                logger.warning(f"Obstacle detected by {sensor} sensor at {distance:.1f}cm!")
            else:
                self.obstacle_event.clear()
                if self.state.status == RobotStatus.OBSTACLE_DETECTED:
                    self.state.status = RobotStatus.IDLE
                logger.info("Obstacle detection cleared")
    
    def _obstacle_detection_loop(self):
        """Background obstacle detection loop with proper state management and auto-clear"""
        while not self.shutdown_event.is_set():
            try:
                # Always check sensors, whether moving or not
                sensor_readings = self._get_all_distances_with_names()
                
                if sensor_readings:
                    # Update sensor distances in state atomically
                    with self.state_lock:
                        self.state.sensor_distances = sensor_readings.copy()
                        # Update last_distances for backward compatibility
                        self.state.last_distances = list(sensor_readings.values())
                    
                    # Find the closest obstacle from valid readings
                    valid_distances = [d for d in sensor_readings.values() if d > 0]
                    
                    if valid_distances:
                        min_distance = min(valid_distances)
                        closest_sensor = ""
                        
                        # Find which sensor has the minimum distance
                        for sensor_name, distance in sensor_readings.items():
                            if distance == min_distance and distance > 0:
                                closest_sensor = sensor_name
                                break
                        
                        # Check for new obstacle detection
                        if min_distance < self.config.obstacle_threshold and not self.state.obstacle_detected:
                            self._update_obstacle_state(True, closest_sensor, min_distance)
                            self.emergency_stop()
                            
                        # Check for obstacle clearing (AUTO-RESET) - only if we have valid readings
                        elif self.state.obstacle_detected and min_distance > self.config.obstacle_clear_threshold:
                            logger.info(f"Obstacle cleared! All sensors show > {self.config.obstacle_clear_threshold}cm")
                            self._update_obstacle_state(False)
                
                time.sleep(OBSTACLE_CHECK_INTERVAL)
                    
            except Exception as e:
                logger.error(f"Error in obstacle detection: {e}")
                time.sleep(OBSTACLE_CHECK_INTERVAL)
    
    def _status_update_loop(self):
        """Background status update loop with proper state management"""
        while not self.shutdown_event.is_set():
            try:
                with self.state_lock:
                    self.state.uptime = time.time() - self.start_time
                    
                    # Update status based on current state (only if not in obstacle mode)
                    if not self.state.obstacle_detected:
                        if self._is_moving():
                            self.state.status = RobotStatus.MOVING
                        else:
                            self.state.status = RobotStatus.IDLE
                
                time.sleep(STATUS_UPDATE_INTERVAL)
                
            except Exception as e:
                logger.error(f"Error in status update: {e}")
                time.sleep(STATUS_UPDATE_INTERVAL)
    
    def _is_moving(self) -> bool:
        """Check if robot is currently moving (call within state_lock)"""
        return any([
            self.state.motor1_speed > 0,
            self.state.motor2_speed > 0,
            self.state.motor3_speed > 0
        ])
    
    def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
        """Get distance from a single ultrasonic sensor - matching old code get_distance function"""
        if not self.gpio_initialized:
            return 100.0  # Simulation mode - return safe distance
        
        try:
            # Clean state - exactly like old code
            GPIO.output(trig_pin, False)
            time.sleep(0.000002)
            
            # Send trigger pulse - exactly like old code
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            # Wait for echo start with timeout
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 0:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return DEFAULT_SENSOR_ERROR_DISTANCE
            pulse_start = time.time()
            
            # Wait for echo end with timeout
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return DEFAULT_SENSOR_ERROR_DISTANCE
            pulse_end = time.time()
            
            # Calculate distance using proper physics - exactly like old code
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Same calculation as old code
            distance = round(distance, 2)
            
            # Validate distance range - exactly like old code
            if MIN_VALID_DISTANCE_CM <= distance <= MAX_VALID_DISTANCE_CM:
                return distance
            else:
                return DEFAULT_SENSOR_ERROR_DISTANCE
            
        except Exception as e:
            logger.error(f"Distance measurement error on pins {trig_pin}/{echo_pin}: {e}")
            return DEFAULT_SENSOR_ERROR_DISTANCE
    
    def _get_all_distances(self) -> List[float]:
        """Get distances from all sensors (backward compatibility) - only valid readings"""
        sensor_configs = [
            (self.config.trig1, self.config.echo1),
            (self.config.trig2, self.config.echo2),
            (self.config.trig3, self.config.echo3)
        ]
        
        distances = []
        for trig, echo in sensor_configs:
            dist = self._get_distance(trig, echo)
            if dist > 0:  # Only include valid readings
                distances.append(dist)
        
        return distances
    
    def _get_all_distances_with_names(self) -> Dict[str, float]:
        """Get distances from all sensors with sensor names - only include valid readings"""
        sensor_configs = [
            (self.config.trig1, self.config.echo1, "front"),   # Sensor 1 = Front
            (self.config.trig2, self.config.echo2, "left"),    # Sensor 2 = Left  
            (self.config.trig3, self.config.echo3, "right")    # Sensor 3 = Right
        ]
        
        sensor_readings = {}
        for trig, echo, name in sensor_configs:
            dist = self._get_distance(trig, echo)
            if dist > 0:  # Only include valid readings
                sensor_readings[name] = dist
            # Don't add invalid readings - this prevents false obstacle clearing
        
        return sensor_readings
    
    def _change_speeds_smooth(self, new_speeds: Tuple[int, int, int]):
        """Smoothly change motor speeds with proper thread safety - enhanced version of old changeSpeedSmooth"""
        if not self.gpio_initialized:
            with self.state_lock:
                self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
            return
        
        with self.movement_lock:
            # Check obstacle state once at the beginning
            with self.state_lock:
                if self.state.obstacle_detected or self.interrupt_event.is_set():
                    self._stop_motors_immediate()
                    return
                
                current_speeds = (self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed)
            
            # Calculate steps for smooth transition - exactly like old code
            speed_diffs = [abs(new_speeds[i] - current_speeds[i]) for i in range(3)]
            steps = max(speed_diffs) if speed_diffs else 0
            
            if steps < 1:
                steps = 1  # Minimum one step to avoid division by zero
            
            # Perform smooth transition - exactly like old code logic
            for step in range(steps + 1):
                # Check for interruption during transition
                with self.state_lock:
                    if self.state.obstacle_detected or self.interrupt_event.is_set():
                        self._stop_motors_immediate()
                        return
                
                progress = step / steps if steps > 0 else 1
                speeds = [
                    int(current_speeds[i] + (new_speeds[i] - current_speeds[i]) * progress)
                    for i in range(3)
                ]
                
                # Clamp speeds to valid range
                speeds = [max(0, min(100, speed)) for speed in speeds]
                
                # Apply PWM safely - exactly like old code
                try:
                    if self.motor1_pwm:
                        self.motor1_pwm.ChangeDutyCycle(speeds[0])
                    if self.motor2_pwm:
                        self.motor2_pwm.ChangeDutyCycle(speeds[1])
                    if self.motor3_pwm:
                        self.motor3_pwm.ChangeDutyCycle(speeds[2])
                except Exception as e:
                    logger.error(f"PWM update error: {e}")
                    self._stop_motors_immediate()
                    return
                
                time.sleep(SMOOTH_SPEED_STEP_DELAY)  # Same as old code 0.01
            
            # Update state atomically if successful
            with self.state_lock:
                if not self.state.obstacle_detected and not self.interrupt_event.is_set():
                    self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
                else:
                    self._stop_motors_immediate()
    
    def _stop_motors_immediate(self):
        """Immediately stop all motors (call within appropriate lock) - like old stopNoTime"""
        try:
            if self.gpio_initialized and all([self.motor1_pwm, self.motor2_pwm, self.motor3_pwm]):
                self.motor1_pwm.ChangeDutyCycle(0)
                self.motor2_pwm.ChangeDutyCycle(0)
                self.motor3_pwm.ChangeDutyCycle(0)
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
        
        # Always update state regardless of GPIO errors
        self.state.motor1_speed = 0
        self.state.motor2_speed = 0
        self.state.motor3_speed = 0
    
    # Public API methods
    
    def get_status(self) -> Dict:
        """Get current robot status with enhanced obstacle information"""
        with self.state_lock:
            obstacle_message = "Robot operational"
            if self.state.obstacle_detected:
                obstacle_message = f"Obstacle detected by {self.state.obstacle_sensor} sensor at {self.state.obstacle_distance:.1f}cm"
            
            return {
                "status": self.state.status.value,
                "message": obstacle_message,
                "obstacle_detected": self.state.obstacle_detected,
                "obstacle_sensor": self.state.obstacle_sensor,
                "obstacle_distance": self.state.obstacle_distance,
                "current_speeds": {
                    "motor1": self.state.motor1_speed,
                    "motor2": self.state.motor2_speed,
                    "motor3": self.state.motor3_speed
                },
                "sensor_distances": self.state.sensor_distances.copy(),
                "last_distances": self.state.last_distances.copy(),
                "last_command": self.state.last_command,
                "uptime": self.state.uptime,
                "gpio_available": self.gpio_initialized
            }
    
    def get_sensor_data(self) -> Dict:
        """Get current sensor readings"""
        distances = self._get_all_distances()
        with self.state_lock:
            return {
                "distances": distances,
                "min_distance": min(distances) if distances else None,
                "obstacle_threshold": self.config.obstacle_threshold,
                "obstacle_detected": self.state.obstacle_detected,
                "sensor_distances": self.state.sensor_distances.copy(),
                "obstacle_sensor": self.state.obstacle_sensor
            }
    
    def move(self, direction: Union[str, MovementDirection], speed: Optional[int] = None, 
             duration_ms: Optional[int] = None) -> bool:
        """Move robot in specified direction with proper validation - using exact old code motor configurations"""
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
        
        # Check obstacle - but allow stop command
        with self.state_lock:
            if self.state.obstacle_detected and direction != MovementDirection.STOP:
                logger.warning(f"Movement blocked - obstacle detected by {self.state.obstacle_sensor} sensor")
                return False
            
            speed = speed or self.config.default_speed
            self.state.last_command = direction.value
        
        # Handle stop command with obstacle check
        if direction == MovementDirection.STOP:
            # Check if obstacle is cleared when stopping
            sensor_readings = self._get_all_distances_with_names()
            if sensor_readings:
                valid_distances = [d for d in sensor_readings.values() if d > 0]
                if valid_distances:
                    min_distance = min(valid_distances)
                    if min_distance > self.config.obstacle_clear_threshold:
                        self._update_obstacle_state(False)
        
        # Direction configurations using class constants - EXACTLY MATCHING OLD CODE CONFIGURATIONS
        direction_configs = {
            # From old code: "forward" - Motor1=0, Motor2=speed, Motor3=speed+compensate
            MovementDirection.FORWARD: {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            # From old code: "backward" - Motor1=0, Motor2=speed, Motor3=speed+compensate
            MovementDirection.BACKWARD: {
                "dirs": [self.HIGH, self.LOW, self.HIGH],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            # From old code: "turnLeft" - Motor1=speed, Motor2=speed, Motor3=speed+compensate
            MovementDirection.TURN_LEFT: {
                "dirs": [self.HIGH, self.LOW, self.LOW],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            # From old code: "turnRight" - Motor1=speed, Motor2=speed, Motor3=speed+compensate
            MovementDirection.TURN_RIGHT: {
                "dirs": [self.LOW, self.HIGH, self.HIGH],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            # From old code: "moveLeft" - Motor1=speed*1.5, Motor2=speed, Motor3=0
            MovementDirection.MOVE_LEFT: {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [int(speed * 1.5), speed, 0]
            },
            # From old code: "moveRight" - Motor1=speed*1.5, Motor2=speed, Motor3=speed+compensate
            MovementDirection.MOVE_RIGHT: {
                "dirs": [self.LOW, self.LOW, self.HIGH],
                "speeds": [int(speed * 1.5), speed, speed + self.config.motor3_compensate]
            },
            MovementDirection.STOP: {
                "dirs": [self.LOW, self.LOW, self.LOW],
                "speeds": [0, 0, 0]
            }
        }
        
        config = direction_configs[direction]
        
        # Set motor directions safely - exactly like old code
        if self.gpio_initialized:
            try:
                GPIO.output(self.config.motor1_dir, config["dirs"][0])
                GPIO.output(self.config.motor2_dir, config["dirs"][1])
                GPIO.output(self.config.motor3_dir, config["dirs"][2])
            except Exception as e:
                logger.error(f"Error setting motor directions: {e}")
                return False
        
        # Apply speeds
        self._change_speeds_smooth(tuple(config["speeds"]))
        
        # Handle duration
        if duration_ms is not None and duration_ms > 0:
            time.sleep(duration_ms / 1000)
            with self.state_lock:
                if not self.state.obstacle_detected:
                    self._change_speeds_smooth((0, 0, 0))
        
        return True
    
    def stop(self) -> bool:
        """Stop the robot"""
        return self.move(MovementDirection.STOP)
    
    def emergency_stop(self) -> bool:
        """Emergency stop with state reset - like old emergency_stop"""
        logger.warning("Emergency stop activated")
        
        with self.movement_lock:
            with self.state_lock:
                self.state.status = RobotStatus.EMERGENCY_STOP
                self._stop_motors_immediate()
        
        return True
    
    def reset_obstacle_detection(self) -> bool:
        """Reset obstacle detection state"""
        self._update_obstacle_state(False)
        logger.info("Obstacle detection reset - robot can move again")
        return True
    
    def set_speed(self, speed: int) -> bool:
        """Set default movement speed"""
        if 0 <= speed <= 100:
            self.config.default_speed = speed
            logger.info(f"Default speed set to {speed}")
            return True
        logger.warning(f"Invalid speed value: {speed}. Must be between 0 and 100.")
        return False
    
    def set_motor_compensation(self, compensation: int) -> bool:
        """Set motor 3 compensation value"""
        if 0 <= compensation <= 50:
            self.config.motor3_compensate = compensation
            logger.info(f"Motor 3 compensation set to {compensation}")
            return True
        logger.warning(f"Invalid compensation value: {compensation}. Must be between 0 and 50.")
        return False
    
    def parse_command(self, command: str) -> Optional[Tuple[str, Optional[int]]]:
        """Parse natural language command to movement direction and duration - matching old code logic"""
        command = command.lower().strip()
        
        # Movement direction mappings - exactly like old code
        directions = {
            "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
            "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
            "stop": ["stop", "halt", "stand still", "brake", "freeze"],
            "turnleft": ["turn left", "rotate left", "spin left"],
            "turnright": ["turn right", "rotate right", "spin right"],
            "moveleft": ["move left", "strafe left", "slide left", "sidestep left"],
            "moveright": ["move right", "strafe right", "slide right", "sidestep right"]
        }
        
        # Time patterns - exactly like old code
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
                try:
                    value = float(match.group(1))
                    if unit == "seconds":
                        duration_ms = int(value * 1000)
                    elif unit == "minutes":
                        duration_ms = int(value * 60000)
                    elif unit == "hours":
                        duration_ms = int(value * 3600000)
                    break
                except (ValueError, OverflowError):
                    logger.warning(f"Invalid duration value: {match.group(1)}")
                    continue
        
        return (direction, duration_ms) if direction else None
    
    def get_direction(self, user_input: str) -> Optional[str]:
        """Extract movement direction from user input - matching old code function"""
        user_input = user_input.lower()
        directions = {
            "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
            "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
            "stop": ["stop", "halt", "stand still", "brake", "freeze"],
            "turnLeft": ["turn left", "rotate left", "spin left"],
            "turnRight": ["turn right", "rotate right", "spin right"],
            "moveLeft": ["move left", "strafe left", "slide left", "sidestep left"],
            "moveRight": ["move right", "strafe right", "slide right", "sidestep right"]
        }
        
        for direction, phrases in directions.items():
            if any(phrase in user_input for phrase in phrases):
                return direction
        return None
    
    def convert_to_milliseconds(self, text: str) -> Optional[int]:
        """Convert time expressions to milliseconds - matching old code function"""
        text = text.lower()
        time_patterns = {
            "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
            "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
            "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
        }
        
        for unit, pattern in time_patterns.items():
            match = re.search(pattern, text)
            if match:
                try:
                    value = float(match.group(1))
                    if unit == "seconds":
                        return int(value * 1000)
                    elif unit == "minutes":
                        return int(value * 60000)
                    elif unit == "hours":
                        return int(value * 3600000)
                except (ValueError, OverflowError):
                    logger.warning(f"Invalid duration value: {match.group(1)}")
                    continue
        return None
    
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
                    
                with self.state_lock:
                    if self.state.obstacle_detected:
                        break
        
        return True
    
    async def process_user_input(self, user_input: str, context: str = "") -> bool:
        """Process user input for movement commands - matching old code process_user_input logic"""
        if not user_input:
            return False
        
        # Split commands by "then" for sequential execution - exactly like old code
        instructions = [instr.strip() for instr in user_input.split("then")]
        command_sequence = []
        has_movement = False
        
        # Reset states
        with self.state_lock:
            self.state.obstacle_detected = False
        
        # Parse instructions - exactly like old code
        for instruction in instructions:
            direction = self.get_direction(instruction)
            time_ms = self.convert_to_milliseconds(instruction)
            
            # Default turn time - exactly like old code
            if direction in ["turnLeft", "turnRight"] and time_ms is None:
                time_ms = 1500
            
            if direction and time_ms is not None:
                command_sequence.append((direction, time_ms))
                has_movement = True
            elif direction == "stop":
                command_sequence.append(("stop", 0))
                has_movement = True
        
        # Execute command sequence - exactly like old code logic
        if has_movement:
            logger.info(f"Executing command sequence: {command_sequence}")
            
            for command, duration in command_sequence:
                with self.state_lock:
                    self.state.last_command = command
                
                # Convert old code command names to new enum names
                command_map = {
                    "forward": "forward",
                    "backward": "backward",
                    "turnLeft": "turnleft",
                    "turnRight": "turnright",
                    "moveLeft": "moveleft",
                    "moveRight": "moveright",
                    "stop": "stop"
                }
                
                mapped_command = command_map.get(command, command)
                
                if not self.move(mapped_command, duration_ms=duration):
                    break
                
                # Check for interrupts - exactly like old code
                with self.state_lock:
                    if self.state.obstacle_detected:
                        break
                
                # Wait for command completion
                if duration > 0:
                    await asyncio.sleep(duration / 1000)
            
            return True
        
        return False
    
    async def chat(self, message: str, context: str = "") -> str:
        """Chat with AI model - exactly like old code AI chat logic"""
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
                "obstacle_clear_threshold": self.config.obstacle_clear_threshold,
                "pwm_frequency": self.config.pwm_frequency,
                "sensor_timeout": self.config.sensor_timeout,
                "motor3_compensate": self.config.motor3_compensate
            }
        }
    
    # Movement functions matching old code exactly
    def go_forwards(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Move forward - matching old goForwards function"""
        return self.move("forward", speed, time_ms)
    
    def go_backwards(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Move backward - matching old goBackwards function"""
        return self.move("backward", speed, time_ms)
    
    def turn_left(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Turn left - matching old turnLeft function"""
        return self.move("turnleft", speed, time_ms)
    
    def turn_right(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Turn right - matching old turnRight function"""
        return self.move("turnright", speed, time_ms)
    
    def move_left(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Move left - matching old moveLeft function"""
        return self.move("moveleft", speed, time_ms)
    
    def move_right(self, speed: Optional[int] = None, time_ms: Optional[int] = None) -> bool:
        """Move right - matching old moveRight function"""
        return self.move("moveright", speed, time_ms)
    
    def start_forward(self) -> bool:
        """Start forward movement - matching old startForward function"""
        return self.move("forward")
    
    def start_backward(self) -> bool:
        """Start backward movement - matching old startBackward function"""
        return self.move("backward")
    
    def start_turn_left(self) -> bool:
        """Start left turn - matching old startTurnLeft function"""
        return self.move("turnleft")
    
    def start_turn_right(self) -> bool:
        """Start right turn - matching old startTurnRight function"""
        return self.move("turnright")
    
    def start_move_left(self) -> bool:
        """Start left movement - matching old startMoveLeft function"""
        return self.move("moveleft")
    
    def start_move_right(self) -> bool:
        """Start right movement - matching old startMoveRight function"""
        return self.move("moveright")
    
    def immediate_stop(self) -> bool:
        """Immediate stop - matching old immediateStop function"""
        with self.state_lock:
            self.state.obstacle_detected = False
        self.interrupt_event.clear()
        return self.stop()
    
    def stop_motors(self, time_ms: int = 0) -> bool:
        """Stop motors with optional delay - matching old stopMotors function"""
        with self.state_lock:
            self.state.obstacle_detected = False
        self.interrupt_event.clear()
        
        success = self.stop()
        
        if time_ms > 0:
            time.sleep(time_ms / 1000)
        
        return success
    
    def process_immediate_command(self, command: str) -> bool:
        """Process immediate movement commands - matching old processImmediateCommand"""
        command = command.strip().lower()
        command_map = {
            "forward": self.start_forward,
            "backward": self.start_backward,
            "left": self.start_turn_left,
            "right": self.start_turn_right,
            "moveleft": self.start_move_left,
            "moveright": self.start_move_right,
            "stop": self.immediate_stop
        }
        
        if command in command_map:
            return command_map[command]()
        else:
            logger.warning(f"Unknown immediate command: {command}")
            return False
    
    def process_command(self, command: str, time_ms: int) -> bool:
        """Process timed movement commands - matching old processCommand"""
        with self.state_lock:
            if self.state.obstacle_detected and command != "stop":
                logger.warning("Command blocked - obstacle detected")
                return False
        
        command_map = {
            "forward": lambda: self.go_forwards(time_ms=time_ms),
            "backward": lambda: self.go_backwards(time_ms=time_ms),
            "turnRight": lambda: self.turn_right(time_ms=time_ms),
            "turnLeft": lambda: self.turn_left(time_ms=time_ms),
            "moveRight": lambda: self.move_right(time_ms=time_ms),
            "moveLeft": lambda: self.move_left(time_ms=time_ms),
            "stop": lambda: self.stop_motors(time_ms)
        }
        
        if command in command_map:
            return command_map[command]()
        else:
            logger.warning(f"Unknown timed command: {command}")
            return False
    
    def shutdown(self):
        """Shutdown robot controller with proper cleanup - matching old cleanup logic"""
        logger.info("Shutting down robot controller...")
        
        # Signal shutdown to all threads
        self.shutdown_event.set()
        
        # Stop all motors immediately
        with self.movement_lock:
            with self.state_lock:
                self._stop_motors_immediate()
        
        # Wait for background threads to finish
        threads_to_join = [
            (self.obstacle_thread, "obstacle detection"),
            (self.status_thread, "status update")
        ]
        
        for thread, name in threads_to_join:
            if thread and thread.is_alive():
                logger.info(f"Waiting for {name} thread to finish...")
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"{name} thread did not finish gracefully")
        
        # Cleanup GPIO and PWM - exactly like old code cleanup
        if self.gpio_initialized:
            try:
                # Stop PWM objects - exactly like old code
                pwm_objects = [
                    (self.motor1_pwm, "motor1"),
                    (self.motor2_pwm, "motor2"), 
                    (self.motor3_pwm, "motor3")
                ]
                
                for pwm, name in pwm_objects:
                    if pwm:
                        try:
                            pwm.stop()
                            logger.debug(f"{name} PWM stopped")
                        except Exception as e:
                            logger.error(f"Error stopping {name} PWM: {e}")
                
                # Cleanup GPIO - exactly like old code
                try:
                    GPIO.remove_event_detect(self.config.echo1)
                except:
                    pass
                try:
                    GPIO.remove_event_detect(self.config.echo2)
                except:
                    pass
                try:
                    GPIO.remove_event_detect(self.config.echo3)
                except:
                    pass
                
                GPIO.cleanup()
                logger.info("GPIO cleanup completed")
                
            except Exception as e:
                logger.error(f"GPIO cleanup error: {e}")
        
        logger.info("Robot controller shutdown complete")


# Global robot controller instance
_robot_controller = None
_controller_lock = threading.Lock()

def get_robot_controller():
    """Get or create the global robot controller instance (thread-safe)"""
    global _robot_controller
    with _controller_lock:
        if _robot_controller is None:
            _robot_controller = RobotController()
        return _robot_controller

# Public API functions for FastAPI backend compatibility - matching old code public functions
def get_status() -> Dict:
    """Get current robot status - called by FastAPI"""
    try:
        return get_robot_controller().get_status()
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        return {
            "status": "error",
            "message": f"Error getting status: {str(e)}",
            "obstacle_detected": False,
            "gpio_available": False
        }

def move(direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
    """Move robot in specified direction - called by FastAPI"""
    try:
        return get_robot_controller().move(direction, speed, duration_ms)
    except Exception as e:
        logger.error(f"Error moving robot: {e}")
        return False

def stop() -> bool:
    """Stop the robot - called by FastAPI"""
    try:
        return get_robot_controller().stop()
    except Exception as e:
        logger.error(f"Error stopping robot: {e}")
        return False

def parse_command(command: str) -> Optional[Tuple[str, Optional[int]]]:
    """Parse natural language command - called by FastAPI"""
    try:
        return get_robot_controller().parse_command(command)
    except Exception as e:
        logger.error(f"Error parsing command: {e}")
        return None

async def chat(message: str, context: str = "") -> str:
    """Chat with AI model - called by FastAPI"""
    try:
        return await get_robot_controller().chat(message, context)
    except Exception as e:
        logger.error(f"Error in chat: {e}")
        return "Error processing chat message."

def emergency_stop() -> bool:
    """Emergency stop - called by FastAPI"""
    try:
        return get_robot_controller().emergency_stop()
    except Exception as e:
        logger.error(f"Error in emergency stop: {e}")
        return False

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection - called by FastAPI"""
    try:
        return get_robot_controller().reset_obstacle_detection()
    except Exception as e:
        logger.error(f"Error resetting obstacle detection: {e}")
        return False

def get_sensor_data() -> Dict:
    """Get sensor data - called by FastAPI"""
    try:
        return get_robot_controller().get_sensor_data()
    except Exception as e:
        logger.error(f"Error getting sensor data: {e}")
        return {
            "distances": [],
            "min_distance": None,
            "obstacle_threshold": 30.0,
            "obstacle_detected": False,
            "sensor_distances": {},
            "obstacle_sensor": ""
        }

def set_speed(speed: int) -> bool:
    """Set default speed - called by FastAPI"""
    try:
        return get_robot_controller().set_speed(speed)
    except Exception as e:
        logger.error(f"Error setting speed: {e}")
        return False

def set_motor_compensation(compensation: int) -> bool:
    """Set motor compensation - called by FastAPI"""
    try:
        return get_robot_controller().set_motor_compensation(compensation)
    except Exception as e:
        logger.error(f"Error setting motor compensation: {e}")
        return False

def get_config() -> Dict:
    """Get configuration - called by FastAPI"""
    try:
        return get_robot_controller().get_config()
    except Exception as e:
        logger.error(f"Error getting config: {e}")
        return {}

def shutdown_robot():
    """Shutdown robot controller - called by FastAPI on app shutdown"""
    global _robot_controller
    with _controller_lock:
        if _robot_controller is not None:
            _robot_controller.shutdown()
            _robot_controller = None

# Legacy function aliases for backward compatibility with old code
def goForwards(speed: int, time_ms: int) -> bool:
    """Legacy function - use go_forwards instead"""
    return get_robot_controller().go_forwards(speed, time_ms)

def goBackwards(speed: int, time_ms: int) -> bool:
    """Legacy function - use go_backwards instead"""
    return get_robot_controller().go_backwards(speed, time_ms)

def turnLeft(speed: int, time_ms: int) -> bool:
    """Legacy function - use turn_left instead"""
    return get_robot_controller().turn_left(speed, time_ms)

def turnRight(speed: int, time_ms: int) -> bool:
    """Legacy function - use turn_right instead"""
    return get_robot_controller().turn_right(speed, time_ms)

def moveLeft(speed: int, time_ms: int) -> bool:
    """Legacy function - use move_left instead"""
    return get_robot_controller().move_left(speed, time_ms)

def moveRight(speed: int, time_ms: int) -> bool:
    """Legacy function - use move_right instead"""
    return get_robot_controller().move_right(speed, time_ms)

def startForward() -> bool:
    """Legacy function - use start_forward instead"""
    return get_robot_controller().start_forward()

def startBackward() -> bool:
    """Legacy function - use start_backward instead"""
    return get_robot_controller().start_backward()

def startTurnLeft() -> bool:
    """Legacy function - use start_turn_left instead"""
    return get_robot_controller().start_turn_left()

def startTurnRight() -> bool:
    """Legacy function - use start_turn_right instead"""
    return get_robot_controller().start_turn_right()

def startMoveLeft() -> bool:
    """Legacy function - use start_move_left instead"""
    return get_robot_controller().start_move_left()

def startMoveRight() -> bool:
    """Legacy function - use start_move_right instead"""
    return get_robot_controller().start_move_right()

def immediateStop() -> bool:
    """Legacy function - use immediate_stop instead"""
    return get_robot_controller().immediate_stop()

def stopMotors(time_ms: int) -> bool:
    """Legacy function - use stop_motors instead"""
    return get_robot_controller().stop_motors(time_ms)

def processImmediateCommand(command: str) -> bool:
    """Legacy function - use process_immediate_command instead"""
    return get_robot_controller().process_immediate_command(command)

def processCommand(command: str, time_ms: int) -> bool:
    """Legacy function - use process_command instead"""
    return get_robot_controller().process_command(command, time_ms)

def get_distance(trig_pin: int, echo_pin: int) -> float:
    """Legacy function - get distance from ultrasonic sensor"""
    return get_robot_controller()._get_distance(trig_pin, echo_pin)

# Factory function for easy initialization
def create_robot(config: Optional[RobotConfig] = None) -> RobotController:
    """Create a robot controller instance"""
    return RobotController(config)

# Context manager for proper resource cleanup
class RobotManager:
    """Context manager for robot controller with automatic cleanup"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        self.config = config
        self.robot = None
    
    def __enter__(self) -> RobotController:
        self.robot = RobotController(self.config)
        return self.robot
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.robot:
            self.robot.shutdown()

if __name__ == "__main__":
    print("🤖 Robot Movement Controller - Production Version")
    print("=" * 50)
    print("Features:")
    print("• Full backward compatibility with old code")
    print("• Proper motor compensation")
    print("• Thread-safe operation")
    print("• Enhanced obstacle detection")
    print("• AI integration")
    print("• FastAPI integration ready")
    print("=" * 50)
    print("✅ Robot controller module loaded successfully")