# enhanced_robot_movement.py - Fixed version with proper camera handling
import asyncio
import re
import threading
import time
import logging
import cv2
import os
import platform
import subprocess
from typing import Dict, Optional, List, Tuple, Union
from dataclasses import dataclass, field
from enum import Enum
import json
from collections import deque

# Import your existing modules
try:
    import face_recognition
    import numpy as np
    FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    FACE_RECOGNITION_AVAILABLE = False
    logging.warning("Face recognition not available - install face_recognition library")

try:
    import speech_recognition as sr
    SPEECH_RECOGNITION_AVAILABLE = True
except ImportError:
    SPEECH_RECOGNITION_AVAILABLE = False
    logging.warning("Speech recognition not available - install SpeechRecognition library")

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    logging.warning("MediaPipe not available - hand detection disabled")

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

# Face Recognition Helper Class
class FaceRecognitionHelper:
    """Face recognition helper with speech synthesis"""
    
    def __init__(self, images_path: str = '/home/robot/summer-research-robot/RGB-cam/images'):
        self.images_path = images_path
        self.images = []
        self.class_names = []
        self.encode_list_known = []
        self.cap = None
        self.initialize_face_recognition()
    
    def initialize_face_recognition(self):
        """Initialize face recognition system"""
        if not FACE_RECOGNITION_AVAILABLE:
            logger.warning("Face recognition not available")
            return
        
        try:
            # Create images directory if it doesn't exist
            os.makedirs(self.images_path, exist_ok=True)
            
            # Load existing face images
            my_list = os.listdir(self.images_path)
            for cl in my_list:
                if cl.lower().endswith(('.png', '.jpg', '.jpeg')):
                    cur_img = cv2.imread(f'{self.images_path}/{cl}')
                    if cur_img is not None:
                        self.images.append(cur_img)
                        self.class_names.append(os.path.splitext(cl)[0])
            
            self.encode_list_known = self.find_encodings(self.images)
            logger.info(f"Loaded {len(self.class_names)} known faces")
            
        except Exception as e:
            logger.error(f"Face recognition initialization failed: {e}")
    
    def find_encodings(self, images):
        """Generate face encodings for known faces"""
        if not FACE_RECOGNITION_AVAILABLE:
            return []
        
        encode_list = []
        for img in images:
            try:
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                encodings = face_recognition.face_encodings(img_rgb)
                if encodings:
                    encode_list.append(encodings[0])
                else:
                    logger.warning("No face found in image")
            except Exception as e:
                logger.error(f"Error encoding face: {e}")
        return encode_list
    
    def speak(self, text: str):
        """Cross-platform text-to-speech"""
        system = platform.system().lower()
        
        try:
            if system == "windows":
                ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
                subprocess.run(["powershell", "-Command", ps_command], 
                              capture_output=True, check=True)
            elif system == "darwin":  # macOS
                subprocess.run(["say", text], check=True)
            elif system == "linux":
                try:
                    subprocess.run(["espeak", text], check=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    try:
                        subprocess.run(["echo", text, "|", "festival", "--tts"], 
                                     shell=True, check=True)
                    except (subprocess.CalledProcessError, FileNotFoundError):
                        subprocess.run(["spd-say", text], check=True)
            else:
                logger.info(f"TTS: {text}")
                
        except (subprocess.CalledProcessError, FileNotFoundError):
            logger.info(f"TTS: {text}")
    
    def listen(self) -> Optional[str]:
        """Listen for speech input"""
        if not SPEECH_RECOGNITION_AVAILABLE:
            logger.warning("Speech recognition not available")
            return None
        
        try:
            r = sr.Recognizer()
            with sr.Microphone() as source:
                logger.info("Listening...")
                audio = r.listen(source, timeout=5)
                text = r.recognize_google(audio)
                logger.info(f"Heard: {text}")
                return text.lower()
        except sr.UnknownValueError:
            logger.warning("Could not understand audio")
        except sr.RequestError as e:
            logger.error(f"Speech recognition error: {e}")
        except sr.WaitTimeoutError:
            logger.warning("Listening timeout")
        except Exception as e:
            logger.error(f"Speech recognition error: {e}")
        return None
    
    def take_picture(self, name: str, frame=None) -> bool:
        """Take a picture for face registration using provided frame"""
        try:
            if frame is not None:
                path = os.path.join(self.images_path, f'{name}.jpg')
                cv2.imwrite(path, frame)
                logger.info(f"Picture saved for {name}")
                return True
            else:
                logger.error("No frame provided for picture")
                return False
        except Exception as e:
            logger.error(f"Error taking picture: {e}")
            return False
    
    def find_match(self, frame, mode: str = "auto") -> Optional[str]:
        """Find face match in provided frame"""
        if not FACE_RECOGNITION_AVAILABLE or frame is None:
            return "Unknown"
        
        try:
            img_small = cv2.resize(frame, (0, 0), None, 0.25, 0.25)
            img_rgb = cv2.cvtColor(img_small, cv2.COLOR_BGR2RGB)
            
            faces_cur_frame = face_recognition.face_locations(img_rgb)
            encodes_cur_frame = face_recognition.face_encodings(img_rgb, faces_cur_frame)
            
            for encode_face, face_loc in zip(encodes_cur_frame, faces_cur_frame):
                if not self.encode_list_known:
                    # No known faces, would register this person
                    return "Unknown"
                
                matches = face_recognition.compare_faces(self.encode_list_known, encode_face)
                face_distances = face_recognition.face_distance(self.encode_list_known, encode_face)
                
                if face_distances.size > 0:
                    match_index = np.argmin(face_distances)
                    best_match_distance = face_distances[match_index]
                    
                    if best_match_distance <= 0.42 and matches[match_index]:  # Good match
                        name = self.class_names[match_index].upper()
                        return name
            
            return "Unknown"
                
        except Exception as e:
            logger.error(f"Face recognition error: {e}")
            return "Unknown"

# Hand Detection Helper Class
class HandDetector:
    """Hand gesture detection using MediaPipe"""
    
    def __init__(self, mode=False, max_hands=2, detection_con=0.5, track_con=0.5):
        if not MEDIAPIPE_AVAILABLE:
            logger.warning("MediaPipe not available - hand detection disabled")
            return
        
        self.mode = mode
        self.max_hands = max_hands
        self.detection_con = detection_con
        self.track_con = track_con
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.max_hands,
            min_detection_confidence=self.detection_con,
            min_tracking_confidence=self.track_con
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.results = None
    
    def find_hands(self, img, draw=True):
        """Find hands in image"""
        if not MEDIAPIPE_AVAILABLE:
            return img
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        
        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
        return img
    
    def find_position(self, img, hand_no=0, draw=True):
        """Find hand landmark positions"""
        lm_list = []
        if self.results and self.results.multi_hand_landmarks:
            try:
                my_hand = self.results.multi_hand_landmarks[hand_no]
                for id, lm in enumerate(my_hand.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lm_list.append([id, cx, cy])
                    if draw:
                        cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            except IndexError:
                pass
        return lm_list
    
    def is_waving(self, lm_list, index_x_history):
        """Detect waving gesture"""
        if len(lm_list) < 21:
            return False
        
        # Check if hand is in open position
        if not self.fingers_up(lm_list):
            return False
        
        index_x = lm_list[8][1]  # index fingertip x position
        index_x_history.append(index_x)
        
        if len(index_x_history) < 5:
            return False
        
        # Check if index finger tip is swinging side to side
        index_motion = max(index_x_history) - min(index_x_history)
        return index_motion > 40
    
    def fingers_up(self, lm_list):
        """Check if fingers are extended (open hand)"""
        if len(lm_list) < 21:
            return False
        
        # Thumb
        if lm_list[4][1] < lm_list[3][1]:  # Thumb tip left of thumb joint
            thumb_up = True
        else:
            thumb_up = False
        
        # Four fingers
        fingers = [thumb_up]
        for id in [8, 12, 16, 20]:  # Index, middle, ring, pinky tips
            if lm_list[id][2] < lm_list[id - 2][2]:  # Tip above joint
                fingers.append(True)
            else:
                fingers.append(False)
        
        return sum(fingers) >= 3  # At least 3 fingers up
    
    def is_shaking_hands(self, lm_list):
        """Detect handshake gesture"""
        if len(lm_list) < 21:
            return False
        
        # Check if fingers are partially closed (handshake position)
        tips = [8, 12, 16, 20]  # Index, middle, ring, pinky
        joints = [6, 10, 14, 18]  # Corresponding joints
        
        closed_fingers = 0
        for tip, joint in zip(tips, joints):
            if lm_list[tip][2] > lm_list[joint][2]:  # Tip below joint (closed)
                closed_fingers += 1
        
        return closed_fingers >= 2  # At least 2 fingers closed

# Enhanced Robot Configuration
@dataclass
class EnhancedRobotConfig:
    """Enhanced robot configuration with vision and speech settings"""
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
    motor3_compensate: int = 3
    
    # Obstacle detection
    obstacle_threshold: float = 30.0
    sensor_timeout: float = 0.5
    obstacle_clear_threshold: float = 50.0
    
    # Vision settings
    camera_index: int = 0
    face_images_path: str = '/home/robot/summer-research-robot/RGB-cam/images'
    enable_face_recognition: bool = True
    enable_hand_detection: bool = True
    
    # Speech settings
    enable_speech_synthesis: bool = True
    enable_speech_recognition: bool = True
    
    # AI model
    ai_model: str = "llama3"

# Enhanced Robot State
@dataclass
class EnhancedRobotState:
    """Enhanced robot state with vision and speech data"""
    # Motor states
    motor1_speed: int = 0
    motor2_speed: int = 0
    motor3_speed: int = 0
    
    # Basic states
    status: str = "idle"
    obstacle_detected: bool = False
    last_distances: List[float] = field(default_factory=list)
    sensor_distances: Dict[str, float] = field(default_factory=lambda: {"front": 0.0, "left": 0.0, "right": 0.0})
    obstacle_sensor: str = ""
    obstacle_distance: float = 0.0
    last_command: str = ""
    uptime: float = 0.0
    
    # Vision states
    current_user: str = "Unknown"
    faces_detected: List[str] = field(default_factory=list)
    hand_gesture: str = "none"
    camera_active: bool = False
    
    # Speech states
    last_speech_output: str = ""
    listening: bool = False
    speech_recognition_active: bool = False
    
    # Interaction mode
    interaction_mode: str = "idle"  # idle, speech, text, keyboard, auto

class EnhancedRobotController:
    """Enhanced robot controller with face recognition and speech capabilities"""
    
    def __init__(self, config: Optional[EnhancedRobotConfig] = None):
        self.config = config or EnhancedRobotConfig()
        self.state = EnhancedRobotState()
        self.start_time = time.time()
        
        # Threading controls
        self.movement_lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.vision_lock = threading.RLock()
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Thread references
        self.obstacle_thread = None
        self.status_thread = None
        self.vision_thread = None
        
        # Vision components
        self.face_helper = None
        self.hand_detector = None
        self.camera = None
        self.index_x_history = deque(maxlen=10)
        self.current_frame = None  # Store current frame for streaming
        
        # PWM references
        self.motor1_pwm = None
        self.motor2_pwm = None
        self.motor3_pwm = None
        
        # Direction constants
        self.HIGH = GPIO.HIGH if GPIO_AVAILABLE else 1
        self.LOW = GPIO.LOW if GPIO_AVAILABLE else 0
        
        # Initialize components
        self.gpio_initialized = False
        self._setup_gpio()
        self._setup_ai()
        self._setup_vision()
        self._start_background_tasks()
        
        logger.info("Enhanced robot controller initialized successfully")
    
    def _setup_vision(self):
        """Setup camera and vision components with better error handling"""
        if not self.config.enable_face_recognition and not self.config.enable_hand_detection:
            logger.info("Vision components disabled in config")
            return
        
        try:
            # Try multiple camera indices
            camera_indices = [0, 1, 2, -1]  # -1 for auto-detect
            camera_initialized = False
            
            for idx in camera_indices:
                try:
                    logger.info(f"Trying camera index {idx}")
                    self.camera = cv2.VideoCapture(idx)
                    
                    if self.camera.isOpened():
                        # Set camera properties for better performance
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.camera.set(cv2.CAP_PROP_FPS, 30)
                        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer lag
                        
                        # Test camera
                        ret, test_frame = self.camera.read()
                        if ret and test_frame is not None:
                            with self.state_lock:
                                self.state.camera_active = True
                            self.config.camera_index = idx
                            camera_initialized = True
                            logger.info(f"Camera initialized successfully on index {idx}")
                            break
                        else:
                            self.camera.release()
                            
                except Exception as e:
                    logger.warning(f"Failed to initialize camera on index {idx}: {e}")
                    if self.camera:
                        self.camera.release()
                    continue
            
            if not camera_initialized:
                logger.error("Failed to initialize camera on any index")
                with self.state_lock:
                    self.state.camera_active = False
                return
                
            # Initialize face recognition
            if self.config.enable_face_recognition and FACE_RECOGNITION_AVAILABLE:
                self.face_helper = FaceRecognitionHelper(self.config.face_images_path)
                logger.info("Face recognition initialized")
            
            # Initialize hand detection
            if self.config.enable_hand_detection and MEDIAPIPE_AVAILABLE:
                self.hand_detector = HandDetector()
                logger.info("Hand detection initialized")
                
        except Exception as e:
            logger.error(f"Vision setup failed: {e}")
            with self.state_lock:
                self.state.camera_active = False
    
    def _setup_gpio(self):
        """Setup GPIO pins and PWM"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            
            # Setup ultrasonic sensors
            GPIO.setup(self.config.echo1, GPIO.IN)
            GPIO.setup(self.config.echo2, GPIO.IN)
            GPIO.setup(self.config.echo3, GPIO.IN)
            GPIO.setup(self.config.trig1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig3, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup motors
            GPIO.setup(self.config.motor1_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor1_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_dir, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup PWM
            self.motor1_pwm = GPIO.PWM(self.config.motor1_speed, self.config.pwm_frequency)
            self.motor2_pwm = GPIO.PWM(self.config.motor2_speed, self.config.pwm_frequency)
            self.motor3_pwm = GPIO.PWM(self.config.motor3_speed, self.config.pwm_frequency)
            
            self.motor1_pwm.start(0)
            self.motor2_pwm.start(0)
            self.motor3_pwm.start(0)
            
            self.gpio_initialized = True
            logger.info("GPIO initialized successfully")
            
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            self.gpio_initialized = False
    
    def _setup_ai(self):
        """Setup AI model for conversation"""
        if not AI_AVAILABLE:
            logger.warning("AI not available")
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
        try:
            # Obstacle detection
            self.obstacle_thread = threading.Thread(
                target=self._obstacle_detection_loop, 
                daemon=True, 
                name="ObstacleDetection"
            )
            self.obstacle_thread.start()
            
            # Status updates
            self.status_thread = threading.Thread(
                target=self._status_update_loop, 
                daemon=True, 
                name="StatusUpdate"
            )
            self.status_thread.start()
            
            # Unified vision processing (replaces separate vision and hand detection threads)
            if self.state.camera_active:
                self.vision_thread = threading.Thread(
                    target=self._unified_vision_loop,
                    daemon=True,
                    name="UnifiedVision"
                )
                self.vision_thread.start()
            
            logger.info("Background tasks started successfully")
        except Exception as e:
            logger.error(f"Failed to start background tasks: {e}")
    
    def _unified_vision_loop(self):
        """Unified vision processing loop for both face and hand detection"""
        face_check_counter = 0
        face_check_interval = 60  # Check faces every 60 frames (2 seconds at 30fps)
        last_user_check = time.time()
        
        while not self.shutdown_event.is_set():
            try:
                if self.camera and self.camera.isOpened():
                    success, frame = self.camera.read()
                    if not success:
                        logger.warning("Failed to read camera frame")
                        time.sleep(0.1)
                        continue
                    
                    frame = cv2.flip(frame, 1)
                    
                    # Hand detection on every frame (high priority)
                    if self.hand_detector:
                        frame_copy = frame.copy()  # Work on copy for hand detection
                        self.hand_detector.find_hands(frame_copy, draw=False)
                        lm_list = self.hand_detector.find_position(frame_copy, draw=False)
                        
                        if lm_list:
                            gesture = "none"
                            if self.hand_detector.is_waving(lm_list, self.index_x_history):
                                gesture = "waving"
                            elif self.hand_detector.is_shaking_hands(lm_list):
                                gesture = "shaking_hands"
                            
                            with self.state_lock:
                                if self.state.hand_gesture != gesture and gesture != "none":
                                    self.state.hand_gesture = gesture
                                    self.state.last_speech_output = f"I see you're {gesture.replace('_', ' ')}!"
                                    # Non-blocking speech
                                    if self.config.enable_speech_synthesis and self.face_helper:
                                        threading.Thread(
                                            target=self.face_helper.speak, 
                                            args=(f"I see you're {gesture.replace('_', ' ')}!",),
                                            daemon=True
                                        ).start()
                                    logger.info(f"Hand gesture detected: {gesture}")
                        else:
                            # Reset gesture if no hands detected
                            with self.state_lock:
                                if self.state.hand_gesture != "none":
                                    self.state.hand_gesture = "none"
                    
                    # Face recognition every 2 seconds and only if no active gestures
                    current_time = time.time()
                    if (current_time - last_user_check > 2.0 and 
                        self.face_helper and 
                        self.state.hand_gesture == "none"):
                        
                        last_user_check = current_time
                        try:
                            current_user = self.face_helper.find_match(frame, "auto")
                            if current_user and current_user != "Unknown":
                                with self.state_lock:
                                    if self.state.current_user != current_user:
                                        self.state.current_user = current_user
                                        self.state.last_speech_output = f"Hello {current_user}!"
                                        # Non-blocking speech
                                        if self.config.enable_speech_synthesis:
                                            threading.Thread(
                                                target=self.face_helper.speak,
                                                args=(f"Hello {current_user}!",),
                                                daemon=True
                                            ).start()
                                        logger.info(f"User recognized: {current_user}")
                        except Exception as e:
                            logger.error(f"Face recognition error: {e}")
                    
                    # Store current frame for web streaming
                    self.current_frame = frame.copy()
                    
                    time.sleep(0.033)  # ~30 FPS
                    
                else:
                    logger.warning("Camera not available")
                    time.sleep(0.5)
                    
            except Exception as e:
                logger.error(f"Unified vision processing error: {e}")
                time.sleep(0.1)
    
    def _obstacle_detection_loop(self):
        """Background obstacle detection loop"""
        while not self.shutdown_event.is_set():
            try:
                sensor_readings = self._get_all_distances_with_names()
                
                if sensor_readings:
                    with self.state_lock:
                        self.state.sensor_distances = sensor_readings.copy()
                        self.state.last_distances = list(sensor_readings.values())
                    
                    valid_distances = [d for d in sensor_readings.values() if d > 0]
                    
                    if valid_distances:
                        min_distance = min(valid_distances)
                        closest_sensor = ""
                        
                        for sensor_name, distance in sensor_readings.items():
                            if distance == min_distance and distance > 0:
                                closest_sensor = sensor_name
                                break
                        
                        # Check for new obstacle detection
                        if min_distance < self.config.obstacle_threshold and not self.state.obstacle_detected:
                            self._update_obstacle_state(True, closest_sensor, min_distance)
                            self.emergency_stop()
                            
                        # Check for obstacle clearing
                        elif self.state.obstacle_detected and min_distance > self.config.obstacle_clear_threshold:
                            logger.info(f"Obstacle cleared! All sensors show > {self.config.obstacle_clear_threshold}cm")
                            self._update_obstacle_state(False)
                
                time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Error in obstacle detection: {e}")
                time.sleep(0.1)
    
    def _status_update_loop(self):
        """Background status update loop"""
        while not self.shutdown_event.is_set():
            try:
                with self.state_lock:
                    self.state.uptime = time.time() - self.start_time
                    
                    # Update status based on current state
                    if not self.state.obstacle_detected:
                        if self._is_moving():
                            self.state.status = "moving"
                        else:
                            self.state.status = "idle"
                
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in status update: {e}")
                time.sleep(0.1)
    
    def _update_obstacle_state(self, detected: bool, sensor: str = "", distance: float = 0.0):
        """Update obstacle detection state"""
        with self.state_lock:
            self.state.obstacle_detected = detected
            self.state.obstacle_sensor = sensor
            self.state.obstacle_distance = distance
            
            if detected:
                self.state.status = "obstacle_detected"
                self.obstacle_event.set()
                message = f"Obstacle detected by {sensor} sensor at {distance:.1f}cm!"
                self.state.last_speech_output = message
                if self.config.enable_speech_synthesis and self.face_helper:
                    threading.Thread(
                        target=self.face_helper.speak,
                        args=("Obstacle detected! Stopping.",),
                        daemon=True
                    ).start()
                logger.warning(message)
            else:
                self.obstacle_event.clear()
                if self.state.status == "obstacle_detected":
                    self.state.status = "idle"
                logger.info("Obstacle detection cleared")
    
    def _is_moving(self) -> bool:
        """Check if robot is currently moving"""
        return any([
            self.state.motor1_speed > 0,
            self.state.motor2_speed > 0,
            self.state.motor3_speed > 0
        ])
    
    def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
        """Get distance from ultrasonic sensor"""
        if not self.gpio_initialized:
            return 100.0  # Simulation mode
        
        try:
            GPIO.output(trig_pin, False)
            time.sleep(0.000002)
            
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
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
            
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
            
            if 2 <= distance <= 400:
                return distance
            else:
                return -1
                
        except Exception as e:
            logger.error(f"Distance measurement error: {e}")
            return -1
    
    def _get_all_distances_with_names(self) -> Dict[str, float]:
        """Get distances from all sensors with names"""
        sensor_configs = [
            (self.config.trig1, self.config.echo1, "front"),
            (self.config.trig2, self.config.echo2, "left"),
            (self.config.trig3, self.config.echo3, "right")
        ]
        
        sensor_readings = {}
        for trig, echo, name in sensor_configs:
            dist = self._get_distance(trig, echo)
            if dist > 0:
                sensor_readings[name] = dist
        
        return sensor_readings
    
    def _change_speeds_smooth(self, new_speeds: Tuple[int, int, int]):
        """Smoothly change motor speeds"""
        if not self.gpio_initialized:
            with self.state_lock:
                self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
            return
        
        with self.movement_lock:
            with self.state_lock:
                if self.state.obstacle_detected or self.interrupt_event.is_set():
                    self._stop_motors_immediate()
                    return
                
                current_speeds = (self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed)
            
            speed_diffs = [abs(new_speeds[i] - current_speeds[i]) for i in range(3)]
            steps = max(speed_diffs) if speed_diffs else 0
            
            if steps < 1:
                steps = 1
            
            for step in range(steps + 1):
                with self.state_lock:
                    if self.state.obstacle_detected or self.interrupt_event.is_set():
                        self._stop_motors_immediate()
                        return
                
                progress = step / steps if steps > 0 else 1
                speeds = [
                    int(current_speeds[i] + (new_speeds[i] - current_speeds[i]) * progress)
                    for i in range(3)
                ]
                
                speeds = [max(0, min(100, speed)) for speed in speeds]
                
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
                
                time.sleep(0.01)
            
            with self.state_lock:
                if not self.state.obstacle_detected and not self.interrupt_event.is_set():
                    self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
                else:
                    self._stop_motors_immediate()
    
    def _stop_motors_immediate(self):
        """Immediately stop all motors"""
        try:
            if self.gpio_initialized and all([self.motor1_pwm, self.motor2_pwm, self.motor3_pwm]):
                self.motor1_pwm.ChangeDutyCycle(0)
                self.motor2_pwm.ChangeDutyCycle(0)
                self.motor3_pwm.ChangeDutyCycle(0)
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
        
        self.state.motor1_speed = 0
        self.state.motor2_speed = 0
        self.state.motor3_speed = 0
    
    # Public API methods
    
    def get_status(self) -> Dict:
        """Get current robot status with enhanced information"""
        with self.state_lock:
            obstacle_message = "Robot operational"
            if self.state.obstacle_detected:
                obstacle_message = f"Obstacle detected by {self.state.obstacle_sensor} sensor at {self.state.obstacle_distance:.1f}cm"
            
            return {
                "status": self.state.status,
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
                "gpio_available": self.gpio_initialized,
                # Enhanced status fields
                "current_user": self.state.current_user,
                "faces_detected": self.state.faces_detected.copy(),
                "hand_gesture": self.state.hand_gesture,
                "camera_active": self.state.camera_active,
                "last_speech_output": self.state.last_speech_output,
                "listening": self.state.listening,
                "speech_recognition_active": self.state.speech_recognition_active,
                "interaction_mode": self.state.interaction_mode,
                "face_recognition_available": FACE_RECOGNITION_AVAILABLE,
                "speech_recognition_available": SPEECH_RECOGNITION_AVAILABLE,
                "mediapipe_available": MEDIAPIPE_AVAILABLE
            }
    
    def speak(self, text: str) -> bool:
        """Make robot speak text"""
        try:
            with self.state_lock:
                self.state.last_speech_output = text
            
            if self.config.enable_speech_synthesis and self.face_helper:
                # Use threading to prevent blocking
                threading.Thread(
                    target=self.face_helper.speak,
                    args=(text,),
                    daemon=True
                ).start()
                logger.info(f"Robot speaking: {text}")
                return True
            else:
                logger.info(f"Speech synthesis disabled. Would say: {text}")
                return False
        except Exception as e:
            logger.error(f"Speech synthesis error: {e}")
            return False
    
    def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
        """Listen for speech input"""
        if not self.config.enable_speech_recognition or not self.face_helper:
            return None
        
        try:
            with self.state_lock:
                self.state.listening = True
                self.state.speech_recognition_active = True
            
            result = self.face_helper.listen()
            
            with self.state_lock:
                self.state.listening = False
                self.state.speech_recognition_active = False
            
            return result
        except Exception as e:
            logger.error(f"Speech recognition error: {e}")
            with self.state_lock:
                self.state.listening = False
                self.state.speech_recognition_active = False
            return None
    
    def set_interaction_mode(self, mode: str) -> bool:
        """Set interaction mode (speech, text, keyboard, auto)"""
        valid_modes = ["idle", "speech", "text", "keyboard", "auto"]
        if mode in valid_modes:
            with self.state_lock:
                self.state.interaction_mode = mode
            logger.info(f"Interaction mode set to: {mode}")
            return True
        else:
            logger.warning(f"Invalid interaction mode: {mode}")
            return False
    
    def recognize_user(self, mode: str = "auto") -> Optional[str]:
        """Recognize current user using face recognition"""
        if not self.face_helper or not self.current_frame:
            return None
        
        try:
            user = self.face_helper.find_match(self.current_frame, mode)
            if user and user != "Unknown":
                with self.state_lock:
                    self.state.current_user = user
                return user
            return None
        except Exception as e:
            logger.error(f"Face recognition error: {e}")
            return None
    
    def register_new_user(self, name: str) -> bool:
        """Register a new user with face recognition"""
        if not self.face_helper or not self.current_frame:
            return False
        
        try:
            success = self.face_helper.take_picture(name, self.current_frame)
            if success:
                # Update encodings
                self.face_helper.initialize_face_recognition()
                with self.state_lock:
                    self.state.current_user = name
                logger.info(f"New user registered: {name}")
                return True
            return False
        except Exception as e:
            logger.error(f"User registration error: {e}")
            return False
    
    def move(self, direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
        """Move robot in specified direction"""
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
        
        mapped_direction = direction_map.get(direction.lower(), direction.lower())
        
        # Check obstacle
        with self.state_lock:
            if self.state.obstacle_detected and mapped_direction != "stop":
                message = f"Movement blocked - obstacle detected by {self.state.obstacle_sensor} sensor"
                self.state.last_speech_output = message
                if self.config.enable_speech_synthesis and self.face_helper:
                    threading.Thread(
                        target=self.face_helper.speak,
                        args=("Cannot move, obstacle detected!",),
                        daemon=True
                    ).start()
                logger.warning(message)
                return False
            
            speed = speed or self.config.default_speed
            self.state.last_command = mapped_direction
        
        # Direction configurations
        direction_configs = {
            "forward": {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            "backward": {
                "dirs": [self.HIGH, self.LOW, self.HIGH],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            "turnleft": {
                "dirs": [self.HIGH, self.LOW, self.LOW],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            "turnright": {
                "dirs": [self.LOW, self.HIGH, self.HIGH],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            "moveleft": {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [int(speed * 1.5), speed, 0]
            },
            "moveright": {
                "dirs": [self.LOW, self.LOW, self.HIGH],
                "speeds": [int(speed * 1.5), speed, speed + self.config.motor3_compensate]
            },
            "stop": {
                "dirs": [self.LOW, self.LOW, self.LOW],
                "speeds": [0, 0, 0]
            }
        }
        
        if mapped_direction not in direction_configs:
            logger.error(f"Invalid direction: {direction}")
            return False
        
        config = direction_configs[mapped_direction]
        
        # Set motor directions
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
        return self.move("stop")
    
    def emergency_stop(self) -> bool:
        """Emergency stop with speech notification"""
        logger.warning("Emergency stop activated")
        
        with self.movement_lock:
            with self.state_lock:
                self.state.status = "emergency_stop"
                self.state.last_speech_output = "Emergency stop activated"
                self._stop_motors_immediate()
        
        if self.config.enable_speech_synthesis and self.face_helper:
            threading.Thread(
                target=self.face_helper.speak,
                args=("Emergency stop activated",),
                daemon=True
            ).start()
        
        return True
    
    def reset_obstacle_detection(self) -> bool:
        """Reset obstacle detection state"""
        self._update_obstacle_state(False)
        message = "Obstacle detection reset - robot can move again"
        with self.state_lock:
            self.state.last_speech_output = message
        if self.config.enable_speech_synthesis and self.face_helper:
            threading.Thread(
                target=self.face_helper.speak,
                args=("Obstacle cleared, ready to move",),
                daemon=True
            ).start()
        logger.info(message)
        return True
    
    def parse_command(self, command: str) -> Optional[Tuple[str, Optional[int]]]:
        """Parse natural language command"""
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
    
    async def chat(self, message: str, context: str = "") -> str:
        """Chat with AI model and respond with speech"""
        if not self.ai_chain:
            response = "AI model not available. Please check Ollama installation."
        else:
            try:
                result = self.ai_chain.invoke({"context": context, "question": message})
                response = str(result)
            except Exception as e:
                logger.error(f"AI chat error: {e}")
                response = "Error processing message with AI model."
        
        # Update state and speak response
        with self.state_lock:
            self.state.last_speech_output = response
        
        if self.config.enable_speech_synthesis and self.face_helper:
            threading.Thread(
                target=self.face_helper.speak,
                args=(response,),
                daemon=True
            ).start()
        
        return response
    
    async def handle_conversation_mode(self, mode: str) -> str:
        """Handle different conversation modes (speech, text, auto)"""
        try:
            with self.state_lock:
                self.state.interaction_mode = mode
            
            if mode == "speech":
                # Recognize user first
                user = self.recognize_user("speech")
                if user:
                    greeting = f"Hi {user}, what would you like to ask today?"
                    self.speak(greeting)
                    
                    # Listen for commands
                    user_input = self.listen_for_speech()
                    if user_input:
                        # Try to parse as movement command first
                        parsed = self.parse_command(user_input)
                        if parsed:
                            direction, duration = parsed
                            success = self.move(direction, duration_ms=duration)
                            response = f"Movement command {direction} {'executed' if success else 'failed'}"
                        else:
                            # Use AI for general conversation
                            response = await self.chat(user_input)
                        
                        return response
                    else:
                        return "I didn't hear anything clearly."
                else:
                    return "I couldn't recognize you. Please look at the camera."
            
            elif mode == "text":
                user = self.recognize_user("text")
                if user:
                    greeting = f"Hi {user}, what would you like to ask today?"
                    self.speak(greeting)
                    return greeting
                else:
                    return "Please look at the camera for identification."
            
            elif mode == "auto":
                # Automatic interaction based on detected gestures/faces
                if self.state.hand_gesture in ["waving", "shaking_hands"]:
                    response = f"I see you're {self.state.hand_gesture.replace('_', ' ')}! How can I help you?"
                    self.speak(response)
                    return response
                elif self.state.current_user != "Unknown":
                    response = f"Hello {self.state.current_user}! I'm ready for commands."
                    self.speak(response)
                    return response
                else:
                    return "I'm watching and ready to help!"
            
            return "Unknown conversation mode"
            
        except Exception as e:
            logger.error(f"Conversation mode error: {e}")
            return f"Error in conversation mode: {str(e)}"
    
    def get_camera_frame(self) -> Optional[bytes]:
        """Get current camera frame for web streaming"""
        if not self.camera or not self.camera.isOpened() or self.current_frame is None:
            return None
        
        try:
            frame = self.current_frame.copy()
            
            # Add hand detection visualization if enabled
            if self.hand_detector and self.state.hand_gesture != "none":
                cv2.putText(frame, f"Gesture: {self.state.hand_gesture.replace('_', ' ')}", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Add status text overlay
            cv2.putText(frame, f"User: {self.state.current_user}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Status: {self.state.status}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                return buffer.tobytes()
            
            return None
        except Exception as e:
            logger.error(f"Camera frame error: {e}")
            return None
    
    def shutdown(self):
        """Shutdown enhanced robot controller with cleanup"""
        logger.info("Shutting down enhanced robot controller...")
        
        # Signal shutdown to all threads
        self.shutdown_event.set()
        
        # Stop all motors immediately
        with self.movement_lock:
            with self.state_lock:
                self._stop_motors_immediate()
        
        # Wait for background threads to finish
        threads_to_join = [
            (self.obstacle_thread, "obstacle detection"),
            (self.status_thread, "status update"),
            (self.vision_thread, "unified vision")
        ]
        
        for thread, name in threads_to_join:
            if thread and thread.is_alive():
                logger.info(f"Waiting for {name} thread to finish...")
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"{name} thread did not finish gracefully")
        
        # Cleanup camera
        if self.camera and self.camera.isOpened():
            try:
                self.camera.release()
                logger.info("Camera released")
            except Exception as e:
                logger.error(f"Camera cleanup error: {e}")
        
        # Cleanup GPIO and PWM
        if self.gpio_initialized:
            try:
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
                
                GPIO.cleanup()
                logger.info("GPIO cleanup completed")
                
            except Exception as e:
                logger.error(f"GPIO cleanup error: {e}")
        
        logger.info("Enhanced robot controller shutdown complete")


# Global enhanced robot controller instance
_enhanced_robot_controller = None
_enhanced_controller_lock = threading.Lock()

def get_enhanced_robot_controller():
    """Get or create the global enhanced robot controller instance"""
    global _enhanced_robot_controller
    with _enhanced_controller_lock:
        if _enhanced_robot_controller is None:
            _enhanced_robot_controller = EnhancedRobotController()
        return _enhanced_robot_controller

# Public API functions for FastAPI backend compatibility
def get_status() -> Dict:
    """Get current robot status - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().get_status()
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        return {
            "status": "error",
            "message": f"Error getting status: {str(e)}",
            "obstacle_detected": False,
            "gpio_available": False,
            "camera_active": False,
            "face_recognition_available": False,
            "speech_recognition_available": False
        }

def move(direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
    """Move robot in specified direction - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().move(direction, speed, duration_ms)
    except Exception as e:
        logger.error(f"Error moving robot: {e}")
        return False

def stop() -> bool:
    """Stop the robot - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().stop()
    except Exception as e:
        logger.error(f"Error stopping robot: {e}")
        return False

def speak(text: str) -> bool:
    """Make robot speak - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().speak(text)
    except Exception as e:
        logger.error(f"Error speaking: {e}")
        return False

def listen_for_speech(timeout: int = 5) -> Optional[str]:
    """Listen for speech input - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().listen_for_speech(timeout)
    except Exception as e:
        logger.error(f"Error listening for speech: {e}")
        return None

def recognize_user(mode: str = "auto") -> Optional[str]:
    """Recognize current user - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().recognize_user(mode)
    except Exception as e:
        logger.error(f"Error recognizing user: {e}")
        return None

def register_new_user(name: str) -> bool:
    """Register new user - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().register_new_user(name)
    except Exception as e:
        logger.error(f"Error registering user: {e}")
        return False

def set_interaction_mode(mode: str) -> bool:
    """Set interaction mode - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().set_interaction_mode(mode)
    except Exception as e:
        logger.error(f"Error setting interaction mode: {e}")
        return False

async def handle_conversation_mode(mode: str) -> str:
    """Handle conversation mode - called by FastAPI"""
    try:
        return await get_enhanced_robot_controller().handle_conversation_mode(mode)
    except Exception as e:
        logger.error(f"Error handling conversation mode: {e}")
        return f"Error: {str(e)}"

def get_camera_frame() -> Optional[bytes]:
    """Get camera frame - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().get_camera_frame()
    except Exception as e:
        logger.error(f"Error getting camera frame: {e}")
        return None

def parse_command(command: str) -> Optional[Tuple[str, Optional[int]]]:
    """Parse natural language command - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().parse_command(command)
    except Exception as e:
        logger.error(f"Error parsing command: {e}")
        return None

async def chat(message: str, context: str = "") -> str:
    """Chat with AI model - called by FastAPI"""
    try:
        return await get_enhanced_robot_controller().chat(message, context)
    except Exception as e:
        logger.error(f"Error in chat: {e}")
        return "Error processing chat message."

def emergency_stop() -> bool:
    """Emergency stop - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().emergency_stop()
    except Exception as e:
        logger.error(f"Error in emergency stop: {e}")
        return False

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().reset_obstacle_detection()
    except Exception as e:
        logger.error(f"Error resetting obstacle detection: {e}")
        return False

def shutdown_robot():
    """Shutdown robot controller - called by FastAPI on app shutdown"""
    global _enhanced_robot_controller
    with _enhanced_controller_lock:
        if _enhanced_robot_controller is not None:
            _enhanced_robot_controller.shutdown()
            _enhanced_robot_controller = None

if __name__ == "__main__":
    print("🤖 Enhanced Robot Movement Controller - Fixed Version")
    print("=" * 60)
    print("Features:")
    print("• Fixed camera access and threading issues")
    print("• Unified vision loop for face and hand detection") 
    print("• Non-blocking speech synthesis")
    print("• Improved camera initialization with fallbacks")
    print("• Better error handling and logging")
    print("• Enhanced obstacle detection")
    print("• AI integration with speech responses")
    print("• Camera streaming support")
    print("• FastAPI integration ready")
    print("=" * 60)
    print("✅ Enhanced robot controller module loaded successfully")