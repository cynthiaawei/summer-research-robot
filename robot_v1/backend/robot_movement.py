"""
robot_movement.py - Unified module integrating face recognition, hand detection, and robot movement
This module provides the interface expected by the FastAPI web application.
"""

import threading
import time
import cv2
import asyncio
import json
import logging
import numpy as np
from collections import deque
from typing import Optional, Dict, List, Tuple, Any
import traceback

# Import your existing modules
try:
    import hand
    import face_helper as FR
    import movement_RGB as movement
    MODULES_AVAILABLE = True
except ImportError as e:
    logging.error(f"Failed to import required modules: {e}")
    MODULES_AVAILABLE = False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotController:
    """Unified robot controller integrating all functionality"""
    
    def __init__(self):
        self.cap = None
        self.camera_active = False
        self.hand_detector = None
        self.hand_history = deque(maxlen=10)
        self.current_user = "Unknown"
        self.faces_detected = []
        self.hand_gesture = "none"
        self.last_speech_output = ""
        self.listening = False
        self.speech_recognition_active = False
        self.interaction_mode = "idle"
        self.face_recognition_attempts = 0
        self.awaiting_registration = False
        self.face_recognition_available = False
        self.speech_recognition_available = False
        self.mediapipe_available = False
        self.gpio_available = False
        
        # Threading
        self.hand_thread = None
        self.face_thread = None
        self.camera_thread = None
        self.movement_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        # Status tracking
        self.last_distances = []
        self.obstacle_detected = False
        self.current_speeds = {"motor1": 0, "motor2": 0, "motor3": 0}
        self.last_command = ""
        self.start_time = time.time()
        
        # Initialize if modules available
        if MODULES_AVAILABLE:
            self._initialize_components()
    
    def _initialize_components(self):
        """Initialize all components"""
        try:
            # Initialize camera
            self.cap = cv2.VideoCapture(0, cv2.CAP_ANY)
            if self.cap.isOpened():
                self.camera_active = True
                logger.info("✅ Camera initialized successfully")
            else:
                logger.error("❌ Failed to initialize camera")
                return
            
            # Initialize hand detector
            try:
                self.hand_detector = hand.handDetector()
                self.mediapipe_available = True
                logger.info("✅ Hand detector initialized")
            except Exception as e:
                logger.error(f"❌ Hand detector initialization failed: {e}")
            
            # Initialize face recognition
            try:
                # Set up face helper to use our camera
                FR.cap = self.cap
                self.face_recognition_available = True
                logger.info("✅ Face recognition initialized")
            except Exception as e:
                logger.error(f"❌ Face recognition initialization failed: {e}")
            
            # Check for speech recognition
            try:
                import speech_recognition as sr
                self.speech_recognition_available = True
                logger.info("✅ Speech recognition available")
            except ImportError:
                logger.error("❌ Speech recognition not available")
            
            # Check for GPIO
            try:
                import RPi.GPIO as GPIO
                self.gpio_available = True
                logger.info("✅ GPIO available")
            except ImportError:
                logger.info("ℹ️ GPIO not available (simulation mode)")
            
            # Start background threads
            self._start_background_threads()
            
        except Exception as e:
            logger.error(f"❌ Component initialization failed: {e}")
            traceback.print_exc()
    
    def _start_background_threads(self):
        """Start background processing threads"""
        if not self.camera_active:
            return
            
        self.running = True
        
        # Start hand detection thread
        if self.mediapipe_available:
            self.hand_thread = threading.Thread(target=self._hand_thread_worker, daemon=True)
            self.hand_thread.start()
            logger.info("✅ Hand detection thread started")
        
        # Start movement/conversation thread
        self.movement_thread = threading.Thread(target=self._movement_thread_worker, daemon=True)
        self.movement_thread.start()
        logger.info("✅ Movement thread started")
    
    def _hand_thread_worker(self):
        """Hand detection worker thread"""
        while self.running and self.camera_active:
            try:
                success, frame = self.cap.read()
                if not success:
                    time.sleep(0.03)
                    continue
                
                # Process hand gestures
                if self.hand_detector:
                    img = cv2.flip(frame, 1)
                    self.hand_detector.findHands(img, draw=False)
                    lmList = self.hand_detector.findPosition(img, draw=False)
                    
                    if lmList:
                        if hand.is_waving(lmList, self.hand_detector, self.hand_history):
                            self.hand_gesture = "waving"
                        elif hand.is_shaking_hands(lmList):
                            self.hand_gesture = "shaking_hands"
                        else:
                            self.hand_gesture = "detected"
                    else:
                        self.hand_gesture = "none"
                
                time.sleep(0.03)
                
            except Exception as e:
                logger.error(f"Hand detection error: {e}")
                time.sleep(0.1)
    
    def _movement_thread_worker(self):
        """Movement and conversation worker thread"""
        try:
            # Run the main movement loop
            asyncio.run(movement.main())
        except Exception as e:
            logger.error(f"Movement thread error: {e}")
            traceback.print_exc()
    
    def get_status(self) -> Dict[str, Any]:
        """Get current robot status"""
        return {
            "status": "connected" if self.camera_active else "disconnected",
            "message": "Robot operational" if self.camera_active else "Camera not available",
            "obstacle_detected": self.obstacle_detected,
            "current_speeds": self.current_speeds,
            "last_distances": self.last_distances,
            "last_command": self.last_command,
            "uptime": time.time() - self.start_time,
            "gpio_available": self.gpio_available,
            "current_user": self.current_user,
            "faces_detected": self.faces_detected,
            "hand_gesture": self.hand_gesture,
            "camera_active": self.camera_active,
            "last_speech_output": self.last_speech_output,
            "listening": self.listening,
            "speech_recognition_active": self.speech_recognition_active,
            "interaction_mode": self.interaction_mode,
            "face_recognition_available": self.face_recognition_available,
            "speech_recognition_available": self.speech_recognition_available,
            "mediapipe_available": self.mediapipe_available,
            "face_recognition_attempts": self.face_recognition_attempts,
            "awaiting_registration": self.awaiting_registration
        }
    
    def move(self, direction: str, duration_ms: Optional[int] = None) -> bool:
        """Execute movement command"""
        try:
            self.last_command = f"{direction}"
            if duration_ms:
                self.last_command += f" for {duration_ms}ms"
            
            # Update current speeds based on direction
            if direction == "forward":
                self.current_speeds = {"motor1": 0, "motor2": 25, "motor3": 40}
            elif direction == "backward":
                self.current_speeds = {"motor1": 0, "motor2": 25, "motor3": 40}
            elif direction in ["left", "turnleft"]:
                self.current_speeds = {"motor1": 25, "motor2": 25, "motor3": 40}
            elif direction in ["right", "turnright"]:
                self.current_speeds = {"motor1": 25, "motor2": 25, "motor3": 40}
            elif direction in ["moveleft"]:
                self.current_speeds = {"motor1": 37, "motor2": 25, "motor3": 0}
            elif direction in ["moveright"]:
                self.current_speeds = {"motor1": 37, "motor2": 25, "motor3": 40}
            elif direction == "stop":
                self.current_speeds = {"motor1": 0, "motor2": 0, "motor3": 0}
            
            # If GPIO available, execute actual movement
            if self.gpio_available and hasattr(movement, 'processCommand'):
                if duration_ms:
                    movement.processCommand(direction, duration_ms)
                else:
                    movement.processImmediateCommand(direction)
            
            # Simulate execution time
            if duration_ms:
                time.sleep(duration_ms / 1000)
                self.current_speeds = {"motor1": 0, "motor2": 0, "motor3": 0}
            
            logger.info(f"✅ Movement command executed: {direction}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Movement command failed: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop robot immediately"""
        return self.move("stop")
    
    def speak(self, text: str) -> bool:
        """Make robot speak"""
        try:
            self.last_speech_output = text
            if hasattr(movement, 'speak'):
                movement.speak(text)
            else:
                # Fallback TTS
                import subprocess
                import platform
                system = platform.system().lower()
                if system == "linux":
                    subprocess.run(["espeak", text], check=True)
                elif system == "darwin":
                    subprocess.run(["say", text], check=True)
                else:
                    print(f"🔊 {text}")
            
            logger.info(f"✅ Speech output: {text}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Speech failed: {e}")
            return False
    
    def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
        """Listen for speech input"""
        try:
            self.listening = True
            self.speech_recognition_active = True
            
            if hasattr(movement, 'listen'):
                result = movement.listen()
            else:
                # Fallback speech recognition
                import speech_recognition as sr
                r = sr.Recognizer()
                with sr.Microphone() as source:
                    audio = r.listen(source, timeout=timeout)
                    result = r.recognize_google(audio).lower()
            
            self.listening = False
            self.speech_recognition_active = False
            logger.info(f"✅ Speech input: {result}")
            return result
            
        except Exception as e:
            self.listening = False
            self.speech_recognition_active = False
            logger.error(f"❌ Speech recognition failed: {e}")
            return None
    
    def recognize_user(self, mode: str = "auto") -> Optional[str]:
        """Recognize current user"""
        try:
            if not self.face_recognition_available:
                return None
            
            # Use your face recognition system
            if hasattr(FR, 'findMatch'):
                # Convert mode for your system
                fr_mode = "s" if mode == "speech" else "t"
                name = FR.findMatch(fr_mode)
                if name and name != "UNKNOWN":
                    self.current_user = name
                    self.face_recognition_attempts = 0
                    self.awaiting_registration = False
                    logger.info(f"✅ User recognized: {name}")
                    return name
                else:
                    self.face_recognition_attempts += 1
                    if self.face_recognition_attempts >= 3:
                        self.awaiting_registration = True
                    logger.info(f"❌ User not recognized (attempt {self.face_recognition_attempts}/3)")
                    return None
            
            return None
            
        except Exception as e:
            logger.error(f"❌ User recognition failed: {e}")
            return None
    
    def register_new_user(self, name: str) -> bool:
        """Register new user"""
        try:
            if not self.camera_active:
                return False
            
            # Use your face recognition system to take and save picture
            if hasattr(FR, 'take_picture') and self.cap:
                FR.take_picture(name, self.cap)
                
                # Update the face recognition system with new user
                path = '/home/robot/summer-research-robot/RGB-cam/images'
                new_img_path = f'{path}/{name}.jpg'
                
                if hasattr(FR, 'encodeListKnown') and hasattr(FR, 'classNames'):
                    new_img = cv2.imread(new_img_path)
                    if new_img is not None:
                        import face_recognition
                        new_img_rgb = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
                        new_encoding = face_recognition.face_encodings(new_img_rgb)
                        if new_encoding:
                            FR.encodeListKnown.append(new_encoding[0])
                            FR.classNames.append(name)
                            
                            self.current_user = name
                            self.face_recognition_attempts = 0
                            self.awaiting_registration = False
                            logger.info(f"✅ User registered: {name}")
                            return True
            
            return False
            
        except Exception as e:
            logger.error(f"❌ User registration failed: {e}")
            return False
    
    def reset_face_recognition_state(self) -> bool:
        """Reset face recognition state"""
        try:
            self.face_recognition_attempts = 0
            self.awaiting_registration = False
            self.current_user = "Unknown"
            logger.info("✅ Face recognition state reset")
            return True
        except Exception as e:
            logger.error(f"❌ Face recognition reset failed: {e}")
            return False
    
    def set_interaction_mode(self, mode: str) -> bool:
        """Set interaction mode"""
        try:
            valid_modes = ["idle", "speech", "text", "keyboard", "auto"]
            if mode in valid_modes:
                self.interaction_mode = mode
                logger.info(f"✅ Interaction mode set to: {mode}")
                return True
            return False
        except Exception as e:
            logger.error(f"❌ Set interaction mode failed: {e}")
            return False
    
    async def handle_conversation_mode(self, mode: str) -> str:
        """Handle conversation in specified mode"""
        try:
            if mode == "speech":
                text = self.listen_for_speech()
                if text:
                    response = await self.chat(text)
                    self.speak(response)
                    return response
                return "No speech detected"
            elif mode == "text":
                return "Text mode ready - send message via text command"
            else:
                return "Auto mode active"
        except Exception as e:
            logger.error(f"❌ Conversation mode failed: {e}")
            return f"Error: {str(e)}"
    
    def parse_command(self, text: str) -> Optional[Tuple[str, int]]:
        """Parse natural language command"""
        try:
            # Use your existing parsing logic from movement_RGB
            if hasattr(movement, 'get_direction') and hasattr(movement, 'convert_to_milliseconds'):
                direction = movement.get_direction(text)
                time_ms = movement.convert_to_milliseconds(text)
                
                if direction:
                    # Default turn time
                    if direction in ["turnLeft", "turnRight"] and time_ms is None:
                        time_ms = 1500
                    return (direction, time_ms or 0)
            
            return None
            
        except Exception as e:
            logger.error(f"❌ Command parsing failed: {e}")
            return None
    
    async def chat(self, text: str) -> str:
        """Handle AI chat"""
        try:
            # Use your existing AI chat system
            if hasattr(movement, 'chain') and movement.chain:
                result = movement.chain.invoke({"context": "", "question": text})
                return str(result)
            else:
                return f"I heard: {text}"
        except Exception as e:
            logger.error(f"❌ Chat failed: {e}")
            return "Sorry, I couldn't process that."
    
    def get_camera_frame(self) -> Optional[bytes]:
        """Get current camera frame as JPEG bytes"""
        try:
            if not self.camera_active or not self.cap:
                return None
            
            success, frame = self.cap.read()
            if not success:
                return None
            
            # Add overlays for detected faces and hands
            display_frame = frame.copy()
            
            # Add hand gesture overlay
            if self.hand_gesture != "none":
                cv2.putText(display_frame, f"Gesture: {self.hand_gesture}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add user overlay
            if self.current_user != "Unknown":
                cv2.putText(display_frame, f"User: {self.current_user}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return buffer.tobytes()
            
        except Exception as e:
            logger.error(f"❌ Camera frame capture failed: {e}")
            return None
    
    def reset_obstacle_detection(self) -> bool:
        """Reset obstacle detection"""
        try:
            self.obstacle_detected = False
            if hasattr(movement, 'obstacle_detected'):
                movement.obstacle_detected = False
            if hasattr(movement, 'return_to_mode_selection'):
                movement.return_to_mode_selection = False
            logger.info("✅ Obstacle detection reset")
            return True
        except Exception as e:
            logger.error(f"❌ Obstacle reset failed: {e}")
            return False
    
    def shutdown_robot(self):
        """Shutdown robot controller"""
        try:
            logger.info("🔄 Shutting down robot controller...")
            self.running = False
            
            # Stop motors
            self.move("stop")
            
            # Close camera
            if self.cap:
                self.cap.release()
                self.camera_active = False
            
            # Wait for threads to finish
            if self.hand_thread and self.hand_thread.is_alive():
                self.hand_thread.join(timeout=1)
            
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join(timeout=1)
            
            # GPIO cleanup
            if self.gpio_available:
                try:
                    import RPi.GPIO as GPIO
                    GPIO.cleanup()
                except:
                    pass
            
            logger.info("✅ Robot controller shutdown complete")
            
        except Exception as e:
            logger.error(f"❌ Shutdown error: {e}")

# Create global robot controller instance
robot_controller = RobotController()

# API functions expected by FastAPI app
def get_status() -> Dict[str, Any]:
    """Get robot status"""
    return robot_controller.get_status()

def move(direction: str, duration_ms: Optional[int] = None) -> bool:
    """Move robot"""
    return robot_controller.move(direction, duration_ms)

def stop() -> bool:
    """Stop robot"""
    return robot_controller.stop()

def speak(text: str) -> bool:
    """Make robot speak"""
    return robot_controller.speak(text)

def listen_for_speech(timeout: int = 5) -> Optional[str]:
    """Listen for speech"""
    return robot_controller.listen_for_speech(timeout)

def recognize_user(mode: str = "auto") -> Optional[str]:
    """Recognize user"""
    return robot_controller.recognize_user(mode)

def register_new_user(name: str) -> bool:
    """Register new user"""
    return robot_controller.register_new_user(name)

def reset_face_recognition_state() -> bool:
    """Reset face recognition"""
    return robot_controller.reset_face_recognition_state()

def set_interaction_mode(mode: str) -> bool:
    """Set interaction mode"""
    return robot_controller.set_interaction_mode(mode)

async def handle_conversation_mode(mode: str) -> str:
    """Handle conversation"""
    return await robot_controller.handle_conversation_mode(mode)

def parse_command(text: str) -> Optional[Tuple[str, int]]:
    """Parse command"""
    return robot_controller.parse_command(text)

async def chat(text: str) -> str:
    """Chat with AI"""
    return await robot_controller.chat(text)

def get_camera_frame() -> Optional[bytes]:
    """Get camera frame"""
    return robot_controller.get_camera_frame()

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection"""
    return robot_controller.reset_obstacle_detection()

def shutdown_robot():
    """Shutdown robot"""
    robot_controller.shutdown_robot()

# For compatibility with existing code
if __name__ == "__main__":
    logger.info("🤖 Robot controller module loaded")
    try:
        while True:
            status = get_status()
            logger.info(f"Status: {status['status']}, User: {status['current_user']}, Gesture: {status['hand_gesture']}")
            time.sleep(5)
    except KeyboardInterrupt:
        logger.info("👋 Shutting down...")
        shutdown_robot()
