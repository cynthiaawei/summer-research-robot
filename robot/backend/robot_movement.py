# import asyncio
# import re
# import threading
# import time
# import logging
# import cv2
# import os
# import platform
# import subprocess
# import sys
# from typing import Dict, Optional, List, Tuple, Union
# from dataclasses import dataclass, field
# from enum import Enum
# import json
# from collections import deque

# # Import your existing face_helper module or create FIXED fallback
# try:
#     import face_helper as FR
#     FACE_HELPER_AVAILABLE = True
#     logging.info("Successfully imported face_helper module from current directory")
# except ImportError:
#     try:
#         sys.path.append('/home/robot/summer-research-robot')
#         import face_helper as FR
#         FACE_HELPER_AVAILABLE = True
#         logging.info("Successfully imported face_helper module from parent directory")
#     except ImportError:
#         logging.warning("face_helper not found, creating FIXED fallback")
#         FACE_HELPER_AVAILABLE = True
        
#         class SimpleFaceHelper:
#             def __init__(self):
#                 # FIXED: Use backend/images directory (not facepics!)
#                 self.images_path = '/home/robot/summer-research-robot/RGB-cam/images'
#                 self.cap = None
#                 self.classNames = []
#                 self.encodeListKnown = []
#                 os.makedirs(self.images_path, exist_ok=True)
#                 logging.info(f"Created/verified images directory at {self.images_path}")
            
#             def speak(self, text):
#                 try:
#                     system = platform.system().lower()
#                     if system == "linux":
#                         try:
#                             subprocess.run(["espeak", text], check=True, capture_output=True)
#                         except (subprocess.CalledProcessError, FileNotFoundError):
#                             try:
#                                 subprocess.run(["spd-say", text], check=True, capture_output=True)
#                             except (subprocess.CalledProcessError, FileNotFoundError):
#                                 print(f"🔊 TTS: {text}")
#                     else:
#                         print(f"🔊 TTS: {text}")
#                 except Exception as e:
#                     print(f"🔊 TTS: {text}")
#                     logging.error(f"TTS error: {e}")
            
#             def listen(self):
#                 if not SPEECH_RECOGNITION_AVAILABLE:
#                     return None
#                 try:
#                     import speech_recognition as sr
#                     r = sr.Recognizer()
#                     with sr.Microphone() as source:
#                         print("🎙️ Listening...")
#                         audio = r.listen(source, timeout=5)
#                         text = r.recognize_google(audio)
#                         print(f"🗣️ You said: {text}")
#                         return text.lower()
#                 except Exception as e:
#                     logging.error(f"Speech recognition error: {e}")
#                     return None
            
#             def take_picture(self, name, camera):
#                 """FIXED: Take picture and save to backend/images"""
#                 try:
#                     logging.info(f"📷 Starting picture capture for user: {name}")
#                     if not camera:
#                         logging.error("❌ No camera provided to take_picture")
#                         return False
#                     if not camera.isOpened():
#                         logging.error("❌ Camera is not opened")
#                         return False
                    
#                     # Take the best frame
#                     best_frame = None
#                     for attempt in range(3):
#                         success, image = camera.read()
#                         if success and image is not None:
#                             best_frame = image.copy()
#                             break
                    
#                     if best_frame is None:
#                         logging.error("❌ Failed to capture image from camera")
#                         return False
                    
#                     # Clean filename
#                     clean_name = name.strip().replace(' ', '_').lower()
#                     filename = f'{clean_name}.jpg'
#                     filepath = os.path.join(self.images_path, filename)
                    
#                     logging.info(f"💾 Attempting to save image to: {filepath}")
#                     os.makedirs(os.path.dirname(filepath), exist_ok=True)
                    
#                     result = cv2.imwrite(filepath, best_frame)
#                     if result:
#                         if os.path.exists(filepath):
#                             file_size = os.path.getsize(filepath)
#                             logging.info(f"✅ Picture saved successfully for {name}")
#                             logging.info(f"   📁 Path: {filepath}")
#                             logging.info(f"   📏 Size: {file_size} bytes")
                            
#                             # IMPORTANT: Reload face data immediately
#                             self.reload_face_data()
                            
#                             return True
#                         else:
#                             logging.error(f"❌ File was not saved to {filepath}")
#                             return False
#                     else:
#                         logging.error(f"❌ cv2.imwrite failed for {filepath}")
#                         return False
#                 except Exception as e:
#                     logging.error(f"❌ Error taking picture for {name}: {e}")
#                     logging.error(f"   Exception type: {type(e).__name__}")
#                     import traceback
#                     logging.error(f"   Traceback: {traceback.format_exc()}")
#                     return False
            
#             def reload_face_data(self):
#                 """Reload all face images and regenerate encodings"""
#                 try:
#                     self.images = []
#                     self.classNames = []
                    
#                     if not os.path.exists(self.images_path):
#                         logging.warning(f"Images path does not exist: {self.images_path}")
#                         os.makedirs(self.images_path, exist_ok=True)
#                         return
                    
#                     # Load all images
#                     myList = os.listdir(self.images_path)
#                     for cl in myList:
#                         if cl.lower().endswith(('.png', '.jpg', '.jpeg')):
#                             try:
#                                 full_path = os.path.join(self.images_path, cl)
#                                 cur_img = cv2.imread(full_path)
#                                 if cur_img is not None:
#                                     self.images.append(cur_img)
#                                     self.classNames.append(os.path.splitext(cl)[0])
#                                     logging.info(f"✅ Loaded image for user: {os.path.splitext(cl)[0]}")
#                             except Exception as e:
#                                 logging.error(f"Error loading image {cl}: {e}")
                    
#                     # Generate encodings
#                     self.encodeListKnown = self.findEncodings(self.images)
#                     logging.info(f"📊 Generated {len(self.encodeListKnown)} face encodings total")
                    
#                 except Exception as e:
#                     logging.error(f"Error reloading face data: {e}")
            
#             def findEncodings(self, images):
#                 if not FACE_RECOGNITION_AVAILABLE:
#                     logging.warning("Face recognition not available, returning empty encodings")
#                     return []
#                 encodeList = []
#                 try:
#                     for i, img in enumerate(images):
#                         try:
#                             img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#                             encodings = face_recognition.face_encodings(img_rgb)
#                             if encodings:
#                                 encodeList.append(encodings[0])
#                                 logging.info(f"✅ Generated encoding for image {i+1}")
#                             else:
#                                 logging.warning(f"⚠️ No face found in image {i+1}")
#                         except Exception as e:
#                             logging.error(f"❌ Error encoding image {i+1}: {e}")
#                 except Exception as e:
#                     logging.error(f"❌ Error in findEncodings: {e}")
#                 return encodeList
        
#         FR = SimpleFaceHelper()
#         FR.path = FR.images_path
#         FR.images = []
#         FR.classNames = []
        
#         # Initial load of existing images
#         try:
#             FR.reload_face_data()
#             logging.info(f"Loaded {len(FR.classNames)} known faces: {FR.classNames}")
#         except Exception as e:
#             logging.error(f"Error initializing face recognition: {e}")
#             FR.encodeListKnown = []
#             FR.classNames = []

# # Import other dependencies with fallbacks
# try:
#     import face_recognition
#     import numpy as np
#     FACE_RECOGNITION_AVAILABLE = True
# except ImportError:
#     FACE_RECOGNITION_AVAILABLE = False
#     logging.warning("Face recognition not available - install face_recognition library")

# try:
#     import speech_recognition as sr
#     SPEECH_RECOGNITION_AVAILABLE = True
# except ImportError:
#     SPEECH_RECOGNITION_AVAILABLE = False
#     logging.warning("Speech recognition not available - install SpeechRecognition library")

# try:
#     import mediapipe as mp
#     MEDIAPIPE_AVAILABLE = True
# except ImportError:
#     MEDIAPIPE_AVAILABLE = False
#     logging.warning("MediaPipe not available - hand detection disabled")

# try:
#     import RPi.GPIO as GPIO
#     GPIO_AVAILABLE = True
# except ImportError:
#     GPIO_AVAILABLE = False

# try:
#     from langchain_ollama import OllamaLLM
#     from langchain_core.prompts import ChatPromptTemplate
#     AI_AVAILABLE = True
# except ImportError:
#     AI_AVAILABLE = False

# # Configure logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# # Hand Detection Helper Class (keep as is)
# class HandDetector:
#     """Hand gesture detection using MediaPipe"""
    
#     def __init__(self, mode=False, max_hands=2, detection_con=0.5, track_con=0.5):
#         if not MEDIAPIPE_AVAILABLE:
#             logger.warning("MediaPipe not available - hand detection disabled")
#             return
        
#         self.mode = mode
#         self.max_hands = max_hands
#         self.detection_con = detection_con
#         self.track_con = track_con
        
#         self.mp_hands = mp.solutions.hands
#         self.hands = self.mp_hands.Hands(
#             static_image_mode=self.mode,
#             max_num_hands=self.max_hands,
#             min_detection_confidence=self.detection_con,
#             min_tracking_confidence=self.track_con
#         )
#         self.mp_draw = mp.solutions.drawing_utils
#         self.results = None
    
#     def find_hands(self, img, draw=True):
#         """Find hands in image"""
#         if not MEDIAPIPE_AVAILABLE:
#             return img
        
#         img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#         self.results = self.hands.process(img_rgb)
        
#         if self.results.multi_hand_landmarks:
#             for hand_lms in self.results.multi_hand_landmarks:
#                 if draw:
#                     self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
#         return img
    
#     def find_position(self, img, hand_no=0, draw=True):
#         """Find hand landmark positions"""
#         lm_list = []
#         if self.results and self.results.multi_hand_landmarks:
#             try:
#                 my_hand = self.results.multi_hand_landmarks[hand_no]
#                 for id, lm in enumerate(my_hand.landmark):
#                     h, w, c = img.shape
#                     cx, cy = int(lm.x * w), int(lm.y * h)
#                     lm_list.append([id, cx, cy])
#                     if draw:
#                         cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
#             except IndexError:
#                 pass
#         return lm_list
    
#     def is_waving(self, lm_list, index_x_history):
#         """Detect waving gesture"""
#         if len(lm_list) < 21:
#             return False
        
#         if not self.fingers_up(lm_list):
#             return False
        
#         index_x = lm_list[8][1]
#         index_x_history.append(index_x)
        
#         if len(index_x_history) < 5:
#             return False
        
#         index_motion = max(index_x_history) - min(index_x_history)
#         return index_motion > 40
    
#     def fingers_up(self, lm_list):
#         """Check if fingers are extended (open hand)"""
#         if len(lm_list) < 21:
#             return False
        
#         thumb_up = lm_list[4][1] < lm_list[3][1]
#         fingers = [thumb_up]
#         for id in [8, 12, 16, 20]:
#             if lm_list[id][2] < lm_list[id - 2][2]:
#                 fingers.append(True)
#             else:
#                 fingers.append(False)
        
#         return sum(fingers) >= 3
    
#     def is_shaking_hands(self, lm_list):
#         """Detect handshake gesture"""
#         if len(lm_list) < 21:
#             return False
        
#         tips = [8, 12, 16, 20]
#         joints = [6, 10, 14, 18]
        
#         closed_fingers = 0
#         for tip, joint in zip(tips, joints):
#             if lm_list[tip][2] > lm_list[joint][2]:
#                 closed_fingers += 1
        
#         return closed_fingers >= 2

# # Enhanced Robot Configuration
# @dataclass
# class EnhancedRobotConfig:
#     """Enhanced robot configuration with FIXED paths"""
#     motor1_speed: int = 38
#     motor1_dir: int = 40
#     motor2_speed: int = 32
#     motor2_dir: int = 36
#     motor3_speed: int = 16
#     motor3_dir: int = 26
#     echo1: int = 31
#     echo2: int = 29
#     echo3: int = 22
#     trig1: int = 11
#     trig2: int = 13
#     trig3: int = 15
#     pwm_frequency: int = 1000
#     default_speed: int = 25
#     motor3_compensate: int = 3
#     obstacle_threshold: float = 30.0
#     sensor_timeout: float = 0.5
#     obstacle_clear_threshold: float = 50.0
#     camera_index: int = 0
#     # FIXED: Use correct backend images path
#     face_images_path: str = 'images'  # Relative to backend directory
#     enable_face_recognition: bool = True
#     enable_hand_detection: bool = True
#     enable_speech_synthesis: bool = True
#     enable_speech_recognition: bool = True
#     ai_model: str = "llama3"

# # Enhanced Robot State (keep as is)
# @dataclass
# class EnhancedRobotState:
#     """Enhanced robot state with vision and speech data"""
#     motor1_speed: int = 0
#     motor2_speed: int = 0
#     motor3_speed: int = 0
#     status: str = "idle"
#     obstacle_detected: bool = False
#     last_distances: List[float] = field(default_factory=list)
#     sensor_distances: Dict[str, float] = field(default_factory=lambda: {"front": 0.0, "left": 0.0, "right": 0.0})
#     obstacle_sensor: str = ""
#     obstacle_distance: float = 0.0
#     last_command: str = ""
#     uptime: float = 0.0
#     current_user: str = "Unknown"
#     faces_detected: List[str] = field(default_factory=list)
#     hand_gesture: str = "none"
#     camera_active: bool = False
#     last_speech_output: str = ""
#     listening: bool = False
#     speech_recognition_active: bool = False
#     interaction_mode: str = "idle"
#     face_recognition_attempts: int = 0
#     awaiting_registration: bool = False
#     recognition_complete: bool = False

# class EnhancedRobotController:
#     """Enhanced robot controller with FIXED face recognition"""
    
#     def __init__(self, config: Optional[EnhancedRobotConfig] = None):
#         self.config = config or EnhancedRobotConfig()
#         self.state = EnhancedRobotState()
#         self.start_time = time.time()
#         self.movement_lock = threading.RLock()
#         self.state_lock = threading.RLock()
#         self.vision_lock = threading.RLock()
#         self.interrupt_event = threading.Event()
#         self.obstacle_event = threading.Event()
#         self.shutdown_event = threading.Event()
#         self.obstacle_thread = None
#         self.status_thread = None
#         self.vision_thread = None
#         self.hand_detector = None
#         self.camera = None
#         self.index_x_history = deque(maxlen=10)
#         self.current_frame = None
#         self.motor1_pwm = None
#         self.motor2_pwm = None
#         self.motor3_pwm = None
#         self.HIGH = GPIO.HIGH if GPIO_AVAILABLE else 1
#         self.LOW = GPIO.LOW if GPIO_AVAILABLE else 0
#         self.gpio_initialized = False
#         self._setup_gpio()
#         self._setup_ai()
#         self._setup_vision()
#         self._start_background_tasks()
#         logger.info("Enhanced robot controller initialized successfully")
    
#     # Keep all your existing setup methods as they are...
#     def _setup_vision(self):
#         """Setup camera and vision components with better error handling"""
#         if not self.config.enable_face_recognition and not self.config.enable_hand_detection:
#             logger.info("Vision components disabled in config")
#             return
        
#         try:
#             if FACE_HELPER_AVAILABLE:
#                 try:
#                     FR.cap = cv2.VideoCapture(0)
#                     if FR.cap.isOpened():
#                         logger.info("Face helper camera initialized successfully")
#                         with self.state_lock:
#                             self.state.camera_active = True
#                         self.camera = FR.cap
#                     else:
#                         logger.error("Failed to initialize face helper camera")
#                         with self.state_lock:
#                             self.state.camera_active = False
#                 except Exception as e:
#                     logger.error(f"Face helper camera setup failed: {e}")
#                     with self.state_lock:
#                         self.state.camera_active = False
#             else:
#                 # Fallback camera setup
#                 camera_indices = [0, 1, 2, -1]
#                 camera_initialized = False
                
#                 for idx in camera_indices:
#                     try:
#                         logger.info(f"Trying camera index {idx}")
#                         self.camera = cv2.VideoCapture(idx)
                        
#                         if self.camera.isOpened():
#                             self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#                             self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#                             self.camera.set(cv2.CAP_PROP_FPS, 30)
#                             self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#                             ret, test_frame = self.camera.read()
#                             if ret and test_frame is not None:
#                                 with self.state_lock:
#                                     self.state.camera_active = True
#                                 self.config.camera_index = idx
#                                 camera_initialized = True
#                                 logger.info(f"Camera initialized successfully on index {idx}")
#                                 break
#                             else:
#                                 self.camera.release()
#                     except Exception as e:
#                         logger.warning(f"Failed to initialize camera on index {idx}: {e}")
#                         if self.camera:
#                             self.camera.release()
#                         continue
                
#                 if not camera_initialized:
#                     logger.error("Failed to initialize camera on any index")
#                     with self.state_lock:
#                         self.state.camera_active = False
#                     return
            
#             if self.config.enable_hand_detection and MEDIAPIPE_AVAILABLE:
#                 self.hand_detector = HandDetector()
#                 logger.info("Hand detection initialized")
                
#         except Exception as e:
#             logger.error(f"Vision setup failed: {e}")
#             with self.state_lock:
#                 self.state.camera_active = False
    
#     # Keep all your other setup methods unchanged...
#     def _setup_gpio(self):
#         """Setup GPIO pins and PWM"""
#         if not GPIO_AVAILABLE:
#             logger.warning("GPIO not available - running in simulation mode")
#             return
        
#         try:
#             GPIO.setmode(GPIO.BOARD)
#             GPIO.setup(self.config.echo1, GPIO.IN)
#             GPIO.setup(self.config.echo2, GPIO.IN)
#             GPIO.setup(self.config.echo3, GPIO.IN)
#             GPIO.setup(self.config.trig1, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.trig2, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.trig3, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor1_speed, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor1_dir, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor2_speed, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor2_dir, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor3_speed, GPIO.OUT, initial=GPIO.LOW)
#             GPIO.setup(self.config.motor3_dir, GPIO.OUT, initial=GPIO.LOW)
#             self.motor1_pwm = GPIO.PWM(self.config.motor1_speed, self.config.pwm_frequency)
#             self.motor2_pwm = GPIO.PWM(self.config.motor2_speed, self.config.pwm_frequency)
#             self.motor3_pwm = GPIO.PWM(self.config.motor3_speed, self.config.pwm_frequency)
#             self.motor1_pwm.start(0)
#             self.motor2_pwm.start(0)
#             self.motor3_pwm.start(0)
#             self.gpio_initialized = True
#             logger.info("GPIO initialized successfully")
#         except Exception as e:
#             logger.error(f"GPIO initialization failed: {e}")
#             self.gpio_initialized = False
    
#     def _setup_ai(self):
#         """Setup AI model for conversation"""
#         if not AI_AVAILABLE:
#             logger.warning("AI not available")
#             self.ai_chain = None
#             return
        
#         try:
#             template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
#             model = OllamaLLM(model=self.config.ai_model)
#             prompt = ChatPromptTemplate.from_template(template)
#             self.ai_chain = prompt | model
#             logger.info("AI model initialized successfully")
#         except Exception as e:
#             logger.error(f"AI initialization failed: {e}")
#             self.ai_chain = None
    
#     def _start_background_tasks(self):
#         """Start background monitoring tasks"""
#         try:
#             self.obstacle_thread = threading.Thread(
#                 target=self._obstacle_detection_loop, 
#                 daemon=True, 
#                 name="ObstacleDetection"
#             )
#             self.obstacle_thread.start()
#             self.status_thread = threading.Thread(
#                 target=self._status_update_loop, 
#                 daemon=True, 
#                 name="StatusUpdate"
#             )
#             self.status_thread.start()
#             if self.state.camera_active:
#                 self.vision_thread = threading.Thread(
#                     target=self._unified_vision_loop,
#                     daemon=True,
#                     name="UnifiedVision"
#                 )
#                 self.vision_thread.start()
#             logger.info("Background tasks started successfully")
#         except Exception as e:
#             logger.error(f"Failed to start background tasks: {e}")
    
#     def _unified_vision_loop(self):
#         """COMPLETELY FIXED: Unified vision processing loop"""
#         last_user_check = time.time()
#         last_gesture = "none"
#         gesture_cooldown = time.time()
#         logger.info("🔍 Starting unified vision loop with COMPLETELY FIXED face recognition")
        
#         while not self.shutdown_event.is_set():
#             try:
#                 if self.camera and self.camera.isOpened():
#                     success, frame = self.camera.read()
#                     if not success:
#                         logger.warning("Failed to read camera frame")
#                         time.sleep(0.1)
#                         continue
                    
#                     frame = cv2.flip(frame, 1)
#                     current_time = time.time()
                    
#                     # Hand detection (keep as is)
#                     if self.hand_detector:
#                         frame_copy = frame.copy()
#                         self.hand_detector.find_hands(frame_copy, draw=False)
#                         lm_list = self.hand_detector.find_position(frame_copy, draw=False)
                        
#                         if lm_list:
#                             gesture = "none"
#                             if self.hand_detector.is_waving(lm_list, self.index_x_history):
#                                 gesture = "waving"
#                             elif self.hand_detector.is_shaking_hands(lm_list):
#                                 gesture = "shaking_hands"
                            
#                             if (gesture != "none" and 
#                                 gesture != last_gesture and 
#                                 current_time - gesture_cooldown > 2.0):
                                
#                                 with self.state_lock:
#                                     self.state.hand_gesture = gesture
#                                     self.state.last_speech_output = f"I see you're {gesture.replace('_', ' ')}!"
                                    
#                                 if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#                                     threading.Thread(
#                                         target=FR.speak, 
#                                         args=(f"I see you're {gesture.replace('_', ' ')}!",),
#                                         daemon=True
#                                     ).start()
                                
#                                 logger.info(f"Hand gesture detected: {gesture}")
#                                 last_gesture = gesture
#                                 gesture_cooldown = current_time
#                         else:
#                             if current_time - gesture_cooldown > 1.0:
#                                 with self.state_lock:
#                                     if self.state.hand_gesture != "none":
#                                         self.state.hand_gesture = "none"
#                                 last_gesture = "none"
                    
#                     # COMPLETELY FIXED: Face recognition 
#                     with self.state_lock:
#                         recognition_complete = self.state.recognition_complete
#                         current_attempts = self.state.face_recognition_attempts
#                         awaiting_registration = self.state.awaiting_registration
                    
#                     if (current_time - last_user_check > 2.0 and 
#                         FACE_HELPER_AVAILABLE and 
#                         self.state.hand_gesture == "none" and 
#                         not recognition_complete and 
#                         not awaiting_registration and 
#                         current_attempts < 3):
                        
#                         last_user_check = current_time
#                         try:
#                             # FIXED: Use the correct method
#                             current_user = self._recognize_user_in_frame(frame)
                            
#                             if current_user and current_user != "Unknown":
#                                 # SUCCESS: User recognized
#                                 with self.state_lock:
#                                     self.state.current_user = current_user
#                                     self.state.last_speech_output = f"Hello {current_user}!"
#                                     self.state.face_recognition_attempts = 0
#                                     self.state.awaiting_registration = False
#                                     self.state.recognition_complete = True
                                    
#                                     if self.config.enable_speech_synthesis:
#                                         threading.Thread(
#                                             target=FR.speak,
#                                             args=(f"Hello {current_user}!",),
#                                             daemon=True
#                                         ).start()
#                                     logger.info(f"✅ User recognized: {current_user}")
                            
#                             else:
#                                 # ATTEMPT FAILED: User not recognized
#                                 new_attempts = current_attempts + 1
#                                 with self.state_lock:
#                                     self.state.face_recognition_attempts = new_attempts
                                
#                                 logger.info(f"🔍 Face recognition attempt {new_attempts}/3 - No user found")
                                
#                                 if new_attempts >= 3:
#                                     with self.state_lock:
#                                         self.state.recognition_complete = True
#                                         self.state.current_user = "Unknown"
                                    
#                                     logger.info("❌ Face recognition complete after 3 attempts - user can now choose options")
                                    
#                         except Exception as e:
#                             logger.error(f"Face recognition error: {e}")
#                             new_attempts = current_attempts + 1
#                             with self.state_lock:
#                                 self.state.face_recognition_attempts = new_attempts
                            
#                             if new_attempts >= 3:
#                                 with self.state_lock:
#                                     self.state.recognition_complete = True
#                                     self.state.current_user = "Unknown"
                    
#                     # Store current frame for web streaming
#                     self.current_frame = frame.copy()
#                     time.sleep(0.033)  # ~30 FPS
                    
#                 else:
#                     logger.warning("Camera not available")
#                     time.sleep(0.5)
                    
#             except Exception as e:
#                 logger.error(f"Unified vision processing error: {e}")
#                 time.sleep(0.1)
    
#     def _recognize_user_in_frame(self, frame):
#         """COMPLETELY FIXED: Use face_helper to recognize user from current frame"""
#         if not FACE_HELPER_AVAILABLE or not FACE_RECOGNITION_AVAILABLE:
#             return "Unknown"
        
#         try:
#             # Check if we have the advanced face_recognition_system
#             if hasattr(FR, 'face_recognition_system'):
#                 name, confidence = FR.face_recognition_system.recognize_face_in_frame(frame)
#                 if name and name != "Unknown" and confidence > 0.4:
#                     logger.debug(f"Face recognized: {name} (confidence: {confidence:.3f})")
#                     return name
#                 else:
#                     logger.debug(f"No face recognized (confidence: {confidence:.3f})")
#                     return "Unknown"
#             else:
#                 # Use simple fallback recognition
#                 img = frame
#                 imgS = cv2.resize(img, (0, 0), None, 0.25, 0.25)
#                 imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)
                
#                 if FACE_RECOGNITION_AVAILABLE and FR.encodeListKnown:
#                     faces_cur_frame = face_recognition.face_locations(imgS)
#                     encodes_cur_frame = face_recognition.face_encodings(imgS, faces_cur_frame)
                    
#                     for encode_face in encodes_cur_frame:
#                         matches = face_recognition.compare_faces(FR.encodeListKnown, encode_face, tolerance=0.6)
#                         face_distances = face_recognition.face_distance(FR.encodeListKnown, encode_face)
                        
#                         if len(face_distances) > 0:
#                             match_index = np.argmin(face_distances)
#                             best_match_distance = face_distances[match_index]
                            
#                             if best_match_distance <= 0.6 and matches[match_index]:
#                                 name = FR.classNames[match_index].replace('_', ' ').title()
#                                 logger.debug(f"Face recognized: {name} (distance: {best_match_distance:.3f})")
#                                 return name
                
#                 return "Unknown"
                    
#         except Exception as e:
#             logger.error(f"Face recognition error: {e}")
#             return "Unknown"
    
#     def register_new_user(self, name: str) -> bool:
#         """Use the same simple approach as standalone version"""
#         try:
#             logger.info(f"🆔 REGISTRATION: Starting for {name}")
            
#             with self.state_lock:
#                 self.state.awaiting_registration = True
            
#             if not self.camera or not self.camera.isOpened():
#                 logger.error("❌ Camera not available for registration")
#                 with self.state_lock:
#                     self.state.awaiting_registration = False
#                 return False
            
#             # Clear buffer
#             logger.info("📷 Clearing camera buffer...")
#             for i in range(5):
#                 ret, frame = self.camera.read()
#                 if ret and frame is not None:
#                     logger.info(f"✅ Buffer clear {i+1}: Got frame")
#                 time.sleep(0.1)
            
#             # Use FR.take_picture (same as standalone)
#             logger.info("📸 Taking picture using face_helper...")
#             success = FR.take_picture(name, self.camera)
            
#             # Update state
#             with self.state_lock:
#                 self.state.awaiting_registration = False
#                 if success:
#                     self.state.current_user = name
#                     self.state.recognition_complete = True
#                     self.state.face_recognition_attempts = 0
            
#             logger.info(f"🎉 Registration result for {name}: {success}")
#             return success
            
#         except Exception as e:
#             logger.error(f"❌ Registration error: {e}")
#             with self.state_lock:
#                 self.state.awaiting_registration = False
#             return False
#     def _save_registration_photo(self, name: str, frame) -> bool:
#         """Save registration photo directly from frame"""
#         try:
#             # Clean filename
#             clean_name = name.strip().replace(' ', '_').lower()
#             filename = f"{clean_name}.jpg"
#             filepath = os.path.join("images", filename)
            
#             # Ensure images directory exists
#             os.makedirs("images", exist_ok=True)
            
#             # Save image
#             result = cv2.imwrite(filepath, frame)
            
#             if result and os.path.exists(filepath):
#                 file_size = os.path.getsize(filepath)
#                 logger.info(f"✅ Registration photo saved: {filepath} ({file_size} bytes)")
                
#                 # Reload face recognition data
#                 if hasattr(FR, 'reload_face_data'):
#                     FR.reload_face_data()
#                 elif hasattr(FR, 'load_face_data'):
#                     FR.load_face_data()
                
#                 return True
#             else:
#                 logger.error(f"❌ Failed to save registration photo")
#                 return False
                
#         except Exception as e:
#             logger.error(f"❌ Error saving registration photo: {e}")
#             return False   
#     def reset_face_recognition_state(self):
#         """Reset face recognition state to properly restart 3-attempt cycle"""
#         with self.state_lock:
#             self.state.face_recognition_attempts = 0
#             self.state.awaiting_registration = False
#             self.state.current_user = "Unknown"
#             self.state.recognition_complete = False
#         logger.info("✅ Face recognition state COMPLETELY reset - starting NEW 3-attempt cycle")
#         return True
    
#     # Keep all your existing methods (obstacle detection, status update, etc.)
#     def _obstacle_detection_loop(self):
#         """Background obstacle detection loop"""
#         while not self.shutdown_event.is_set():
#             try:
#                 sensor_readings = self._get_all_distances_with_names()
#                 if sensor_readings:
#                     with self.state_lock:
#                         self.state.sensor_distances = sensor_readings.copy()
#                         self.state.last_distances = list(sensor_readings.values())
#                     valid_distances = [d for d in sensor_readings.values() if d > 0]
#                     if valid_distances:
#                         min_distance = min(valid_distances)
#                         closest_sensor = ""
#                         for sensor_name, distance in sensor_readings.items():
#                             if distance == min_distance and distance > 0:
#                                 closest_sensor = sensor_name
#                                 break
#                         if min_distance < self.config.obstacle_threshold and not self.state.obstacle_detected:
#                             self._update_obstacle_state(True, closest_sensor, min_distance)
#                             self.emergency_stop()
#                         elif self.state.obstacle_detected and min_distance > self.config.obstacle_clear_threshold:
#                             logger.info(f"Obstacle cleared! All sensors show > {self.config.obstacle_clear_threshold}cm")
#                             self._update_obstacle_state(False)
#                 time.sleep(0.1)
#             except Exception as e:
#                 logger.error(f"Error in obstacle detection: {e}")
#                 time.sleep(0.1)
    
#     def _status_update_loop(self):
#         """Background status update loop"""
#         while not self.shutdown_event.is_set():
#             try:
#                 with self.state_lock:
#                     self.state.uptime = time.time() - self.start_time
#                     if not self.state.obstacle_detected:
#                         if self._is_moving():
#                             self.state.status = "moving"
#                         else:
#                             self.state.status = "idle"
#                 time.sleep(0.1)
#             except Exception as e:
#                 logger.error(f"Error in status update: {e}")
#                 time.sleep(0.1)
    
#     def _update_obstacle_state(self, detected: bool, sensor: str = "", distance: float = 0.0):
#         """Update obstacle detection state"""
#         with self.state_lock:
#             self.state.obstacle_detected = detected
#             self.state.obstacle_sensor = sensor
#             self.state.obstacle_distance = distance
#             if detected:
#                 self.state.status = "obstacle_detected"
#                 self.obstacle_event.set()
#                 message = f"Obstacle detected by {sensor} sensor at {distance:.1f}cm!"
#                 self.state.last_speech_output = message
#                 if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#                     threading.Thread(
#                         target=FR.speak,
#                         args=("Obstacle detected! Stopping.",),
#                         daemon=True
#                     ).start()
#                 logger.warning(message)
#             else:
#                 self.obstacle_event.clear()
#                 if self.state.status == "obstacle_detected":
#                     self.state.status = "idle"
#                 logger.info("Obstacle detection cleared")
    
#     def _is_moving(self) -> bool:
#         """Check if robot is currently moving"""
#         return any([
#             self.state.motor1_speed > 0,
#             self.state.motor2_speed > 0,
#             self.state.motor3_speed > 0
#         ])
    
#     def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
#         """Get distance from ultrasonic sensor"""
#         if not self.gpio_initialized:
#             return 100.0
#         try:
#             GPIO.output(trig_pin, False)
#             time.sleep(0.000002)
#             GPIO.output(trig_pin, True)
#             time.sleep(0.00001)
#             GPIO.output(trig_pin, False)
#             timeout_start = time.time()
#             while GPIO.input(echo_pin) == 0:
#                 if time.time() - timeout_start > self.config.sensor_timeout:
#                     return -1
#             pulse_start = time.time()
#             timeout_start = time.time()
#             while GPIO.input(echo_pin) == 1:
#                 if time.time() - timeout_start > self.config.sensor_timeout:
#                     return -1
#             pulse_end = time.time()
#             pulse_duration = pulse_end - pulse_start
#             distance = pulse_duration * 17150
#             distance = round(distance, 2)
#             if 2 <= distance <= 400:
#                 return distance
#             else:
#                 return -1
#         except Exception as e:
#             logger.error(f"Distance measurement error: {e}")
#             return -1
    
#     def _get_all_distances_with_names(self) -> Dict[str, float]:
#         """Get distances from all sensors with names"""
#         sensor_configs = [
#             (self.config.trig1, self.config.echo1, "front"),
#             (self.config.trig2, self.config.echo2, "left"),
#             (self.config.trig3, self.config.echo3, "right")
#         ]
#         sensor_readings = {}
#         for trig, echo, name in sensor_configs:
#             dist = self._get_distance(trig, echo)
#             if dist > 0:
#                 sensor_readings[name] = dist
#         return sensor_readings
    
#     def _change_speeds_smooth(self, new_speeds: Tuple[int, int, int]):
#         """Smoothly change motor speeds"""
#         if not self.gpio_initialized:
#             with self.state_lock:
#                 self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
#             return
        
#         with self.movement_lock:
#             with self.state_lock:
#                 if self.state.obstacle_detected or self.interrupt_event.is_set():
#                     self._stop_motors_immediate()
#                     return
#                 current_speeds = (self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed)
            
#             speed_diffs = [abs(new_speeds[i] - current_speeds[i]) for i in range(3)]
#             steps = max(speed_diffs) if speed_diffs else 0
#             if steps < 1:
#                 steps = 1
            
#             for step in range(steps + 1):
#                 with self.state_lock:
#                     if self.state.obstacle_detected or self.interrupt_event.is_set():
#                         self._stop_motors_immediate()
#                         return
#                 progress = step / steps if steps > 0 else 1
#                 speeds = [
#                     int(current_speeds[i] + (new_speeds[i] - current_speeds[i]) * progress)
#                     for i in range(3)
#                 ]
#                 speeds = [max(0, min(100, speed)) for speed in speeds]
#                 try:
#                     if self.motor1_pwm:
#                         self.motor1_pwm.ChangeDutyCycle(speeds[0])
#                     if self.motor2_pwm:
#                         self.motor2_pwm.ChangeDutyCycle(speeds[1])
#                     if self.motor3_pwm:
#                         self.motor3_pwm.ChangeDutyCycle(speeds[2])
#                 except Exception as e:
#                     logger.error(f"PWM update error: {e}")
#                     self._stop_motors_immediate()
#                     return
#                 time.sleep(0.01)
            
#             with self.state_lock:
#                 if not self.state.obstacle_detected and not self.interrupt_event.is_set():
#                     self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
#                 else:
#                     self._stop_motors_immediate()
    
#     def _stop_motors_immediate(self):
#         """Immediately stop all motors"""
#         try:
#             if self.gpio_initialized and all([self.motor1_pwm, self.motor2_pwm, self.motor3_pwm]):
#                 self.motor1_pwm.ChangeDutyCycle(0)
#                 self.motor2_pwm.ChangeDutyCycle(0)
#                 self.motor3_pwm.ChangeDutyCycle(0)
#         except Exception as e:
#             logger.error(f"Error stopping motors: {e}")
#         self.state.motor1_speed = 0
#         self.state.motor2_speed = 0
#         self.state.motor3_speed = 0
    
#     # Public API methods
#     def get_status(self) -> Dict:
#         """Get current robot status with enhanced information"""
#         with self.state_lock:
#             obstacle_message = "Robot operational"
#             if self.state.obstacle_detected:
#                 obstacle_message = f"Obstacle detected by {self.state.obstacle_sensor} sensor at {self.state.obstacle_distance:.1f}cm"
            
#             return {
#                 "status": self.state.status,
#                 "message": obstacle_message,
#                 "obstacle_detected": self.state.obstacle_detected,
#                 "obstacle_sensor": self.state.obstacle_sensor,
#                 "obstacle_distance": self.state.obstacle_distance,
#                 "current_speeds": {
#                     "motor1": self.state.motor1_speed,
#                     "motor2": self.state.motor2_speed,
#                     "motor3": self.state.motor3_speed
#                 },
#                 "sensor_distances": self.state.sensor_distances.copy(),
#                 "last_distances": self.state.last_distances.copy(),
#                 "last_command": self.state.last_command,
#                 "uptime": self.state.uptime,
#                 "gpio_available": self.gpio_initialized,
#                 "current_user": self.state.current_user,
#                 "faces_detected": self.state.faces_detected.copy(),
#                 "hand_gesture": self.state.hand_gesture,
#                 "camera_active": self.state.camera_active,
#                 "last_speech_output": self.state.last_speech_output,
#                 "listening": self.state.listening,
#                 "speech_recognition_active": self.state.speech_recognition_active,
#                 "interaction_mode": self.state.interaction_mode,
#                 "face_recognition_available": FACE_HELPER_AVAILABLE and FACE_RECOGNITION_AVAILABLE,
#                 "speech_recognition_available": SPEECH_RECOGNITION_AVAILABLE,
#                 "mediapipe_available": MEDIAPIPE_AVAILABLE,
#                 "face_recognition_attempts": self.state.face_recognition_attempts,
#                 "awaiting_registration": self.state.awaiting_registration,
#                 "recognition_complete": self.state.recognition_complete
#             }
    
#     def speak(self, text: str) -> bool:
#         """Make robot speak text using face_helper"""
#         try:
#             with self.state_lock:
#                 self.state.last_speech_output = text
#             if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#                 threading.Thread(
#                     target=FR.speak,
#                     args=(text,),
#                     daemon=True
#                 ).start()
#                 logger.info(f"Robot speaking: {text}")
#                 return True
#             else:
#                 logger.info(f"Speech synthesis disabled. Would say: {text}")
#                 return False
#         except Exception as e:
#             logger.error(f"Speech synthesis error: {e}")
#             return False
    
#     def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
#         """Listen for speech input using face_helper"""
#         if not self.config.enable_speech_recognition or not FACE_HELPER_AVAILABLE:
#             return None
#         try:
#             with self.state_lock:
#                 self.state.listening = True
#                 self.state.speech_recognition_active = True
#             result = FR.listen()
#             with self.state_lock:
#                 self.state.listening = False
#                 self.state.speech_recognition_active = False
#             return result
#         except Exception as e:
#             logger.error(f"Speech recognition error: {e}")
#             with self.state_lock:
#                 self.state.listening = False
#                 self.state.speech_recognition_active = False
#             return None
    
#     def set_interaction_mode(self, mode: str) -> bool:
#         """Set interaction mode (speech, text, keyboard, auto)"""
#         valid_modes = ["idle", "speech", "text", "keyboard", "auto"]
#         if mode in valid_modes:
#             with self.state_lock:
#                 self.state.interaction_mode = mode
#             logger.info(f"Interaction mode set to: {mode}")
#             return True
#         else:
#             logger.warning(f"Invalid interaction mode: {mode}")
#             return False
    
#     def recognize_user(self, mode: str = "auto") -> Optional[str]:
#         """Recognize current user using face_helper"""
#         if not FACE_HELPER_AVAILABLE or not self.current_frame:
#             return None
#         try:
#             user = self._recognize_user_in_frame(self.current_frame)
#             if user and user != "Unknown":
#                 with self.state_lock:
#                     self.state.current_user = user
#                     self.state.face_recognition_attempts = 0
#                     self.state.awaiting_registration = False
#                 return user
#             return None
#         except Exception as e:
#             logger.error(f"Face recognition error: {e}")
#             return None
    
#     # Keep all your existing movement and other methods as they are...
#     def move(self, direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
#         direction_map = {
#             "up": "forward",
#             "down": "backward", 
#             "left": "turnleft",
#             "right": "turnright",
#             "forward": "forward",
#             "backward": "backward",
#             "turnleft": "turnleft",
#             "turnright": "turnright",
#             "moveleft": "moveleft",
#             "moveright": "moveright",
#             "stop": "stop"
#         }
#         mapped_direction = direction_map.get(direction.lower(), direction.lower())
#         with self.state_lock:
#             if self.state.obstacle_detected and mapped_direction != "stop":
#                 message = f"Movement blocked - obstacle detected by {self.state.obstacle_sensor} sensor"
#                 self.state.last_speech_output = message
#                 if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#                     threading.Thread(
#                         target=FR.speak,
#                         args=("Cannot move, obstacle detected!",),
#                         daemon=True
#                     ).start()
#                 logger.warning(message)
#                 return False
#             speed = speed or self.config.default_speed
#             self.state.last_command = mapped_direction
#         direction_configs = {
#             "forward": {
#                 "dirs": [self.HIGH, self.HIGH, self.LOW],
#                 "speeds": [0, speed, speed + self.config.motor3_compensate]
#             },
#             "backward": {
#                 "dirs": [self.HIGH, self.LOW, self.HIGH],
#                 "speeds": [0, speed, speed + self.config.motor3_compensate]
#             },
#             "turnleft": {
#                 "dirs": [self.HIGH, self.LOW, self.LOW],
#                 "speeds": [speed, speed, speed + self.config.motor3_compensate]
#             },
#             "turnright": {
#                 "dirs": [self.LOW, self.HIGH, self.HIGH],
#                 "speeds": [speed, speed, speed + self.config.motor3_compensate]
#             },
#             "moveleft": {
#                 "dirs": [self.HIGH, self.HIGH, self.LOW],
#                 "speeds": [int(speed * 1.5), speed, 0]
#             },
#             "moveright": {
#                 "dirs": [self.LOW, self.LOW, self.HIGH],
#                 "speeds": [int(speed * 1.5), speed, speed + self.config.motor3_compensate]
#             },
#             "stop": {
#                 "dirs": [self.LOW, self.LOW, self.LOW],
#                 "speeds": [0, 0, 0]
#             }
#         }
#         if mapped_direction not in direction_configs:
#             logger.error(f"Invalid direction: {direction}")
#             return False
#         config = direction_configs[mapped_direction]
#         if self.gpio_initialized:
#             try:
#                 GPIO.output(self.config.motor1_dir, config["dirs"][0])
#                 GPIO.output(self.config.motor2_dir, config["dirs"][1])
#                 GPIO.output(self.config.motor3_dir, config["dirs"][2])
#             except Exception as e:
#                 logger.error(f"Error setting motor directions: {e}")
#                 return False
#         self._change_speeds_smooth(tuple(config["speeds"]))
#         if duration_ms is not None and duration_ms > 0:
#             time.sleep(duration_ms / 1000)
#             with self.state_lock:
#                 if not self.state.obstacle_detected:
#                     self._change_speeds_smooth((0, 0, 0))
#         return True
    
#     def stop(self) -> bool:
#         """Stop the robot"""
#         return self.move("stop")
    
#     def emergency_stop(self) -> bool:
#         """Emergency stop with speech notification"""
#         logger.warning("Emergency stop activated")
#         with self.movement_lock:
#             with self.state_lock:
#                 self.state.status = "emergency_stop"
#                 self.state.last_speech_output = "Emergency stop activated"
#                 self._stop_motors_immediate()
#         if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#             threading.Thread(
#                 target=FR.speak,
#                 args=("Emergency stop activated",),
#                 daemon=True
#             ).start()
#         return True
    
#     def reset_obstacle_detection(self) -> bool:
#         """Reset obstacle detection state"""
#         self._update_obstacle_state(False)
#         message = "Obstacle detection reset - robot can move again"
#         with self.state_lock:
#             self.state.last_speech_output = message
#         if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#             threading.Thread(
#                 target=FR.speak,
#                 args=("Obstacle cleared, ready to move",),
#                 daemon=True
#             ).start()
#         logger.info(message)
#         return True
    
#     def parse_command(self, command: str) -> Optional[Tuple[str, Optional[int]]]:
#         """Parse natural language command"""
#         command = command.lower().strip()
#         directions = {
#             "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
#             "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
#             "stop": ["stop", "halt", "stand still", "brake", "freeze"],
#             "turnleft": ["turn left", "rotate left", "spin left"],
#             "turnright": ["turn right", "rotate right", "spin right"],
#             "moveleft": ["move left", "strafe left", "slide left", "sidestep left"],
#             "moveright": ["move right", "strafe right", "slide right", "sidestep right"]
#         }
#         time_patterns = {
#             "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
#             "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
#             "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
#         }
#         direction = None
#         for dir_key, phrases in directions.items():
#             if any(phrase in command for phrase in phrases):
#                 direction = dir_key
#                 break
#         duration_ms = None
#         for unit, pattern in time_patterns.items():
#             match = re.search(pattern, command)
#             if match:
#                 try:
#                     value = float(match.group(1))
#                     if unit == "seconds":
#                         duration_ms = int(value * 1000)
#                     elif unit == "minutes":
#                         duration_ms = int(value * 60000)
#                     elif unit == "hours":
#                         duration_ms = int(value * 3600000)
#                     break
#                 except (ValueError, OverflowError):
#                     logger.warning(f"Invalid duration value: {match.group(1)}")
#                     continue
#         return (direction, duration_ms) if direction else None
    
#     async def chat(self, message: str, context: str = "") -> str:
#         """Chat with AI model and respond with intelligent speech"""
#         if not self.ai_chain:
#             response = "AI model not available. Please check Ollama installation."
#         else:
#             try:
#                 result = self.ai_chain.invoke({"context": context, "question": message})
#                 response = str(result)
#             except Exception as e:
#                 logger.error(f"AI chat error: {e}")
#                 response = "Error processing message with AI model."
#         with self.state_lock:
#             self.state.last_speech_output = response
#         if self._should_speak_response(message):
#             if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
#                 threading.Thread(
#                     target=FR.speak,
#                     args=(response,),
#                     daemon=True
#                 ).start()
#                 logger.info(f"🗣️ Speaking AI response: {response[:50]}...")
#         else:
#             logger.info(f"🤫 Not speaking movement command response: {response[:50]}...")
#         return response
    
#     def _should_speak_response(self, original_message: str) -> bool:
#         """Determine if the robot should speak the response based on the original message"""
#         movement_keywords = [
#             'go forward', 'move forward', 'forward', 'advance',
#             'go backward', 'move backward', 'backward', 'reverse', 'back up',
#             'turn left', 'turn right', 'rotate left', 'rotate right',
#             'move left', 'move right', 'strafe left', 'strafe right',
#             'stop', 'halt', 'brake', 'freeze'
#         ]
#         message_lower = original_message.lower()
#         for keyword in movement_keywords:
#             if keyword in message_lower:
#                 return False
#         return True
    
#     async def handle_conversation_mode(self, mode: str) -> str:
#         """Handle different conversation modes (speech, text, auto)"""
#         try:
#             with self.state_lock:
#                 self.state.interaction_mode = mode
#             if mode == "speech":
#                 user = self.recognize_user("speech")
#                 if user:
#                     greeting = f"Hi {user}, what would you like to ask today?"
#                     self.speak(greeting)
#                     user_input = self.listen_for_speech()
#                     if user_input:
#                         parsed = self.parse_command(user_input)
#                         if parsed:
#                             direction, duration = parsed
#                             success = self.move(direction, duration_ms=duration)
#                             response = f"Movement command {direction} {'executed' if success else 'failed'}"
#                         else:
#                             response = await self.chat(user_input)
#                         return response
#                     else:
#                         return "I didn't hear anything clearly."
#                 else:
#                     return "I couldn't recognize you. Please look at the camera."
#             elif mode == "text":
#                 user = self.recognize_user("text")
#                 if user:
#                     greeting = f"Hi {user}, what would you like to ask today?"
#                     self.speak(greeting)
#                     return greeting
#                 else:
#                     return "Please look at the camera for identification."
#             elif mode == "auto":
#                 if self.state.hand_gesture in ["waving", "shaking_hands"]:
#                     response = f"I see you're {self.state.hand_gesture.replace('_', ' ')}! How can I help you?"
#                     self.speak(response)
#                     return response
#                 elif self.state.current_user != "Unknown":
#                     response = f"Hello {self.state.current_user}! I'm ready for commands."
#                     self.speak(response)
#                     return response
#                 else:
#                     return "I'm watching and ready to help!"
#             return "Unknown conversation mode"
#         except Exception as e:
#             logger.error(f"Conversation mode error: {e}")
#             return f"Error in conversation mode: {str(e)}"
    
#     def get_camera_frame(self) -> Optional[bytes]:
#         """Get current camera frame for web streaming"""
#         if not self.camera or not self.camera.isOpened() or self.current_frame is None:
#             return None
#         try:
#             frame = self.current_frame.copy()
#             if self.hand_detector and self.state.hand_gesture != "none":
#                 cv2.putText(frame, f"Gesture: {self.state.hand_gesture.replace('_', ' ')}", 
#                            (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
#             cv2.putText(frame, f"User: {self.state.current_user}", 
#                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             cv2.putText(frame, f"Status: {self.state.status}", 
#                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             if self.state.awaiting_registration:
#                 cv2.putText(frame, "Ready for Registration", 
#                            (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
#             elif self.state.face_recognition_attempts > 0:
#                 cv2.putText(frame, f"Scanning... ({self.state.face_recognition_attempts}/3)", 
#                            (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 165, 0), 2)
#             ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
#             if ret:
#                 return buffer.tobytes()
#             return None
#         except Exception as e:
#             logger.error(f"Camera frame error: {e}")
#             return None
    
#     def shutdown(self):
#         """Shutdown enhanced robot controller with cleanup"""
#         logger.info("Shutting down enhanced robot controller...")
#         self.shutdown_event.set()
#         with self.movement_lock:
#             with self.state_lock:
#                 self._stop_motors_immediate()
#         threads_to_join = [
#             (self.obstacle_thread, "obstacle detection"),
#             (self.status_thread, "status update"),
#             (self.vision_thread, "unified vision")
#         ]
#         for thread, name in threads_to_join:
#             if thread and thread.is_alive():
#                 logger.info(f"Waiting for {name} thread to finish...")
#                 thread.join(timeout=2.0)
# # Global enhanced robot controller instance
# _enhanced_robot_controller = None
# _enhanced_controller_lock = threading.Lock()

# def get_enhanced_robot_controller():
#     """Get or create the global enhanced robot controller instance"""
#     global _enhanced_robot_controller
#     with _enhanced_controller_lock:
#         if _enhanced_robot_controller is None:
#             _enhanced_robot_controller = EnhancedRobotController()
#         return _enhanced_robot_controller

# # Public API functions for FastAPI backend compatibility
# def get_status() -> Dict:
#     """Get current robot status - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().get_status()
#     except Exception as e:
#         logger.error(f"Error getting status: {e}")
#         return {
#             "status": "error",
#             "message": f"Error getting status: {str(e)}",
#             "obstacle_detected": False,
#             "gpio_available": False,
#             "camera_active": False,
#             "face_recognition_available": False,
#             "speech_recognition_available": False,
#             "face_recognition_attempts": 0,
#             "awaiting_registration": False
#         }

# def move(direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
#     """Move robot in specified direction - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().move(direction, speed, duration_ms)
#     except Exception as e:
#         logger.error(f"Error moving robot: {e}")
#         return False

# def stop() -> bool:
#     """Stop the robot - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().stop()
#     except Exception as e:
#         logger.error(f"Error stopping robot: {e}")
#         return False

# def speak(text: str) -> bool:
#     """Make robot speak - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().speak(text)
#     except Exception as e:
#         logger.error(f"Error speaking: {e}")
#         return False

# def listen_for_speech(timeout: int = 5) -> Optional[str]:
#     """Listen for speech input - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().listen_for_speech(timeout)
#     except Exception as e:
#         logger.error(f"Error listening for speech: {e}")
#         return None

# def recognize_user(mode: str = "auto") -> Optional[str]:
#     """Recognize current user - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().recognize_user(mode)
#     except Exception as e:
#         logger.error(f"Error recognizing user: {e}")
#         return None

# def register_new_user(name: str) -> bool:
#     """Register new user - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().register_new_user(name)
#     except Exception as e:
#         logger.error(f"Error registering user: {e}")
#         return False

# def set_interaction_mode(mode: str) -> bool:
#     """Set interaction mode - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().set_interaction_mode(mode)
#     except Exception as e:
#         logger.error(f"Error setting interaction mode: {e}")
#         return False

# def reset_face_recognition_state() -> bool:
#     """Reset face recognition state - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().reset_face_recognition_state()
#     except Exception as e:
#         logger.error(f"Error resetting face recognition state: {e}")
#         return False

# async def handle_conversation_mode(mode: str) -> str:
#     """Handle conversation mode - called by FastAPI"""
#     try:
#         return await get_enhanced_robot_controller().handle_conversation_mode(mode)
#     except Exception as e:
#         logger.error(f"Error handling conversation mode: {e}")
#         return f"Error: {str(e)}"

# def get_camera_frame() -> Optional[bytes]:
#     """Get camera frame - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().get_camera_frame()
#     except Exception as e:
#         logger.error(f"Error getting camera frame: {e}")
#         return None

# def parse_command(command: str) -> Optional[Tuple[str, Optional[int]]]:
#     """Parse natural language command - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().parse_command(command)
#     except Exception as e:
#         logger.error(f"Error parsing command: {e}")
#         return None

# async def chat(message: str, context: str = "") -> str:
#     """Chat with AI model - called by FastAPI"""
#     try:
#         return await get_enhanced_robot_controller().chat(message, context)
#     except Exception as e:
#         logger.error(f"Error in chat: {e}")
#         return "Error processing chat message."

# def emergency_stop() -> bool:
#     """Emergency stop - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().emergency_stop()
#     except Exception as e:
#         logger.error(f"Error in emergency stop: {e}")
#         return False

# def reset_obstacle_detection() -> bool:
#     """Reset obstacle detection - called by FastAPI"""
#     try:
#         return get_enhanced_robot_controller().reset_obstacle_detection()
#     except Exception as e:
#         logger.error(f"Error resetting obstacle detection: {e}")
#         return False

# def shutdown_robot():
#     """Shutdown robot controller - called by FastAPI on app shutdown"""
#     global _enhanced_robot_controller
#     with _enhanced_controller_lock:
#         if _enhanced_robot_controller is not None:
#             _enhanced_robot_controller.shutdown()
#             _enhanced_robot_controller = None

# if __name__ == "__main__":
#     print("🤖 Enhanced Robot Movement Controller - Complete with Face Helper Integration")
#     print("=" * 80)
#     print("Features:")
#     print("• ✅ INTEGRATED: face_helper.py for face recognition and speech")
#     print("• ✅ FIXED: Registration using actual face_helper functions")
#     print("• ✅ FIXED: Stable WebSocket connections with proper cleanup")
#     print("• ✅ Unified vision loop for face and hand detection") 
#     print("• ✅ Non-blocking speech synthesis using face_helper.speak()")
#     print("• ✅ Improved camera initialization with face_helper integration")
#     print("• ✅ Better error handling and logging")
#     print("• ✅ Enhanced obstacle detection")
#     print("• ✅ AI integration with speech responses")
#     print("• ✅ Camera streaming support")
#     print("• ✅ Face recognition attempt limiting (3 tries then STOP)")
#     print("• ✅ Hand gesture spam prevention")
#     print("• ✅ Proper connection status handling")
#     print("• ✅ FastAPI integration ready")
#     print("• ✅ User registration with face_helper.take_picture()")
#     print("=" * 80)
#     print("✅ Enhanced robot controller module loaded successfully")
#     print("🔧 Key Integration: Uses your existing face_helper.py for all face operations")
#     print("🔧 Key Integration: Shares camera properly between vision systems")
#     print("🔧 Key Integration: Registration saves to /home/robot/summer-research-robot/RGB-cam/images")
#     print("🔧 Key Fix: Face recognition stops after 3 attempts and waits for user choice")
#     print("🔧 Key Fix: No automatic navigation to registration page")
#     print("🔧 Key Fix: Stable WebSocket connections with proper cleanup")
# robot_movement.py - COMPLETE FIXED VERSION with GPIO cleanup and debugging
import asyncio
import re
import threading
import time
import logging
import cv2
import os
import platform
import subprocess
import sys
from typing import Dict, Optional, List, Tuple, Union
from dataclasses import dataclass, field
from enum import Enum
import json
from collections import deque

# Import your existing face_helper module or create FIXED fallback
try:
    import face_helper as FR
    FACE_HELPER_AVAILABLE = True
    logging.info("Successfully imported face_helper module from current directory")
except ImportError:
    try:
        sys.path.append('/home/robot/summer-research-robot')
        import face_helper as FR
        FACE_HELPER_AVAILABLE = True
        logging.info("Successfully imported face_helper module from parent directory")
    except ImportError:
        logging.warning("face_helper not found, creating FIXED fallback")
        FACE_HELPER_AVAILABLE = True
        
        class SimpleFaceHelper:
            def __init__(self):
                # FIXED: Use backend/images directory (not facepics!)
                self.images_path = '/home/robot/summer-research-robot/RGB-cam/images'
                self.cap = None
                self.classNames = []
                self.encodeListKnown = []
                os.makedirs(self.images_path, exist_ok=True)
                logging.info(f"Created/verified images directory at {self.images_path}")
            
            def speak(self, text):
                try:
                    system = platform.system().lower()
                    if system == "linux":
                        try:
                            subprocess.run(["espeak", text], check=True, capture_output=True)
                        except (subprocess.CalledProcessError, FileNotFoundError):
                            try:
                                subprocess.run(["spd-say", text], check=True, capture_output=True)
                            except (subprocess.CalledProcessError, FileNotFoundError):
                                print(f"🔊 TTS: {text}")
                    else:
                        print(f"🔊 TTS: {text}")
                except Exception as e:
                    print(f"🔊 TTS: {text}")
                    logging.error(f"TTS error: {e}")
            
            def listen(self):
                if not SPEECH_RECOGNITION_AVAILABLE:
                    return None
                try:
                    import speech_recognition as sr
                    r = sr.Recognizer()
                    with sr.Microphone() as source:
                        print("🎙️ Listening...")
                        audio = r.listen(source, timeout=5)
                        text = r.recognize_google(audio)
                        print(f"🗣️ You said: {text}")
                        return text.lower()
                except Exception as e:
                    logging.error(f"Speech recognition error: {e}")
                    return None
            
            def take_picture(self, name, camera):
                """FIXED: Take picture and save to backend/images"""
                try:
                    logging.info(f"📷 Starting picture capture for user: {name}")
                    if not camera:
                        logging.error("❌ No camera provided to take_picture")
                        return False
                    if not camera.isOpened():
                        logging.error("❌ Camera is not opened")
                        return False
                    
                    # Take the best frame
                    best_frame = None
                    for attempt in range(3):
                        success, image = camera.read()
                        if success and image is not None:
                            best_frame = image.copy()
                            break
                    
                    if best_frame is None:
                        logging.error("❌ Failed to capture image from camera")
                        return False
                    
                    # Clean filename
                    clean_name = name.strip().replace(' ', '_').lower()
                    filename = f'{clean_name}.jpg'
                    filepath = os.path.join(self.images_path, filename)
                    
                    logging.info(f"💾 Attempting to save image to: {filepath}")
                    os.makedirs(os.path.dirname(filepath), exist_ok=True)
                    
                    result = cv2.imwrite(filepath, best_frame)
                    if result:
                        if os.path.exists(filepath):
                            file_size = os.path.getsize(filepath)
                            logging.info(f"✅ Picture saved successfully for {name}")
                            logging.info(f"   📁 Path: {filepath}")
                            logging.info(f"   📏 Size: {file_size} bytes")
                            
                            # IMPORTANT: Reload face data immediately
                            self.reload_face_data()
                            
                            return True
                        else:
                            logging.error(f"❌ File was not saved to {filepath}")
                            return False
                    else:
                        logging.error(f"❌ cv2.imwrite failed for {filepath}")
                        return False
                except Exception as e:
                    logging.error(f"❌ Error taking picture for {name}: {e}")
                    logging.error(f"   Exception type: {type(e).__name__}")
                    import traceback
                    logging.error(f"   Traceback: {traceback.format_exc()}")
                    return False
            
            def reload_face_data(self):
                """Reload all face images and regenerate encodings"""
                try:
                    self.images = []
                    self.classNames = []
                    
                    if not os.path.exists(self.images_path):
                        logging.warning(f"Images path does not exist: {self.images_path}")
                        os.makedirs(self.images_path, exist_ok=True)
                        return
                    
                    # Load all images
                    myList = os.listdir(self.images_path)
                    for cl in myList:
                        if cl.lower().endswith(('.png', '.jpg', '.jpeg')):
                            try:
                                full_path = os.path.join(self.images_path, cl)
                                cur_img = cv2.imread(full_path)
                                if cur_img is not None:
                                    self.images.append(cur_img)
                                    self.classNames.append(os.path.splitext(cl)[0])
                                    logging.info(f"✅ Loaded image for user: {os.path.splitext(cl)[0]}")
                            except Exception as e:
                                logging.error(f"Error loading image {cl}: {e}")
                    
                    # Generate encodings
                    self.encodeListKnown = self.findEncodings(self.images)
                    logging.info(f"📊 Generated {len(self.encodeListKnown)} face encodings total")
                    
                except Exception as e:
                    logging.error(f"Error reloading face data: {e}")
            
            def findEncodings(self, images):
                if not FACE_RECOGNITION_AVAILABLE:
                    logging.warning("Face recognition not available, returning empty encodings")
                    return []
                encodeList = []
                try:
                    for i, img in enumerate(images):
                        try:
                            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            encodings = face_recognition.face_encodings(img_rgb)
                            if encodings:
                                encodeList.append(encodings[0])
                                logging.info(f"✅ Generated encoding for image {i+1}")
                            else:
                                logging.warning(f"⚠️ No face found in image {i+1}")
                        except Exception as e:
                            logging.error(f"❌ Error encoding image {i+1}: {e}")
                except Exception as e:
                    logging.error(f"❌ Error in findEncodings: {e}")
                return encodeList
        
        FR = SimpleFaceHelper()
        FR.path = FR.images_path
        FR.images = []
        FR.classNames = []
        
        # Initial load of existing images
        try:
            FR.reload_face_data()
            logging.info(f"Loaded {len(FR.classNames)} known faces: {FR.classNames}")
        except Exception as e:
            logging.error(f"Error initializing face recognition: {e}")
            FR.encodeListKnown = []
            FR.classNames = []

# Import other dependencies with fallbacks
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

# FIXED GPIO IMPORT WITH CLEANUP
print("🔧 DEBUGGING GPIO AVAILABILITY...")
try:
    import RPi.GPIO as GPIO
    print("✅ RPi.GPIO import successful")
    
    # FORCE GPIO cleanup first
    try:
        GPIO.cleanup()
        print("🧹 Forced GPIO cleanup completed")
    except Exception as cleanup_error:
        print(f"🧹 GPIO cleanup error (expected): {cleanup_error}")
    
    # Test GPIO access
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.OUT)
    GPIO.cleanup()
    print("✅ GPIO access test successful")
    
    GPIO_AVAILABLE = True
except ImportError as e:
    print(f"❌ RPi.GPIO import failed: {e}")
    print("💡 Solution: pip3 install RPi.GPIO")
    GPIO_AVAILABLE = False
except Exception as e:
    print(f"❌ GPIO access failed: {e}")
    print("💡 Solution: Run with 'sudo' or check Pi 5 compatibility")
    GPIO_AVAILABLE = False

print(f"🔧 GPIO_AVAILABLE = {GPIO_AVAILABLE}")

try:
    from langchain_ollama import OllamaLLM
    from langchain_core.prompts import ChatPromptTemplate
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
        
        if not self.fingers_up(lm_list):
            return False
        
        index_x = lm_list[8][1]
        index_x_history.append(index_x)
        
        if len(index_x_history) < 5:
            return False
        
        index_motion = max(index_x_history) - min(index_x_history)
        return index_motion > 40
    
    def fingers_up(self, lm_list):
        """Check if fingers are extended (open hand)"""
        if len(lm_list) < 21:
            return False
        
        thumb_up = lm_list[4][1] < lm_list[3][1]
        fingers = [thumb_up]
        for id in [8, 12, 16, 20]:
            if lm_list[id][2] < lm_list[id - 2][2]:
                fingers.append(True)
            else:
                fingers.append(False)
        
        return sum(fingers) >= 3
    
    def is_shaking_hands(self, lm_list):
        """Detect handshake gesture"""
        if len(lm_list) < 21:
            return False
        
        tips = [8, 12, 16, 20]
        joints = [6, 10, 14, 18]
        
        closed_fingers = 0
        for tip, joint in zip(tips, joints):
            if lm_list[tip][2] > lm_list[joint][2]:
                closed_fingers += 1
        
        return closed_fingers >= 2

# Enhanced Robot Configuration
@dataclass
class EnhancedRobotConfig:
    """Enhanced robot configuration with FIXED paths"""
    motor1_speed: int = 38
    motor1_dir: int = 40
    motor2_speed: int = 32
    motor2_dir: int = 36
    motor3_speed: int = 16
    motor3_dir: int = 26
    echo1: int = 31
    echo2: int = 29
    echo3: int = 22
    trig1: int = 11
    trig2: int = 13
    trig3: int = 15
    pwm_frequency: int = 1000
    default_speed: int = 25
    motor3_compensate: int = 3
    obstacle_threshold: float = 30.0
    sensor_timeout: float = 0.5
    obstacle_clear_threshold: float = 50.0
    camera_index: int = 0
    # FIXED: Use correct backend images path
    face_images_path: str = 'images'  # Relative to backend directory
    enable_face_recognition: bool = True
    enable_hand_detection: bool = True
    enable_speech_synthesis: bool = True
    enable_speech_recognition: bool = True
    ai_model: str = "llama3"

# Enhanced Robot State
@dataclass
class EnhancedRobotState:
    """Enhanced robot state with vision and speech data"""
    motor1_speed: int = 0
    motor2_speed: int = 0
    motor3_speed: int = 0
    status: str = "idle"
    obstacle_detected: bool = False
    last_distances: List[float] = field(default_factory=list)
    sensor_distances: Dict[str, float] = field(default_factory=lambda: {"front": 0.0, "left": 0.0, "right": 0.0})
    obstacle_sensor: str = ""
    obstacle_distance: float = 0.0
    last_command: str = ""
    uptime: float = 0.0
    current_user: str = "Unknown"
    faces_detected: List[str] = field(default_factory=list)
    hand_gesture: str = "none"
    camera_active: bool = False
    last_speech_output: str = ""
    listening: bool = False
    speech_recognition_active: bool = False
    interaction_mode: str = "idle"
    face_recognition_attempts: int = 0
    awaiting_registration: bool = False
    recognition_complete: bool = False

class EnhancedRobotController:
    """Enhanced robot controller with FIXED face recognition"""
    
    def __init__(self, config: Optional[EnhancedRobotConfig] = None):
        self.config = config or EnhancedRobotConfig()
        self.state = EnhancedRobotState()
        self.start_time = time.time()
        self.movement_lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.vision_lock = threading.RLock()
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.obstacle_thread = None
        self.status_thread = None
        self.vision_thread = None
        self.hand_detector = None
        self.camera = None
        self.index_x_history = deque(maxlen=10)
        self.current_frame = None
        self.motor1_pwm = None
        self.motor2_pwm = None
        self.motor3_pwm = None
        self.HIGH = GPIO.HIGH if GPIO_AVAILABLE else 1
        self.LOW = GPIO.LOW if GPIO_AVAILABLE else 0
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
            if FACE_HELPER_AVAILABLE:
                try:
                    FR.cap = cv2.VideoCapture(0)
                    if FR.cap.isOpened():
                        logger.info("Face helper camera initialized successfully")
                        with self.state_lock:
                            self.state.camera_active = True
                        self.camera = FR.cap
                    else:
                        logger.error("Failed to initialize face helper camera")
                        with self.state_lock:
                            self.state.camera_active = False
                except Exception as e:
                    logger.error(f"Face helper camera setup failed: {e}")
                    with self.state_lock:
                        self.state.camera_active = False
            else:
                # Fallback camera setup
                camera_indices = [0, 1, 2, -1]
                camera_initialized = False
                
                for idx in camera_indices:
                    try:
                        logger.info(f"Trying camera index {idx}")
                        self.camera = cv2.VideoCapture(idx)
                        
                        if self.camera.isOpened():
                            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            self.camera.set(cv2.CAP_PROP_FPS, 30)
                            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
            
            if self.config.enable_hand_detection and MEDIAPIPE_AVAILABLE:
                self.hand_detector = HandDetector()
                logger.info("Hand detection initialized")
                
        except Exception as e:
            logger.error(f"Vision setup failed: {e}")
            with self.state_lock:
                self.state.camera_active = False
    
    def _setup_gpio(self):
        """FIXED: Setup GPIO pins and PWM with FORCED cleanup"""
        print("🔌 STARTING GPIO SETUP WITH FORCED CLEANUP...")
        
        if not GPIO_AVAILABLE:
            print("❌ GPIO LIBRARY NOT AVAILABLE - Running in simulation mode")
            logger.warning("GPIO not available - running in simulation mode")
            return
        
        try:
            # CRITICAL FIX: FORCE cleanup any existing GPIO state
            try:
                GPIO.cleanup()
                print("🧹 FORCED GPIO cleanup completed")
                logger.info("🧹 Forced GPIO cleanup completed")
            except Exception as cleanup_error:
                print(f"🧹 GPIO cleanup error (expected): {cleanup_error}")
                logger.info(f"GPIO cleanup error (expected): {cleanup_error}")
            
            # Small delay to let GPIO settle
            time.sleep(0.2)
            
            print("🔌 Setting GPIO mode to BOARD...")
            GPIO.setmode(GPIO.BOARD)
            
            print("🔌 Setting up ULTRASONIC sensors...")
            # Setup ultrasonic sensors
            sensor_pins = [
                (self.config.echo1, "ECHO1", GPIO.IN),
                (self.config.echo2, "ECHO2", GPIO.IN),
                (self.config.echo3, "ECHO3", GPIO.IN),
                (self.config.trig1, "TRIG1", GPIO.OUT, GPIO.LOW),
                (self.config.trig2, "TRIG2", GPIO.OUT, GPIO.LOW),
                (self.config.trig3, "TRIG3", GPIO.OUT, GPIO.LOW),
            ]
            
            for pin_setup in sensor_pins:
                if len(pin_setup) == 4:
                    pin, name, direction, initial = pin_setup
                    GPIO.setup(pin, direction, initial=initial)
                    print(f"   ✅ {name} (pin {pin}): {direction} = {initial}")
                else:
                    pin, name, direction = pin_setup
                    GPIO.setup(pin, direction)
                    print(f"   ✅ {name} (pin {pin}): {direction}")
            
            print("🔌 Setting up MOTOR pins...")
            # Setup motors
            motor_pins = [
                (self.config.motor1_speed, "MOTOR1_SPEED", GPIO.OUT, GPIO.LOW),
                (self.config.motor1_dir, "MOTOR1_DIR", GPIO.OUT, GPIO.LOW),
                (self.config.motor2_speed, "MOTOR2_SPEED", GPIO.OUT, GPIO.LOW),
                (self.config.motor2_dir, "MOTOR2_DIR", GPIO.OUT, GPIO.LOW),
                (self.config.motor3_speed, "MOTOR3_SPEED", GPIO.OUT, GPIO.LOW),
                (self.config.motor3_dir, "MOTOR3_DIR", GPIO.OUT, GPIO.LOW),
            ]
            
            for pin, name, direction, initial in motor_pins:
                GPIO.setup(pin, direction, initial=initial)
                print(f"   ✅ {name} (pin {pin}): {direction} = {initial}")
            
            print("🔌 Setting up PWM objects...")
            # Setup PWM with debugging
            pwm_configs = [
                (self.config.motor1_speed, "MOTOR1", self.config.pwm_frequency),
                (self.config.motor2_speed, "MOTOR2", self.config.pwm_frequency),
                (self.config.motor3_speed, "MOTOR3", self.config.pwm_frequency),
            ]
            
            for pin, name, freq in pwm_configs:
                try:
                    if name == "MOTOR1":
                        self.motor1_pwm = GPIO.PWM(pin, freq)
                        self.motor1_pwm.start(0)
                        print(f"   ✅ {name}_PWM (pin {pin}): {freq}Hz - STARTED")
                    elif name == "MOTOR2":
                        self.motor2_pwm = GPIO.PWM(pin, freq)
                        self.motor2_pwm.start(0)
                        print(f"   ✅ {name}_PWM (pin {pin}): {freq}Hz - STARTED")
                    elif name == "MOTOR3":
                        self.motor3_pwm = GPIO.PWM(pin, freq)
                        self.motor3_pwm.start(0)
                        print(f"   ✅ {name}_PWM (pin {pin}): {freq}Hz - STARTED")
                except Exception as e:
                    print(f"   ❌ {name}_PWM FAILED: {e}")
                    logger.error(f"PWM initialization failed: {e}")
                    self.gpio_initialized = False
                    return
            
            self.gpio_initialized = True
            print("✅ GPIO INITIALIZATION COMPLETE!")
            print(f"✅ PWM Frequency: {self.config.pwm_frequency}Hz")
            print(f"✅ PWM Period: {1000/self.config.pwm_frequency:.2f}ms")
            print(f"✅ Default Speed: {self.config.default_speed}%")
            logger.info("GPIO initialized successfully")
            
        except Exception as e:
            print(f"❌ GPIO INITIALIZATION FAILED: {e}")
            logger.error(f"GPIO initialization failed: {e}")
            import traceback
            print(f"📊 Traceback: {traceback.format_exc()}")
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
    
    def _obstacle_detection_loop(self):
        """FIXED: Obstacle detection loop with debug and no auto-stops"""
        print("🚀 STARTING OBSTACLE DETECTION WITH DEBUG 🚀")
        print(f"🎯 Obstacle threshold: {self.config.obstacle_threshold}cm")
        print(f"✅ Clear threshold: {self.config.obstacle_clear_threshold}cm")
        print(f"🔌 GPIO initialized: {self.gpio_initialized}")
        print("="*60)
        
        loop_count = 0
        
        while not self.shutdown_event.is_set():
            try:
                loop_count += 1
                sensor_readings = self._get_all_distances_with_names()
                
                # Debug info every 50 iterations
                if loop_count <= 20 or loop_count % 50 == 0:
                    print(f"\n📡 LOOP {loop_count} [{time.strftime('%H:%M:%S')}]")
                    if sensor_readings:
                        sensor_status = []
                        for name, dist in sensor_readings.items():
                            status_icon = "🔴" if dist < self.config.obstacle_threshold else "🟢"
                            sensor_status.append(f"{status_icon}{name.upper()}:{dist:.1f}cm")
                        print(f"   Sensors: {' | '.join(sensor_status)}")
                    else:
                        print("   ❌ NO SENSOR READINGS!")
                    print(f"   🤖 Current obstacle state: {self.state.obstacle_detected}")
                    print("-" * 40)
                
                if sensor_readings:
                    with self.state_lock:
                        self.state.sensor_distances = sensor_readings.copy()
                        self.state.last_distances = list(sensor_readings.values())
                    
                    valid_distances = [d for d in sensor_readings.values() if d > 0]
                    
                    if valid_distances:
                        min_distance = min(valid_distances)
                        min_sensor = min(sensor_readings.keys(), key=sensor_readings.get)
                        
                        obstacle_detected = min_distance < self.config.obstacle_threshold
                        
                        # Update obstacle state but DON'T auto-stop
                        with self.state_lock:
                            self.state.obstacle_detected = obstacle_detected
                            if obstacle_detected:
                                self.state.obstacle_sensor = min_sensor
                                self.state.obstacle_distance = min_distance
                                
                                # DEBUG: Print obstacle detection but DON'T stop robot
                                if loop_count % 10 == 0:  # Print every 10th detection
                                    print(f"🚨 OBSTACLE DETECTED: {min_sensor} at {min_distance:.1f}cm")
                                    print(f"   🔧 DEBUG: Robot continues running (no auto-stop)")
                            else:
                                self.state.obstacle_sensor = ""
                                self.state.obstacle_distance = 0.0
                    else:
                        print(f"⚠️ Loop {loop_count}: No valid distances received")
                        with self.state_lock:
                            self.state.obstacle_detected = False
                            self.state.obstacle_sensor = ""
                            self.state.obstacle_distance = 0.0
                else:
                    if loop_count % 20 == 0:  # Print every 20th failed reading
                        print(f"❌ Loop {loop_count}: No sensor readings at all")
                        print("   Possible causes: GPIO issues, wiring, power")
                    
                    with self.state_lock:
                        self.state.obstacle_detected = False
                        self.state.sensor_distances = {"front": 0.0, "left": 0.0, "right": 0.0}
                
                time.sleep(0.1)  # 100ms loop
                
            except Exception as e:
                print(f"❌ Obstacle detection error in loop {loop_count}: {e}")
                logger.error(f"Obstacle detection error: {e}")
                time.sleep(0.5)  # Longer delay on error
    
    def _status_update_loop(self):
        """Status update loop"""
        while not self.shutdown_event.is_set():
            try:
                with self.state_lock:
                    self.state.uptime = time.time() - self.start_time
                time.sleep(1.0)
            except Exception as e:
                logger.error(f"Status update error: {e}")
                time.sleep(1.0)
    
    def _unified_vision_loop(self):
        """FIXED: Unified vision processing loop"""
        print("📷 STARTING UNIFIED VISION LOOP")
        frame_count = 0
        last_face_check = 0
        
        while not self.shutdown_event.is_set():
            try:
                if not self.camera or not self.camera.isOpened():
                    print("❌ Camera not available, retrying...")
                    time.sleep(1.0)
                    continue
                
                success, frame = self.camera.read()
                if not success or frame is None:
                    print("❌ Failed to read frame from camera")
                    time.sleep(0.1)
                    continue
                
                frame_count += 1
                current_time = time.time()
                
                with self.vision_lock:
                    self.current_frame = frame.copy()
                
                # Face recognition every 30 frames (~1 second)
                if (frame_count % 30 == 0 and 
                    self.config.enable_face_recognition and 
                    FACE_RECOGNITION_AVAILABLE and 
                    current_time - last_face_check > 1.0):
                    
                    try:
                        self._process_face_recognition(frame)
                        last_face_check = current_time
                    except Exception as e:
                        if frame_count % 300 == 0:  # Log every 10 seconds
                            logger.error(f"Face recognition error: {e}")
                
                # Hand detection every frame if enabled
                if (self.config.enable_hand_detection and 
                    self.hand_detector and 
                    MEDIAPIPE_AVAILABLE):
                    try:
                        self._process_hand_detection(frame)
                    except Exception as e:
                        if frame_count % 300 == 0:
                            logger.error(f"Hand detection error: {e}")
                
                # Prevent excessive CPU usage
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                logger.error(f"Vision loop error: {e}")
                time.sleep(0.5)
        
        print("📷 VISION LOOP STOPPED")
    
    def _process_face_recognition(self, frame):
        """Process face recognition on frame"""
        try:
            if not FR or not hasattr(FR, 'encodeListKnown'):
                return
            
            if len(FR.encodeListKnown) == 0:
                with self.state_lock:
                    self.state.current_user = "Unknown"
                    self.state.faces_detected = []
                return
            
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Find faces in current frame
            face_locations = face_recognition.face_locations(frame_rgb)
            face_encodings = face_recognition.face_encodings(frame_rgb, face_locations)
            
            detected_names = []
            
            for face_encoding in face_encodings:
                matches = face_recognition.compare_faces(FR.encodeListKnown, face_encoding)
                face_distances = face_recognition.face_distance(FR.encodeListKnown, face_encoding)
                
                if len(face_distances) > 0:
                    best_match_index = face_distances.argmin()
                    
                    if matches[best_match_index] and face_distances[best_match_index] < 0.6:
                        name = FR.classNames[best_match_index]
                        detected_names.append(name)
            
            with self.state_lock:
                self.state.faces_detected = detected_names
                if detected_names:
                    self.state.current_user = detected_names[0]  # Use first detected face
                    self.state.face_recognition_attempts = 0
                else:
                    self.state.face_recognition_attempts += 1
                    if self.state.face_recognition_attempts >= 3:
                        self.state.current_user = "Unknown"
                
        except Exception as e:
            logger.error(f"Face recognition processing error: {e}")
    
    def _process_hand_detection(self, frame):
        """Process hand detection on frame"""
        try:
            if not self.hand_detector:
                return
            
            frame = self.hand_detector.find_hands(frame, draw=False)
            lm_list = self.hand_detector.find_position(frame, draw=False)
            
            gesture = "none"
            
            if lm_list:
                if self.hand_detector.is_waving(lm_list, self.index_x_history):
                    gesture = "waving"
                elif self.hand_detector.is_shaking_hands(lm_list):
                    gesture = "handshake"
                elif self.hand_detector.fingers_up(lm_list):
                    gesture = "open_hand"
            
            with self.state_lock:
                self.state.hand_gesture = gesture
                
        except Exception as e:
            logger.error(f"Hand detection processing error: {e}")
    
    def _get_all_distances_with_names(self) -> Dict[str, float]:
        """FIXED: Get distances from all sensors with proper error handling"""
        distances = {}
        
        if not self.gpio_initialized:
            return distances
        
        sensor_configs = [
            (self.config.trig1, self.config.echo1, "front"),
            (self.config.trig2, self.config.echo2, "left"), 
            (self.config.trig3, self.config.echo3, "right")
        ]
        
        for trig_pin, echo_pin, name in sensor_configs:
            try:
                distance = self._get_distance(trig_pin, echo_pin)
                if 0 < distance < 400:  # Valid range for HC-SR04
                    distances[name] = distance
                else:
                    distances[name] = 0.0  # Invalid reading
            except Exception as e:
                distances[name] = 0.0
                logger.error(f"Error reading {name} sensor: {e}")
        
        return distances
    
    def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
        """FIXED: Get distance from ultrasonic sensor with timeout"""
        if not self.gpio_initialized:
            return 0.0
        
        try:
            # Send trigger pulse
            GPIO.output(trig_pin, GPIO.LOW)
            time.sleep(0.002)  # 2ms settle time
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10μs pulse
            GPIO.output(trig_pin, GPIO.LOW)
            
            # Wait for echo to go HIGH with timeout
            timeout_start = time.time()
            while GPIO.input(echo_pin) == GPIO.LOW:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return 0.0
                pulse_start = time.time()
            
            # Wait for echo to go LOW with timeout  
            timeout_start = time.time()
            while GPIO.input(echo_pin) == GPIO.HIGH:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return 0.0
                pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 34300) / 2  # Speed of sound = 343 m/s
            
            return round(distance, 2)
            
        except Exception as e:
            logger.error(f"Distance measurement error on pins {trig_pin}/{echo_pin}: {e}")
            return 0.0
    
    def move_forward(self, duration: float = 0, speed: Optional[int] = None):
        """Move robot forward with NO auto-stop on obstacles"""
        with self.movement_lock:
            try:
                actual_speed = speed or self.config.default_speed
                print(f"🚀 MOVING FORWARD: speed={actual_speed}%, duration={duration}s")
                
                if self.gpio_initialized:
                    # Motor 1: Forward
                    GPIO.output(self.config.motor1_dir, self.LOW)
                    self.motor1_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 2: Forward  
                    GPIO.output(self.config.motor2_dir, self.LOW)
                    self.motor2_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 3: Forward (with compensation)
                    GPIO.output(self.config.motor3_dir, self.LOW)
                    self.motor3_pwm.ChangeDutyCycle(actual_speed + self.config.motor3_compensate)
                
                with self.state_lock:
                    self.state.motor1_speed = actual_speed
                    self.state.motor2_speed = actual_speed  
                    self.state.motor3_speed = actual_speed + self.config.motor3_compensate
                    self.state.status = "moving_forward"
                    self.state.last_command = f"forward_{actual_speed}"
                
                if duration > 0:
                    time.sleep(duration)
                    self.stop()
                    
            except Exception as e:
                logger.error(f"Move forward error: {e}")
                self.stop()
    
    def move_backward(self, duration: float = 0, speed: Optional[int] = None):
        """Move robot backward"""
        with self.movement_lock:
            try:
                actual_speed = speed or self.config.default_speed
                print(f"🔄 MOVING BACKWARD: speed={actual_speed}%, duration={duration}s")
                
                if self.gpio_initialized:
                    # Motor 1: Backward
                    GPIO.output(self.config.motor1_dir, self.HIGH)
                    self.motor1_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 2: Backward
                    GPIO.output(self.config.motor2_dir, self.HIGH)  
                    self.motor2_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 3: Backward (with compensation)
                    GPIO.output(self.config.motor3_dir, self.HIGH)
                    self.motor3_pwm.ChangeDutyCycle(actual_speed + self.config.motor3_compensate)
                
                with self.state_lock:
                    self.state.motor1_speed = -actual_speed
                    self.state.motor2_speed = -actual_speed
                    self.state.motor3_speed = -(actual_speed + self.config.motor3_compensate)
                    self.state.status = "moving_backward"
                    self.state.last_command = f"backward_{actual_speed}"
                
                if duration > 0:
                    time.sleep(duration) 
                    self.stop()
                    
            except Exception as e:
                logger.error(f"Move backward error: {e}")
                self.stop()
    
    def turn_left(self, duration: float = 0, speed: Optional[int] = None):
        """Turn robot left"""
        with self.movement_lock:
            try:
                actual_speed = speed or self.config.default_speed
                print(f"↩️ TURNING LEFT: speed={actual_speed}%, duration={duration}s")
                
                if self.gpio_initialized:
                    # Motor 1: Backward (left side)
                    GPIO.output(self.config.motor1_dir, self.HIGH)
                    self.motor1_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 2: Forward (right side)
                    GPIO.output(self.config.motor2_dir, self.LOW)
                    self.motor2_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 3: Stop or slow
                    GPIO.output(self.config.motor3_dir, self.LOW)
                    self.motor3_pwm.ChangeDutyCycle(0)
                
                with self.state_lock:
                    self.state.motor1_speed = -actual_speed
                    self.state.motor2_speed = actual_speed
                    self.state.motor3_speed = 0
                    self.state.status = "turning_left"
                    self.state.last_command = f"left_{actual_speed}"
                
                if duration > 0:
                    time.sleep(duration)
                    self.stop()
                    
            except Exception as e:
                logger.error(f"Turn left error: {e}")
                self.stop()
    
    def turn_right(self, duration: float = 0, speed: Optional[int] = None):
        """Turn robot right"""
        with self.movement_lock:
            try:
                actual_speed = speed or self.config.default_speed
                print(f"↪️ TURNING RIGHT: speed={actual_speed}%, duration={duration}s")
                
                if self.gpio_initialized:
                    # Motor 1: Forward (left side)
                    GPIO.output(self.config.motor1_dir, self.LOW)
                    self.motor1_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 2: Backward (right side)
                    GPIO.output(self.config.motor2_dir, self.HIGH)
                    self.motor2_pwm.ChangeDutyCycle(actual_speed)
                    
                    # Motor 3: Stop or slow
                    GPIO.output(self.config.motor3_dir, self.LOW) 
                    self.motor3_pwm.ChangeDutyCycle(0)
                
                with self.state_lock:
                    self.state.motor1_speed = actual_speed
                    self.state.motor2_speed = -actual_speed
                    self.state.motor3_speed = 0
                    self.state.status = "turning_right"
                    self.state.last_command = f"right_{actual_speed}"
                
                if duration > 0:
                    time.sleep(duration)
                    self.stop()
                    
            except Exception as e:
                logger.error(f"Turn right error: {e}")
                self.stop()
    
    def stop(self):
        """FIXED: Stop robot with debug info"""
        with self.movement_lock:
            try:
                print("🛑 STOPPING ROBOT")
                
                if self.gpio_initialized:
                    # Stop all motors
                    self.motor1_pwm.ChangeDutyCycle(0)
                    self.motor2_pwm.ChangeDutyCycle(0)
                    self.motor3_pwm.ChangeDutyCycle(0)
                    
                    # Set all direction pins to LOW
                    GPIO.output(self.config.motor1_dir, self.LOW)
                    GPIO.output(self.config.motor2_dir, self.LOW)
                    GPIO.output(self.config.motor3_dir, self.LOW)
                
                with self.state_lock:
                    self.state.motor1_speed = 0
                    self.state.motor2_speed = 0
                    self.state.motor3_speed = 0
                    self.state.status = "stopped"
                    self.state.last_command = "stop"
                
                print("✅ Robot stopped successfully")
                    
            except Exception as e:
                print(f"❌ Stop error: {e}")
                logger.error(f"Stop error: {e}")
    
    def get_state(self) -> dict:
        """Get current robot state"""
        with self.state_lock:
            return {
                "motor1_speed": self.state.motor1_speed,
                "motor2_speed": self.state.motor2_speed, 
                "motor3_speed": self.state.motor3_speed,
                "status": self.state.status,
                "obstacle_detected": self.state.obstacle_detected,
                "sensor_distances": self.state.sensor_distances.copy(),
                "obstacle_sensor": self.state.obstacle_sensor,
                "obstacle_distance": self.state.obstacle_distance,
                "last_command": self.state.last_command,
                "uptime": self.state.uptime,
                "current_user": self.state.current_user,
                "faces_detected": self.state.faces_detected.copy(),
                "hand_gesture": self.state.hand_gesture,
                "camera_active": self.state.camera_active,
                "listening": self.state.listening,
                "gpio_initialized": self.gpio_initialized,
                "interaction_mode": self.state.interaction_mode
            }
    
    def take_picture(self, name: str) -> bool:
        """FIXED: Take picture using face helper"""
        try:
            if not self.camera or not self.camera.isOpened():
                logger.error("Camera not available for picture")
                return False
            
            return FR.take_picture(name, self.camera)
            
        except Exception as e:
            logger.error(f"Take picture error: {e}")
            return False
    
    def speak(self, text: str):
        """Speak text using TTS"""
        try:
            if hasattr(FR, 'speak'):
                FR.speak(text)
            else:
                print(f"🔊 TTS: {text}")
            
            with self.state_lock:
                self.state.last_speech_output = text
                
        except Exception as e:
            logger.error(f"Speech error: {e}")
            print(f"🔊 TTS: {text}")
    
    def listen(self) -> Optional[str]:
        """Listen for speech input"""
        try:
            with self.state_lock:
                self.state.listening = True
            
            if hasattr(FR, 'listen'):
                result = FR.listen()
                with self.state_lock:
                    self.state.listening = False
                return result
            else:
                with self.state_lock:
                    self.state.listening = False
                return None
                
        except Exception as e:
            logger.error(f"Listen error: {e}")
            with self.state_lock:
                self.state.listening = False
            return None
    
    def get_camera_frame(self) -> Optional[bytes]:
        """Get current camera frame as JPEG bytes"""
        try:
            if not self.current_frame is None:
                with self.vision_lock:
                    frame = self.current_frame.copy()
                
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                return buffer.tobytes()
            return None
            
        except Exception as e:
            logger.error(f"Get camera frame error: {e}")
            return None
    
    def cleanup(self):
        """FIXED: Cleanup resources with forced GPIO cleanup"""
        try:
            print("🧹 STARTING ROBOT CLEANUP...")
            
            # Stop background tasks
            self.shutdown_event.set()
            
            # Stop robot movement
            self.stop()
            
            # Wait for threads to finish
            for thread in [self.obstacle_thread, self.status_thread, self.vision_thread]:
                if thread and thread.is_alive():
                    thread.join(timeout=2.0)
            
            # Release camera
            if self.camera:
                self.camera.release()
                print("📷 Camera released")
            
            if hasattr(FR, 'cap') and FR.cap:
                FR.cap.release()
                print("📷 Face helper camera released")
            
            # CRITICAL: Force GPIO cleanup
            if GPIO_AVAILABLE:
                try:
                    print("🔌 Stopping PWM...")
                    if self.motor1_pwm:
                        self.motor1_pwm.stop()
                    if self.motor2_pwm:
                        self.motor2_pwm.stop()
                    if self.motor3_pwm:
                        self.motor3_pwm.stop()
                    
                    print("🔌 Forcing GPIO cleanup...")
                    GPIO.cleanup()
                    print("✅ GPIO cleanup completed")
                    
                except Exception as e:
                    print(f"⚠️ GPIO cleanup error: {e}")
            
            print("✅ ROBOT CLEANUP COMPLETE")
            
        except Exception as e:
            print(f"❌ Cleanup error: {e}")
            logger.error(f"Cleanup error: {e}")

# Example usage and testing
if __name__ == "__main__":
    print("🤖 STARTING ENHANCED ROBOT CONTROLLER TEST...")
    
    try:
        # Create robot with debug config
        config = EnhancedRobotConfig()
        robot = EnhancedRobotController(config)
        
        print("✅ Robot initialized successfully")
        print(f"📊 GPIO Status: {'✅ Working' if robot.gpio_initialized else '❌ Failed'}")
        print(f"📷 Camera Status: {'✅ Active' if robot.state.camera_active else '❌ Inactive'}")
        
        # Test basic functionality
        print("\n🧪 RUNNING BASIC TESTS...")
        
        # Test 1: Get initial state
        state = robot.get_state()
        print(f"📊 Initial state: {state['status']}")
        
        # Test 2: Movement test (if GPIO working)
        if robot.gpio_initialized:
            print("🚀 Testing movement (2 seconds each)...")
            robot.move_forward(2.0, 20)
            time.sleep(1)
            robot.turn_left(1.0, 20)
            time.sleep(1)
            robot.turn_right(1.0, 20)
            time.sleep(1)
            robot.move_backward(1.0, 20)
            robot.stop()
            print("✅ Movement test complete")
        else:
            print("⚠️ Skipping movement test (GPIO not initialized)")
        
        # Test 3: Status monitoring
        print("📊 Monitoring status for 10 seconds...")
        for i in range(10):
            state = robot.get_state()
            print(f"  Status: {state['status']}, Uptime: {state['uptime']:.1f}s, "
                  f"User: {state['current_user']}, Obstacles: {state['obstacle_detected']}")
            time.sleep(1)
        
        print("✅ All tests completed successfully!")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'robot' in locals():
            robot.cleanup()
        print("🏁 Test session ended")