# face_helper.py - FIXED with DIRECT ABSOLUTE PATH
import os
import cv2
import face_recognition
import numpy as np
import platform
import subprocess
import speech_recognition as sr
import logging
import json
from datetime import datetime
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FaceRecognitionSystem:
    """Enhanced face recognition system with DIRECT ABSOLUTE PATH"""
    
    def __init__(self, images_path=None):
        # FIXED: Use DIRECT ABSOLUTE PATH - no more confusion!
        if images_path is None:
            # DIRECT PATH - change this to match your exact system
            if platform.system().lower() == "windows":
                # For Windows (your development machine)
                self.images_path = "/home/robot/summer-research-robot/RGB-cam/images"
            else:
                # For Linux/Raspberry Pi
                self.images_path = "/home/robot/summer-research-robot/RGB-cam/images"
        else:
            self.images_path = images_path
        
        # Ensure directory exists
        os.makedirs(self.images_path, exist_ok=True)
        
        # Initialize storage
        self.images = []
        self.classNames = []
        self.encodeListKnown = []
        
        # Camera reference (set by robot_movement.py)
        self.cap = None
        
        # Recognition state
        self.recognition_attempts = 0
        self.max_attempts = 3
        self.last_recognition_time = 0
        
        # Load existing face data
        self.load_face_data()
        
        logger.info(f"✅ Face recognition initialized with {len(self.classNames)} known users")
        logger.info(f"📁 DIRECT Images directory: {self.images_path}")
        
        # Verify the path actually exists and is writable
        if os.path.exists(self.images_path) and os.access(self.images_path, os.W_OK):
            logger.info(f"✅ Directory is accessible and writable")
        else:
            logger.error(f"❌ Directory is not accessible or writable!")
    
    def load_face_data(self):
        """Load all face images and generate encodings"""
        try:
            self.images = []
            self.classNames = []
            
            if not os.path.exists(self.images_path):
                logger.warning(f"Images path does not exist: {self.images_path}")
                os.makedirs(self.images_path, exist_ok=True)
                return
            
            # Get all image files
            image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
            image_files = []
            
            for file in os.listdir(self.images_path):
                file_lower = file.lower()
                if any(file_lower.endswith(ext) for ext in image_extensions):
                    image_files.append(file)
            
            logger.info(f"Found {len(image_files)} image files in {self.images_path}")
            
            for image_file in image_files:
                try:
                    full_path = os.path.join(self.images_path, image_file)
                    img = cv2.imread(full_path)
                    
                    if img is not None:
                        self.images.append(img)
                        username = os.path.splitext(image_file)[0]
                        self.classNames.append(username)
                        logger.info(f"✅ Loaded image for: {username}")
                    else:
                        logger.warning(f"⚠️ Could not load image: {image_file}")
                        
                except Exception as e:
                    logger.error(f"❌ Error loading {image_file}: {e}")
            
            # Generate encodings
            if self.images:
                self.encodeListKnown = self.findEncodings(self.images)
                logger.info(f"Generated {len(self.encodeListKnown)} face encodings")
            else:
                logger.warning("No valid images found for face recognition")
                self.encodeListKnown = []
            
        except Exception as e:
            logger.error(f"Error loading face data: {e}")
            self.images = []
            self.classNames = []
            self.encodeListKnown = []
    
    def findEncodings(self, images):
        """Generate face encodings with proper error handling"""
        encodeList = []
        
        if not images:
            logger.warning("No images provided for encoding")
            return encodeList
        
        for i, img in enumerate(images):
            try:
                if img is None:
                    logger.warning(f"Image {i+1} is None, skipping")
                    continue
                
                # Convert to RGB
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Get face encodings
                face_encodings = face_recognition.face_encodings(img_rgb)
                
                if face_encodings:
                    encode = face_encodings[0]  # Take the first face found
                    encodeList.append(encode)
                    logger.debug(f"✅ Generated encoding for image {i+1}")
                else:
                    logger.warning(f"⚠️ No face found in image {i+1}")
                    encodeList.append(None)
                    
            except Exception as e:
                logger.error(f"❌ Error processing image {i+1}: {e}")
                encodeList.append(None)
                continue
        
        # Filter out None values and update class names accordingly
        valid_encodings = []
        valid_names = []
        for i, encoding in enumerate(encodeList):
            if encoding is not None:
                valid_encodings.append(encoding)
                if i < len(self.classNames):
                    valid_names.append(self.classNames[i])
        
        # Update class names to match valid encodings
        self.classNames = valid_names
        
        logger.info(f"Successfully generated {len(valid_encodings)} valid face encodings")
        return valid_encodings
    
    def take_picture(self, name, camera):
        """Use the EXACT same method that works in standalone"""
        success, image = camera.read()

        if success:
            # Use the exact same path as your working standalone version
            path = os.path.join('/home/robot/summer-research-robot/RGB-cam/images', name + '.jpg')
            result = cv2.imwrite(path, image)
            
            if result and os.path.exists(path):
                print(f"✅ Image saved successfully to {path}")
                file_size = os.path.getsize(path)
                print(f"📏 File size: {file_size} bytes")
                
                # Reload face data like in standalone
                self.load_face_data()
                return True
            else:
                print(f"❌ Failed to save image to {path}")
                return False
        else:
            print("❌ Failed to capture image from camera")
            return False
    def recognize_face_in_frame(self, img):
        """Recognize faces in a single frame with improved logic"""
        try:
            if img is None:
                return None, 0.0
            
            # Check if we have any known encodings
            if not self.encodeListKnown or len(self.encodeListKnown) == 0:
                logger.debug("No known face encodings available")
                return None, 0.0
            
            # Resize for faster processing but keep good quality
            height, width = img.shape[:2]
            scale = min(640/width, 480/height) if width > 640 or height > 480 else 1.0
            
            if scale < 1.0:
                new_width = int(width * scale)
                new_height = int(height * scale)
                imgS = cv2.resize(img, (new_width, new_height))
            else:
                imgS = img.copy()
            
            # Convert to RGB
            imgS_rgb = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)
            
            # Find faces in the frame
            facesCurFrame = face_recognition.face_locations(imgS_rgb)
            
            if not facesCurFrame:
                logger.debug("No faces detected in frame")
                return None, 0.0
            
            # Get face encodings
            encodesCurFrame = face_recognition.face_encodings(imgS_rgb, facesCurFrame)
            
            if not encodesCurFrame:
                logger.debug("No face encodings generated from detected faces")
                return None, 0.0
            
            best_match_name = None
            best_confidence = 0.0
            
            # Process each face found
            for encodeFace in encodesCurFrame:
                # Compare with known faces
                matches = face_recognition.compare_faces(self.encodeListKnown, encodeFace, tolerance=0.6)
                faceDis = face_recognition.face_distance(self.encodeListKnown, encodeFace)
                
                if len(faceDis) > 0:
                    matchIndex = np.argmin(faceDis)
                    best_distance = faceDis[matchIndex]
                    
                    logger.debug(f"Best distance: {best_distance}, Match: {matches[matchIndex]}")
                    
                    # Use more lenient threshold for recognition
                    if best_distance < 0.6 and matches[matchIndex]:
                        confidence = 1.0 - best_distance
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_match_name = self.classNames[matchIndex]
                            logger.debug(f"Match found: {best_match_name} (confidence: {confidence:.3f})")
            
            if best_match_name:
                # Convert to display format
                display_name = best_match_name.replace('_', ' ').title()
                logger.info(f"✅ Face recognized: {display_name} (confidence: {best_confidence:.3f})")
                return display_name, best_confidence
            else:
                logger.debug("No matches found above threshold")
            
            return None, 0.0
            
        except Exception as e:
            logger.error(f"❌ Error in face recognition: {e}")
            return None, 0.0
    
    def findMatch(self, mode="auto", single_frame=None):
        """Enhanced findMatch with proper recognition logic"""
        if single_frame is not None:
            # Single frame mode for web interface
            name, confidence = self.recognize_face_in_frame(single_frame)
            return name if name else "Unknown"
        
        # Continuous mode for direct camera access
        if not self.cap or not self.cap.isOpened():
            logger.error("❌ Camera not available for continuous recognition")
            return "Unknown"
        
        return self._continuous_recognition(mode)
    
    def _continuous_recognition(self, mode):
        """Continuous face recognition with proper logic"""
        unknown_count = 0
        recognition_frames = 0
        max_unknown_before_registration = 10
        min_recognition_frames = 5
        
        consecutive_matches = {}  # Track consecutive matches for each person
        
        while self.recognition_attempts < self.max_attempts:
            if not self.cap or not self.cap.isOpened():
                logger.error("❌ Camera not available")
                return "Unknown"
            
            success, img = self.cap.read()
            if not success or img is None:
                continue
            
            recognition_frames += 1
            name, confidence = self.recognize_face_in_frame(img)
            
            if name and name != "Unknown":
                # Person recognized
                if name not in consecutive_matches:
                    consecutive_matches[name] = 0
                consecutive_matches[name] += 1
                
                # Reset other counters
                unknown_count = 0
                for other_name in consecutive_matches:
                    if other_name != name:
                        consecutive_matches[other_name] = 0
                
                # Require multiple consecutive recognitions for confirmation
                if consecutive_matches[name] >= 3:
                    logger.info(f"✅ User confirmed after {consecutive_matches[name]} consecutive recognitions: {name}")
                    return name
                    
            else:
                # No recognition
                unknown_count += 1
                # Reset all consecutive matches
                for name in consecutive_matches:
                    consecutive_matches[name] = 0
                
                # If we've tried enough frames without recognition
                if unknown_count >= max_unknown_before_registration and recognition_frames >= min_recognition_frames:
                    self.recognition_attempts += 1
                    logger.info(f"❌ Recognition attempt {self.recognition_attempts}/{self.max_attempts} failed after {recognition_frames} frames")
                    
                    if self.recognition_attempts >= self.max_attempts:
                        logger.info("❌ Max recognition attempts reached")
                        return "Unknown"
                    
                    # Register new user if in appropriate mode
                    if mode in ["t", "s"]:
                        try:
                            if mode == "t":
                                name = input("Welcome new visitor! What is your name? : ")
                            elif mode == "s":
                                speak("Welcome new visitor! What is your name?")
                                name = listen()
                            
                            if name and self.take_picture(name, self.cap):
                                return name.title()
                        except Exception as e:
                            logger.error(f"Registration error: {e}")
                    
                    # Reset for next attempt
                    unknown_count = 0
                    recognition_frames = 0
                    consecutive_matches = {}
        
        return "Unknown"
    
    def reset_recognition_state(self):
        """Reset recognition attempts"""
        self.recognition_attempts = 0
        self.last_recognition_time = 0
        logger.info("🔄 Face recognition state reset")
    
    def get_registered_users(self):
        """Get list of registered users"""
        return [name.replace('_', ' ').title() for name in self.classNames]
    
    def delete_user(self, username):
        """Delete a user's image and reload data"""
        try:
            clean_name = username.strip().replace(' ', '_').lower()
            
            # Try different extensions
            extensions = ['.jpg', '.jpeg', '.png', '.bmp']
            deleted = False
            
            for ext in extensions:
                image_path = os.path.join(self.images_path, f"{clean_name}{ext}")
                if os.path.exists(image_path):
                    os.remove(image_path)
                    logger.info(f"✅ Deleted image for user: {username} ({ext})")
                    deleted = True
                    break
            
            if deleted:
                # Reload face data
                self.load_face_data()
                return True
            else:
                logger.warning(f"⚠️ No image found for user: {username}")
                return False
                
        except Exception as e:
            logger.error(f"❌ Error deleting user {username}: {e}")
            return False

# Create global instance
face_recognition_system = FaceRecognitionSystem()

def speak(text):
    """Cross-platform TTS implementation"""
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
                subprocess.run(["espeak", text], check=True, capture_output=True)
            except (subprocess.CalledProcessError, FileNotFoundError):
                try:
                    subprocess.run(["spd-say", text], check=True, capture_output=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    print(f"🔊 TTS: {text}")
        else:
            print(f"🔊 TTS: {text}")
            
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            if system == "windows":
                vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
                subprocess.run(["wscript", "/nologo", "-"], 
                              input=vbs_script, text=True, capture_output=True)
            elif system == "darwin":
                applescript = f'say "{text}"'
                subprocess.run(["osascript", "-e", applescript], check=True)
            else:
                raise subprocess.CalledProcessError(1, "TTS failed")
        except:
            print(f"🔊 TTS: {text}")

def listen():
    """Speech recognition"""
    if not 'sr' in globals():
        return None
        
    try:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("🎙️ Listening...")
            r.adjust_for_ambient_noise(source, duration=1)
            audio = r.listen(source, timeout=5, phrase_time_limit=5)
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
    except sr.UnknownValueError:
        print("❌ Didn't catch that.")
    except sr.RequestError as e:
        print(f"❌ Speech recognition API error: {e}")
    except sr.WaitTimeoutError:
        print("❌ Listening timeout.")
    except Exception as e:
        print(f"❌ Speech recognition error: {e}")
    return None

# Backward compatibility functions
def take_picture(name, camera):
    """FIXED: Backward compatibility wrapper - uses DIRECT PATH"""
    return face_recognition_system.take_picture(name, camera)

def findMatch(mode="auto", single_frame=None):
    """Backward compatibility wrapper"""
    return face_recognition_system.findMatch(mode, single_frame)

def findEncodings(images):
    """Backward compatibility wrapper"""
    return face_recognition_system.findEncodings(images)

def reload_face_data():
    """Reload face recognition data"""
    face_recognition_system.load_face_data()

# Initialize global variables for backward compatibility
def initialize_globals():
    """Initialize global variables for backward compatibility"""
    global images, classNames, encodeListKnown, path, cap
    
    images = face_recognition_system.images
    classNames = face_recognition_system.classNames
    encodeListKnown = face_recognition_system.encodeListKnown
    path = '/home/robot/summer-research-robot/RGB-cam/images'
    cap = face_recognition_system.cap

# Initialize on import
initialize_globals()

# Camera reference (will be set by robot_movement.py)
cap = None