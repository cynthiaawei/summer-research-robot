import os
import cv2
import face_recognition
import numpy as np
import platform
import subprocess
import speech_recognition as sr
import logging

def speak(text):
    """Cross-platform TTS implementation"""
    system = platform.system().lower()
    
    try:
        if system == "windows":
            # Method 1: Use Windows Speech API via PowerShell (most reliable)
            ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_command], 
                          capture_output=True, check=True)
        elif system == "darwin":  # macOS
            # Use macOS built-in 'say' command
            subprocess.run(["say", text], check=True)
        elif system == "linux":
            # Try multiple Linux TTS options
            try:
                # Try espeak first
                subprocess.run(["espeak", text], check=True)
            except (subprocess.CalledProcessError, FileNotFoundError):
                try:
                    # Try festival if espeak not available
                    subprocess.run(["echo", text, "|", "festival", "--tts"], 
                                 shell=True, check=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    # Fallback to spd-say if available
                    subprocess.run(["spd-say", text], check=True)
        else:
            # Unknown system, just print
            print(f"🔊 TTS: {text}")
            
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback methods for each platform
        try:
            if system == "windows":
                # Method 2: Use Windows SAPI via wscript (fallback)
                vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
                subprocess.run(["wscript", "/nologo", "-"], 
                              input=vbs_script, text=True, capture_output=True)
            elif system == "darwin":
                # Alternative macOS method using osascript
                applescript = f'say "{text}"'
                subprocess.run(["osascript", "-e", applescript], check=True)
            else:
                raise subprocess.CalledProcessError(1, "TTS failed")
        except:
            # Final fallback: print to console
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
    
def take_picture(name, camera):
    """Take a picture for face registration with better error handling"""
    try:
        success, image = camera.read()

        if success and image is not None:
            # Use local facepics directory instead of the original path
            backend_dir = os.path.dirname(__file__)
            facepics_dir = os.path.join(backend_dir, 'facepics')
            
            # Create directory if it doesn't exist
            os.makedirs(facepics_dir, exist_ok=True)
            
            path = os.path.join(facepics_dir, name + '.jpg')
            result = cv2.imwrite(path, image)
            
            if result and os.path.exists(path):
                print(f"✅ Picture saved successfully: {path}")
                return True
            else:
                print(f"❌ Failed to save image to {path}")
                return False
        else:
            print("❌ Failed to capture image from camera")
            return False
    except Exception as e:
        print(f"❌ Error taking picture: {e}")
        return False

# FIXED: Use facepics directory and handle empty directories
backend_dir = os.path.dirname(__file__)
facepics_dir = os.path.join(backend_dir, 'facepics')

# Create facepics directory if it doesn't exist
os.makedirs(facepics_dir, exist_ok=True)

# Use facepics as the main path
path = facepics_dir
images = []
classNames = []

# Load existing images safely
try:
    myList = os.listdir(path)
    print(f"📁 Found {len(myList)} files in facepics directory")
    
    for cl in myList:
        if cl.lower().endswith(('.png', '.jpg', '.jpeg')):
            try:
                img_path = os.path.join(path, cl)
                curImg = cv2.imread(img_path)
                if curImg is not None:
                    images.append(curImg)
                    classNames.append(os.path.splitext(cl)[0])
                    print(f"✅ Loaded image for: {os.path.splitext(cl)[0]}")
                else:
                    print(f"⚠️ Could not load image: {cl}")
            except Exception as e:
                print(f"❌ Error loading {cl}: {e}")
                
except Exception as e:
    print(f"❌ Error reading facepics directory: {e}")
    myList = []

print(f"📊 Loaded {len(images)} valid face images")
print(f"👥 Known users: {classNames}")

# FIXED: Enhanced findEncodings function with error handling
def findEncodings(images):
    """Generate face encodings with proper error handling"""
    encodeList = []
    
    if not images:
        print("⚠️ No images provided to findEncodings")
        return encodeList
    
    for i, img in enumerate(images):
        try:
            if img is None:
                print(f"⚠️ Image {i+1} is None, skipping")
                continue
                
            # Convert to RGB
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Get face encodings
            face_encodings = face_recognition.face_encodings(img_rgb)
            
            if face_encodings:
                # Take the first face encoding
                encode = face_encodings[0]
                encodeList.append(encode)
                print(f"✅ Generated encoding for image {i+1}")
            else:
                print(f"⚠️ No face found in image {i+1} - skipping")
                
        except Exception as e:
            print(f"❌ Error processing image {i+1}: {e}")
            continue
    
    print(f"📊 Successfully generated {len(encodeList)} face encodings")
    return encodeList

# Generate encodings for existing images
encodeListKnown = findEncodings(images)

# Camera will be set by robot_movement.py
cap = None

# FIXED: Enhanced findMatch function for single-frame recognition
def findMatch(mode=None, single_frame=None):
    """
    Find face match - can work with single frame or continuous capture
    """
    if single_frame is not None:
        # Single frame mode for web interface
        return _process_single_frame(single_frame)
    else:
        # Original continuous mode
        return _continuous_face_recognition(mode)

def _process_single_frame(img):
    """Process a single frame for face recognition"""
    try:
        if img is None:
            return "Unknown"
            
        # Resize for faster processing
        imgS = cv2.resize(img, (0, 0), None, 0.25, 0.25)
        imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)

        # Find faces in the frame
        facesCurFrame = face_recognition.face_locations(imgS)
        encodesCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)

        # Process each face found
        for encodeFace, faceLoc in zip(encodesCurFrame, facesCurFrame):
            if not encodeListKnown:
                return "Unknown"  # No known faces to compare against
                
            # Compare with known faces
            matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
            faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
            
            if len(faceDis) > 0:
                matchIndex = np.argmin(faceDis)
                best_match_distance = faceDis[matchIndex]
                
                # Check if it's a good match (distance < 0.42)
                if best_match_distance <= 0.42 and matches[matchIndex]:
                    name = classNames[matchIndex].upper()
                    print(f"✅ Face recognized: {name} (distance: {best_match_distance:.3f})")
                    return name
        
        return "Unknown"
        
    except Exception as e:
        print(f"❌ Error in face recognition: {e}")
        return "Unknown"

def _continuous_face_recognition(mode):
    """Original continuous face recognition mode"""
    unknown_num = 0
    known_num = 0
    
    while True:
        if cap is None or not cap.isOpened():
            print("❌ Camera not available")
            return None
            
        success, img = cap.read()
        if not success:
            continue
            
        # Process the frame
        result = _process_single_frame(img)
        
        if result != "Unknown":
            known_num += 1
            unknown_num = 0
            if known_num > 2:
                known_num = 0
                return result
        else:
            unknown_num += 1
            known_num = 0
            
            if unknown_num > 3:  # Unknown face detected 3 times
                if mode == "t": 
                    name = input("Welcome new visitor! What is your name? : ")
                elif mode == "s": 
                    speak("Welcome new visitor! What is your name?")
                    name = listen()
                else:
                    return "Unknown"  # Don't auto-register in web mode
                
                if name:
                    # Take picture and update encodings
                    if take_picture(name, cap):
                        # Reload the face recognition data
                        reload_face_data()
                        return name.upper()
                
                unknown_num = 0

def reload_face_data():
    """Reload face recognition data after new registration"""
    global images, classNames, encodeListKnown
    
    try:
        print("🔄 Reloading face recognition data...")
        
        # Clear existing data
        images = []
        classNames = []
        
        # Reload images
        myList = os.listdir(path)
        for cl in myList:
            if cl.lower().endswith(('.png', '.jpg', '.jpeg')):
                try:
                    img_path = os.path.join(path, cl)
                    curImg = cv2.imread(img_path)
                    if curImg is not None:
                        images.append(curImg)
                        classNames.append(os.path.splitext(cl)[0])
                except Exception as e:
                    print(f"❌ Error reloading {cl}: {e}")
        
        # Regenerate encodings
        encodeListKnown = findEncodings(images)
        print(f"✅ Reloaded {len(classNames)} users: {classNames}")
        
    except Exception as e:
        print(f"❌ Error reloading face data: {e}")
