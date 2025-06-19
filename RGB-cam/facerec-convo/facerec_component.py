import os
import cv2
import face_recognition
import numpy as np
import platform
import subprocess
import speech_recognition as sr

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
    success, image = camera.read()

    if success:
        path = os.path.join('/home/robot/summer-research-robot/RGB-cam/images', name + '.jpg')
        cv2.imwrite(path, image)
        if not os.path.exists(path):
            print(f"❌ Failed to save image to {path}")
    else:
        print("❌ Failed to capture image from camera")

path = '/home/robot/summer-research-robot/RGB-cam/images'
images = []
classNames = []
myList = os.listdir(path)

# remove ".jpg" from the names & put in classNames
for cl in myList:
    curImg = cv2.imread(f'{path}/{cl}')
    images.append(curImg)
    classNames.append(os.path.splitext(cl)[0])

# make a list of all image encodings (images represented as vectors)
def findEncodings(images):
    encodeList = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encode = face_recognition.face_encodings(img)[0]
        encodeList.append(encode)
    return encodeList

encodeListKnown = findEncodings(images)

# turn on camera
cap = cv2.VideoCapture(0)


# find matches
def findMatch(mode):
    unknown_num = 0
    known_num = 0
    while True:
        success, img = cap.read() # give image
        imgS = cv2.resize(img,(0,0), None, 0.25, 0.25) # crop image (speed up process)
        imgS = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # convert image to rbg

        facesCurFrame = face_recognition.face_locations(imgS)  # find exact location of live face
        encodesCurFrame = face_recognition.face_encodings(imgS, facesCurFrame) # get encoding of live face (vector format)

        for encodeFace, faceLoc in zip(encodesCurFrame, facesCurFrame): # iterate through live faces & compare to image in saved file
            matches = face_recognition.compare_faces(encodeListKnown, encodeFace) # compare list known & live face
            faceDis = face_recognition.face_distance(encodeListKnown,encodeFace)
            
            matchIndex = np.argmin(faceDis) # obtain index of the exa

            # smallest faceDis value
            best_match_distance = faceDis[matchIndex]
            #print(best_match_distance)
            
            if(best_match_distance > 0.42): # unknown face
                #print("UNKNOWN")
                unknown_num += 1 
                known_num = 0

                if(unknown_num > 3): # check three times that its unknown just to be sure
                    
                    #name = input("Welcome new visitor! What is your name? : ")
                    if mode == "t": name = input("Welcome new visitor! What is your name? : ")
                    elif mode == "s": 
                        speak(str("Welcome new visitor! What is your name?"))
                        name = listen()
                    take_picture(name, cap) # enter your name

                    unknown_num = 0 # reset unknown count to 0
                    new_img = cv2.imread(f'{path}/{name}.jpg') # update list
                    new_img_rgb = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
                    new_encoding = face_recognition.face_encodings(new_img_rgb)
                    if new_encoding:
                        encodeListKnown.append(new_encoding[0])
                        classNames.append(name)

            elif matches[matchIndex]: # found match!
                unknown_num = 0
                known_num += 1
                name = classNames[matchIndex].upper() # name of matched person
                #print(name)
                if known_num > 2 : 
                    known_num = 0
                    return name


