import cv2
import mediapipe as mp
import time
from collections import deque
import RPi.GPIO as GPIO

# THESE NUMBERS ARE TO BE MODIFIED
# Need to disable I2C communication on RPI5
wave = 3
hand = 5

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(wave, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(hand, GPIO.OUT,  initial=GPIO.LOW)

current_time = 0
wave_time = 2
handshake_time = 2
wait_time = 2

class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon=0.5,trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
                static_image_mode=self.mode,
                max_num_hands=self.maxHands,
                min_detection_confidence=self.detectionCon,
                min_tracking_confidence=self.trackCon
            )
        self.mpDraw= mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        #print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw: 
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS) # draw hand landmarks
        return img

    def findPosition(self, img, handNo=0, draw = True):

        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                lmList.append([id,cx,cy])
                #if id == 4:
                if draw:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

        return lmList

def is_right_hand(detector):
    if detector.results.multi_handedness:
        return detector.results.multi_handedness[0].classification[0].label == "Right"
    return False

def high_five_position(lmList, detector):
    if len(lmList)  < 21: return False

     # Each finger: [tip, dip, pip, mcp]
    fingers = [
        [4, 3, 2, 1],    # Thumb
        [8, 7, 6, 5],    # Index
        [12,11,10,9],    # Middle
        [16,15,14,13],   # Ring
        [20,19,18,17]    # Pinky
    ]

    for finger in fingers:
        for i in range(len(finger)-1): # loop through each finger
            if lmList[finger[i]][2] >= lmList[finger[i+1]][2]: # y position of tip should be lower than mcp
                return False # if not lower, fingers are bent

    wrist_y = lmList[0][2] # wrist position
    for tip in [4, 8, 12, 16, 20]:
        if lmList[tip][2] >= wrist_y: # fingers should be higher than wrist
            return False
    

    joints = list(zip(*fingers)) # transpose fingers 
    '''
    joints = [ (thumb, index, middle, ring, pinky)
        [4, 8, 12, 16, 20], tips
        [3, 7, 11, 15, 19], dips
        [2, 6, 10, 14, 18]. pips
        [1, 5, 9, 14, 17] mcps
    ]
    '''
            
    if is_right_hand(detector) :
        for joint in joints: # for every joint
            for i in range(1, 4): # all fingers except pinky
                if lmList[joint[i]][1] >= lmList[joint[i + 1]][1]: 
                    return False # pinky should have a lower x value then thumb
    else :
        for joint in joints: # for every joint
            for i in range(1, 4): # pinky to thumb
                if lmList[joint[i]][1] <= lmList[joint[i + 1]][1]: 
                    return False # pinky should have a greater x value then thumb
    return True


def is_waving(lmList, detector, index_x_history):
    if ((time.time - current_time) < wait_time): return False
    if len(lmList) < 21:
        return False
    
    if not high_five_position(lmList, detector):
        return False
    
    index_x = lmList[8][1]  # index fingertip x position
    index_x_history.append(index_x) # add new position
    
    # Check if index finger tip is swinging side to side
    index_motion = max(index_x_history) - min(index_x_history)

    if(index_motion <= 40): return False
    current_time = time.time()
    GPIO.output(wave, GPIO.HIGH)
    return True

def fingers_close_together(lmList):
    if len(lmList) < 21:
        return False
    
    tips = [4, 8, 12, 16, 20]
    for i in range(1, 4):
        if abs(lmList[tips[i]][1] - lmList[tips[i+1]][1]) > 50 :
            return False
    return True

def is_shaking_hands(lmList):
    if ((time.time - current_time) < wait_time): return False
    if not fingers_close_together(lmList): return False
    if abs(lmList[5][2]-lmList[0][2]) > 50:
        return False
    current_time = time.time()
    GPIO.output(hand, GPIO.HIGH)
    return True

def main():
    cap = cv2.VideoCapture(0)
    detector = handDetector()
    index_x_history = deque(maxlen=10)

    while True:
        success, img = cap.read()
        if not success:
            continue
        img = cv2.flip(img, 1)
        img = detector.findHands(img)
        lmList = detector.findPosition(img)
        if len(lmList) != 0:
            if is_waving(lmList, detector, index_x_history):
                print("waving")
            elif is_shaking_hands(lmList):
                print("shaking hands")
            else:
                GPIO.output(hand, GPIO.LOW) # prev was "False" instad of GPIO.LOW?
                GPIO.output(wave, GPIO.HIGH)
                
def process_hand_frame(frame, history, detector): #  only the while loop inside of main
    """
    Given a BGR frame, a history deque, and a handDetector,
    prints “Waving” or “Shaking Hands” if it sees them.
    """
    img = cv2.flip(frame, 1)
    detector.findHands(img, draw=False)
    lmList = detector.findPosition(img, draw=False)
    if lmList:
        if is_waving(lmList, detector, history):
            print("Waving")
        elif is_shaking_hands(lmList):
            print("Shaking Hands")
            
