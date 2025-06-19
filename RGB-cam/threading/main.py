# main.py
import threading, time, cv2, asyncio
from collections import deque

import hand, face, face_helper

def hand_thread(cap):
    detector = hand.handDetector()
    history  = deque(maxlen=10)
    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(0.03)
            continue
        hand.process_hand_frame(frame, history, detector)
        time.sleep(0.03)

def face_thread(cap):
    # tell your face_helper to use the same cap
    face_helper.cap = cap
    asyncio.run(face.main())

if __name__ == "__main__":
    cap = cv2.VideoCapture(0, cv2.CAP_ANY)
    if not cap.isOpened():
        raise RuntimeError("Couldn't open camera")
    # start hand‐tracking as a daemon
    t1 = threading.Thread(target=hand_thread,  args=(cap,), daemon=True)
    t1.start()
    # then run your face/convo loop in the main thread
    face_thread(cap)
