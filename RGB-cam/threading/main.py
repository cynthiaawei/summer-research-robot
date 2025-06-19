import threading
import asyncio
import cv2

import hand     # your hand.py
import face     # your face.py
import face_helper as FR  # fixes from step (1)

def hand_thread(cap):
    detector    = hand.handDetector()
    history     = deque(maxlen=10)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        # process and print waving/shaking
        hand.process_hand_frame(frame, history, detector)
        time.sleep(0.03)

def face_thread(cap):
    # tell face_helper to reuse the same camera
    FR.cap = cap
    # run your async convo loop
    asyncio.run(face.main())

if __name__=="__main__":
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Couldn’t open camera!")

    # start the hand‐tracking thread
    t1 = threading.Thread(target=hand_thread, args=(cap,), daemon=True)
    t1.start()

    # then in this main thread (or another) run face
    try:
        face_thread(cap)
    except KeyboardInterrupt:
        print("Shutting down…")
    finally:
        cap.release()
