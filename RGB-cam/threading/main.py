import threading
import time
from collections import deque
import hand
import face 
import asyncio

def hand_thread(): 
    hand.main()
    time.sleep(0.03)  # ~30 FPS

# face recognition & convo > this file is kept the same
def face_thread():
    asyncio.run(face.main())

# launch both
if __name__ == "__main__":
    t1 = threading.Thread(target=hand_thread)
    t2 = threading.Thread(target=face_thread)

    t1.start()
    t2.start()

    t1.join()
    t2.join()
