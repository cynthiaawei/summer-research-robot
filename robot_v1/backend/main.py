# main.py - Your existing main.py with minimal web interface addition
import threading, time, cv2, asyncio, sys
from collections import deque
import hand, movement_RGB, face_helper

def hand_thread(cap):
    """Hand detection thread - EXACTLY as you had it"""
    detector = hand.handDetector() # what was outside of the main while loop in the main function
    history = deque(maxlen=10)
    while True:
        success, frame = cap.read() # read again
        if not success:
            time.sleep(0.03)
            continue
        hand.process_hand_frame(frame, history, detector) # only include the infinite loop inside the main function
        time.sleep(0.03)

def face_thread(cap):
    """Face recognition thread - EXACTLY as you had it"""
    # tell your face_helper to use the same cap
    face_helper.cap = cap
    asyncio.run(movement_RGB.main())

def start_web_server():
    """Start web server in background thread - NEW ADDITION"""
    try:
        import uvicorn
        from app import app  # Import your existing app.py
        
        print("🌐 Starting web interface on http://localhost:8000")
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
    except ImportError:
        print("⚠️ Web interface not available (app.py not found)")
    except Exception as e:
        print(f"⚠️ Web interface error: {e}")

if __name__ == "__main__":
    # Check if user wants web interface - NEW
    enable_web = "--no-web" not in sys.argv
    
    # Your original camera setup - UNCHANGED
    cap = cv2.VideoCapture(0, cv2.CAP_ANY)
    if not cap.isOpened():
        raise RuntimeError("Couldn't open camera")
    
    print("🤖 Starting robot system...")
    
    # Start web server in background if requested - NEW
    if enable_web:
        web_thread = threading.Thread(target=start_web_server, daemon=True)
        web_thread.start()
        print("🌐 Web interface will be available at: http://localhost:8000")
        print("   (Use --no-web flag to disable)")
    
    # Your original hand threading - UNCHANGED
    # start hand‐tracking as a daemon
    t1 = threading.Thread(target=hand_thread, args=(cap,), daemon=True)
    t1.start()
    
    print("✅ Hand detection started")
    print("🔍 Starting face recognition...")
    
    # Your original face/conversation loop - UNCHANGED
    # then run your face/convo loop in the main thread
    face_thread(cap)
