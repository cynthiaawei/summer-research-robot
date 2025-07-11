"""
robot_movement.py - Minimal wrapper for your existing modules
This just provides the interface the FastAPI app expects
"""

import time
import logging
from typing import Optional, Dict, Any, Tuple

# Import your existing modules - use exact names from your files
try:
    import movement_RGB as movement
    import face_helper as FR
    import hand
    MODULES_AVAILABLE = True
    print("✅ All modules imported successfully")
except ImportError as e:
    logging.error(f"Failed to import modules: {e}")
    MODULES_AVAILABLE = False

logger = logging.getLogger(__name__)

# Global state to track what's happening
_status = {
    "status": "connected" if MODULES_AVAILABLE else "disconnected",
    "message": "Robot operational" if MODULES_AVAILABLE else "Modules not available",
    "obstacle_detected": False,
    "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
    "last_distances": [],
    "last_command": "",
    "uptime": time.time(),
    "gpio_available": MODULES_AVAILABLE,
    "current_user": "Unknown",
    "faces_detected": [],
    "hand_gesture": "none",
    "camera_active": MODULES_AVAILABLE,
    "last_speech_output": "",
    "listening": False,
    "speech_recognition_active": False,
    "interaction_mode": "auto",
    "face_recognition_available": MODULES_AVAILABLE,
    "speech_recognition_available": MODULES_AVAILABLE,
    "mediapipe_available": MODULES_AVAILABLE,
    "face_recognition_attempts": 0,
    "awaiting_registration": False
}

def get_status() -> Dict[str, Any]:
    """Get current robot status"""
    return _status.copy()

def move(direction: str, duration_ms: Optional[int] = None) -> bool:
    """Execute movement command"""
    try:
        _status["last_command"] = f"{direction}" + (f" for {duration_ms}ms" if duration_ms else "")
        
        if not MODULES_AVAILABLE:
            return False
            
        # Use your existing movement functions
        if hasattr(movement, 'processCommand') and duration_ms:
            movement.processCommand(direction, duration_ms)
        elif hasattr(movement, 'processImmediateCommand'):
            movement.processImmediateCommand(direction)
        
        # Update speeds for web interface
        if direction == "forward":
            _status["current_speeds"] = {"motor1": 0, "motor2": 25, "motor3": 40}
        elif direction == "backward":
            _status["current_speeds"] = {"motor1": 0, "motor2": 25, "motor3": 40}
        elif direction in ["left", "turnleft"]:
            _status["current_speeds"] = {"motor1": 25, "motor2": 25, "motor3": 40}
        elif direction in ["right", "turnright"]:
            _status["current_speeds"] = {"motor1": 25, "motor2": 25, "motor3": 40}
        elif direction == "stop":
            _status["current_speeds"] = {"motor1": 0, "motor2": 0, "motor3": 0}
        
        return True
    except Exception as e:
        logger.error(f"Movement error: {e}")
        return False

def stop() -> bool:
    """Stop robot"""
    return move("stop")

def speak(text: str) -> bool:
    """Make robot speak"""
    try:
        _status["last_speech_output"] = text
        if MODULES_AVAILABLE and hasattr(movement, 'speak'):
            movement.speak(text)
        return True
    except Exception as e:
        logger.error(f"Speech error: {e}")
        return False

def listen_for_speech(timeout: int = 5) -> Optional[str]:
    """Listen for speech"""
    try:
        _status["listening"] = True
        _status["speech_recognition_active"] = True
        
        result = None
        if MODULES_AVAILABLE and hasattr(movement, 'listen'):
            result = movement.listen()
        
        _status["listening"] = False
        _status["speech_recognition_active"] = False
        return result
    except Exception as e:
        _status["listening"] = False
        _status["speech_recognition_active"] = False
        logger.error(f"Listen error: {e}")
        return None

def listen() -> Optional[str]:
    """Alias for compatibility"""
    return listen_for_speech()

def recognize_user(mode: str = "auto") -> Optional[str]:
    """Recognize user"""
    try:
        if not MODULES_AVAILABLE:
            return None
        
        if hasattr(FR, 'findMatch'):
            fr_mode = "s" if mode == "speech" else "t"
            name = FR.findMatch(fr_mode)
            if name and name != "UNKNOWN":
                _status["current_user"] = name
                return name
        return None
    except Exception as e:
        logger.error(f"Recognition error: {e}")
        return None

def register_new_user(name: str) -> bool:
    """Register new user"""
    try:
        if not MODULES_AVAILABLE:
            return False
        
        if hasattr(FR, 'take_picture') and hasattr(FR, 'cap'):
            FR.take_picture(name, FR.cap)
            _status["current_user"] = name
            return True
        return False
    except Exception as e:
        logger.error(f"Registration error: {e}")
        return False

def reset_face_recognition_state() -> bool:
    """Reset face recognition"""
    _status["face_recognition_attempts"] = 0
    _status["awaiting_registration"] = False
    return True

def set_interaction_mode(mode: str) -> bool:
    """Set interaction mode"""
    _status["interaction_mode"] = mode
    return True

async def handle_conversation_mode(mode: str) -> str:
    """Handle conversation"""
    return f"Conversation mode: {mode}"

def parse_command(text: str) -> Optional[Tuple[str, int]]:
    """Parse natural language command"""
    try:
        if not MODULES_AVAILABLE:
            return None
            
        # Simple parsing
        text = text.lower()
        direction = None
        duration = 1000  # Default duration
        
        if "forward" in text:
            direction = "forward"
        elif "backward" in text:
            direction = "backward"
        elif "left" in text:
            direction = "turnleft"
            duration = 1500
        elif "right" in text:
            direction = "turnright" 
            duration = 1500
        elif "stop" in text:
            direction = "stop"
            duration = 0
        
        if direction:
            return (direction, duration)
        return None
    except Exception as e:
        logger.error(f"Parse error: {e}")
        return None

async def chat(text: str) -> str:
    """Chat response"""
    return f"I heard: {text}"

def get_camera_frame() -> Optional[bytes]:
    """Get camera frame"""
    try:
        if MODULES_AVAILABLE and hasattr(FR, 'cap') and FR.cap:
            import cv2
            ret, frame = FR.cap.read()
            if ret:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                return buffer.tobytes()
        return None
    except Exception as e:
        logger.error(f"Camera error: {e}")
        return None

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection"""
    _status["obstacle_detected"] = False
    return True

def shutdown_robot():
    """Shutdown robot"""
    if MODULES_AVAILABLE:
        try:
            move("stop")
            if hasattr(FR, 'cap') and FR.cap:
                FR.cap.release()
        except:
            pass
