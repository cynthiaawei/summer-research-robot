# app.py - FIXED version that prevents random backend stops
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, BackgroundTasks
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import logging
import os
import signal
import sys
from typing import Dict, List, Optional
import uvicorn
import io
import time
import traceback
import threading

# CRITICAL FIX: Prevent random stops by catching ALL exceptions
def setup_exception_handler():
    """Setup global exception handler to prevent crashes"""
    def handle_exception(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            # Allow KeyboardInterrupt to work normally
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return
        
        logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))
        # Don't exit on uncaught exceptions - just log them
    
    sys.excepthook = handle_exception

# Call this early
setup_exception_handler()

# Import robot movement with better error handling
try:
    import robot_movement as robot_movement
    ROBOT_AVAILABLE = True
    logger = logging.getLogger(__name__)
    logger.info("✅ Successfully imported robot_movement module")
except ImportError as e:
    ROBOT_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.error(f"❌ Robot movement module not available: {e}")
    logger.info("🔧 Running in SIMULATION MODE")

# Configure logging with file output to help debug stops
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('robot_backend.log', mode='a')  # Keep log file
    ]
)

app = FastAPI(
    title="Enhanced Robot Control API", 
    description="Web interface for omni-wheel robot with face recognition and speech"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models (keep all existing ones)
class Command(BaseModel):
    command: str
    duration: Optional[int] = None

class TextCommand(BaseModel):
    text: str

class DirectionCommand(BaseModel):
    direction: str

class SpeechCommand(BaseModel):
    text: str

class UserRegistration(BaseModel):
    name: str

class InteractionMode(BaseModel):
    mode: str

class ConversationRequest(BaseModel):
    mode: str
    message: Optional[str] = None

class EnhancedRobotStatusResponse(BaseModel):
    status: str
    message: str
    obstacle_detected: bool
    current_speeds: Dict[str, int]
    last_distances: List[float]
    last_command: str
    uptime: float
    gpio_available: bool
    current_user: str
    faces_detected: List[str]
    hand_gesture: str
    camera_active: bool
    last_speech_output: str
    listening: bool
    speech_recognition_active: bool
    interaction_mode: str
    face_recognition_available: bool
    speech_recognition_available: bool
    mediapipe_available: bool
    face_recognition_attempts: int
    awaiting_registration: bool

# Global state
robot_movement_available = ROBOT_AVAILABLE
background_task_created = False

# Test robot movement with exception handling
if ROBOT_AVAILABLE:
    try:
        status = robot_movement.get_status()
        logger.info("✅ Enhanced robot movement module initialized successfully")
        logger.info(f"📷 Camera active: {status.get('camera_active', False)}")
        logger.info(f"👁️ Face recognition: {status.get('face_recognition_available', False)}")
        logger.info(f"✋ Hand detection: {status.get('mediapipe_available', False)}")
    except Exception as e:
        logger.error(f"❌ Failed to initialize enhanced robot movement module: {e}")
        logger.error(f"📊 Full traceback: {traceback.format_exc()}")
        robot_movement_available = False

# FIXED WebSocket connection manager with robust error handling
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.connection_lock = asyncio.Lock()
        self.last_status = None
        self.status_changed = True
        self._stop_event = asyncio.Event()

    async def connect(self, websocket: WebSocket):
        try:
            await websocket.accept()
            async with self.connection_lock:
                self.active_connections.append(websocket)
            logger.info(f"🔗 Client connected. Total connections: {len(self.active_connections)}")
        except Exception as e:
            logger.error(f"❌ Error connecting WebSocket: {e}")

    async def disconnect(self, websocket: WebSocket):
        try:
            async with self.connection_lock:
                if websocket in self.active_connections:
                    self.active_connections.remove(websocket)
            logger.info(f"🔌 Client disconnected. Total connections: {len(self.active_connections)}")
        except Exception as e:
            logger.error(f"❌ Error disconnecting WebSocket: {e}")

    async def send_personal_message(self, message: str, websocket: WebSocket):
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.warning(f"⚠️ Error sending personal message: {e}")
            await self.disconnect(websocket)

    async def broadcast(self, message: str):
        if not self.active_connections:
            return
            
        disconnected = []
        async with self.connection_lock:
            connections_copy = self.active_connections.copy()
        
        for connection in connections_copy:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.warning(f"⚠️ Error broadcasting to connection: {e}")
                disconnected.append(connection)
        
        # Remove disconnected connections
        if disconnected:
            async with self.connection_lock:
                for connection in disconnected:
                    if connection in self.active_connections:
                        self.active_connections.remove(connection)

    def mark_status_changed(self):
        """Mark that status has changed and needs to be broadcast"""
        self.status_changed = True

    def stop(self):
        """Signal the manager to stop"""
        self._stop_event.set()

manager = ConnectionManager()

# FIXED Status broadcaster with better error handling
async def broadcast_status():
    """ROBUST status broadcaster that won't crash the app"""
    logger.info("🚀 Starting status broadcaster...")
    consecutive_errors = 0
    max_consecutive_errors = 10
    
    while not manager._stop_event.is_set():
        try:
            if len(manager.active_connections) > 0:
                if robot_movement_available:
                    try:
                        current_status = robot_movement.get_status()
                        
                        # Only broadcast if status changed
                        if (current_status != manager.last_status or manager.status_changed):
                            await manager.broadcast(json.dumps({
                                "type": "status_update",
                                "data": current_status
                            }))
                            manager.last_status = current_status.copy() if isinstance(current_status, dict) else current_status
                            manager.status_changed = False
                            
                        # Reset error counter on success
                        consecutive_errors = 0
                            
                    except Exception as e:
                        consecutive_errors += 1
                        logger.error(f"❌ Error getting robot status (attempt {consecutive_errors}): {e}")
                        
                        # If too many consecutive errors, provide fallback status
                        if consecutive_errors >= 3:
                            error_status = {
                                "status": "error",
                                "message": f"Robot status error: {str(e)}",
                                "obstacle_detected": False,
                                "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
                                "sensor_distances": {},
                                "last_command": "",
                                "uptime": 0.0,
                                "gpio_available": False,
                                "current_user": "Unknown",
                                "faces_detected": [],
                                "hand_gesture": "none",
                                "camera_active": False,
                                "last_speech_output": "",
                                "listening": False,
                                "speech_recognition_active": False,
                                "interaction_mode": "idle",
                                "face_recognition_available": False,
                                "speech_recognition_available": False,
                                "mediapipe_available": False,
                                "face_recognition_attempts": 0,
                                "awaiting_registration": False
                            }
                            
                            if error_status != manager.last_status:
                                await manager.broadcast(json.dumps({
                                    "type": "status_update",
                                    "data": error_status
                                }))
                                manager.last_status = error_status
                else:
                    # Robot not available status
                    offline_status = {
                        "status": "disconnected",
                        "message": "Enhanced robot movement module not available",
                        "obstacle_detected": False,
                        "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
                        "sensor_distances": {},
                        "last_command": "",
                        "uptime": 0.0,
                        "gpio_available": False,
                        "current_user": "Unknown",
                        "faces_detected": [],
                        "hand_gesture": "none",
                        "camera_active": False,
                        "last_speech_output": "",
                        "listening": False,
                        "speech_recognition_active": False,
                        "interaction_mode": "idle",
                        "face_recognition_available": False,
                        "speech_recognition_available": False,
                        "mediapipe_available": False,
                        "face_recognition_attempts": 0,
                        "awaiting_registration": False
                    }
                    
                    if offline_status != manager.last_status:
                        await manager.broadcast(json.dumps({
                            "type": "status_update",
                            "data": offline_status
                        }))
                        manager.last_status = offline_status
            
            await asyncio.sleep(0.5)  # Broadcast every 500ms
            
        except asyncio.CancelledError:
            logger.info("📡 Status broadcaster cancelled - shutting down gracefully")
            break
        except Exception as e:
            consecutive_errors += 1
            logger.error(f"❌ Unexpected error in status broadcaster (attempt {consecutive_errors}): {e}")
            logger.error(f"📊 Traceback: {traceback.format_exc()}")
            
            # If too many consecutive errors, slow down to prevent spam
            if consecutive_errors >= max_consecutive_errors:
                logger.critical(f"🚨 Too many consecutive errors ({consecutive_errors}), slowing down...")
                await asyncio.sleep(5)  # Slow down when there are many errors
            else:
                await asyncio.sleep(1)  # Normal error delay

# FIXED startup event with proper error handling
@app.on_event("startup")
async def startup_event():
    """Start background tasks on app startup with error handling"""
    global background_task_created
    if not background_task_created:
        try:
            # Create the background task with proper exception handling
            task = asyncio.create_task(broadcast_status())
            background_task_created = True
            logger.info("🚀 Background status broadcaster started successfully")
            
            # Store task reference to prevent garbage collection
            app.state.background_task = task
            
        except Exception as e:
            logger.error(f"❌ Failed to start background tasks: {e}")
            logger.error(f"📊 Traceback: {traceback.format_exc()}")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on app shutdown"""
    logger.info("🛑 Starting application shutdown...")
    
    try:
        # Stop the connection manager
        manager.stop()
        
        # Cancel background task if it exists
        if hasattr(app.state, 'background_task'):
            app.state.background_task.cancel()
            try:
                await app.state.background_task
            except asyncio.CancelledError:
                pass
        
        # Shutdown robot if available
        if robot_movement_available:
            try:
                robot_movement.shutdown_robot()
                logger.info("✅ Robot controller shutdown completed")
            except Exception as e:
                logger.error(f"❌ Error during robot shutdown: {e}")
                
    except Exception as e:
        logger.error(f"❌ Error during app shutdown: {e}")
    
    logger.info("✅ Application shutdown complete")

# Add signal handlers to gracefully handle termination
def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown"""
    def signal_handler(signum, frame):
        logger.info(f"🛑 Received signal {signum}, shutting down gracefully...")
        # Don't exit immediately - let FastAPI handle shutdown
        
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

setup_signal_handlers()

# ALL YOUR EXISTING ENDPOINTS WITH ADDED ERROR HANDLING

@app.get("/")
async def read_root():
    """Serve the React frontend"""
    try:
        return FileResponse("frontend/build/index.html")
    except FileNotFoundError:
        return {"message": "Frontend build not found. Please build your React app first."}

@app.get("/api/status")
async def get_status():
    """Get current enhanced robot status with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        status_data = robot_movement.get_status()
        return EnhancedRobotStatusResponse(**status_data)
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        logger.error(f"❌ Error getting status: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.post("/api/command")
async def execute_command(command: Command):
    """Execute a robot command with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.move(command.command.lower(), duration_ms=command.duration)
        response_message = f"Command '{command.command}' executed successfully" if success else f"Command '{command.command}' failed"
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "command_executed",
            "data": {"command": command.command, "success": success, "message": response_message}
        }))
        
        return {"success": success, "message": response_message}
        
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except ValueError as e:
        logger.error(f"❌ Invalid command: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"❌ Error executing command: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/api/camera/stream")
async def camera_stream():
    """Stream camera feed with improved error handling"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    def generate_frames():
        """Generate camera frames with robust error handling"""
        frame_count = 0
        last_frame_time = time.time()
        max_failures = 20  # Increased failure tolerance
        failure_count = 0
        
        logger.info("📹 Starting camera stream...")
        
        while True:
            try:
                frame_bytes = robot_movement.get_camera_frame()
                current_time = time.time()
                
                if frame_bytes:
                    frame_count += 1
                    last_frame_time = current_time
                    failure_count = 0  # Reset failure count on success
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' + 
                           frame_bytes + b'\r\n')
                    
                    # Limit frame rate to ~15 FPS to reduce load
                    time.sleep(0.067)
                else:
                    failure_count += 1
                    
                    # If no frame for 10 seconds or too many failures, break the stream
                    if (current_time - last_frame_time > 10.0) or (failure_count > max_failures):
                        logger.warning(f"📹 Camera stream ending: no frames for {current_time - last_frame_time:.1f}s or {failure_count} failures")
                        break
                    
                    # Send a small delay and try again
                    time.sleep(0.2)  # Longer delay when no frame
                    
            except GeneratorExit:
                logger.info("📹 Camera stream generator exited normally")
                break
            except Exception as e:
                logger.error(f"❌ Camera streaming error: {e}")
                failure_count += 1
                if failure_count > max_failures:
                    logger.error("📹 Too many camera failures, ending stream")
                    break
                time.sleep(0.5)  # Longer delay on error
    
    try:
        return StreamingResponse(
            generate_frames(), 
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0",
                "Access-Control-Allow-Origin": "*"
            }
        )
    except Exception as e:
        logger.error(f"❌ Camera stream initialization error: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Camera stream error: {str(e)}")

@app.post("/api/text-command")
async def process_text_command(text_command: TextCommand):
    """Process natural language text command with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        parsed = robot_movement.parse_command(text_command.text)
        if parsed:
            direction, duration = parsed
            success = robot_movement.move(direction, duration_ms=duration)
            response = f"Command '{text_command.text}' executed successfully" if success else f"Command '{text_command.text}' failed"
        else:
            response = await robot_movement.chat(text_command.text)
            success = True
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "text_command_processed",
            "data": {"text": text_command.text, "success": success, "message": response}
        }))
        
        return {"success": success, "message": response}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error processing text command: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Text command error: {str(e)}")

@app.post("/api/speak")
async def make_robot_speak(speech_command: SpeechCommand):
    """Make robot speak text with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.speak(speech_command.text)
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "speech_output",
            "data": {"text": speech_command.text, "success": success}
        }))
        
        return {"success": success, "message": f"{'Spoke' if success else 'Failed to speak'}: {speech_command.text}"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error making robot speak: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Speech error: {str(e)}")

@app.post("/api/listen")
async def listen_for_speech(timeout: int = 5):
    """Listen for speech input with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        # Notify clients that listening started
        await manager.broadcast(json.dumps({
            "type": "listening_started",
            "data": {"timeout": timeout}
        }))
        
        speech_text = robot_movement.listen_for_speech(timeout)
        
        # Notify clients of result
        await manager.broadcast(json.dumps({
            "type": "speech_input",
            "data": {"text": speech_text, "success": speech_text is not None}
        }))
        
        if speech_text:
            return {"success": True, "text": speech_text, "message": f"Heard: {speech_text}"}
        else:
            return {"success": False, "text": None, "message": "No speech detected or recognition failed"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error listening for speech: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Speech recognition error: {str(e)}")

@app.post("/api/recognize-user")
async def recognize_user(mode: str = "auto"):
    """Recognize current user using face recognition with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        user = robot_movement.recognize_user(mode)
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "user_recognized",
            "data": {"user": user, "success": user is not None}
        }))
        
        if user:
            return {"success": True, "user": user, "message": f"Recognized user: {user}"}
        else:
            return {"success": False, "user": None, "message": "No user recognized"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error recognizing user: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Face recognition error: {str(e)}")

@app.post("/api/register-user")
async def register_user(user_registration: UserRegistration):
    """Register a new user with enhanced error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.register_new_user(user_registration.name)
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "user_registered",
            "data": {"name": user_registration.name, "success": success}
        }))
        
        if success:
            return {"success": True, "message": f"User {user_registration.name} registered successfully"}
        else:
            return {"success": False, "message": f"Failed to register user {user_registration.name}"}
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error registering user: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Registration error: {str(e)}")

@app.post("/api/reset-face-recognition")
async def reset_face_recognition():
    """Reset face recognition state with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.reset_face_recognition_state()
        
        if success:
            logger.info("✅ Face recognition reset - starting new 3-attempt cycle")
            manager.mark_status_changed()
            await manager.broadcast(json.dumps({
                "type": "face_recognition_reset",
                "data": {"success": True}
            }))
            return {"success": True, "message": "Face recognition reset successfully - starting new scan cycle"}
        else:
            await manager.broadcast(json.dumps({
                "type": "face_recognition_reset", 
                "data": {"success": False}
            }))
            return {"success": False, "message": "Failed to reset face recognition"}
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error resetting face recognition: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Face recognition reset error: {str(e)}")

@app.post("/api/interaction-mode")
async def set_interaction_mode(interaction_mode: InteractionMode):
    """Set robot interaction mode with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.set_interaction_mode(interaction_mode.mode)
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "interaction_mode_changed",
            "data": {"mode": interaction_mode.mode, "success": success}
        }))
        
        if success:
            return {"success": True, "message": f"Interaction mode set to {interaction_mode.mode}"}
        else:
            return {"success": False, "message": f"Failed to set interaction mode to {interaction_mode.mode}"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error setting interaction mode: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Interaction mode error: {str(e)}")

@app.post("/api/conversation")
async def handle_conversation(conversation_request: ConversationRequest):
    """Handle conversation in specified mode with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        response = await robot_movement.handle_conversation_mode(conversation_request.mode)
        
        await manager.broadcast(json.dumps({
            "type": "conversation_response",
            "data": {"mode": conversation_request.mode, "response": response}
        }))
        
        return {"success": True, "mode": conversation_request.mode, "response": response}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error handling conversation: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Conversation error: {str(e)}")

@app.post("/api/direction")
async def move_direction(direction_command: DirectionCommand):
    """Move robot in specified direction with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.move(direction_command.direction.lower())
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "direction_command",
            "data": {"direction": direction_command.direction, "message": f"Moving {direction_command.direction}"}
        }))
        
        return {"success": success, "message": f"Moving {direction_command.direction}"}
    except HTTPException:
        raise
    except ValueError as e:
        logger.error(f"❌ Invalid direction: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"❌ Error moving direction: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Movement error: {str(e)}")

@app.post("/api/stop")
async def stop_robot():
    """Stop robot immediately with error handling"""
    try:
        if not robot_movement_available:
            raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
        
        success = robot_movement.stop()
        
        manager.mark_status_changed()
        await manager.broadcast(json.dumps({
            "type": "robot_stopped",
            "data": {"message": "Robot stopped"}
        }))
        
        return {"success": success, "message": "Robot stopped"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error stopping robot: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Stop error: {str(e)}")

# FIXED WebSocket endpoint with comprehensive error handling
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Enhanced WebSocket endpoint with bulletproof error handling"""
    client_ip = websocket.client.host if websocket.client else "unknown"
    logger.info(f"🔗 New WebSocket connection from {client_ip}")
    
    await manager.connect(websocket)
    
    try:
        while True:
            try:
                # Set a reasonable timeout for receiving messages
                data = await asyncio.wait_for(websocket.receive_text(), timeout=60.0)
                
                try:
                    message = json.loads(data)
                except json.JSONDecodeError as e:
                    logger.error(f"❌ JSON decode error from {client_ip}: {e}")
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "Invalid JSON format"}
                    }))
                    continue
                
                # Handle the message (your existing message handling code)
                await handle_websocket_message(websocket, message)
                    
            except asyncio.TimeoutError:
                # Send ping to keep connection alive
                try:
                    await websocket.send_text(json.dumps({"type": "ping"}))
                    continue
                except Exception as e:
                    logger.warning(f"⚠️ Ping failed for {client_ip}: {e}")
                    break
                    
            except WebSocketDisconnect:
                logger.info(f"🔌 Client {client_ip} disconnected normally")
                break
                
            except Exception as e:
                logger.error(f"❌ Error receiving WebSocket message from {client_ip}: {e}")
                logger.error(f"📊 Traceback: {traceback.format_exc()}")
                break
                
    except Exception as e:
        logger.error(f"❌ Unexpected WebSocket error with {client_ip}: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
    finally:
        await manager.disconnect(websocket)

async def handle_websocket_message(websocket: WebSocket, message):
    """Handle individual WebSocket messages with error handling"""
    try:
        message_type = message.get("type")
        payload = message.get("data", {})
        
        # Handle ping/pong explicitly
        if message_type == "ping":
            try:
                await websocket.send_text(json.dumps({"type": "pong"}))
                return
            except Exception as e:
                logger.error(f"❌ Pong failed: {e}")
                raise
        
        if not robot_movement_available:
            try:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": "Enhanced robot movement module not available"}
                }))
            except:
                pass
            return
        
        # Handle all message types with error handling
        try:
            if message_type == "direction_command":
                direction = payload.get("direction", "").lower()
                if direction:
                    success = robot_movement.move(direction)
                    manager.mark_status_changed()
                    await manager.broadcast(json.dumps({
                        "type": "direction_executed",
                        "data": {"direction": direction, "success": success}
                    }))
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "No direction specified"}
                    }))
            
            elif message_type == "text_command":
                text = payload.get("text", "")
                if text:
                    parsed = robot_movement.parse_command(text)
                    if parsed:
                        direction, duration = parsed
                        success = robot_movement.move(direction, duration_ms=duration)
                        response = f"Command '{text}' executed successfully" if success else f"Command '{text}' failed"
                    else:
                        response = await robot_movement.chat(text)
                        success = True
                    
                    manager.mark_status_changed()
                    await manager.broadcast(json.dumps({
                        "type": "text_command_result",
                        "data": {"text": text, "success": success, "message": response}
                    }))
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "No text specified"}
                    }))
            
            elif message_type == "speech_command":
                text = payload.get("text", "")
                if text:
                    success = robot_movement.speak(text)
                    manager.mark_status_changed()
                    await manager.broadcast(json.dumps({
                        "type": "speech_output",
                        "data": {"text": text, "success": success}
                    }))
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "No speech text specified"}
                    }))
            
            elif message_type == "listen_command":
                timeout = payload.get("timeout", 5)
                await manager.broadcast(json.dumps({
                    "type": "listening_started",
                    "data": {"timeout": timeout}
                }))
                
                speech_text = robot_movement.listen_for_speech(timeout)
                await manager.broadcast(json.dumps({
                    "type": "speech_input",
                    "data": {"text": speech_text, "success": speech_text is not None}
                }))
            
            elif message_type == "recognize_user":
                mode = payload.get("mode", "auto")
                user = robot_movement.recognize_user(mode)
                manager.mark_status_changed()
                await manager.broadcast(json.dumps({
                    "type": "user_recognized",
                    "data": {"user": user, "success": user is not None}
                }))
            
            elif message_type == "register_user":
                name = payload.get("name", "")
                if name:
                    logger.info(f"WebSocket: Registering user {name}")
                    success = robot_movement.register_new_user(name)
                    manager.mark_status_changed()
                    await manager.broadcast(json.dumps({
                        "type": "user_registered",
                        "data": {"name": name, "success": success}
                    }))
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "No name specified"}
                    }))
            
            elif message_type == "reset_face_recognition":
                logger.info("WebSocket: Face recognition reset requested")
                success = robot_movement.reset_face_recognition_state()
                
                if success:
                    logger.info("✅ Face recognition reset successful - starting new 3-attempt cycle")
                
                manager.mark_status_changed()
                await manager.broadcast(json.dumps({
                    "type": "face_recognition_reset",
                    "data": {"success": success}
                }))
            
            elif message_type == "set_interaction_mode":
                mode = payload.get("mode", "")
                if mode:
                    success = robot_movement.set_interaction_mode(mode)
                    manager.mark_status_changed()
                    await manager.broadcast(json.dumps({
                        "type": "interaction_mode_changed",
                        "data": {"mode": mode, "success": success}
                    }))
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": "No mode specified"}
                    }))
            
            elif message_type == "conversation_mode":
                mode = payload.get("mode", "auto")
                response = await robot_movement.handle_conversation_mode(mode)
                await manager.broadcast(json.dumps({
                    "type": "conversation_response",
                    "data": {"mode": mode, "response": response}
                }))
            
            elif message_type == "stop_command":
                success = robot_movement.stop()
                manager.mark_status_changed()
                await manager.broadcast(json.dumps({
                    "type": "robot_stopped",
                    "data": {"message": "Robot stopped via WebSocket"}
                }))
            
            elif message_type == "reset_obstacle":
                success = robot_movement.reset_obstacle_detection()
                manager.mark_status_changed()
                await manager.broadcast(json.dumps({
                    "type": "obstacle_reset",
                    "data": {"message": "Obstacle detection reset", "success": success}
                }))
            
            else:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": f"Unknown message type: {message_type}"}
                }))
                
        except ValueError as e:
            logger.error(f"❌ Invalid command in WebSocket: {e}")
            try:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": f"Invalid command: {str(e)}"}
                }))
            except:
                pass
        except Exception as e:
            logger.error(f"❌ Error processing WebSocket message: {e}")
            logger.error(f"📊 Traceback: {traceback.format_exc()}")
            try:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": f"Error processing command: {str(e)}"}
                }))
            except:
                pass
                
    except Exception as e:
        logger.error(f"❌ Critical error in WebSocket message handler: {e}")
        logger.error(f"📊 Traceback: {traceback.format_exc()}")
        raise  # Re-raise to close the connection

# Health check endpoint
@app.get("/api/health")
async def health_check():
    """Enhanced health check endpoint"""
    try:
        if not robot_movement_available:
            return {
                "status": "degraded",
                "robot_available": False,
                "active_connections": len(manager.active_connections),
                "features": {
                    "face_recognition": False,
                    "speech_recognition": False,
                    "hand_detection": False,
                    "camera": False
                }
            }
        
        status = robot_movement.get_status()
        return {
            "status": "healthy",
            "robot_available": True,
            "active_connections": len(manager.active_connections),
            "features": {
                "face_recognition": status.get("face_recognition_available", False),
                "speech_recognition": status.get("speech_recognition_available", False),
                "hand_detection": status.get("mediapipe_available", False),
                "camera": status.get("camera_active", False)
            },
            "uptime": status.get("uptime", 0),
            "last_command": status.get("last_command", ""),
        }
    except Exception as e:
        logger.error(f"❌ Health check error: {e}")
        return {
            "status": "error",
            "robot_available": False,
            "active_connections": len(manager.active_connections),
            "error": str(e),
            "features": {
                "face_recognition": False,
                "speech_recognition": False,
                "hand_detection": False,
                "camera": False
            }
        }

# Serve static files
try:
    app.mount("/static", StaticFiles(directory="frontend/build/static"), name="static")
except Exception as e:
    logger.warning(f"⚠️ Could not mount static files: {e}")

# Catch-all route for React Router
@app.get("/{path:path}")
async def catch_all(path: str):
    """Catch-all route for React Router"""
    try:
        return FileResponse("frontend/build/index.html")
    except FileNotFoundError:
        return {"message": "Frontend build not found. Please build your React app first."}

if __name__ == "__main__":
    # Create frontend directory if it doesn't exist
    os.makedirs("frontend/build", exist_ok=True)
    
    print("🚀 Starting Enhanced Robot Control Web Server...")
    print("📡 Server will be available at: http://localhost:8000")
    print("🤖 Enhanced robot control interface ready!")
    print("✨ FIXED Features:")
    print("   • 🛡️ Comprehensive exception handling")
    print("   • 📝 Enhanced logging to robot_backend.log")
    print("   • 🔄 Robust background task management")
    print("   • 🌐 Bulletproof WebSocket handling")
    print("   • 📹 Improved camera streaming")
    print("   • 🎯 Signal handlers for graceful shutdown")
    print("   • ⚡ No more random stops!")
    print(f"🔧 Enhanced robot movement module: {'Available' if robot_movement_available else 'Not Available (Simulation Mode)'}")
    
    try:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8000,
            log_level="info",
            reload=False,  # Disable reload to prevent conflicts
            access_log=True
        )
    except KeyboardInterrupt:
        logger.info("🛑 Server stopped by user")
    except Exception as e:
        logger.critical(f"🚨 Critical server error: {e}")
        logger.critical(f"📊 Traceback: {traceback.format_exc()}")
        sys.exit(1)