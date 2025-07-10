# enhanced_app.py - Complete fixed FastAPI backend
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, BackgroundTasks
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import logging
import os
from typing import Dict, List, Optional
import uvicorn
import io
import time

# Import enhanced robot movement
try:
    import robot_movement as robot_movement
    ROBOT_AVAILABLE = True
except ImportError as e:
    ROBOT_AVAILABLE = False
    logging.error(f"Robot movement module not available: {e}")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Enhanced Robot Control API", 
    description="Web interface for omni-wheel robot with face recognition and speech"
)

# Enable CORS with more specific settings
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
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
    mode: str  # speech, text, keyboard, auto

class ConversationRequest(BaseModel):
    mode: str  # speech, text, auto
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
    # Enhanced fields
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

# Test robot movement availability
if ROBOT_AVAILABLE:
    try:
        status = robot_movement.get_status()
        logger.info("Enhanced robot movement module initialized successfully")
        logger.info(f"Camera active: {status.get('camera_active', False)}")
        logger.info(f"Face recognition: {status.get('face_recognition_available', False)}")
        logger.info(f"Hand detection: {status.get('mediapipe_available', False)}")
    except Exception as e:
        logger.error(f"Failed to initialize enhanced robot movement module: {e}")
        robot_movement_available = False

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.connection_lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self.connection_lock:
            self.active_connections.append(websocket)
        logger.info(f"Client connected. Total connections: {len(self.active_connections)}")

    async def disconnect(self, websocket: WebSocket):
        async with self.connection_lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")

    async def send_personal_message(self, message: str, websocket: WebSocket):
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"Error sending personal message: {e}")

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
                logger.error(f"Error broadcasting message: {e}")
                disconnected.append(connection)
        
        # Remove disconnected connections
        if disconnected:
            async with self.connection_lock:
                for connection in disconnected:
                    if connection in self.active_connections:
                        self.active_connections.remove(connection)

manager = ConnectionManager()

# Status update broadcaster
async def broadcast_status():
    """Broadcast enhanced robot status to all connected clients"""
    last_status = None
    
    while True:
        try:
            if len(manager.active_connections) > 0:
                if robot_movement_available:
                    try:
                        status = robot_movement.get_status()
                        
                        # Only broadcast if status changed to reduce network traffic
                        if status != last_status:
                            await manager.broadcast(json.dumps({
                                "type": "status_update",
                                "data": status
                            }))
                            last_status = status.copy()
                            
                    except Exception as e:
                        logger.error(f"Error getting robot status: {e}")
                        # Send error status if robot fails
                        error_status = {
                            "status": "error",
                            "message": f"Robot status error: {str(e)}",
                            "obstacle_detected": False,
                            "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
                            "last_distances": [],
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
                        
                        if error_status != last_status:
                            await manager.broadcast(json.dumps({
                                "type": "status_update",
                                "data": error_status
                            }))
                            last_status = error_status
                else:
                    offline_status = {
                        "status": "disconnected",
                        "message": "Enhanced robot movement module not available",
                        "obstacle_detected": False,
                        "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
                        "last_distances": [],
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
                    
                    if offline_status != last_status:
                        await manager.broadcast(json.dumps({
                            "type": "status_update",
                            "data": offline_status
                        }))
                        last_status = offline_status
            
            await asyncio.sleep(0.5)  # Broadcast every 500ms
        except Exception as e:
            logger.error(f"Error in status broadcaster: {e}")
            await asyncio.sleep(1)

@app.on_event("startup")
async def startup_event():
    """Start background tasks on app startup"""
    global background_task_created
    if not background_task_created:
        asyncio.create_task(broadcast_status())
        background_task_created = True
        logger.info("Background status broadcaster started")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on app shutdown"""
    if robot_movement_available:
        try:
            robot_movement.shutdown_robot()
            logger.info("Robot controller shutdown completed")
        except Exception as e:
            logger.error(f"Error during robot shutdown: {e}")

@app.get("/")
async def read_root():
    """Serve the React frontend"""
    try:
        return FileResponse("frontend/build/index.html")
    except FileNotFoundError:
        return {"message": "Frontend build not found. Please build your React app first."}

@app.get("/api/status")
async def get_status():
    """Get current enhanced robot status"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    try:
        status_data = robot_movement.get_status()
        return EnhancedRobotStatusResponse(**status_data)
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/command")
async def execute_command(command: Command):
    """Execute a robot command"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.move(command.command.lower(), duration_ms=command.duration)
        response_message = f"Command '{command.command}' executed successfully" if success else f"Command '{command.command}' failed"
        
        await manager.broadcast(json.dumps({
            "type": "command_executed",
            "data": {"command": command.command, "success": success, "message": response_message}
        }))
        
        return {"success": success, "message": response_message}
    except ValueError as e:
        logger.error(f"Invalid command: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error executing command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/text-command")
async def process_text_command(text_command: TextCommand):
    """Process natural language text command"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        parsed = robot_movement.parse_command(text_command.text)
        if parsed:
            direction, duration = parsed
            success = robot_movement.move(direction, duration_ms=duration)
            response = f"Command '{text_command.text}' executed successfully" if success else f"Command '{text_command.text}' failed"
        else:
            response = await robot_movement.chat(text_command.text)
            success = True
        
        await manager.broadcast(json.dumps({
            "type": "text_command_processed",
            "data": {"text": text_command.text, "success": success, "message": response}
        }))
        
        return {"success": success, "message": response}
    except Exception as e:
        logger.error(f"Error processing text command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/speak")
async def make_robot_speak(speech_command: SpeechCommand):
    """Make robot speak text"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.speak(speech_command.text)
        
        await manager.broadcast(json.dumps({
            "type": "speech_output",
            "data": {"text": speech_command.text, "success": success}
        }))
        
        return {"success": success, "message": f"{'Spoke' if success else 'Failed to speak'}: {speech_command.text}"}
    except Exception as e:
        logger.error(f"Error making robot speak: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/listen")
async def listen_for_speech(timeout: int = 5):
    """Listen for speech input"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
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
    except Exception as e:
        logger.error(f"Error listening for speech: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/recognize-user")
async def recognize_user(mode: str = "auto"):
    """Recognize current user using face recognition"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        user = robot_movement.recognize_user(mode)
        
        await manager.broadcast(json.dumps({
            "type": "user_recognized",
            "data": {"user": user, "success": user is not None}
        }))
        
        if user:
            return {"success": True, "user": user, "message": f"Recognized user: {user}"}
        else:
            return {"success": False, "user": None, "message": "No user recognized"}
    except Exception as e:
        logger.error(f"Error recognizing user: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/register-user")
async def register_user(user_registration: UserRegistration):
    """Register a new user with face recognition"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.register_new_user(user_registration.name)
        
        await manager.broadcast(json.dumps({
            "type": "user_registered",
            "data": {"name": user_registration.name, "success": success}
        }))
        
        if success:
            return {"success": True, "message": f"User {user_registration.name} registered successfully"}
        else:
            return {"success": False, "message": f"Failed to register user {user_registration.name}"}
    except Exception as e:
        logger.error(f"Error registering user: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/reset-face-recognition")
async def reset_face_recognition():
    """Reset face recognition state to allow new attempts"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.reset_face_recognition_state()
        
        await manager.broadcast(json.dumps({
            "type": "face_recognition_reset",
            "data": {"success": success}
        }))
        
        return {"success": success, "message": "Face recognition state reset"}
    except Exception as e:
        logger.error(f"Error resetting face recognition: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/interaction-mode")
async def set_interaction_mode(interaction_mode: InteractionMode):
    """Set robot interaction mode"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.set_interaction_mode(interaction_mode.mode)
        
        await manager.broadcast(json.dumps({
            "type": "interaction_mode_changed",
            "data": {"mode": interaction_mode.mode, "success": success}
        }))
        
        if success:
            return {"success": True, "message": f"Interaction mode set to {interaction_mode.mode}"}
        else:
            return {"success": False, "message": f"Failed to set interaction mode to {interaction_mode.mode}"}
    except Exception as e:
        logger.error(f"Error setting interaction mode: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/conversation")
async def handle_conversation(conversation_request: ConversationRequest):
    """Handle conversation in specified mode"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        response = await robot_movement.handle_conversation_mode(conversation_request.mode)
        
        await manager.broadcast(json.dumps({
            "type": "conversation_response",
            "data": {"mode": conversation_request.mode, "response": response}
        }))
        
        return {"success": True, "mode": conversation_request.mode, "response": response}
    except Exception as e:
        logger.error(f"Error handling conversation: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/camera/stream")
async def camera_stream():
    """Stream camera feed with improved error handling"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    def generate_frames():
        """Generate camera frames with better error handling"""
        frame_count = 0
        last_frame_time = time.time()
        max_failures = 10
        failure_count = 0
        
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
                    
                    # If no frame for 5 seconds or too many failures, break the stream
                    if (current_time - last_frame_time > 5.0) or (failure_count > max_failures):
                        logger.warning(f"Camera stream ending: no frames for {current_time - last_frame_time:.1f}s or {failure_count} failures")
                        break
                    
                    # Send a small delay and try again
                    time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Camera streaming error: {e}")
                failure_count += 1
                if failure_count > max_failures:
                    break
                time.sleep(0.1)
    
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
        logger.error(f"Camera stream initialization error: {e}")
        raise HTTPException(status_code=500, detail=f"Camera stream error: {str(e)}")

@app.post("/api/direction")
async def move_direction(direction_command: DirectionCommand):
    """Move robot in specified direction"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.move(direction_command.direction.lower())
        
        await manager.broadcast(json.dumps({
            "type": "direction_command",
            "data": {"direction": direction_command.direction, "message": f"Moving {direction_command.direction}"}
        }))
        
        return {"success": success, "message": f"Moving {direction_command.direction}"}
    except ValueError as e:
        logger.error(f"Invalid direction: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error moving direction: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/stop")
async def stop_robot():
    """Stop robot immediately"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
    try:
        success = robot_movement.stop()
        
        await manager.broadcast(json.dumps({
            "type": "robot_stopped",
            "data": {"message": "Robot stopped"}
        }))
        
        return {"success": success, "message": "Robot stopped"}
    except Exception as e:
        logger.error(f"Error stopping robot: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Enhanced WebSocket endpoint for real-time communication"""
    await manager.connect(websocket)
    
    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": "Invalid JSON format"}
                }))
                continue
            
            message_type = message.get("type")
            payload = message.get("data", {})
            
            if not robot_movement_available:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": "Enhanced robot movement module not available"}
                }))
                continue
            
            try:
                if message_type == "direction_command":
                    direction = payload.get("direction", "").lower()
                    if direction:
                        success = robot_movement.move(direction)
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
                    await manager.broadcast(json.dumps({
                        "type": "user_recognized",
                        "data": {"user": user, "success": user is not None}
                    }))
                
                elif message_type == "register_user":
                    name = payload.get("name", "")
                    if name:
                        success = robot_movement.register_new_user(name)
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
                    success = robot_movement.reset_face_recognition_state()
                    await manager.broadcast(json.dumps({
                        "type": "face_recognition_reset",
                        "data": {"success": success}
                    }))
                
                elif message_type == "set_interaction_mode":
                    mode = payload.get("mode", "")
                    if mode:
                        success = robot_movement.set_interaction_mode(mode)
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
                    await manager.broadcast(json.dumps({
                        "type": "robot_stopped",
                        "data": {"message": "Robot stopped via WebSocket"}
                    }))
                
                elif message_type == "reset_obstacle":
                    success = robot_movement.reset_obstacle_detection()
                    await manager.broadcast(json.dumps({
                        "type": "obstacle_reset",
                        "data": {"message": "Obstacle detection reset", "success": success}
                    }))
                
                elif message_type == "ping":
                    await websocket.send_text(json.dumps({"type": "pong"}))
                
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": f"Unknown message type: {message_type}"}
                    }))
                    
            except ValueError as e:
                logger.error(f"Invalid command: {e}")
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": f"Invalid command: {str(e)}"}
                }))
            except Exception as e:
                logger.error(f"Error processing WebSocket message: {e}")
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": f"Error processing command: {str(e)}"}
                }))
    
    except WebSocketDisconnect:
        await manager.disconnect(websocket)
        logger.info("Client disconnected from WebSocket")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await manager.disconnect(websocket)

# Health check endpoint with better error handling
@app.get("/api/health")
async def health_check():
    """Enhanced health check endpoint"""
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
    
    try:
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
            "face_recognition_attempts": status.get("face_recognition_attempts", 0),
            "awaiting_registration": status.get("awaiting_registration", False),
            "current_user": status.get("current_user", "Unknown")
        }
    except Exception as e:
        logger.error(f"Health check error: {e}")
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

# Serve static files (React frontend)
try:
    app.mount("/static", StaticFiles(directory="frontend/build/static"), name="static")
except Exception as e:
    logger.warning(f"Could not mount static files: {e}")

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
    print("✨ New Features:")
    print("   • ✅ Face Recognition & User Management")
    print("   • ✅ Speech Synthesis & Recognition") 
    print("   • ✅ Hand Gesture Detection")
    print("   • ✅ Camera Streaming")
    print("   • ✅ Enhanced AI Conversations")
    print("   • ✅ Fixed threading and camera issues")
    print("   • ✅ Gesture spam prevention")
    print("   • ✅ 3-attempt face recognition")
    print("   • ✅ Improved connection handling")
    print(f"🔧 Enhanced robot movement module: {'Available' if robot_movement_available else 'Not Available (Simulation Mode)'}")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info",
        reload=False
    )