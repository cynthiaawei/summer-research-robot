"""
app.py - Simplified FastAPI application for your robot system
This integrates with your existing face_helper.py, hand.py, and movement_RGB.py modules
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
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
import time

# Import the unified robot movement module
try:
    import robot_movement
    ROBOT_AVAILABLE = True
except ImportError as e:
    ROBOT_AVAILABLE = False
    logging.error(f"Robot movement module not available: {e}")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Robot Control API", 
    description="Web interface for omni-wheel robot with face recognition and hand detection"
)

# Enable CORS
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

class SpeechCommand(BaseModel):
    text: str

class UserRegistration(BaseModel):
    name: str

class RobotStatusResponse(BaseModel):
    status: str
    message: str
    current_user: str
    hand_gesture: str
    camera_active: bool
    face_recognition_available: bool
    speech_recognition_available: bool
    current_speeds: Dict[str, int]
    face_recognition_attempts: int
    awaiting_registration: bool

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.last_status = None

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Client connected. Total: {len(self.active_connections)}")

    async def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting: {e}")
                disconnected.append(connection)
        
        for connection in disconnected:
            await self.disconnect(connection)

manager = ConnectionManager()

# Status broadcaster
async def broadcast_status():
    """Broadcast robot status to all connected clients"""
    while True:
        try:
            if len(manager.active_connections) > 0 and ROBOT_AVAILABLE:
                current_status = robot_movement.get_status()
                
                if current_status != manager.last_status:
                    await manager.broadcast(json.dumps({
                        "type": "status_update",
                        "data": current_status
                    }))
                    manager.last_status = current_status.copy()
            
            await asyncio.sleep(1.0)  # Update every second
        except Exception as e:
            logger.error(f"Status broadcast error: {e}")
            await asyncio.sleep(1)

@app.on_event("startup")
async def startup_event():
    """Start background tasks"""
    asyncio.create_task(broadcast_status())
    logger.info("🚀 Robot control server started")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    if ROBOT_AVAILABLE:
        try:
            robot_movement.shutdown_robot()
            logger.info("Robot shutdown completed")
        except Exception as e:
            logger.error(f"Shutdown error: {e}")

# API Routes
@app.get("/")
async def read_root():
    """Serve main page"""
    return {
        "message": "Robot Control API", 
        "status": "online",
        "robot_available": ROBOT_AVAILABLE
    }

@app.get("/api/status")
async def get_status():
    """Get current robot status"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    try:
        status_data = robot_movement.get_status()
        return RobotStatusResponse(**status_data)
    except Exception as e:
        logger.error(f"Status error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/command")
async def execute_command(command: Command):
    """Execute robot movement command"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    try:
        success = robot_movement.move(command.command.lower(), duration_ms=command.duration)
        message = f"Command '{command.command}' {'executed' if success else 'failed'}"
        
        await manager.broadcast(json.dumps({
            "type": "command_executed",
            "data": {"command": command.command, "success": success, "message": message}
        }))
        
        return {"success": success, "message": message}
    except Exception as e:
        logger.error(f"Command error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/text-command")
async def process_text_command(text_command: TextCommand):
    """Process natural language text command"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    try:
        # Try to parse as movement command first
        parsed = robot_movement.parse_command(text_command.text)
        if parsed:
            direction, duration = parsed
            success = robot_movement.move(direction, duration_ms=duration)
            response = f"Movement command executed: {direction}"
        else:
            # Process as chat
            response = await robot_movement.chat(text_command.text)
            success = True
        
        await manager.broadcast(json.dumps({
            "type": "text_command_processed",
            "data": {"text": text_command.text, "success": success, "response": response}
        }))
        
        return {"success": success, "message": response}
    except Exception as e:
        logger.error(f"Text command error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/speak")
async def make_robot_speak(speech_command: SpeechCommand):
    """Make robot speak"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    try:
        success = robot_movement.speak(speech_command.text)
        
        await manager.broadcast(json.dumps({
            "type": "speech_output",
            "data": {"text": speech_command.text, "success": success}
        }))
        
        return {"success": success, "message": f"Spoke: {speech_command.text}"}
    except Exception as e:
        logger.error(f"Speech error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/listen")
async def listen_for_speech(timeout: int = 5):
    """Listen for speech input"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    try:
        await manager.broadcast(json.dumps({
            "type": "listening_started",
            "data": {"timeout": timeout}
        }))
        
        speech_text = robot_movement.listen_for_speech(timeout)
        
        await manager.broadcast(json.dumps({
            "type": "speech_input",
            "data": {"text": speech_text, "success": speech_text is not None}
        }))
        
        if speech_text:
            return {"success": True, "text": speech_text}
        else:
            return {"success": False, "text": None, "message": "No speech detected"}
    except Exception as e:
        logger.error(f"Listen error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/recognize-user")
async def recognize_user(mode: str = "auto"):
    """Recognize current user using face recognition"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
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
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
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

@app.get("/api/camera/stream")
async def camera_stream():
    """Stream camera feed"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
    def generate_frames():
        while True:
            try:
                frame_bytes = robot_movement.get_camera_frame()
                if frame_bytes:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' + 
                           frame_bytes + b'\r\n')
                
                time.sleep(0.1)  # ~10 FPS
            except Exception as e:
                logger.error(f"Camera stream error: {e}")
                time.sleep(1)
    
    return StreamingResponse(
        generate_frames(), 
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.post("/api/stop")
async def stop_robot():
    """Stop robot immediately"""
    if not ROBOT_AVAILABLE:
        raise HTTPException(status_code=503, detail="Robot not available")
    
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
    """WebSocket endpoint for real-time communication"""
    await manager.connect(websocket)
    
    try:
        while True:
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=30.0)
                message = json.loads(data)
            except asyncio.TimeoutError:
                # Send ping to keep connection alive
                await websocket.send_text(json.dumps({"type": "ping"}))
                continue
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": "Invalid JSON format"}
                }))
                continue
                
            message_type = message.get("type")
            payload = message.get("data", {})
            
            # Handle ping/pong
            if message_type == "ping":
                await websocket.send_text(json.dumps({"type": "pong"}))
                continue
            
            if not ROBOT_AVAILABLE:
                await websocket.send_text(json.dumps({
                    "type": "error",
                    "data": {"message": "Robot not available"}
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
                
                elif message_type == "text_command":
                    text = payload.get("text", "")
                    if text:
                        parsed = robot_movement.parse_command(text)
                        if parsed:
                            direction, duration = parsed
                            success = robot_movement.move(direction, duration_ms=duration)
                            response = f"Movement command executed: {direction}"
                        else:
                            response = await robot_movement.chat(text)
                            success = True
                        
                        await manager.broadcast(json.dumps({
                            "type": "text_command_result",
                            "data": {"text": text, "success": success, "message": response}
                        }))
                
                elif message_type == "speech_command":
                    text = payload.get("text", "")
                    if text:
                        success = robot_movement.speak(text)
                        await manager.broadcast(json.dumps({
                            "type": "speech_output",
                            "data": {"text": text, "success": success}
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
                
                elif message_type == "stop_command":
                    success = robot_movement.stop()
                    await manager.broadcast(json.dumps({
                        "type": "robot_stopped",
                        "data": {"message": "Robot stopped via WebSocket"}
                    }))
                
                else:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "data": {"message": f"Unknown message type: {message_type}"}
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

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    if not ROBOT_AVAILABLE:
        return {
            "status": "degraded",
            "robot_available": False,
            "active_connections": len(manager.active_connections)
        }
    
    try:
        status = robot_movement.get_status()
        return {
            "status": "healthy",
            "robot_available": True,
            "active_connections": len(manager.active_connections),
            "current_user": status.get("current_user", "Unknown")
        }
    except Exception as e:
        logger.error(f"Health check error: {e}")
        return {
            "status": "error",
            "robot_available": False,
            "active_connections": len(manager.active_connections),
            "error": str(e)
        }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
