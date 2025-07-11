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
            "data": {"
