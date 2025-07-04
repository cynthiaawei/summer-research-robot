from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import logging
import threading
import time
import os
from typing import Dict, List, Optional
import uvicorn

# Import your robot control functions
from robot_movement import (
    RobotController,
    process_immediate_command,
    process_user_input,
    listen,
    speak
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Control API", description="Web interface for omni-wheel robot control")

# Enable CORS for frontend
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

class RobotStatus(BaseModel):
    status: str
    message: str
    obstacle_detected: bool
    current_speeds: Dict[str, int]

# Global state
robot_controller = None
connected_clients: List[WebSocket] = []
robot_status = {
    "status": "idle",
    "message": "Robot ready",
    "obstacle_detected": False,
    "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0}
}

# Initialize robot controller
try:
    robot_controller = RobotController()
    logger.info("Robot controller initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize robot controller: {e}")
    robot_controller = None

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Client connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")

    async def send_personal_message(self, message: str, websocket: WebSocket):
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"Error sending personal message: {e}")

    async def broadcast(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")
                disconnected.append(connection)
        
        for connection in disconnected:
            self.disconnect(connection)

manager = ConnectionManager()

# Status update broadcaster
async def broadcast_status():
    """Broadcast robot status to all connected clients"""
    while True:
        try:
            if robot_controller:
                robot_status.update({
                    "status": robot_controller.get_status(),
                    "obstacle_detected": robot_controller.obstacle_detected,
                    "current_speeds": robot_controller.get_current_speeds()
                })
            
            await manager.broadcast(json.dumps({
                "type": "status_update",
                "data": robot_status
            }))
            await asyncio.sleep(0.5)
        except Exception as e:
            logger.error(f"Error in status broadcaster: {e}")
            await asyncio.sleep(1)

asyncio.create_task(broadcast_status())

@app.get("/")
async def read_root():
    """Serve the React frontend"""
    return FileResponse("frontend/build/index.html")

@app.get("/api/status")
async def get_status():
    """Get current robot status"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    return RobotStatus(**robot_status)

@app.post("/api/command")
async def execute_command(command: Command):
    """Execute a robot command"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    
    try:
        success = await robot_controller.execute_command(command.command, command.duration)
        response_message = f"Command '{command.command}' executed successfully" if success else f"Command '{command.command}' failed"
        
        await manager.broadcast(json.dumps({
            "type": "command_executed",
            "data": {"command": command.command, "success": success, "message": response_message}
        }))
        
        return {"success": success, "message": response_message}
    except Exception as e:
        logger.error(f"Error executing command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/text-command")
async def process_text_command(text_command: TextCommand):
    """Process natural language text command"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    
    try:
        success, response = await process_user_input(text_command.text, "")
        await speak(response)  # Verbalize AI response
        
        await manager.broadcast(json.dumps({
            "type": "text_command_processed",
            "data": {"text": text_command.text, "success": success, "message": response}
        }))
        
        return {"success": success, "message": response}
    except Exception as e:
        logger.error(f"Error processing text command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/direction")
async def move_direction(direction_command: DirectionCommand):
    """Move robot in specified direction"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    
    try:
        process_immediate_command(direction_command.direction.lower())
        
        await manager.broadcast(json.dumps({
            "type": "direction_command",
            "data": {"direction": direction_command.direction, "message": f"Moving {direction_command.direction}"}
        }))
        
        return {"success": True, "message": f"Moving {direction_command.direction}"}
    except Exception as e:
        logger.error(f"Error moving direction: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/stop")
async def stop_robot():
    """Stop robot immediately"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    
    try:
        process_immediate_command("stop")
        
        await manager.broadcast(json.dumps({
            "type": "robot_stopped",
            "data": {"message": "Robot stopped"}
        }))
        
        return {"success": True, "message": "Robot stopped"}
    except Exception as e:
        logger.error(f"Error stopping robot: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/speech/start")
async def start_speech_recognition():
    """Start speech recognition"""
    if not robot_controller:
        raise HTTPException(status_code=503, detail="Robot controller not available")
    
    try:
        asyncio.create_task(handle_speech_recognition())
        return {"success": True, "message": "Speech recognition started"}
    except Exception as e:
        logger.error(f"Error starting speech recognition: {e}")
        raise HTTPException(status_code=500, detail=str(e))

async def handle_speech_recognition():
    """Handle speech recognition in background"""
    try:
        while True:
            user_input = listen()
            
            if user_input:
                success, response = await process_user_input(user_input, "")
                await speak(response)  # Verbalize AI response
                
                await manager.broadcast(json.dumps({
                    "type": "speech_recognized",
                    "data": {
                        "text": user_input,
                        "success": success,
                        "message": response
                    }
                }))
            
            await asyncio.sleep(0.1)
    except Exception as e:
        logger.error(f"Error in speech recognition: {e}")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time communication"""
    await manager.connect(websocket)
    
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            
            message_type = message.get("type")
            payload = message.get("data", {})
            
            if message_type == "direction_command":
                direction = payload.get("direction", "")
                if direction:
                    process_immediate_command(direction.lower())
                    await manager.broadcast(json.dumps({
                        "type": "direction_executed",
                        "data": {"direction": direction}
                    }))
            
            elif message_type == "text_command":
                text = payload.get("text", "")
                if text:
                    success, response = await process_user_input(text, "")
                    await speak(response)  # Verbalize AI response
                    await manager.broadcast(json.dumps({
                        "type": "text_command_result",
                        "data": {"text": text, "success": success, "message": response}
                    }))
            
            elif message_type == "stop_command":
                process_immediate_command("stop")
                await manager.broadcast(json.dumps({
                    "type": "robot_stopped",
                    "data": {"message": "Robot stopped via WebSocket"}
                }))
            
            elif message_type == "ping":
                await websocket.send_text(json.dumps({"type": "pong"}))
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

# Serve static files (React frontend)
app.mount("/static", StaticFiles(directory="frontend/build/static"), name="static")

# Catch-all route for React Router
@app.get("/{path:path}")
async def catch_all(path: str):
    """Catch-all route for React Router"""
    return FileResponse("frontend/build/index.html")

if __name__ == "__main__":
    os.makedirs("frontend/build", exist_ok=True)
    print("🚀 Starting Robot Control Web Server...")
    print("📡 Server will be available at: http://localhost:8000")
    print("🤖 Robot control interface ready!")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info",
        reload=False
    )