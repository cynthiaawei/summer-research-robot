from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import logging
import os
from typing import Dict, List, Optional
import uvicorn

# Import robot_movement
import robot_movement

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

class RobotStatusResponse(BaseModel):
    status: str
    message: str
    obstacle_detected: bool
    current_speeds: Dict[str, int]
    last_distances: List[float]
    last_command: str
    uptime: float
    gpio_available: bool

# Global state
robot_movement_available = True
background_task_created = False

# Check if robot_movement is available
try:
    robot_movement.get_status()  # Test if module is functional
    logger.info("Robot movement module initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize robot movement module: {e}")
    robot_movement_available = False

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
        if not self.active_connections:
            return
            
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
            if len(manager.active_connections) > 0:  # Only broadcast if there are connections
                if robot_movement_available:
                    status = robot_movement.get_status()
                    await manager.broadcast(json.dumps({
                        "type": "status_update",
                        "data": status
                    }))
                else:
                    await manager.broadcast(json.dumps({
                        "type": "status_update",
                        "data": {
                            "status": "error",
                            "message": "Robot movement module not available",
                            "obstacle_detected": False,
                            "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
                            "last_distances": [],
                            "last_command": "",
                            "uptime": 0.0,
                            "gpio_available": False
                        }
                    }))
            await asyncio.sleep(0.5)
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

@app.get("/")
async def read_root():
    """Serve the React frontend"""
    try:
        return FileResponse("frontend/build/index.html")
    except FileNotFoundError:
        return {"message": "Frontend build not found. Please build your React app first."}

@app.get("/api/status")
async def get_status():
    """Get current robot status"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Robot movement module not available")
    try:
        status_data = robot_movement.get_status()
        return RobotStatusResponse(**status_data)
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/command")
async def execute_command(command: Command):
    """Execute a robot command"""
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Robot movement module not available")
    
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
        raise HTTPException(status_code=503, detail="Robot movement module not available")
    
    try:
        parsed = robot_movement.parse_command(text_command.text)
        if parsed:
            direction, duration = parsed
            success = robot_movement.move(direction, duration_ms=duration)
            response = f"Command '{text_command.text}' executed successfully" if success else f"Command '{text_command.text}' failed"
        else:
            response = await robot_movement.chat(text_command.text)
            success = True  # Chat responses are considered successful
        
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
    if not robot_movement_available:
        raise HTTPException(status_code=503, detail="Robot movement module not available")
    
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
        raise HTTPException(status_code=503, detail="Robot movement module not available")
    
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
                    "data": {"message": "Robot movement module not available"}
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
        manager.disconnect(websocket)
        logger.info("Client disconnected from WebSocket")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

# Health check endpoint
@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "robot_available": robot_movement_available,
        "active_connections": len(manager.active_connections)
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
    
    print("🚀 Starting Robot Control Web Server...")
    print("📡 Server will be available at: http://localhost:8000")
    print("🤖 Robot control interface ready!")
    print(f"🔧 Robot movement module: {'Available' if robot_movement_available else 'Not Available (Simulation Mode)'}")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info",
        reload=False
    )