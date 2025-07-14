# # enhanced_app.py - Complete fixed FastAPI backend with ALL original functionality
# from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, BackgroundTasks
# from fastapi.staticfiles import StaticFiles
# from fastapi.responses import FileResponse, StreamingResponse
# from fastapi.middleware.cors import CORSMiddleware
# from pydantic import BaseModel
# import asyncio
# import json
# import logging
# import os
# from typing import Dict, List, Optional
# import uvicorn
# import io
# import time

# # Import enhanced robot movement
# try:
#     import robot_movement as robot_movement
#     ROBOT_AVAILABLE = True
# except ImportError as e:
#     ROBOT_AVAILABLE = False
#     logging.error(f"Robot movement module not available: {e}")

# # Configure logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# app = FastAPI(
#     title="Enhanced Robot Control API", 
#     description="Web interface for omni-wheel robot with face recognition and speech"
# )

# # Enable CORS with more specific settings
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # Pydantic models - ALL ORIGINAL MODELS RESTORED
# class Command(BaseModel):
#     command: str
#     duration: Optional[int] = None

# class TextCommand(BaseModel):
#     text: str

# class DirectionCommand(BaseModel):
#     direction: str

# class SpeechCommand(BaseModel):
#     text: str

# class UserRegistration(BaseModel):
#     name: str

# class InteractionMode(BaseModel):
#     mode: str  # speech, text, keyboard, auto

# class ConversationRequest(BaseModel):
#     mode: str  # speech, text, auto
#     message: Optional[str] = None

# class EnhancedRobotStatusResponse(BaseModel):
#     status: str
#     message: str
#     obstacle_detected: bool
#     current_speeds: Dict[str, int]
#     last_distances: List[float]
#     last_command: str
#     uptime: float
#     gpio_available: bool
#     # Enhanced fields
#     current_user: str
#     faces_detected: List[str]
#     hand_gesture: str
#     camera_active: bool
#     last_speech_output: str
#     listening: bool
#     speech_recognition_active: bool
#     interaction_mode: str
#     face_recognition_available: bool
#     speech_recognition_available: bool
#     mediapipe_available: bool
#     face_recognition_attempts: int
#     awaiting_registration: bool

# # Global state
# robot_movement_available = ROBOT_AVAILABLE
# background_task_created = False

# # Test robot movement availability
# if ROBOT_AVAILABLE:
#     try:
#         status = robot_movement.get_status()
#         logger.info("Enhanced robot movement module initialized successfully")
#         logger.info(f"Camera active: {status.get('camera_active', False)}")
#         logger.info(f"Face recognition: {status.get('face_recognition_available', False)}")
#         logger.info(f"Hand detection: {status.get('mediapipe_available', False)}")
#     except Exception as e:
#         logger.error(f"Failed to initialize enhanced robot movement module: {e}")
#         robot_movement_available = False

# # WebSocket connection manager - IMPROVED VERSION
# class ConnectionManager:
#     def __init__(self):
#         self.active_connections: List[WebSocket] = []
#         self.connection_lock = asyncio.Lock()
#         self.last_status = None
#         self.status_changed = True

#     async def connect(self, websocket: WebSocket):
#         await websocket.accept()
#         async with self.connection_lock:
#             self.active_connections.append(websocket)
#         logger.info(f"Client connected. Total connections: {len(self.active_connections)}")

#     async def disconnect(self, websocket: WebSocket):
#         async with self.connection_lock:
#             if websocket in self.active_connections:
#                 self.active_connections.remove(websocket)
#         logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")

#     async def send_personal_message(self, message: str, websocket: WebSocket):
#         try:
#             await websocket.send_text(message)
#         except Exception as e:
#             logger.error(f"Error sending personal message: {e}")
#             await self.disconnect(websocket)

#     async def broadcast(self, message: str):
#         if not self.active_connections:
#             return
            
#         disconnected = []
#         async with self.connection_lock:
#             connections_copy = self.active_connections.copy()
        
#         for connection in connections_copy:
#             try:
#                 await connection.send_text(message)
#             except Exception as e:
#                 logger.error(f"Error broadcasting message: {e}")
#                 disconnected.append(connection)
        
#         # Remove disconnected connections
#         if disconnected:
#             async with self.connection_lock:
#                 for connection in disconnected:
#                     if connection in self.active_connections:
#                         self.active_connections.remove(connection)

#     def mark_status_changed(self):
#         """Mark that status has changed and needs to be broadcast"""
#         self.status_changed = True

# manager = ConnectionManager()

# # Status update broadcaster - OPTIMIZED VERSION
# async def broadcast_status():
#     """Broadcast enhanced robot status to all connected clients - OPTIMIZED"""
    
#     while True:
#         try:
#             if len(manager.active_connections) > 0:
#                 if robot_movement_available:
#                     try:
#                         current_status = robot_movement.get_status()
                        
#                         # Only broadcast if status changed OR it's been 5 seconds since last update
#                         if (current_status != manager.last_status or 
#                             manager.status_changed):
                            
#                             await manager.broadcast(json.dumps({
#                                 "type": "status_update",
#                                 "data": current_status
#                             }))
#                             manager.last_status = current_status.copy()
#                             manager.status_changed = False
                            
#                     except Exception as e:
#                         logger.error(f"Error getting robot status: {e}")
#                         # Send error status if robot fails
#                         error_status = {
#                             "status": "error",
#                             "message": f"Robot status error: {str(e)}",
#                             "obstacle_detected": False,
#                             "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
#                             "last_distances": [],
#                             "last_command": "",
#                             "uptime": 0.0,
#                             "gpio_available": False,
#                             "current_user": "Unknown",
#                             "faces_detected": [],
#                             "hand_gesture": "none",
#                             "camera_active": False,
#                             "last_speech_output": "",
#                             "listening": False,
#                             "speech_recognition_active": False,
#                             "interaction_mode": "idle",
#                             "face_recognition_available": False,
#                             "speech_recognition_available": False,
#                             "mediapipe_available": False,
#                             "face_recognition_attempts": 0,
#                             "awaiting_registration": False
#                         }
                        
#                         if error_status != manager.last_status:
#                             await manager.broadcast(json.dumps({
#                                 "type": "status_update",
#                                 "data": error_status
#                             }))
#                             manager.last_status = error_status
#                 else:
#                     offline_status = {
#                         "status": "disconnected",
#                         "message": "Enhanced robot movement module not available",
#                         "obstacle_detected": False,
#                         "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
#                         "last_distances": [],
#                         "last_command": "",
#                         "uptime": 0.0,
#                         "gpio_available": False,
#                         "current_user": "Unknown",
#                         "faces_detected": [],
#                         "hand_gesture": "none",
#                         "camera_active": False,
#                         "last_speech_output": "",
#                         "listening": False,
#                         "speech_recognition_active": False,
#                         "interaction_mode": "idle",
#                         "face_recognition_available": False,
#                         "speech_recognition_available": False,
#                         "mediapipe_available": False,
#                         "face_recognition_attempts": 0,
#                         "awaiting_registration": False
#                     }
                    
#                     if offline_status != manager.last_status:
#                         await manager.broadcast(json.dumps({
#                             "type": "status_update",
#                             "data": offline_status
#                         }))
#                         manager.last_status = offline_status
            
#             await asyncio.sleep(0.5)  # Broadcast every 500ms
#         except Exception as e:
#             logger.error(f"Error in status broadcaster: {e}")
#             await asyncio.sleep(1)

# @app.on_event("startup")
# async def startup_event():
#     """Start background tasks on app startup"""
#     global background_task_created
#     if not background_task_created:
#         asyncio.create_task(broadcast_status())
#         background_task_created = True
#         logger.info("Background status broadcaster started")

# @app.on_event("shutdown")
# async def shutdown_event():
#     """Cleanup on app shutdown"""
#     if robot_movement_available:
#         try:
#             robot_movement.shutdown_robot()
#             logger.info("Robot controller shutdown completed")
#         except Exception as e:
#             logger.error(f"Error during robot shutdown: {e}")

# @app.get("/")
# async def read_root():
#     """Serve the React frontend"""
#     try:
#         return FileResponse("frontend/build/index.html")
#     except FileNotFoundError:
#         return {"message": "Frontend build not found. Please build your React app first."}

# @app.get("/api/status")
# async def get_status():
#     """Get current enhanced robot status"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
#     try:
#         status_data = robot_movement.get_status()
#         return EnhancedRobotStatusResponse(**status_data)
#     except Exception as e:
#         logger.error(f"Error getting status: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/command")
# async def execute_command(command: Command):
#     """Execute a robot command"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.move(command.command.lower(), duration_ms=command.duration)
#         response_message = f"Command '{command.command}' executed successfully" if success else f"Command '{command.command}' failed"
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "command_executed",
#             "data": {"command": command.command, "success": success, "message": response_message}
#         }))
        
#         return {"success": success, "message": response_message}
#     except ValueError as e:
#         logger.error(f"Invalid command: {e}")
#         raise HTTPException(status_code=400, detail=str(e))
#     except Exception as e:
#         logger.error(f"Error executing command: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/text-command")
# async def process_text_command(text_command: TextCommand):
#     """Process natural language text command"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         parsed = robot_movement.parse_command(text_command.text)
#         if parsed:
#             direction, duration = parsed
#             success = robot_movement.move(direction, duration_ms=duration)
#             response = f"Command '{text_command.text}' executed successfully" if success else f"Command '{text_command.text}' failed"
#         else:
#             response = await robot_movement.chat(text_command.text)
#             success = True
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "text_command_processed",
#             "data": {"text": text_command.text, "success": success, "message": response}
#         }))
        
#         return {"success": success, "message": response}
#     except Exception as e:
#         logger.error(f"Error processing text command: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/speak")
# async def make_robot_speak(speech_command: SpeechCommand):
#     """Make robot speak text"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.speak(speech_command.text)
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "speech_output",
#             "data": {"text": speech_command.text, "success": success}
#         }))
        
#         return {"success": success, "message": f"{'Spoke' if success else 'Failed to speak'}: {speech_command.text}"}
#     except Exception as e:
#         logger.error(f"Error making robot speak: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/listen")
# async def listen_for_speech(timeout: int = 5):
#     """Listen for speech input"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         # Notify clients that listening started
#         await manager.broadcast(json.dumps({
#             "type": "listening_started",
#             "data": {"timeout": timeout}
#         }))
        
#         speech_text = robot_movement.listen_for_speech(timeout)
        
#         # Notify clients of result
#         await manager.broadcast(json.dumps({
#             "type": "speech_input",
#             "data": {"text": speech_text, "success": speech_text is not None}
#         }))
        
#         if speech_text:
#             return {"success": True, "text": speech_text, "message": f"Heard: {speech_text}"}
#         else:
#             return {"success": False, "text": None, "message": "No speech detected or recognition failed"}
#     except Exception as e:
#         logger.error(f"Error listening for speech: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/recognize-user")
# async def recognize_user(mode: str = "auto"):
#     """Recognize current user using face recognition"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         user = robot_movement.recognize_user(mode)
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "user_recognized",
#             "data": {"user": user, "success": user is not None}
#         }))
        
#         if user:
#             return {"success": True, "user": user, "message": f"Recognized user: {user}"}
#         else:
#             return {"success": False, "user": None, "message": "No user recognized"}
#     except Exception as e:
#         logger.error(f"Error recognizing user: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/register-user")
# async def register_user(user_registration: UserRegistration):
#     """Register a new user with face recognition"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.register_new_user(user_registration.name)
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "user_registered",
#             "data": {"name": user_registration.name, "success": success}
#         }))
        
#         if success:
#             return {"success": True, "message": f"User {user_registration.name} registered successfully"}
#         else:
#             return {"success": False, "message": f"Failed to register user {user_registration.name}"}
#     except Exception as e:
#         logger.error(f"Error registering user: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/reset-face-recognition")
# async def reset_face_recognition():
#     """Reset face recognition state to allow new attempts - FIXED VERSION"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         # FIXED: Properly reset the face recognition system
#         success = robot_movement.reset_face_recognition_state()
        
#         if success:
#             logger.info("✅ Face recognition reset - starting new 3-attempt cycle")
#             manager.mark_status_changed()  # Mark that status changed
#             await manager.broadcast(json.dumps({
#                 "type": "face_recognition_reset",
#                 "data": {"success": True}
#             }))
#             return {"success": True, "message": "Face recognition reset successfully - starting new scan cycle"}
#         else:
#             await manager.broadcast(json.dumps({
#                 "type": "face_recognition_reset", 
#                 "data": {"success": False}
#             }))
#             return {"success": False, "message": "Failed to reset face recognition"}
            
#     except Exception as e:
#         logger.error(f"Error resetting face recognition: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/interaction-mode")
# async def set_interaction_mode(interaction_mode: InteractionMode):
#     """Set robot interaction mode"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.set_interaction_mode(interaction_mode.mode)
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "interaction_mode_changed",
#             "data": {"mode": interaction_mode.mode, "success": success}
#         }))
        
#         if success:
#             return {"success": True, "message": f"Interaction mode set to {interaction_mode.mode}"}
#         else:
#             return {"success": False, "message": f"Failed to set interaction mode to {interaction_mode.mode}"}
#     except Exception as e:
#         logger.error(f"Error setting interaction mode: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/conversation")
# async def handle_conversation(conversation_request: ConversationRequest):
#     """Handle conversation in specified mode"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         response = await robot_movement.handle_conversation_mode(conversation_request.mode)
        
#         await manager.broadcast(json.dumps({
#             "type": "conversation_response",
#             "data": {"mode": conversation_request.mode, "response": response}
#         }))
        
#         return {"success": True, "mode": conversation_request.mode, "response": response}
#     except Exception as e:
#         logger.error(f"Error handling conversation: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.get("/api/camera/stream")
# async def camera_stream():
#     """Stream camera feed with improved error handling"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     def generate_frames():
#         """Generate camera frames with better error handling"""
#         frame_count = 0
#         last_frame_time = time.time()
#         max_failures = 10
#         failure_count = 0
        
#         while True:
#             try:
#                 frame_bytes = robot_movement.get_camera_frame()
#                 current_time = time.time()
                
#                 if frame_bytes:
#                     frame_count += 1
#                     last_frame_time = current_time
#                     failure_count = 0  # Reset failure count on success
                    
#                     yield (b'--frame\r\n'
#                            b'Content-Type: image/jpeg\r\n'
#                            b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' + 
#                            frame_bytes + b'\r\n')
                    
#                     # Limit frame rate to ~15 FPS to reduce load
#                     time.sleep(0.067)
#                 else:
#                     failure_count += 1
                    
#                     # If no frame for 5 seconds or too many failures, break the stream
#                     if (current_time - last_frame_time > 5.0) or (failure_count > max_failures):
#                         logger.warning(f"Camera stream ending: no frames for {current_time - last_frame_time:.1f}s or {failure_count} failures")
#                         break
                    
#                     # Send a small delay and try again
#                     time.sleep(0.1)
                    
#             except Exception as e:
#                 logger.error(f"Camera streaming error: {e}")
#                 failure_count += 1
#                 if failure_count > max_failures:
#                     break
#                 time.sleep(0.1)
    
#     try:
#         return StreamingResponse(
#             generate_frames(), 
#             media_type="multipart/x-mixed-replace; boundary=frame",
#             headers={
#                 "Cache-Control": "no-cache, no-store, must-revalidate",
#                 "Pragma": "no-cache",
#                 "Expires": "0",
#                 "Access-Control-Allow-Origin": "*"
#             }
#         )
#     except Exception as e:
#         logger.error(f"Camera stream initialization error: {e}")
#         raise HTTPException(status_code=500, detail=f"Camera stream error: {str(e)}")

# @app.post("/api/direction")
# async def move_direction(direction_command: DirectionCommand):
#     """Move robot in specified direction"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.move(direction_command.direction.lower())
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "direction_command",
#             "data": {"direction": direction_command.direction, "message": f"Moving {direction_command.direction}"}
#         }))
        
#         return {"success": success, "message": f"Moving {direction_command.direction}"}
#     except ValueError as e:
#         logger.error(f"Invalid direction: {e}")
#         raise HTTPException(status_code=400, detail=str(e))
#     except Exception as e:
#         logger.error(f"Error moving direction: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# @app.post("/api/stop")
# async def stop_robot():
#     """Stop robot immediately"""
#     if not robot_movement_available:
#         raise HTTPException(status_code=503, detail="Enhanced robot movement module not available")
    
#     try:
#         success = robot_movement.stop()
        
#         manager.mark_status_changed()  # Mark that status changed
#         await manager.broadcast(json.dumps({
#             "type": "robot_stopped",
#             "data": {"message": "Robot stopped"}
#         }))
        
#         return {"success": success, "message": "Robot stopped"}
#     except Exception as e:
#         logger.error(f"Error stopping robot: {e}")
#         raise HTTPException(status_code=500, detail=str(e))

# # Fixed WebSocket endpoint in app.py
# @app.websocket("/ws")
# async def websocket_endpoint(websocket: WebSocket):
#     """Enhanced WebSocket endpoint with better error handling"""
#     await manager.connect(websocket)
    
#     try:
#         while True:
#             try:
#                 # Set a reasonable timeout for receiving messages
#                 data = await asyncio.wait_for(websocket.receive_text(), timeout=60.0)
                
#                 try:
#                     message = json.loads(data)
#                 except json.JSONDecodeError as e:
#                     logger.error(f"JSON decode error: {e}")
#                     await websocket.send_text(json.dumps({
#                         "type": "error",
#                         "data": {"message": "Invalid JSON format"}
#                     }))
#                     continue
                    
#             except asyncio.TimeoutError:
#                 # Send ping to keep connection alive
#                 try:
#                     await websocket.send_text(json.dumps({"type": "ping"}))
#                     continue
#                 except Exception as e:
#                     logger.error(f"Ping failed: {e}")
#                     break
#             except Exception as e:
#                 logger.error(f"Error receiving message: {e}")
#                 break
                
#             message_type = message.get("type")
#             payload = message.get("data", {})
            
#             # Handle ping/pong explicitly
#             if message_type == "ping":
#                 try:
#                     await websocket.send_text(json.dumps({"type": "pong"}))
#                     continue
#                 except Exception as e:
#                     logger.error(f"Pong failed: {e}")
#                     break
            
#             if not robot_movement_available:
#                 try:
#                     await websocket.send_text(json.dumps({
#                         "type": "error",
#                         "data": {"message": "Enhanced robot movement module not available"}
#                     }))
#                 except:
#                     break
#                 continue
            
#             try:
#                 if message_type == "direction_command":
#                     direction = payload.get("direction", "").lower()
#                     if direction:
#                         success = robot_movement.move(direction)
#                         manager.mark_status_changed()
#                         await manager.broadcast(json.dumps({
#                             "type": "direction_executed",
#                             "data": {"direction": direction, "success": success}
#                         }))
#                     else:
#                         await websocket.send_text(json.dumps({
#                             "type": "error",
#                             "data": {"message": "No direction specified"}
#                         }))
                
#                 elif message_type == "text_command":
#                     text = payload.get("text", "")
#                     if text:
#                         parsed = robot_movement.parse_command(text)
#                         if parsed:
#                             direction, duration = parsed
#                             success = robot_movement.move(direction, duration_ms=duration)
#                             response = f"Command '{text}' executed successfully" if success else f"Command '{text}' failed"
#                         else:
#                             response = await robot_movement.chat(text)
#                             success = True
                        
#                         manager.mark_status_changed()
#                         await manager.broadcast(json.dumps({
#                             "type": "text_command_result",
#                             "data": {"text": text, "success": success, "message": response}
#                         }))
#                     else:
#                         await websocket.send_text(json.dumps({
#                             "type": "error",
#                             "data": {"message": "No text specified"}
#                         }))
                
#                 elif message_type == "speech_command":
#                     text = payload.get("text", "")
#                     if text:
#                         success = robot_movement.speak(text)
#                         manager.mark_status_changed()
#                         await manager.broadcast(json.dumps({
#                             "type": "speech_output",
#                             "data": {"text": text, "success": success}
#                         }))
#                     else:
#                         await websocket.send_text(json.dumps({
#                             "type": "error",
#                             "data": {"message": "No speech text specified"}
#                         }))
                
#                 elif message_type == "listen_command":
#                     timeout = payload.get("timeout", 5)
#                     await manager.broadcast(json.dumps({
#                         "type": "listening_started",
#                         "data": {"timeout": timeout}
#                     }))
                    
#                     speech_text = robot_movement.listen_for_speech(timeout)
#                     await manager.broadcast(json.dumps({
#                         "type": "speech_input",
#                         "data": {"text": speech_text, "success": speech_text is not None}
#                     }))
                
#                 elif message_type == "recognize_user":
#                     mode = payload.get("mode", "auto")
#                     user = robot_movement.recognize_user(mode)
#                     manager.mark_status_changed()
#                     await manager.broadcast(json.dumps({
#                         "type": "user_recognized",
#                         "data": {"user": user, "success": user is not None}
#                     }))
                
#                 elif message_type == "register_user":
#                     name = payload.get("name", "")
#                     if name:
#                         logger.info(f"WebSocket: Registering user {name}")
#                         success = robot_movement.register_new_user(name)
#                         manager.mark_status_changed()
#                         await manager.broadcast(json.dumps({
#                             "type": "user_registered",
#                             "data": {"name": name, "success": success}
#                         }))
#                     else:
#                         await websocket.send_text(json.dumps({
#                             "type": "error",
#                             "data": {"message": "No name specified"}
#                         }))
                
#                 elif message_type == "reset_face_recognition":
#                     logger.info("WebSocket: Face recognition reset requested")
#                     success = robot_movement.reset_face_recognition_state()
                    
#                     if success:
#                         logger.info("✅ Face recognition reset successful - starting new 3-attempt cycle")
                    
#                     manager.mark_status_changed()
#                     await manager.broadcast(json.dumps({
#                         "type": "face_recognition_reset",
#                         "data": {"success": success}
#                     }))
                
#                 elif message_type == "set_interaction_mode":
#                     mode = payload.get("mode", "")
#                     if mode:
#                         success = robot_movement.set_interaction_mode(mode)
#                         manager.mark_status_changed()
#                         await manager.broadcast(json.dumps({
#                             "type": "interaction_mode_changed",
#                             "data": {"mode": mode, "success": success}
#                         }))
#                     else:
#                         await websocket.send_text(json.dumps({
#                             "type": "error",
#                             "data": {"message": "No mode specified"}
#                         }))
                
#                 elif message_type == "conversation_mode":
#                     mode = payload.get("mode", "auto")
#                     response = await robot_movement.handle_conversation_mode(mode)
#                     await manager.broadcast(json.dumps({
#                         "type": "conversation_response",
#                         "data": {"mode": mode, "response": response}
#                     }))
                
#                 elif message_type == "stop_command":
#                     success = robot_movement.stop()
#                     manager.mark_status_changed()
#                     await manager.broadcast(json.dumps({
#                         "type": "robot_stopped",
#                         "data": {"message": "Robot stopped via WebSocket"}
#                     }))
                
#                 elif message_type == "reset_obstacle":
#                     success = robot_movement.reset_obstacle_detection()
#                     manager.mark_status_changed()
#                     await manager.broadcast(json.dumps({
#                         "type": "obstacle_reset",
#                         "data": {"message": "Obstacle detection reset", "success": success}
#                     }))
                
#                 else:
#                     await websocket.send_text(json.dumps({
#                         "type": "error",
#                         "data": {"message": f"Unknown message type: {message_type}"}
#                     }))
                    
#             except ValueError as e:
#                 logger.error(f"Invalid command: {e}")
#                 try:
#                     await websocket.send_text(json.dumps({
#                         "type": "error",
#                         "data": {"message": f"Invalid command: {str(e)}"}
#                     }))
#                 except:
#                     break
#             except Exception as e:
#                 logger.error(f"Error processing WebSocket message: {e}")
#                 try:
#                     await websocket.send_text(json.dumps({
#                         "type": "error",
#                         "data": {"message": f"Error processing command: {str(e)}"}
#                     }))
#                 except:
#                     break
    
#     except WebSocketDisconnect:
#         logger.info("Client disconnected from WebSocket normally")
#     except Exception as e:
#         logger.error(f"WebSocket error: {e}")
#     finally:
#         await manager.disconnect(websocket)


# # Improved Connection Manager
# class ConnectionManager:
#     def __init__(self):
#         self.active_connections: List[WebSocket] = []
#         self.connection_lock = asyncio.Lock()
#         self.last_status = None
#         self.status_changed = True

#     async def connect(self, websocket: WebSocket):
#         try:
#             await websocket.accept()
#             async with self.connection_lock:
#                 self.active_connections.append(websocket)
#             logger.info(f"Client connected. Total connections: {len(self.active_connections)}")
#         except Exception as e:
#             logger.error(f"Error accepting WebSocket connection: {e}")

#     async def disconnect(self, websocket: WebSocket):
#         async with self.connection_lock:
#             if websocket in self.active_connections:
#                 self.active_connections.remove(websocket)
#         logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")

#     async def send_personal_message(self, message: str, websocket: WebSocket):
#         try:
#             if websocket.client_state.value == 1:  # Connected
#                 await websocket.send_text(message)
#         except Exception as e:
#             logger.error(f"Error sending personal message: {e}")
#             await self.disconnect(websocket)

#     async def broadcast(self, message: str):
#         if not self.active_connections:
#             return
            
#         disconnected = []
#         async with self.connection_lock:
#             connections_copy = self.active_connections.copy()
        
#         for connection in connections_copy:
#             try:
#                 if connection.client_state.value == 1:  # Connected
#                     await connection.send_text(message)
#             except Exception as e:
#                 logger.warning(f"Error broadcasting to connection: {e}")
#                 disconnected.append(connection)
        
#         # Remove disconnected connections
#         if disconnected:
#             async with self.connection_lock:
#                 for connection in disconnected:
#                     if connection in self.active_connections:
#                         self.active_connections.remove(connection)

#     def mark_status_changed(self):
#         """Mark that status has changed and needs to be broadcast"""
#         self.status_changed = True


# # Improved status broadcaster
# async def broadcast_status():
#     """Broadcast enhanced robot status to all connected clients - IMPROVED"""
    
#     while True:
#         try:
#             if len(manager.active_connections) > 0:
#                 if robot_movement_available:
#                     try:
#                         current_status = robot_movement.get_status()
                        
#                         # Only broadcast if status changed
#                         if (current_status != manager.last_status or 
#                             manager.status_changed):
                            
#                             await manager.broadcast(json.dumps({
#                                 "type": "status_update",
#                                 "data": current_status
#                             }))
#                             manager.last_status = current_status.copy() if isinstance(current_status, dict) else current_status
#                             manager.status_changed = False
                            
#                     except Exception as e:
#                         logger.error(f"Error getting robot status: {e}")
#                         # Send error status if robot fails
#                         error_status = {
#                             "status": "error",
#                             "message": f"Robot status error: {str(e)}",
#                             "obstacle_detected": False,
#                             "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
#                             "sensor_distances": {},
#                             "last_command": "",
#                             "uptime": 0.0,
#                             "gpio_available": False,
#                             "current_user": "Unknown",
#                             "faces_detected": [],
#                             "hand_gesture": "none",
#                             "camera_active": False,
#                             "last_speech_output": "",
#                             "listening": False,
#                             "speech_recognition_active": False,
#                             "interaction_mode": "idle",
#                             "face_recognition_available": False,
#                             "speech_recognition_available": False,
#                             "mediapipe_available": False,
#                             "face_recognition_attempts": 0,
#                             "awaiting_registration": False
#                         }
                        
#                         if error_status != manager.last_status:
#                             await manager.broadcast(json.dumps({
#                                 "type": "status_update",
#                                 "data": error_status
#                             }))
#                             manager.last_status = error_status
#                 else:
#                     offline_status = {
#                         "status": "disconnected",
#                         "message": "Enhanced robot movement module not available",
#                         "obstacle_detected": False,
#                         "current_speeds": {"motor1": 0, "motor2": 0, "motor3": 0},
#                         "sensor_distances": {},
#                         "last_command": "",
#                         "uptime": 0.0,
#                         "gpio_available": False,
#                         "current_user": "Unknown",
#                         "faces_detected": [],
#                         "hand_gesture": "none",
#                         "camera_active": False,
#                         "last_speech_output": "",
#                         "listening": False,
#                         "speech_recognition_active": False,
#                         "interaction_mode": "idle",
#                         "face_recognition_available": False,
#                         "speech_recognition_available": False,
#                         "mediapipe_available": False,
#                         "face_recognition_attempts": 0,
#                         "awaiting_registration": False
#                     }
                    
#                     if offline_status != manager.last_status:
#                         await manager.broadcast(json.dumps({
#                             "type": "status_update",
#                             "data": offline_status
#                         }))
#                         manager.last_status = offline_status
            
#             await asyncio.sleep(0.5)  # Broadcast every 500ms
#         except Exception as e:
#             logger.error(f"Error in status broadcaster: {e}")
#             await asyncio.sleep(1)
            
             
# # Health check endpoint with better error handling
# @app.get("/api/health")
# async def health_check():
#     """Enhanced health check endpoint"""
#     if not robot_movement_available:
#         return {
#             "status": "degraded",
#             "robot_available": False,
#             "active_connections": len(manager.active_connections),
#             "features": {
#                 "face_recognition": False,
#                 "speech_recognition": False,
#                 "hand_detection": False,
#                 "camera": False
#             }
#         }
    
#     try:
#         status = robot_movement.get_status()
#         return {
#             "status": "healthy",
#             "robot_available": True,
#             "active_connections": len(manager.active_connections),
#             "features": {
#                 "face_recognition": status.get("face_recognition_available", False),
#                 "speech_recognition": status.get("speech_recognition_available", False),
#                 "hand_detection": status.get("mediapipe_available", False),
#                 "camera": status.get("camera_active", False)
#             },
#             "face_recognition_attempts": status.get("face_recognition_attempts", 0),
#             "awaiting_registration": status.get("awaiting_registration", False),
#             "current_user": status.get("current_user", "Unknown")
#         }
#     except Exception as e:
#         logger.error(f"Health check error: {e}")
#         return {
#             "status": "error",
#             "robot_available": False,
#             "active_connections": len(manager.active_connections),
#             "error": str(e),
#             "features": {
#                 "face_recognition": False,
#                 "speech_recognition": False,
#                 "hand_detection": False,
#                 "camera": False
#             }
#         }

# # Serve static files (React frontend)
# try:
#     app.mount("/static", StaticFiles(directory="frontend/build/static"), name="static")
# except Exception as e:
#     logger.warning(f"Could not mount static files: {e}")

# # Catch-all route for React Router
# @app.get("/{path:path}")
# async def catch_all(path: str):
#     """Catch-all route for React Router"""
#     try:
#         return FileResponse("frontend/build/index.html")
#     except FileNotFoundError:
#         return {"message": "Frontend build not found. Please build your React app first."}

# if __name__ == "__main__":
#     # Create frontend directory if it doesn't exist
#     os.makedirs("frontend/build", exist_ok=True)
    
#     print("🚀 Starting Enhanced Robot Control Web Server...")
#     print("📡 Server will be available at: http://localhost:8000")
#     print("🤖 Enhanced robot control interface ready!")
#     print("✨ New Features:")
#     print("   • ✅ Face Recognition & User Management")
#     print("   • ✅ Speech Synthesis & Recognition") 
#     print("   • ✅ Hand Gesture Detection")
#     print("   • ✅ Camera Streaming")
#     print("   • ✅ Enhanced AI Conversations")
#     print("   • ✅ Fixed threading and camera issues")
#     print("   • ✅ Gesture spam prevention")
#     print("   • ✅ 3-attempt face recognition")
#     print("   • ✅ Improved connection handling")
#     print(f"🔧 Enhanced robot movement module: {'Available' if robot_movement_available else 'Not Available (Simulation Mode)'}")
    
#     uvicorn.run(
#         app,
#         host="0.0.0.0",
#         port=8000,
#         log_level="info",
#         reload=False
#     )
# robot_movement.py - Complete version with IMPROVED face_helper integration
import asyncio
import re
import threading
import time
import logging
import cv2
import os
import platform
import subprocess
import sys
from typing import Dict, Optional, List, Tuple, Union
from dataclasses import dataclass, field
from enum import Enum
import json
from collections import deque

# Import face_helper with fallback
try:
    import face_helper as FR
    FACE_HELPER_AVAILABLE = True
    logging.info("Successfully imported face_helper module")
except ImportError:
    FACE_HELPER_AVAILABLE = False
    logging.warning("face_helper not found - face recognition disabled")

# Import dependencies with fallbacks
try:
    import face_recognition
    import numpy as np
    FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    FACE_RECOGNITION_AVAILABLE = False
    logging.warning("Face recognition not available - install face_recognition library")

try:
    import speech_recognition as sr
    SPEECH_RECOGNITION_AVAILABLE = True
except ImportError:
    SPEECH_RECOGNITION_AVAILABLE = False
    logging.warning("Speech recognition not available - install SpeechRecognition library")

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    logging.warning("MediaPipe not available - hand detection disabled")

# Optional imports with graceful fallbacks
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

try:
    from langchain_ollama import OllamaLLM
    from langchain_core.prompts import ChatPromptTemplate
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Hand Detection Helper Class
class HandDetector:
    """Hand gesture detection using MediaPipe"""
    
    def __init__(self, mode=False, max_hands=2, detection_con=0.5, track_con=0.5):
        if not MEDIAPIPE_AVAILABLE:
            logger.warning("MediaPipe not available - hand detection disabled")
            return
        
        self.mode = mode
        self.max_hands = max_hands
        self.detection_con = detection_con
        self.track_con = track_con
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.max_hands,
            min_detection_confidence=self.detection_con,
            min_tracking_confidence=self.track_con
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.results = None
    
    def find_hands(self, img, draw=True):
        """Find hands in image"""
        if not MEDIAPIPE_AVAILABLE:
            return img
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        
        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
        return img
    
    def find_position(self, img, hand_no=0, draw=True):
        """Find hand landmark positions"""
        lm_list = []
        if self.results and self.results.multi_hand_landmarks:
            try:
                my_hand = self.results.multi_hand_landmarks[hand_no]
                for id, lm in enumerate(my_hand.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lm_list.append([id, cx, cy])
                    if draw:
                        cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            except IndexError:
                pass
        return lm_list
    
    def is_waving(self, lm_list, index_x_history):
        """Detect waving gesture"""
        if len(lm_list) < 21:
            return False
        
        # Check if hand is in open position
        if not self.fingers_up(lm_list):
            return False
        
        index_x = lm_list[8][1]  # index fingertip x position
        index_x_history.append(index_x)
        
        if len(index_x_history) < 5:
            return False
        
        # Check if index finger tip is swinging side to side
        index_motion = max(index_x_history) - min(index_x_history)
        return index_motion > 40
    
    def fingers_up(self, lm_list):
        """Check if fingers are extended (open hand)"""
        if len(lm_list) < 21:
            return False
        
        # Thumb
        if lm_list[4][1] < lm_list[3][1]:  # Thumb tip left of thumb joint
            thumb_up = True
        else:
            thumb_up = False
        
        # Four fingers
        fingers = [thumb_up]
        for id in [8, 12, 16, 20]:  # Index, middle, ring, pinky tips
            if lm_list[id][2] < lm_list[id - 2][2]:  # Tip above joint
                fingers.append(True)
            else:
                fingers.append(False)
        
        return sum(fingers) >= 3  # At least 3 fingers up
    
    def is_shaking_hands(self, lm_list):
        """Detect handshake gesture"""
        if len(lm_list) < 21:
            return False
        
        # Check if fingers are partially closed (handshake position)
        tips = [8, 12, 16, 20]  # Index, middle, ring, pinky
        joints = [6, 10, 14, 18]  # Corresponding joints
        
        closed_fingers = 0
        for tip, joint in zip(tips, joints):
            if lm_list[tip][2] > lm_list[joint][2]:  # Tip below joint (closed)
                closed_fingers += 1
        
        return closed_fingers >= 2  # At least 2 fingers closed

# Enhanced Robot Configuration
@dataclass
class EnhancedRobotConfig:
    """Enhanced robot configuration with vision and speech settings"""
    # Motor pins
    motor1_speed: int = 38
    motor1_dir: int = 40
    motor2_speed: int = 32
    motor2_dir: int = 36
    motor3_speed: int = 16
    motor3_dir: int = 26
    
    # Ultrasonic sensor pins
    echo1: int = 31
    echo2: int = 29
    echo3: int = 22
    trig1: int = 11
    trig2: int = 13
    trig3: int = 15
    
    # Motor settings
    pwm_frequency: int = 1000
    default_speed: int = 25
    motor3_compensate: int = 3
    
    # Obstacle detection
    obstacle_threshold: float = 30.0
    sensor_timeout: float = 0.5
    obstacle_clear_threshold: float = 50.0
    
    # Vision settings
    camera_index: int = 0
    enable_face_recognition: bool = True
    enable_hand_detection: bool = True
    
    # Speech settings
    enable_speech_synthesis: bool = True
    enable_speech_recognition: bool = True
    
    # AI model
    ai_model: str = "llama3"

# Enhanced Robot State
@dataclass
class EnhancedRobotState:
    """Enhanced robot state with vision and speech data"""
    # Motor states
    motor1_speed: int = 0
    motor2_speed: int = 0
    motor3_speed: int = 0
    
    # Basic states
    status: str = "idle"
    obstacle_detected: bool = False
    last_distances: List[float] = field(default_factory=list)
    sensor_distances: Dict[str, float] = field(default_factory=lambda: {"front": 0.0, "left": 0.0, "right": 0.0})
    obstacle_sensor: str = ""
    obstacle_distance: float = 0.0
    last_command: str = ""
    uptime: float = 0.0
    
    # Vision states
    current_user: str = "Unknown"
    faces_detected: List[str] = field(default_factory=list)
    hand_gesture: str = "none"
    camera_active: bool = False
    
    # Speech states
    last_speech_output: str = ""
    listening: bool = False
    speech_recognition_active: bool = False
    
    # Interaction mode
    interaction_mode: str = "idle"  # idle, speech, text, keyboard, auto
    
    # Face recognition state management
    face_recognition_attempts: int = 0
    awaiting_registration: bool = False
    recognition_complete: bool = False

class EnhancedRobotController:
    """Enhanced robot controller with face recognition capabilities"""
    
    def __init__(self, config: Optional[EnhancedRobotConfig] = None):
        self.config = config or EnhancedRobotConfig()
        self.state = EnhancedRobotState()
        self.start_time = time.time()
        
        # Threading controls
        self.movement_lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.vision_lock = threading.RLock()
        self.interrupt_event = threading.Event()
        self.obstacle_event = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Thread references
        self.obstacle_thread = None
        self.status_thread = None
        self.vision_thread = None
        
        # Vision components
        self.hand_detector = None
        self.camera = None
        self.index_x_history = deque(maxlen=10)
        self.current_frame = None
        
        # PWM references
        self.motor1_pwm = None
        self.motor2_pwm = None
        self.motor3_pwm = None
        
        # Direction constants
        self.HIGH = GPIO.HIGH if GPIO_AVAILABLE else 1
        self.LOW = GPIO.LOW if GPIO_AVAILABLE else 0
        
        # Initialize components
        self.gpio_initialized = False
        self._setup_gpio()
        self._setup_ai()
        self._setup_vision()
        self._start_background_tasks()
        
        logger.info("Enhanced robot controller initialized successfully")
    
    def _setup_vision(self):
        """Setup camera and vision components with face_helper integration"""
        if not self.config.enable_face_recognition and not self.config.enable_hand_detection:
            logger.info("Vision components disabled in config")
            return
        
        try:
            # Initialize camera for face_helper integration
            camera_initialized = False
            for camera_index in [0, 1, 2]:
                try:
                    logger.info(f"Trying camera index {camera_index}")
                    self.camera = cv2.VideoCapture(camera_index)
                    
                    if self.camera.isOpened():
                        # Set camera properties
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.camera.set(cv2.CAP_PROP_FPS, 30)
                        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        
                        # Test camera
                        ret, test_frame = self.camera.read()
                        if ret and test_frame is not None:
                            with self.state_lock:
                                self.state.camera_active = True
                            
                            # Set camera reference in face_helper
                            if FACE_HELPER_AVAILABLE:
                                FR.face_recognition_system.cap = self.camera
                                if hasattr(FR, 'cap'):
                                    FR.cap = self.camera
                                logger.info("✅ Camera reference set in face_helper")
                            
                            camera_initialized = True
                            logger.info(f"✅ Camera setup completed on index {camera_index}")
                            break
                        else:
                            self.camera.release()
                            
                except Exception as e:
                    logger.warning(f"Failed to initialize camera on index {camera_index}: {e}")
                    if self.camera:
                        self.camera.release()
                    continue
            
            if not camera_initialized:
                logger.error("❌ Camera setup failed on all indices")
                with self.state_lock:
                    self.state.camera_active = False
                return
            
            # Initialize hand detection
            if self.config.enable_hand_detection and MEDIAPIPE_AVAILABLE:
                self.hand_detector = HandDetector()
                logger.info("✅ Hand detection initialized")
                
        except Exception as e:
            logger.error(f"❌ Vision setup failed: {e}")
            with self.state_lock:
                self.state.camera_active = False
    
    def _setup_gpio(self):
        """Setup GPIO pins and PWM"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            
            # Setup ultrasonic sensors
            GPIO.setup(self.config.echo1, GPIO.IN)
            GPIO.setup(self.config.echo2, GPIO.IN)
            GPIO.setup(self.config.echo3, GPIO.IN)
            GPIO.setup(self.config.trig1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.trig3, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup motors
            GPIO.setup(self.config.motor1_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor1_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor2_dir, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_speed, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.config.motor3_dir, GPIO.OUT, initial=GPIO.LOW)
            
            # Setup PWM
            self.motor1_pwm = GPIO.PWM(self.config.motor1_speed, self.config.pwm_frequency)
            self.motor2_pwm = GPIO.PWM(self.config.motor2_speed, self.config.pwm_frequency)
            self.motor3_pwm = GPIO.PWM(self.config.motor3_speed, self.config.pwm_frequency)
            
            self.motor1_pwm.start(0)
            self.motor2_pwm.start(0)
            self.motor3_pwm.start(0)
            
            self.gpio_initialized = True
            logger.info("GPIO initialized successfully")
            
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            self.gpio_initialized = False
    
    def _setup_ai(self):
        """Setup AI model for conversation"""
        if not AI_AVAILABLE:
            logger.warning("AI not available")
            self.ai_chain = None
            return
        
        try:
            template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
            model = OllamaLLM(model=self.config.ai_model)
            prompt = ChatPromptTemplate.from_template(template)
            self.ai_chain = prompt | model
            logger.info("AI model initialized successfully")
        except Exception as e:
            logger.error(f"AI initialization failed: {e}")
            self.ai_chain = None
    
    def _start_background_tasks(self):
        """Start background monitoring tasks"""
        try:
            # Obstacle detection
            self.obstacle_thread = threading.Thread(
                target=self._obstacle_detection_loop, 
                daemon=True, 
                name="ObstacleDetection"
            )
            self.obstacle_thread.start()
            
            # Status updates
            self.status_thread = threading.Thread(
                target=self._status_update_loop, 
                daemon=True, 
                name="StatusUpdate"
            )
            self.status_thread.start()
            
            # Unified vision processing
            if self.state.camera_active:
                self.vision_thread = threading.Thread(
                    target=self._unified_vision_loop,
                    daemon=True,
                    name="UnifiedVision"
                )
                self.vision_thread.start()
            
            logger.info("Background tasks started successfully")
        except Exception as e:
            logger.error(f"Failed to start background tasks: {e}")
    
    def _unified_vision_loop(self):
        """Unified vision processing loop with proper 3-attempt face recognition"""
        last_user_check = time.time()
        
        # Hand gesture state
        last_gesture = "none"
        gesture_cooldown = time.time()
        
        logger.info("🔍 Starting unified vision loop with 3-attempt face recognition")
        
        while not self.shutdown_event.is_set():
            try:
                if self.camera and self.camera.isOpened():
                    success, frame = self.camera.read()
                    if not success:
                        logger.warning("Failed to read camera frame")
                        time.sleep(0.1)
                        continue
                    
                    frame = cv2.flip(frame, 1)
                    current_time = time.time()
                    
                    # Hand detection with cooldown
                    if self.hand_detector:
                        frame_copy = frame.copy()
                        self.hand_detector.find_hands(frame_copy, draw=False)
                        lm_list = self.hand_detector.find_position(frame_copy, draw=False)
                        
                        if lm_list:
                            gesture = "none"
                            if self.hand_detector.is_waving(lm_list, self.index_x_history):
                                gesture = "waving"
                            elif self.hand_detector.is_shaking_hands(lm_list):
                                gesture = "shaking_hands"
                            
                            if (gesture != "none" and 
                                gesture != last_gesture and 
                                current_time - gesture_cooldown > 2.0):
                                
                                with self.state_lock:
                                    self.state.hand_gesture = gesture
                                    self.state.last_speech_output = f"I see you're {gesture.replace('_', ' ')}!"
                                    
                                if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
                                    threading.Thread(
                                        target=FR.speak, 
                                        args=(f"I see you're {gesture.replace('_', ' ')}!",),
                                        daemon=True
                                    ).start()
                                
                                logger.info(f"Hand gesture detected: {gesture}")
                                last_gesture = gesture
                                gesture_cooldown = current_time
                        else:
                            if current_time - gesture_cooldown > 1.0:
                                with self.state_lock:
                                    if self.state.hand_gesture != "none":
                                        self.state.hand_gesture = "none"
                                last_gesture = "none"
                    
                    # Face recognition with proper 3-attempt system
                    with self.state_lock:
                        recognition_complete = self.state.recognition_complete
                        current_attempts = self.state.face_recognition_attempts
                        awaiting_registration = self.state.awaiting_registration
                    
                    # Only try face recognition under specific conditions
                    if (current_time - last_user_check > 2.0 and
                        FACE_HELPER_AVAILABLE and 
                        self.state.hand_gesture == "none" and
                        not recognition_complete and
                        not awaiting_registration and
                        current_attempts < 3):
                        
                        last_user_check = current_time
                        try:
                            current_user, confidence = self._recognize_user_with_face_helper_proper(frame)
                            
                            if current_user and current_user != "Unknown":
                                # SUCCESS
                                with self.state_lock:
                                    self.state.current_user = current_user
                                    self.state.last_speech_output = f"Hello {current_user}!"
                                    self.state.face_recognition_attempts = 0
                                    self.state.awaiting_registration = False
                                    self.state.recognition_complete = True
                                    
                                if self.config.enable_speech_synthesis:
                                    threading.Thread(
                                        target=FR.speak,
                                        args=(f"Hello {current_user}!",),
                                        daemon=True
                                    ).start()
                                logger.info(f"✅ User recognized: {current_user} (confidence: {confidence:.3f})")
                            
                            else:
                                # FAILED ATTEMPT
                                new_attempts = current_attempts + 1
                                with self.state_lock:
                                    self.state.face_recognition_attempts = new_attempts
                                
                                logger.info(f"🔍 Face recognition attempt {new_attempts}/3 - No user found")
                                
                                if new_attempts >= 3:
                                    with self.state_lock:
                                        self.state.recognition_complete = True
                                        self.state.current_user = "Unknown"
                                    
                                    logger.info("❌ Face recognition complete after 3 attempts - showing options")
                            
                        except Exception as e:
                            logger.error(f"Face recognition error: {e}")
                            new_attempts = current_attempts + 1
                            with self.state_lock:
                                self.state.face_recognition_attempts = new_attempts
                            
                            if new_attempts >= 3:
                                with self.state_lock:
                                    self.state.recognition_complete = True
                                    self.state.current_user = "Unknown"
                    
                    # Store current frame for web streaming
                    self.current_frame = frame.copy()
                    
                    time.sleep(0.033)  # ~30 FPS
                    
                else:
                    logger.warning("Camera not available")
                    time.sleep(0.5)
                    
            except Exception as e:
                logger.error(f"Unified vision processing error: {e}")
                time.sleep(0.1)
    
    def _recognize_user_with_face_helper_proper(self, frame):
        """Proper integration with face_helper system"""
        if not FACE_HELPER_AVAILABLE or frame is None:
            return "Unknown", 0.0
        
        try:
            # Use the face_helper system's recognition function
            name, confidence = FR.face_recognition_system.recognize_single_frame(frame)
            
            if name and name != "Unknown":
                return name, confidence
            else:
                return "Unknown", 0.0
                
        except Exception as e:
            logger.error(f"Face recognition with face_helper error: {e}")
            return "Unknown", 0.0
    
    def _obstacle_detection_loop(self):
        """Background obstacle detection loop"""
        while not self.shutdown_event.is_set():
            try:
                sensor_readings = self._get_all_distances_with_names()
                
                if sensor_readings:
                    with self.state_lock:
                        self.state.sensor_distances = sensor_readings.copy()
                        self.state.last_distances = list(sensor_readings.values())
                    
                    valid_distances = [d for d in sensor_readings.values() if d > 0]
                    
                    if valid_distances:
                        min_distance = min(valid_distances)
                        closest_sensor = ""
                        
                        for sensor_name, distance in sensor_readings.items():
                            if distance == min_distance and distance > 0:
                                closest_sensor = sensor_name
                                break
                        
                        # Check for new obstacle detection
                        if min_distance < self.config.obstacle_threshold and not self.state.obstacle_detected:
                            self._update_obstacle_state(True, closest_sensor, min_distance)
                            self.emergency_stop()
                            
                        # Check for obstacle clearing
                        elif self.state.obstacle_detected and min_distance > self.config.obstacle_clear_threshold:
                            logger.info(f"Obstacle cleared! All sensors show > {self.config.obstacle_clear_threshold}cm")
                            self._update_obstacle_state(False)
                
                time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Error in obstacle detection: {e}")
                time.sleep(0.1)
    
    def _status_update_loop(self):
        """Background status update loop"""
        while not self.shutdown_event.is_set():
            try:
                with self.state_lock:
                    self.state.uptime = time.time() - self.start_time
                    
                    # Update status based on current state
                    if not self.state.obstacle_detected:
                        if self._is_moving():
                            self.state.status = "moving"
                        else:
                            self.state.status = "idle"
                
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in status update: {e}")
                time.sleep(0.1)
    
    def _update_obstacle_state(self, detected: bool, sensor: str = "", distance: float = 0.0):
        """Update obstacle detection state"""
        with self.state_lock:
            self.state.obstacle_detected = detected
            self.state.obstacle_sensor = sensor
            self.state.obstacle_distance = distance
            
            if detected:
                self.state.status = "obstacle_detected"
                self.obstacle_event.set()
                message = f"Obstacle detected by {sensor} sensor at {distance:.1f}cm!"
                self.state.last_speech_output = message
                if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
                    threading.Thread(
                        target=FR.speak,
                        args=("Obstacle detected! Stopping.",),
                        daemon=True
                    ).start()
                logger.warning(message)
            else:
                self.obstacle_event.clear()
                if self.state.status == "obstacle_detected":
                    self.state.status = "idle"
                logger.info("Obstacle detection cleared")
    
    def _is_moving(self) -> bool:
        """Check if robot is currently moving"""
        return any([
            self.state.motor1_speed > 0,
            self.state.motor2_speed > 0,
            self.state.motor3_speed > 0
        ])
    
    def _get_distance(self, trig_pin: int, echo_pin: int) -> float:
        """Get distance from ultrasonic sensor"""
        if not self.gpio_initialized:
            return 100.0  # Simulation mode
        
        try:
            GPIO.output(trig_pin, False)
            time.sleep(0.000002)
            
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 0:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return -1
            pulse_start = time.time()
            
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() - timeout_start > self.config.sensor_timeout:
                    return -1
            pulse_end = time.time()
            
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
            
            if 2 <= distance <= 400:
                return distance
            else:
                return -1
                
        except Exception as e:
            logger.error(f"Distance measurement error: {e}")
            return -1
    
    def _get_all_distances_with_names(self) -> Dict[str, float]:
        """Get distances from all sensors with names"""
        sensor_configs = [
            (self.config.trig1, self.config.echo1, "front"),
            (self.config.trig2, self.config.echo2, "left"),
            (self.config.trig3, self.config.echo3, "right")
        ]
        
        sensor_readings = {}
        for trig, echo, name in sensor_configs:
            dist = self._get_distance(trig, echo)
            if dist > 0:
                sensor_readings[name] = dist
        
        return sensor_readings
    
    def _change_speeds_smooth(self, new_speeds: Tuple[int, int, int]):
        """Smoothly change motor speeds"""
        if not self.gpio_initialized:
            with self.state_lock:
                self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
            return
        
        with self.movement_lock:
            with self.state_lock:
                if self.state.obstacle_detected or self.interrupt_event.is_set():
                    self._stop_motors_immediate()
                    return
                
                current_speeds = (self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed)
            
            speed_diffs = [abs(new_speeds[i] - current_speeds[i]) for i in range(3)]
            steps = max(speed_diffs) if speed_diffs else 0
            
            if steps < 1:
                steps = 1
            
            for step in range(steps + 1):
                with self.state_lock:
                    if self.state.obstacle_detected or self.interrupt_event.is_set():
                        self._stop_motors_immediate()
                        return
                
                progress = step / steps if steps > 0 else 1
                speeds = [
                    int(current_speeds[i] + (new_speeds[i] - current_speeds[i]) * progress)
                    for i in range(3)
                ]
                
                speeds = [max(0, min(100, speed)) for speed in speeds]
                
                try:
                    if self.motor1_pwm:
                        self.motor1_pwm.ChangeDutyCycle(speeds[0])
                    if self.motor2_pwm:
                        self.motor2_pwm.ChangeDutyCycle(speeds[1])
                    if self.motor3_pwm:
                        self.motor3_pwm.ChangeDutyCycle(speeds[2])
                except Exception as e:
                    logger.error(f"PWM update error: {e}")
                    self._stop_motors_immediate()
                    return
                
                time.sleep(0.01)
            
            with self.state_lock:
                if not self.state.obstacle_detected and not self.interrupt_event.is_set():
                    self.state.motor1_speed, self.state.motor2_speed, self.state.motor3_speed = new_speeds
                else:
                    self._stop_motors_immediate()
    
    def _stop_motors_immediate(self):
        """Immediately stop all motors"""
        try:
            if self.gpio_initialized and all([self.motor1_pwm, self.motor2_pwm, self.motor3_pwm]):
                self.motor1_pwm.ChangeDutyCycle(0)
                self.motor2_pwm.ChangeDutyCycle(0)
                self.motor3_pwm.ChangeDutyCycle(0)
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
        
        self.state.motor1_speed = 0
        self.state.motor2_speed = 0
        self.state.motor3_speed = 0
    
    # Public API methods
    
    def get_status(self) -> Dict:
        """Get status with proper face recognition state"""
        with self.state_lock:
            obstacle_message = "Robot operational"
            if self.state.obstacle_detected:
                obstacle_message = f"Obstacle detected by {self.state.obstacle_sensor} sensor at {self.state.obstacle_distance:.1f}cm"
            
            # Get face recognition system status
            face_users = []
            face_available = False
            registered_users_count = 0
            if FACE_HELPER_AVAILABLE:
                try:
                    face_users = FR.face_recognition_system.get_registered_users()
                    face_available = True
                    registered_users_count = len(face_users)
                except Exception as e:
                    logger.error(f"Error getting face users: {e}")
            
            return {
                "status": self.state.status,
                "message": obstacle_message,
                "obstacle_detected": self.state.obstacle_detected,
                "obstacle_sensor": self.state.obstacle_sensor,
                "obstacle_distance": self.state.obstacle_distance,
                "current_speeds": {
                    "motor1": self.state.motor1_speed,
                    "motor2": self.state.motor2_speed,
                    "motor3": self.state.motor3_speed
                },
                "sensor_distances": self.state.sensor_distances.copy(),
                "last_distances": self.state.last_distances.copy(),
                "last_command": self.state.last_command,
                "uptime": self.state.uptime,
                "gpio_available": self.gpio_initialized,
                # Enhanced status fields with face recognition
                "current_user": self.state.current_user,
                "faces_detected": face_users,
                "hand_gesture": self.state.hand_gesture,
                "camera_active": self.state.camera_active,
                "last_speech_output": self.state.last_speech_output,
                "listening": self.state.listening,
                "speech_recognition_active": self.state.speech_recognition_active,
                "interaction_mode": self.state.interaction_mode,
                "face_recognition_available": face_available,
                "speech_recognition_available": SPEECH_RECOGNITION_AVAILABLE,
                "mediapipe_available": MEDIAPIPE_AVAILABLE,
                "face_recognition_attempts": self.state.face_recognition_attempts,
                "awaiting_registration": self.state.awaiting_registration,
                "recognition_complete": self.state.recognition_complete,
                "max_attempts_reached": self.state.face_recognition_attempts >= 3,
                "registered_users_count": registered_users_count
            }
    
    def speak(self, text: str) -> bool:
        """Make robot speak text using face_helper"""
        try:
            with self.state_lock:
                self.state.last_speech_output = text
            
            if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
                # Use face_helper's speak function in a thread
                threading.Thread(
                    target=FR.speak,
                    args=(text,),
                    daemon=True
                ).start()
                logger.info(f"Robot speaking: {text}")
                return True
            else:
                logger.info(f"Speech synthesis disabled. Would say: {text}")
                return False
        except Exception as e:
            logger.error(f"Speech synthesis error: {e}")
            return False
    
    def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
        """Listen for speech input using face_helper"""
        if not self.config.enable_speech_recognition or not FACE_HELPER_AVAILABLE:
            return None
        
        try:
            with self.state_lock:
                self.state.listening = True
                self.state.speech_recognition_active = True
            
            # Use face_helper's listen function
            result = FR.listen()
            
            with self.state_lock:
                self.state.listening = False
                self.state.speech_recognition_active = False
            
            return result
        except Exception as e:
            logger.error(f"Speech recognition error: {e}")
            with self.state_lock:
                self.state.listening = False
                self.state.speech_recognition_active = False
            return None
    
    def set_interaction_mode(self, mode: str) -> bool:
        """Set interaction mode (speech, text, keyboard, auto)"""
        valid_modes = ["idle", "speech", "text", "keyboard", "auto"]
        if mode in valid_modes:
            with self.state_lock:
                self.state.interaction_mode = mode
            logger.info(f"Interaction mode set to: {mode}")
            return True
        else:
            logger.warning(f"Invalid interaction mode: {mode}")
            return False
    
    def recognize_user(self, mode: str = "auto") -> Optional[str]:
        """Recognize user using face_helper system"""
        if not FACE_HELPER_AVAILABLE or not self.current_frame:
            logger.warning("Face helper or camera frame not available")
            return None
        
        try:
            # Use the improved recognition method
            user, confidence = self._recognize_user_with_face_helper_proper(self.current_frame)
            
            if user and user != "Unknown":
                with self.state_lock:
                    self.state.current_user = user
                    self.state.face_recognition_attempts = 0
                    self.state.awaiting_registration = False
                    self.state.recognition_complete = True
                
                logger.info(f"✅ User recognition: {user} (confidence: {confidence:.3f})")
                return user
            else:
                logger.info("❌ Recognition: No user found")
                return None
                
        except Exception as e:
            logger.error(f"❌ Recognition error: {e}")
            return None
    
    def register_new_user(self, name: str) -> bool:
        """Register new user using face_helper system"""
        if not FACE_HELPER_AVAILABLE:
            logger.error("❌ Face helper not available for registration")
            return False
        
        try:
            logger.info(f"🆔 Starting registration for user: {name}")
            
            # Check camera availability
            if not self.camera or not self.camera.isOpened():
                logger.error("❌ Camera not available for registration")
                return False
            
            logger.info("📷 Camera is available for registration")
            
            # Use face_helper's registration system
            success = FR.face_recognition_system.take_picture(name, self.camera)
            
            if success:
                # Update robot state
                with self.state_lock:
                    self.state.current_user = name
                    self.state.face_recognition_attempts = 0
                    self.state.awaiting_registration = False
                    self.state.recognition_complete = True
                
                logger.info(f"🎉 Registration completed for: {name}")
                logger.info(f"   📊 Total users now: {len(FR.face_recognition_system.classNames)}")
                return True
            else:
                logger.error(f"❌ Registration failed for: {name}")
                return False
                
        except Exception as e:
            logger.error(f"❌ Registration error for {name}: {e}")
            import traceback
            logger.error(f"   Full traceback: {traceback.format_exc()}")
            return False
    
    def reset_face_recognition_state(self) -> bool:
        """Reset face recognition state properly"""
        try:
            with self.state_lock:
                # Reset ALL recognition state variables
                self.state.face_recognition_attempts = 0
                self.state.awaiting_registration = False
                self.state.current_user = "Unknown"
                self.state.recognition_complete = False
            
            # Also reset the face_helper system if available
            if FACE_HELPER_AVAILABLE:
                FR.face_recognition_system.reset_recognition_state()
            
            logger.info("✅ Face recognition state reset completed")
            logger.info(f"   • Attempts: 0/3")
            logger.info(f"   • Recognition complete: False")
            logger.info(f"   • Ready for new recognition cycle")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error resetting face recognition state: {e}")
            return False
    
    def move(self, direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
        """Move robot in specified direction"""
        direction_map = {
            "up": "forward",
            "down": "backward", 
            "left": "turnleft",
            "right": "turnright",
            "forward": "forward",
            "backward": "backward",
            "turnleft": "turnleft",
            "turnright": "turnright",
            "moveleft": "moveleft",
            "moveright": "moveright",
            "stop": "stop"
        }
        
        mapped_direction = direction_map.get(direction.lower(), direction.lower())
        
        # Check obstacle
        with self.state_lock:
            if self.state.obstacle_detected and mapped_direction != "stop":
                message = f"Movement blocked - obstacle detected by {self.state.obstacle_sensor} sensor"
                self.state.last_speech_output = message
                if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
                    threading.Thread(
                        target=FR.speak,
                        args=("Cannot move, obstacle detected!",),
                        daemon=True
                    ).start()
                logger.warning(message)
                return False
            
            speed = speed or self.config.default_speed
            self.state.last_command = mapped_direction
        
        # Direction configurations
        direction_configs = {
            "forward": {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            "backward": {
                "dirs": [self.HIGH, self.LOW, self.HIGH],
                "speeds": [0, speed, speed + self.config.motor3_compensate]
            },
            "turnleft": {
                "dirs": [self.HIGH, self.LOW, self.LOW],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            "turnright": {
                "dirs": [self.LOW, self.HIGH, self.HIGH],
                "speeds": [speed, speed, speed + self.config.motor3_compensate]
            },
            "moveleft": {
                "dirs": [self.HIGH, self.HIGH, self.LOW],
                "speeds": [int(speed * 1.5), speed, 0]
            },
            "moveright": {
                "dirs": [self.LOW, self.LOW, self.HIGH],
                "speeds": [int(speed * 1.5), speed, speed + self.config.motor3_compensate]
            },
            "stop": {
                "dirs": [self.LOW, self.LOW, self.LOW],
                "speeds": [0, 0, 0]
            }
        }
        
        if mapped_direction not in direction_configs:
            logger.error(f"Invalid direction: {direction}")
            return False
        
        config = direction_configs[mapped_direction]
        
        # Set motor directions
        if self.gpio_initialized:
            try:
                GPIO.output(self.config.motor1_dir, config["dirs"][0])
                GPIO.output(self.config.motor2_dir, config["dirs"][1])
                GPIO.output(self.config.motor3_dir, config["dirs"][2])
            except Exception as e:
                logger.error(f"Error setting motor directions: {e}")
                return False
        
        # Apply speeds
        self._change_speeds_smooth(tuple(config["speeds"]))
        
        # Handle duration
        if duration_ms is not None and duration_ms > 0:
            time.sleep(duration_ms / 1000)
            with self.state_lock:
                if not self.state.obstacle_detected:
                    self._change_speeds_smooth((0, 0, 0))
        
        return True
    
    def stop(self) -> bool:
        """Stop the robot"""
        return self.move("stop")
    
    def emergency_stop(self) -> bool:
        """Emergency stop with speech notification"""
        logger.warning("Emergency stop activated")
        
        with self.movement_lock:
            with self.state_lock:
                self.state.status = "emergency_stop"
                self.state.last_speech_output = "Emergency stop activated"
                self._stop_motors_immediate()
        
        if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
            threading.Thread(
                target=FR.speak,
                args=("Emergency stop activated",),
                daemon=True
            ).start()
        
        return True
    
    def reset_obstacle_detection(self) -> bool:
        """Reset obstacle detection state"""
        self._update_obstacle_state(False)
        message = "Obstacle detection reset - robot can move again"
        with self.state_lock:
            self.state.last_speech_output = message
        if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
            threading.Thread(
                target=FR.speak,
                args=("Obstacle cleared, ready to move",),
                daemon=True
            ).start()
        logger.info(message)
        return True
    
    def parse_command(self, command: str) -> Optional[Tuple[str, Optional[int]]]:
        """Parse natural language command"""
        command = command.lower().strip()
        
        directions = {
            "forward": ["go forward", "move forward", "move ahead", "advance", "go straight"],
            "backward": ["go backward", "move backward", "reverse", "back up", "go back"],
            "stop": ["stop", "halt", "stand still", "brake", "freeze"],
            "turnleft": ["turn left", "rotate left", "spin left"],
            "turnright": ["turn right", "rotate right", "spin right"],
            "moveleft": ["move left", "strafe left", "slide left", "sidestep left"],
            "moveright": ["move right", "strafe right", "slide right", "sidestep right"]
        }
        
        time_patterns = {
            "seconds": r"(\d+(?:\.\d+)?)\s*(?:second|sec|s)s?",
            "minutes": r"(\d+(?:\.\d+)?)\s*(?:minute|min|m)s?",
            "hours": r"(\d+(?:\.\d+)?)\s*(?:hour|hr|h)s?"
        }
        
        # Find direction
        direction = None
        for dir_key, phrases in directions.items():
            if any(phrase in command for phrase in phrases):
                direction = dir_key
                break
        
        # Find duration
        duration_ms = None
        for unit, pattern in time_patterns.items():
            match = re.search(pattern, command)
            if match:
                try:
                    value = float(match.group(1))
                    if unit == "seconds":
                        duration_ms = int(value * 1000)
                    elif unit == "minutes":
                        duration_ms = int(value * 60000)
                    elif unit == "hours":
                        duration_ms = int(value * 3600000)
                    break
                except (ValueError, OverflowError):
                    logger.warning(f"Invalid duration value: {match.group(1)}")
                    continue
        
        return (direction, duration_ms) if direction else None
    
    async def chat(self, message: str, context: str = "") -> str:
        """Chat with AI model and respond with intelligent speech"""
        if not self.ai_chain:
            response = "AI model not available. Please check Ollama installation."
        else:
            try:
                result = self.ai_chain.invoke({"context": context, "question": message})
                response = str(result)
            except Exception as e:
                logger.error(f"AI chat error: {e}")
                response = "Error processing message with AI model."
        
        # Update state
        with self.state_lock:
            self.state.last_speech_output = response
        
        # INTELLIGENT SPEECH: Only speak for non-movement commands
        if self._should_speak_response(message):
            if self.config.enable_speech_synthesis and FACE_HELPER_AVAILABLE:
                threading.Thread(
                    target=FR.speak,
                    args=(response,),
                    daemon=True
                ).start()
                logger.info(f"🗣️ Speaking AI response: {response[:50]}...")
        else:
            logger.info(f"🤫 Not speaking movement command response: {response[:50]}...")
        
        return response
    
    def _should_speak_response(self, original_message: str) -> bool:
        """Determine if the robot should speak the response based on the original message"""
        movement_keywords = [
            'go forward', 'move forward', 'forward', 'advance',
            'go backward', 'move backward', 'backward', 'reverse', 'back up',
            'turn left', 'turn right', 'rotate left', 'rotate right',
            'move left', 'move right', 'strafe left', 'strafe right',
            'stop', 'halt', 'brake', 'freeze'
        ]
        
        message_lower = original_message.lower()
        
        # If the message contains movement keywords, don't speak
        for keyword in movement_keywords:
            if keyword in message_lower:
                return False
        
        # For conversational messages, do speak
        return True
    
    async def handle_conversation_mode(self, mode: str) -> str:
        """Handle different conversation modes (speech, text, auto)"""
        try:
            with self.state_lock:
                self.state.interaction_mode = mode
            
            if mode == "speech":
                # Recognize user first
                user = self.recognize_user("speech")
                if user:
                    greeting = f"Hi {user}, what would you like to ask today?"
                    self.speak(greeting)
                    
                    # Listen for commands
                    user_input = self.listen_for_speech()
                    if user_input:
                        # Try to parse as movement command first
                        parsed = self.parse_command(user_input)
                        if parsed:
                            direction, duration = parsed
                            success = self.move(direction, duration_ms=duration)
                            response = f"Movement command {direction} {'executed' if success else 'failed'}"
                        else:
                            # Use AI for general conversation
                            response = await self.chat(user_input)
                        
                        return response
                    else:
                        return "I didn't hear anything clearly."
                else:
                    return "I couldn't recognize you. Please look at the camera."
            
            elif mode == "text":
                user = self.recognize_user("text")
                if user:
                    greeting = f"Hi {user}, what would you like to ask today?"
                    self.speak(greeting)
                    return greeting
                else:
                    return "Please look at the camera for identification."
            
            elif mode == "auto":
                # Automatic interaction based on detected gestures/faces
                if self.state.hand_gesture in ["waving", "shaking_hands"]:
                    response = f"I see you're {self.state.hand_gesture.replace('_', ' ')}! How can I help you?"
                    self.speak(response)
                    return response
                elif self.state.current_user != "Unknown":
                    response = f"Hello {self.state.current_user}! I'm ready for commands."
                    self.speak(response)
                    return response
                else:
                    return "I'm watching and ready to help!"
            
            return "Unknown conversation mode"
            
        except Exception as e:
            logger.error(f"Conversation mode error: {e}")
            return f"Error in conversation mode: {str(e)}"
    
    def get_camera_frame(self) -> Optional[bytes]:
        """Get current camera frame for web streaming"""
        if not self.camera or not self.camera.isOpened() or self.current_frame is None:
            return None
        
        try:
            frame = self.current_frame.copy()
            
            # Add hand detection visualization if enabled
            if self.hand_detector and self.state.hand_gesture != "none":
                cv2.putText(frame, f"Gesture: {self.state.hand_gesture.replace('_', ' ')}", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Add status text overlay
            cv2.putText(frame, f"User: {self.state.current_user}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Status: {self.state.status}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show recognition attempts if scanning
            if self.state.awaiting_registration:
                cv2.putText(frame, "Ready for Registration", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
            elif self.state.face_recognition_attempts > 0:
                cv2.putText(frame, f"Scanning... ({self.state.face_recognition_attempts}/3)", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 165, 0), 2)
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                return buffer.tobytes()
            
            return None
        except Exception as e:
            logger.error(f"Camera frame error: {e}")
            return None
    
    def shutdown(self):
        """Shutdown enhanced robot controller with cleanup"""
        logger.info("Shutting down enhanced robot controller...")
        
        # Signal shutdown to all threads
        self.shutdown_event.set()
        
        # Stop all motors immediately
        with self.movement_lock:
            with self.state_lock:
                self._stop_motors_immediate()
        
        # Wait for background threads to finish
        threads_to_join = [
            (self.obstacle_thread, "obstacle detection"),
            (self.status_thread, "status update"),
            (self.vision_thread, "unified vision")
        ]
        
        for thread, name in threads_to_join:
            if thread and thread.is_alive():
                logger.info(f"Waiting for {name} thread to finish...")
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"{name} thread did not finish gracefully")
        
        # Cleanup camera
        if self.camera and self.camera.isOpened():
            try:
                self.camera.release()
                logger.info("Camera released")
            except Exception as e:
                logger.error(f"Camera cleanup error: {e}")
        
        # Cleanup GPIO and PWM
        if self.gpio_initialized:
            try:
                pwm_objects = [
                    (self.motor1_pwm, "motor1"),
                    (self.motor2_pwm, "motor2"), 
                    (self.motor3_pwm, "motor3")
                ]
                
                for pwm, name in pwm_objects:
                    if pwm:
                        try:
                            pwm.stop()
                            logger.debug(f"{name} PWM stopped")
                        except Exception as e:
                            logger.error(f"Error stopping {name} PWM: {e}")
                
                GPIO.cleanup()
                logger.info("GPIO cleanup completed")
                
            except Exception as e:
                logger.error(f"GPIO cleanup error: {e}")
        
        logger.info("Enhanced robot controller shutdown complete")


# Global enhanced robot controller instance
_enhanced_robot_controller = None
_enhanced_controller_lock = threading.Lock()

def get_enhanced_robot_controller():
    """Get or create the global enhanced robot controller instance"""
    global _enhanced_robot_controller
    with _enhanced_controller_lock:
        if _enhanced_robot_controller is None:
            _enhanced_robot_controller = EnhancedRobotController()
        return _enhanced_robot_controller

# Public API functions for FastAPI backend compatibility
def get_status() -> Dict:
    """Get current robot status - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().get_status()
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        return {
            "status": "error",
            "message": f"Error getting status: {str(e)}",
            "obstacle_detected": False,
            "gpio_available": False,
            "camera_active": False,
            "face_recognition_available": False,
            "speech_recognition_available": False,
            "face_recognition_attempts": 0,
            "awaiting_registration": False
        }

def move(direction: str, speed: Optional[int] = None, duration_ms: Optional[int] = None) -> bool:
    """Move robot in specified direction - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().move(direction, speed, duration_ms)
    except Exception as e:
        logger.error(f"Error moving robot: {e}")
        return False

def stop() -> bool:
    """Stop the robot - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().stop()
    except Exception as e:
        logger.error(f"Error stopping robot: {e}")
        return False

def speak(text: str) -> bool:
    """Make robot speak - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().speak(text)
    except Exception as e:
        logger.error(f"Error speaking: {e}")
        return False

def listen_for_speech(timeout: int = 5) -> Optional[str]:
    """Listen for speech input - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().listen_for_speech(timeout)
    except Exception as e:
        logger.error(f"Error listening for speech: {e}")
        return None

def recognize_user(mode: str = "auto") -> Optional[str]:
    """Recognize current user - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().recognize_user(mode)
    except Exception as e:
        logger.error(f"Error recognizing user: {e}")
        return None

def register_new_user(name: str) -> bool:
    """Register new user - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().register_new_user(name)
    except Exception as e:
        logger.error(f"Error registering user: {e}")
        return False

def set_interaction_mode(mode: str) -> bool:
    """Set interaction mode - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().set_interaction_mode(mode)
    except Exception as e:
        logger.error(f"Error setting interaction mode: {e}")
        return False

def reset_face_recognition_state() -> bool:
    """Reset face recognition state - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().reset_face_recognition_state()
    except Exception as e:
        logger.error(f"Error resetting face recognition state: {e}")
        return False

async def handle_conversation_mode(mode: str) -> str:
    """Handle conversation mode - called by FastAPI"""
    try:
        return await get_enhanced_robot_controller().handle_conversation_mode(mode)
    except Exception as e:
        logger.error(f"Error handling conversation mode: {e}")
        return f"Error: {str(e)}"

def get_camera_frame() -> Optional[bytes]:
    """Get camera frame - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().get_camera_frame()
    except Exception as e:
        logger.error(f"Error getting camera frame: {e}")
        return None

def parse_command(command: str) -> Optional[Tuple[str, Optional[int]]]:
    """Parse natural language command - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().parse_command(command)
    except Exception as e:
        logger.error(f"Error parsing command: {e}")
        return None

async def chat(message: str, context: str = "") -> str:
    """Chat with AI model - called by FastAPI"""
    try:
        return await get_enhanced_robot_controller().chat(message, context)
    except Exception as e:
        logger.error(f"Error in chat: {e}")
        return "Error processing chat message."

def emergency_stop() -> bool:
    """Emergency stop - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().emergency_stop()
    except Exception as e:
        logger.error(f"Error in emergency stop: {e}")
        return False

def reset_obstacle_detection() -> bool:
    """Reset obstacle detection - called by FastAPI"""
    try:
        return get_enhanced_robot_controller().reset_obstacle_detection()
    except Exception as e:
        logger.error(f"Error resetting obstacle detection: {e}")
        return False

def shutdown_robot():
    """Shutdown robot controller - called by FastAPI on app shutdown"""
    global _enhanced_robot_controller
    with _enhanced_controller_lock:
        if _enhanced_robot_controller is not None:
            _enhanced_robot_controller.shutdown()
            _enhanced_robot_controller = None

if __name__ == "__main__":
    print("🤖 Enhanced Robot Movement Controller - Complete with Face Helper Integration")
    print("=" * 80)
    print("Features:")
    print("• ✅ INTEGRATED: face_helper.py for face recognition and speech")
    print("• ✅ FIXED: Registration using actual face_helper functions")
    print("• ✅ FIXED: Stable WebSocket connections with proper cleanup")
    print("• ✅ Unified vision loop for face and hand detection") 
    print("• ✅ Non-blocking speech synthesis using face_helper.speak()")
    print("• ✅ Improved camera initialization with face_helper integration")
    print("• ✅ Better error handling and logging")
    print("• ✅ Enhanced obstacle detection")
    print("• ✅ AI integration with speech responses")
    print("• ✅ Camera streaming support")
    print("• ✅ Face recognition attempt limiting (3 tries then STOP)")
    print("• ✅ Hand gesture spam prevention")
    print("• ✅ Proper connection status handling")
    print("• ✅ FastAPI integration ready")
    print("• ✅ User registration with face_helper.take_picture()")
    print("=" * 80)
    print("✅ Enhanced robot controller module loaded successfully")
    print("🔧 Key Integration: Uses your existing face_helper.py for all face operations")
    print("🔧 Key Integration: Shares camera properly between vision systems")
    print("🔧 Key Integration: Registration saves to backend/images directory")
    print("🔧 Key Fix: Face recognition stops after 3 attempts and waits for user choice")
    print("🔧 Key Fix: No automatic navigation to registration page")
    print("🔧 Key Fix: Stable WebSocket connections with proper cleanup")