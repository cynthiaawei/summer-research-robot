#!/usr/bin/env python3
"""
Navigation AI System - EXPLAINED VERSION
This is the main AI navigation code that takes over robot control
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import numpy as np
import cv2
import base64
import json
import time
import math
import threading
import asyncio
import sys
import os

# ═══════════════════════════════════════════════════════════════════
# PART 1: IMPORT YOUR ROBOT MOVEMENT FUNCTIONS
# ═══════════════════════════════════════════════════════════════════
# This connects to your robot/backend/ folder to use your movement functions
try:
    backend_path = os.path.expanduser("~/robot/backend")  # Path to your backend
    sys.path.append(backend_path)
    from robot_movement import *  # Import ALL your movement functions
    MOVEMENT_AVAILABLE = True
    print("✅ SUCCESS: Your robot_movement.py functions are imported!")
    print("   AI can now use: move_forward(), turn_left(), etc.")
except ImportError as e:
    MOVEMENT_AVAILABLE = False
    print(f"❌ FAILED: Could not import robot_movement.py: {e}")
    print(f"   Expected path: {backend_path}")
    print("   AI will use basic fallback movements")

# Import AI model for decision making
try:
    from langchain_ollama import OllamaLLM
    AI_AVAILABLE = True
except ImportError:
    AI_AVAILABLE = False
    print("⚠️ Install langchain_ollama for AI features")

class NavigationAI(Node):
    """
    MAIN AI NAVIGATION CLASS
    
    What this does:
    1. Listens to your SLAM map updates
    2. Listens to Nav2 planned paths  
    3. Gets robot position from your odom_node.py
    4. Sends everything to AI for analysis
    5. Executes navigation using your robot_movement.py functions
    """
    
    def __init__(self):
        super().__init__('navigation_ai')
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 2: AI MODEL SETUP
        # ═══════════════════════════════════════════════════════════════════
        # Connect to Ollama AI for decision making
        self.ai_model = None
        if AI_AVAILABLE:
            try:
                self.ai_model = OllamaLLM(model="mistral-nemo")  # Connect to Ollama
                self.get_logger().info("🤖 AI BRAIN: Ollama connected!")
            except Exception as e:
                self.get_logger().error(f"❌ AI BRAIN: Ollama failed: {e}")
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 3: ROBOT LOCALIZATION SETUP  
        # ═══════════════════════════════════════════════════════════════════
        # This helps us know WHERE the robot is on the map
        self.tf_buffer = Buffer()              # Stores coordinate transforms
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Listens to robot position
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 4: DATA STORAGE
        # ═══════════════════════════════════════════════════════════════════
        # Store current state of everything
        self.current_map = None        # Latest map from SLAM
        self.current_path = None       # Latest path from Nav2
        self.robot_pose = None         # Current robot position  
        self.path_image = None         # Image for AI to see
        self.executing = False         # Are we currently moving?
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 5: ROBOT CONTROL PUBLISHER
        # ═══════════════════════════════════════════════════════════════════
        # This sends movement commands to robot (fallback if backend fails)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 6: LISTENERS (SUBSCRIBERS)
        # ═══════════════════════════════════════════════════════════════════
        # Listen to important topics from your robot system
        
        # Listen to SLAM map updates
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        print("📡 LISTENING: /map topic (from your SLAM system)")
        
        # Listen to Nav2 planned paths  
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        print("📡 LISTENING: /plan topic (from Nav2 path planner)")
        
        # Listen when you set robot pose in RViz
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        print("📡 LISTENING: /initialpose topic (when you set robot pose)")
        
        # ═══════════════════════════════════════════════════════════════════
        # PART 7: TIMER FOR CONTINUOUS ROBOT TRACKING
        # ═══════════════════════════════════════════════════════════════════
        # Check robot position every 0.5 seconds
        self.pose_timer = self.create_timer(0.5, self.update_robot_pose)
        
        self.get_logger().info("🎯 NAVIGATION AI: System ready!")
        self.print_system_status()
    
    def print_system_status(self):
        """Print what systems are connected and working"""
        print("\n" + "="*60)
        print("🤖 NAVIGATION AI SYSTEM STATUS")
        print("="*60)
        print("Integration Status:")
        print(f"  🧠 AI Brain (Ollama): {'✅ Connected' if self.ai_model else '❌ Not available'}")
        print(f"  🔧 Robot Backend: {'✅ Connected' if MOVEMENT_AVAILABLE else '❌ Not found'}")
        print("")
        print("Your System Flow:")
        print("  Arduino → arduino_bridge.py → odom_node.py → /odom")
        print("  SLAM → /map → Nav2 → /plan → 🤖 AI → robot_movement.py")
        print("")
        print("What This AI Does:")
        print("  1. 👀 Watches your SLAM map updates")
        print("  2. 📍 Tracks robot position from your odom_node.py")  
        print("  3. 🛣️ Gets planned paths from Nav2")
        print("  4. 🧠 Sends everything to AI for smart analysis")
        print("  5. 🚀 Executes using your robot_movement.py functions")
        print("="*60)
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 8: MAP HANDLING
    # ═══════════════════════════════════════════════════════════════════
    def map_callback(self, msg):
        """
        CALLED EVERY TIME YOUR SLAM UPDATES THE MAP
        
        What happens:
        1. Store the new map
        2. Convert map to image so AI can "see" it
        """
        self.current_map = msg
        self.convert_map_to_image()
        # Log occasionally so you know SLAM is working
        if hasattr(self, '_map_count'):
            self._map_count += 1
            if self._map_count % 50 == 0:  # Every 50th update
                self.get_logger().info(f"🗺️ SLAM UPDATE: Map updated {self._map_count} times")
        else:
            self._map_count = 1
    
    def convert_map_to_image(self):
        """
        CONVERT SLAM MAP TO IMAGE FOR AI TO SEE
        
        SLAM gives us numbers, AI needs to "see" the map like a picture
        """
        if not self.current_map:
            return
        
        try:
            # Get map as numbers (0=free, 100=wall, -1=unknown)
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height, self.current_map.info.width)
            
            # Convert numbers to colors for AI to understand
            map_img = np.zeros_like(map_data, dtype=np.uint8)
            map_img[map_data == 0] = 255      # Free space = WHITE
            map_img[map_data == -1] = 128     # Unknown = GRAY
            map_img[map_data == 100] = 0      # Walls = BLACK
            
            # Flip image so it matches RViz view
            map_img = np.flipud(map_img)
            # Convert to color image
            self.map_image = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
            
        except Exception as e:
            self.get_logger().error(f"❌ MAP CONVERSION ERROR: {e}")
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 9: ROBOT POSITION TRACKING
    # ═══════════════════════════════════════════════════════════════════
    def initial_pose_callback(self, msg):
        """CALLED WHEN YOU SET ROBOT POSE IN RVIZ"""
        self.get_logger().info("📍 ROBOT POSE: Set in RViz - ready for navigation!")
    
    def update_robot_pose(self):
        """
        CONTINUOUSLY TRACK WHERE ROBOT IS
        
        This runs every 0.5 seconds to know robot's current position
        Uses data from your odom_node.py
        """
        try:
            # Ask TF system: "Where is robot relative to map?"
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            # Extract position (x, y) and orientation
            pos = transform.transform.translation
            rot = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            
            # Store robot position in easy-to-use format
            self.robot_pose = {
                'x': pos.x,                    # Robot X position on map
                'y': pos.y,                    # Robot Y position on map  
                'yaw': yaw,                    # Robot orientation (radians)
                'yaw_deg': math.degrees(yaw),  # Robot orientation (degrees)
                'source': 'your_odom_node'     # Data comes from your system
            }
            
        except Exception as e:
            # Robot position not available yet (normal during startup)
            pass
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 10: NAVIGATION PATH HANDLING
    # ═══════════════════════════════════════════════════════════════════
    def path_callback(self, msg):
        """
        🚨 MAIN TRIGGER FUNCTION 🚨
        
        CALLED WHEN NAV2 PLANS A PATH (when you set goal in RViz)
        
        This is where the magic starts:
        1. Nav2 planned a path
        2. Create visualization for AI
        3. Send everything to AI for analysis
        4. Execute AI's plan using your robot functions
        """
        self.current_path = msg
        self.get_logger().info(f"🛣️ PATH RECEIVED: Nav2 planned {len(msg.poses)} waypoints")
        
        # Create image showing map + robot + path for AI to analyze
        self.create_path_visualization()
        
        # If we have all data needed, start AI navigation
        if self.current_map and self.robot_pose and not self.executing:
            self.get_logger().info("🧠 STARTING AI: All data ready - sending to AI...")
            self.start_ai_navigation()
        else:
            missing = []
            if not self.current_map: missing.append("map")
            if not self.robot_pose: missing.append("robot_pose") 
            if self.executing: missing.append("already_executing")
            self.get_logger().warn(f"⚠️ WAITING FOR: {missing}")
    
    def create_path_visualization(self):
        """
        CREATE PICTURE FOR AI TO SEE
        
        Make an image showing:
        - Map (white=free, black=walls, gray=unknown)
        - Robot position (blue circle)
        - Robot orientation (yellow arrow)
        - Planned path (green line)
        - Goal (red circle)
        """
        if not all([self.current_map, self.current_path, self.map_image]):
            return
        
        try:
            # Start with map image
            vis_image = self.map_image.copy()
            
            # Get map coordinate info
            resolution = self.current_map.info.resolution  # meters per pixel
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            height = self.current_map.info.height
            
            # Convert path waypoints from world coordinates to pixel coordinates
            path_pixels = []
            for pose_stamped in self.current_path.poses:
                world_x = pose_stamped.pose.position.x  # Real world X
                world_y = pose_stamped.pose.position.y  # Real world Y
                
                # Convert to image pixel coordinates
                pixel_x = int((world_x - origin_x) / resolution)
                pixel_y = int(height - (world_y - origin_y) / resolution)
                
                # Only add if pixel is within image bounds
                if 0 <= pixel_x < self.current_map.info.width and 0 <= pixel_y < height:
                    path_pixels.append((pixel_x, pixel_y))
            
            # Draw green path line
            if len(path_pixels) > 1:
                for i in range(len(path_pixels) - 1):
                    cv2.line(vis_image, path_pixels[i], path_pixels[i+1], (0, 255, 0), 4)
                
                # Mark start (blue) and goal (red)
                cv2.circle(vis_image, path_pixels[0], 12, (255, 0, 0), -1)   # Start = blue
                cv2.circle(vis_image, path_pixels[-1], 15, (0, 0, 255), -1)  # Goal = red
            
            # Draw robot position and orientation
            if self.robot_pose:
                robot_x = self.robot_pose['x']
                robot_y = self.robot_pose['y']
                robot_yaw = self.robot_pose['yaw']
                
                # Convert robot position to pixels
                robot_pixel_x = int((robot_x - origin_x) / resolution)
                robot_pixel_y = int(height - (robot_y - origin_y) / resolution)
                
                if 0 <= robot_pixel_x < self.current_map.info.width and 0 <= robot_pixel_y < height:
                    # Draw robot as blue circle
                    cv2.circle(vis_image, (robot_pixel_x, robot_pixel_y), 14, (255, 0, 0), -1)
                    
                    # Draw orientation arrow
                    arrow_length = 25
                    end_x = int(robot_pixel_x + arrow_length * math.cos(robot_yaw))
                    end_y = int(robot_pixel_y - arrow_length * math.sin(robot_yaw))
                    cv2.arrowedLine(vis_image, (robot_pixel_x, robot_pixel_y), (end_x, end_y), (255, 255, 0), 3)
            
            # Save visualization image
            self.path_image = vis_image
            cv2.imwrite('ai_navigation_plan.png', vis_image)
            self.get_logger().info("🖼️ VISUALIZATION: Saved ai_navigation_plan.png for AI analysis")
            
        except Exception as e:
            self.get_logger().error(f"❌ VISUALIZATION ERROR: {e}")
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 11: AI ANALYSIS
    # ═══════════════════════════════════════════════════════════════════
    def extract_navigation_data(self) -> dict:
        """
        PREPARE ALL DATA FOR AI
        
        Collect everything AI needs to make smart decisions:
        - Map information
        - Robot current position
        - Planned path waypoints
        """
        # Map details
        map_info = {}
        if self.current_map:
            info = self.current_map.info
            map_info = {
                "width": info.width,
                "height": info.height,
                "resolution": round(info.resolution, 4),
                "origin_x": round(info.origin.position.x, 3),
                "origin_y": round(info.origin.position.y, 3)
            }
        
        # Robot current state
        robot_data = self.robot_pose or {}
        
        # Path waypoints (convert to simple coordinates)
        waypoints = []
        if self.current_path:
            for i, pose_stamped in enumerate(self.current_path.poses):
                pos = pose_stamped.pose.position
                orient = pose_stamped.pose.orientation
                
                _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
                
                waypoints.append({
                    "index": i,
                    "x": round(pos.x, 3),
                    "y": round(pos.y, 3),
                    "theta_deg": round(math.degrees(yaw), 1)
                })
        
        return {
            "robot_system": "arduino_based_robot",
            "backend_available": MOVEMENT_AVAILABLE,
            "map_info": map_info,
            "robot_pose": robot_data,
            "path_waypoints": waypoints[:10],  # First 10 waypoints
            "total_waypoints": len(waypoints)
        }
    
    async def send_to_ai(self) -> dict:
        """
        SEND EVERYTHING TO AI FOR ANALYSIS
        
        This is the AI "brain" function:
        1. Convert image to format AI can process
        2. Create detailed prompt explaining the situation
        3. Ask AI to create execution plan using your robot functions
        4. Return AI's response
        """
        if not self.ai_model or not self.path_image:
            return {"error": "AI model or visualization not available"}
        
        try:
            # Convert image to base64 so AI can "see" it
            _, buffer = cv2.imencode('.png', self.path_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Get all navigation data
            nav_data = self.extract_navigation_data()
            
            # Create detailed prompt for AI
            prompt = f"""
I'm controlling an Arduino-based robot with your custom movement functions. Here's the complete situation:

ROBOT HARDWARE SYSTEM:
- Arduino microcontroller with custom firmware
- Your arduino_bridge.py handles communication
- Your odom_node.py provides position tracking
- Your robot_movement.py contains movement functions
- SLAM system provides live mapping
- Nav2 handles path planning

IMAGE ANALYSIS:
The attached image shows:
- WHITE areas: Free navigable space (safe to drive)
- BLACK areas: Obstacles/walls (cannot pass through)
- GRAY areas: Unknown/unexplored space (proceed with caution)
- BLUE CIRCLE: Robot current position
- YELLOW ARROW: Robot current orientation/direction
- GREEN LINE: Optimal path planned by Nav2
- RED CIRCLE: Final destination/goal

CURRENT NAVIGATION DATA:
{json.dumps(nav_data, indent=2)}

YOUR AVAILABLE ROBOT FUNCTIONS:
Based on your robot_movement.py backend, you can use:
- move_forward(distance_meters, speed=0.3): Move robot straight
- turn_left(angle_degrees): Turn robot left by specified angle
- turn_right(angle_degrees): Turn robot right by specified angle
- move_to_coordinate(x, y, speed=0.3): Navigate directly to coordinates
- rotate_to_heading(target_degrees): Rotate to absolute compass heading
- stop_robot(): Emergency stop all movement

TASK: Create an intelligent execution plan using your specific robot functions.

IMPORTANT CONSIDERATIONS:
1. Your robot's current position and orientation
2. The efficiency of the planned path
3. Your robot's movement capabilities and limitations
4. Safe speeds for your hardware
5. Smooth transitions between movements
6. Error handling and safety

Return a JSON execution plan in this exact format:
{{
    "analysis": {{
        "robot_system": "arduino_based_with_custom_backend",
        "path_assessment": "detailed evaluation of the planned route",
        "current_robot_state": "description of robot position and orientation",
        "estimated_difficulty": "easy/medium/hard",
        "key_challenges": ["list any potential issues or difficult segments"]
    }},
    "execution_plan": {{
        "strategy": "high-level approach description",
        "estimated_total_time": 45.0,
        "steps": [
            {{"action": "rotate_to_heading", "params": {{"target_degrees": 45.0}}, "duration": 3.0, "reason": "align robot with path direction"}},
            {{"action": "move_to_coordinate", "params": {{"x": 2.0, "y": 1.5, "speed": 0.3}}, "duration": 12.0, "reason": "navigate to first major waypoint"}},
            {{"action": "stop_robot", "params": {{}}, "duration": 1.0, "reason": "complete navigation safely"}}
        ]
    }}
}}

Focus on creating an efficient, safe execution plan using your specific robot's movement functions.
"""
            
            # Send to AI and get response
            result = self.ai_model.invoke(prompt)
            self.get_logger().info("🧠 AI ANALYSIS: Complete! Processing response...")
            
            try:
                # Try to parse AI response as JSON
                analysis = json.loads(result)
                return analysis
            except json.JSONDecodeError:
                # If AI didn't return proper JSON, create fallback plan
                self.get_logger().warn("⚠️ AI RESPONSE: Not valid JSON, creating fallback plan")
                goal_waypoint = nav_data["path_waypoints"][-1] if nav_data["path_waypoints"] else {"x": 0, "y": 0}
                return {
                    "analysis": {"robot_system": "arduino_based_fallback"},
                    "execution_plan": {
                        "strategy": "Simple direct movement to goal using your backend",
                        "estimated_total_time": 20.0,
                        "steps": [
                            {"action": "move_to_coordinate", "params": {"x": goal_waypoint["x"], "y": goal_waypoint["y"], "speed": 0.3}, "duration": 18.0},
                            {"action": "stop_robot", "params": {}, "duration": 1.0}
                        ]
                    }
                }
            
        except Exception as e:
            self.get_logger().error(f"❌ AI ANALYSIS ERROR: {e}")
            return {"error": str(e)}
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 12: ROBOT EXECUTION
    # ═══════════════════════════════════════════════════════════════════
    def execute_plan(self, analysis_result: dict):
        """
        EXECUTE AI'S PLAN USING YOUR ROBOT FUNCTIONS
        
        This is where AI's decisions become robot actions
        """
        if "error" in analysis_result:
            self.get_logger().error(f"❌ EXECUTION BLOCKED: {analysis_result['error']}")
            return
        
        # Print AI's analysis
        analysis = analysis_result.get("analysis", {})
        execution_plan = analysis_result.get("execution_plan", {})
        
        self.get_logger().info("🧠 AI ANALYSIS RESULTS:")
        self.get_logger().info(f"   Robot system: {analysis.get('robot_system', 'unknown')}")
        self.get_logger().info(f"   Path assessment: {analysis.get('path_assessment', 'unknown')}")
        self.get_logger().info(f"   Current state: {analysis.get('current_robot_state', 'unknown')}")
        self.get_logger().info(f"   Difficulty: {analysis.get('estimated_difficulty', 'unknown')}")
        
        challenges = analysis.get('key_challenges', [])
        if challenges:
            self.get_logger().info(f"   Challenges: {', '.join(challenges)}")
        
        strategy = execution_plan.get("strategy", "No strategy provided")
        estimated_time = execution_plan.get("estimated_total_time", "unknown")
        
        self.get_logger().info(f"🎯 EXECUTION STRATEGY: {strategy}")
        self.get_logger().info(f"⏱️ ESTIMATED TIME: {estimated_time} seconds")
        
        # Start execution
        self.executing = True
        
        try:
            steps = execution_plan.get("steps", [])
            self.get_logger().info(f"🚀 STARTING EXECUTION: {len(steps)} steps planned")
            
            for i, step in enumerate(steps):
                if not self.executing:  # Allow emergency stop
                    self.get_logger().warn("🛑 EXECUTION STOPPED: Emergency stop activated")
                    break
                
                action = step.get("action")
                params = step.get("params", {})
                reason = step.get("reason", "")
                duration = step.get("duration", 5.0)
                
                self.get_logger().info(f"📋 STEP {i+1}/{len(steps)}: {action}")
                self.get_logger().info(f"    Reason: {reason}")
                self.get_logger().info(f"    Parameters: {params}")
                
                # Execute the action using your robot functions
                if MOVEMENT_AVAILABLE:
                    self.execute_your_movement(action, params)
                else:
                    self.get_logger().warn("⚠️ FALLBACK: Using basic movement (robot_movement.py not available)")
                    self.execute_fallback_movement(action, params)
                
                # Brief pause between actions
                time.sleep(0.5)
            
            self.get_logger().info("✅ EXECUTION COMPLETE: All steps finished successfully!")
            
        except Exception as e:
            self.get_logger().error(f"❌ EXECUTION ERROR: {e}")
        finally:
            # Always stop and cleanup
            self.executing = False
            if MOVEMENT_AVAILABLE:
                stop_robot()  # Use your function
            else:
                cmd = Twist()  # Fallback stop
                self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("🏁 EXECUTION FINISHED: Robot stopped safely")
    
    def execute_your_movement(self, action: str, params: dict):
        """
        EXECUTE USING YOUR ROBOT_MOVEMENT.PY FUNCTIONS
        
        This calls your specific robot movement functions
        """
        try:
            self.get_logger().info(f"🔧 USING YOUR BACKEND: Calling {action}")
            
            if action == "move_forward":
                distance = params.get("distance_meters", 1.0)
                speed = params.get("speed", 0.3)
                move_forward(distance, speed)  # Your function
                
            elif action == "turn_left":
                angle = params.get("angle_degrees", 90.0)
                turn_left(angle)  # Your function
                
            elif action == "turn_right":
                angle = params.get("angle_degrees", 90.0)
                turn_right(angle)  # Your function
                
            elif action == "move_to_coordinate":
                x = params.get("x", 0.0)
                y = params.get("y", 0.0)
                speed = params.get("speed", 0.3)
                move_to_coordinate(x, y, speed)  # Your function
                
            elif action == "rotate_to_heading":
                target_deg = params.get("target_degrees", 0.0)
                rotate_to_heading(target_deg)  # Your function
                
            elif action == "stop_robot":
                stop_robot()  # Your function
                
            else:
                self.get_logger().warn(f"⚠️ UNKNOWN ACTION: {action} not recognized")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOUR FUNCTION ERROR: {e}")
            # Fall back to basic movement
            self.execute_fallback_movement(action, params)
    
    def execute_fallback_movement(self, action: str, params: dict):
        """
        FALLBACK MOVEMENT IF YOUR FUNCTIONS FAIL
        
        Basic movement using ROS /cmd_vel topic
        """
        self.get_logger().info(f"⚙️ FALLBACK MOVEMENT: Using basic {action}")
        cmd = Twist()
        
        if action == "move_forward":
            distance = params.get("distance_meters", 1.0)
            speed = params.get("speed", 0.3)
            duration = distance / speed
            
            cmd.linear.x = speed
            for _ in range(int(duration * 10)):
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
                
        elif action in ["turn_left", "turn_right"]:
            angle_deg = params.get("angle_degrees", 90.0)
            direction = 1.0 if action == "turn_left" else -1.0
            duration = abs(angle_deg) / 45.0  # Rough timing
            
            cmd.angular.z = 0.5 * direction
            for _ in range(int(duration * 10)):
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
        
        # Always stop after movement
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    # ═══════════════════════════════════════════════════════════════════
    # PART 13: MAIN COORDINATION FUNCTION
    # ═══════════════════════════════════════════════════════════════════
    def start_ai_navigation(self):
        """
        🚨 MAIN COORDINATION FUNCTION 🚨
        
        This orchestrates the entire AI navigation process:
        1. Check we have all data needed
        2. Send everything to AI for analysis
        3. Execute AI's plan using your robot functions
        """
        if self.executing:
            self.get_logger().warn("⚠️ ALREADY EXECUTING: Navigation in progress!")
            return
        
        # Check if we have all required data
        required_data = [self.current_map, self.current_path, self.robot_pose, self.path_image]
        missing = []
        
        if not self.current_map:
            missing.append("map")
        if not self.current_path:
            missing.append("path")
        if not self.robot_pose:
            missing.append("robot_pose")
        if not self.path_image:
            missing.append("visualization")
        
        if missing:
            self.get_logger().error(f"❌ MISSING DATA: {missing}")
            self.get_logger().info("   Make sure:")
            self.get_logger().info("   - SLAM is running (map)")
            self.get_logger().info("   - Robot pose is set in RViz (robot_pose)")
            self.get_logger().info("   - Goal is set in RViz (path)")
            return
        
        self.get_logger().info("🚀 STARTING AI NAVIGATION: All data ready!")
        self.get_logger().info(f"   Map: {self.current_map.info.width}x{self.current_map.info.height}")
        self.get_logger().info(f"   Path: {len(self.current_path.poses)} waypoints")
        self.get_logger().info(f"   Robot: ({self.robot_pose['x']:.2f}, {self.robot_pose['y']:.2f}, {self.robot_pose['yaw_deg']:.1f}°)")
        
        # Run AI analysis and execution in separate thread to avoid blocking
        def navigation_thread():
            """
            BACKGROUND THREAD FOR AI PROCESSING
            
            This runs the AI analysis and execution without blocking the main ROS loop
            """
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                self.get_logger().info("🧠 AI ANALYSIS: Sending data to Ollama...")
                
                # Send everything to AI and get execution plan
                ai_result = loop.run_until_complete(self.send_to_ai())
                
                if "error" in ai_result:
                    self.get_logger().error(f"❌ AI FAILED: {ai_result['error']}")
                    return
                
                self.get_logger().info("🧠 AI ANALYSIS: Success! Executing plan...")
                
                # Execute the AI's plan using your robot functions
                self.execute_plan(ai_result)
                
            except Exception as e:
                self.get_logger().error(f"❌ NAVIGATION THREAD ERROR: {e}")
            finally:
                # Clean up event loop
                loop.close()
        
        # Start the navigation thread
        thread = threading.Thread(target=navigation_thread, daemon=True)
        thread.start()


def main():
    """
    MAIN FUNCTION - Entry point for the Navigation AI system
    
    This initializes ROS2 and starts the AI navigation node
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create the navigation AI node
        nav_ai = NavigationAI()
        
        # Print startup information
        print("\n" + "="*60)
        print("🤖 NAVIGATION AI SYSTEM RUNNING!")
        print("="*60)
        print("Integration Status:")
        print(f"  🧠 AI Brain: {'✅ Ready' if nav_ai.ai_model else '❌ Not available'}")
        print(f"  🔧 Robot Backend: {'✅ Connected' if MOVEMENT_AVAILABLE else '❌ Not found'}")
        print("")
        print("System Ready! Here's what to do:")
        print("1. 📍 Set robot pose in RViz with '2D Pose Estimate'")
        print("2. 🎯 Set navigation goal with '2D Goal Pose'")
        print("3. 🤖 Watch AI analyze and execute navigation!")
        print("4. 🗺️ SLAM will keep updating map during navigation")
        print("")
        print("Data Flow:")
        print("  Your SLAM → /map → Nav2 → /plan → 🤖 AI → Your robot_movement.py")
        print("="*60)
        
        # Start spinning (this keeps the node running and processing messages)
        rclpy.spin(nav_ai)
        
    except KeyboardInterrupt:
        print("\n🛑 NAVIGATION AI: Shutting down gracefully...")
        
        # Stop any ongoing navigation
        if 'nav_ai' in locals() and nav_ai.executing:
            nav_ai.executing = False
            if MOVEMENT_AVAILABLE:
                try:
                    stop_robot()  # Use your function to stop
                except:
                    pass
            
        print("✅ Navigation AI stopped safely")
        
    except Exception as e:
        print(f"❌ FATAL ERROR: {e}")
        
    finally:
        # Clean shutdown
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()