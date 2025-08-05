import asyncio
import time
import math
import logging
import re
from typing import Optional, Dict, Tuple, List
from enum import Enum
from robot_movement import get_enhanced_robot_controller
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quarternion
from nav_msgs.msg import OccupancyGrid
from tf_transformations import quarternion_from_euler
NAV2_AVAILABLE = True
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class NavigationMode(Enum):
    IDLE= "idle"
    NAV2_PLANNING = "nav2_plan"
    NAV2_EXECUTING= "nav2_exec"
    AI_ANALYZING= "ai_analyze"
    AI_EXECUTING= "ai_execute"
    FAILED= "failed"
    SUCCEEDED= "succeeded"

class StuckReason(Enum):
    PHYSICAL_OBSTACLE= "physical"
    NAVIGATION_CONFUSED= "confused"
    SENSOR_BLOCKED= "sensor"
    MECHANICAL= "mechanical"
    UNKNOWN= "unknown"

class Nav2Interface(Node):
    def __init__(self):    
        super().__init__('hybrid_nav2_interface')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') 

        self.current_goal_handle= None
        self.navigation_result= None
        self.navigation_feedback= None

        if not self.nav_client.wait_for_server(timout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
        else:
            self.get_logger().info("Nav2 action server connected")

    async def navigate_to_pose(self, x: float, y: float, theta: float = 0.0):
        goal_msg= NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map' #position is relative to the map
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y  
        goal_msg.pose.pose.position.z = 0.0

        q= quarternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        try:
            self.get_logger().info(f"Sending goal to Nav2: ({x:.2f}, {y:.2f} {math.degrees(theta):.1f})")
            send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback = self. _feedback_callback)
            rclpy.spin_until_future_complete(self.send_goal_future, timeout_sec=5.0) # Wait for Nav2 to accept or reject the goal

            if send_goal_future.done():
                self.current_goal_handle = send_goal_future.result()
                if self.current_goal_handle.accepted: #check if Nav2 accepted this goal
                    self.get_logger().info("Nav2 accepted the goal")
                    return True
                else:
                    self.get_logger().info("Nav2 rejected the goal")
                    return False
            else:
                self.get_logger().info("Nav2 goal send timed out")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Error sending the goal to Nav2: {e}")
            return False
        
    def _feedback_callback(self, feedback_msg):
        self.navigation_feedback = feedback_msg.feedback

    def get_navigation_status(self):
        if not self.current_goal_handle:
            return 'idle' 
        
        if self.current_goal_handle.is_ready():
            result = self.current_goal_handle.get_result()
            if result.status == 4:
                return 'succeeded'
            else:
                return 'failed'
        else:
            return 'executing'
        
    def cancel_navigation(self):
        if self.current_goal_handle:
            self.get_logger().info("Cancelling navigation")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

class HybridNavigator:
    def __init__(self):
        self.robot = get_enhanced_robot_controller
        self.nav2 = None

        if NAV2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()

                self.nav2 = Nav2Interface() 
                print("Nav2 interface initialized")
            except Exception as e:
                print(f"Nav2 initialization failed: {e}")
                self.nav2 = None

        self.current_mode = NavigationMode.IDLE
        self.current_goal = None
        self.navigation_active = False

        self.stuck_detection_enabled = True
        self.stuck_check_interval = 2.0
        self.stuck_time_threshold = 8.0
        self.stuck_distance_threshold = 0.1

        self.last_position = None
        self.stuck_start_time = None
        self.stuck_count = 0

        self.max_ai_attempts = 3
        self.ai_attempt_count = 0

        print("Hybrid Navigator READY")

    async def navigate_to_goal(self, goal_x: float, goal_y: float, goal_theta: float = 0.0):
        print(f"Starting navigation to ({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f})")

        self.current_goal = (goal_x, goal_y, goal_theta)
        self.navigation_active = True
        self.ai_attempt_count = 0
        self.stuck_count = 0

        try:
            if self.nav2:
                print("Attempting")
                success = await self._navigate_with_nav2()

                if success:
                    print("Successful")
                    return True
                else:
                    print("Not successful")

            print("Trying Ai-assissted navigation")
            success = await self._navigate_with_ai()

            return success
        except Exception as e:
            logger.error(f"Navigation error: {e}")
            return False
        
        finally:
            self._cleanup_navigation()


    def get_navigation_status(self):
        try:
            robot_status = self.robot.get_status()
            return{
                'navigation_active': self.navigation_active,
                'current_mode': self.current_mode.value,
                'current_goal': self.current_goal,

                'current_position': robot_status.get('current_position', {}),
                'distance_to_goal': self._calculate_distance_to_goal(),

                'nav2_available': self.nav2 is not None,
                'ai_available': hasattr(self.robot, 'chat'),
                'robot_status': robot_status.get('status','unknown'),

                'stuck_detection_enabled': self.stuck_detection_enabled,
                'currently_stuck': self.stuck_start_time is not None,
                'stuck_duration': time.time() - self.stuck_start_time if self.stuck_start_time else 0,
                'stuck_count': self.stuck_count,

                'ai_attempts': self.ai_attempt_count,
                'max_ai_attempts': self.max_ai_attempts,

                'sensors': robot_status.get('sensor_distances', {}),
                'obstacle_detected': robot_status.get('obstacle_detected',False)
            }
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return {'error':str(e)}
        
    async def cancel_navigation(self):
        print("Cancelling navigation")

        try:
            if self.nav2 and self.current_mode in [NavigationMode.NAV2_PLANNING, NavigationMode.NAV2_EXECUTING]:
               self.nav2.cancel_navigation()#self.nav2 is the instance of the nav2interface class

            self.robot.stop()

            self._cleanup_navigation()

            print("Navigation Cancelled")
            return True
        except Exception as e:
            logger.error(f"Error cancelling navigation: {e}")
            return False
    async def _navigate_with_nav2(self):
        if not self.nav2:
            print("Nav2 not available")
            return False
        
        goal_x, goal_y, goal_theta = self.current_goal
        
        self.current_mode = NavigationMode.NAV2_PLANNING
        nav2_started = await self.nav2.navigate_to_pose(goal_x, goal_y, goal_theta)

        if not nav2_started:
            print("Nav2 failed to start navigation")
            return False
        
        self.current_mode = NavigationMode.NAV2_EXECUTING

        stuck_detection_task = asyncio.create_task(self._stuck_detection_loop())

        try:
            while self.navigation_active:
                nav2_status= self.nav2.get_navigation_status()
                print(f"Nav2 status: {nav2_status}")

                if nav2_status == 'succeeded':
                    print("Nav2 reached the goal")
                    return True
                elif nav2_status == 'failed':
                    print("Nav2 Failed")
                    return False
                elif self.current_mode == NavigationMode.AI_ANALYZING:
                    print("AI intervention in process")
                    self.nav2.cancel_navigation()

                    ai_success = await self._handle_ai_intervention() #order of methods inside the class don't matter

                    if ai_success:
                        print("AI intervention successful, resuming Nav2")
                        self.current_mode = NavigationMode.NAV2_PLANNING
                        nav2_started = await self.nav2.navigate_to_pose(goal_x,goal_y,goal_theta)
                        if nav2_started:
                            self.current_mode = NavigationMode.NAV2_EXECUTING
                        else:
                            print("Failed to resume Nav2")
                            return False
                    else:
                        print("AI intervention failed")
                        return False
                await asyncio.sleep(1)

            return False
        finally:
            stuck_detection_task.cancel()
            try:
                await stuck_detection_task
            except asyncio.CancelledError:
                pass
    async def _stuck_detection_loop(self):
        print("Starting stuck detection monitoring")
        while self.navigation_active and self.current_mode in [NavigationMode.NAV2_EXECUTING]:
            try:
                is_stuck = self._check_physical_movement()

                if is_stuck:
                    print("Physical Stuck Detected")
                    print("Robot hasnt moved despite Nav2 running")

                    if self.ai_attempt_count < self.max_ai_attempts:
                        self.current_mode = NavigationMode.AI_ANALYZING
                        self.ai_attempt_count+=1
                        self.stuck_count +=1

                        print("Requesting AI intervention")
                        break
                    else:
                        print("Max AI attempts reached")
                        self.navigation_active = False
                        break
                await asyncio.sleep(self.stuck_check_interval)
            except asyncio.CancelledError:
                print("Stuck detection cancelled")
                break
            except Exception as e:
                logger.error(f"Error in stuck detection: {e}")
                await asyncio.sleep(self.stuck_check_interval)

    def _check_physical_movement(self):
        try:
            status = self.robot.get_status()
            current_x = status.get('current_position', {}).get('x', 0)
            current_y = status.get('current_position', {}).get('y', 0)
            current_pos = (current_x, current_y)

            if self.last_position is None:
                self.last_position = current_pos
                self.stuck_start_time = None
                return False
            
            distance_moved = math.sqrt((current_pos[0] - self.last_position[0])**2
                                       + (current_pos[1] - self.last_position[1])**2)
            print(f"Moved {distance_moved:.3f} m in {self.stuck_check_interval}s")

            if distance_moved < self.stuck_distance_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()#if it is the first time the robot stopped record the time
                    print(f"Robot stopped moving (threshold: {self.stuck_time_threshold}s)")

                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    print(f"Stuck for {stuck_duration:.1f}s")

                    if stuck_duration > self.stuck_time_duration:
                        return True
            else:
                if self.stuck_start_time is not None:
                    print("Robot moving again")
                self.stuck_start_time = None
                self.last_position = current_pos
            return False
        
        except Exception as e:
            logger.error(f"Error checking movement : {e}")
            return False
        
    async def _handle_ai_intervention(self):
        print("AI taking control to resolve stuck situation")

        try:
            print("AI analyzing why robot is stuck")
            analysis = await self._get_ai_situation_analysis()
            print(f"AI analysis: {analysis}")

            print("AI creating recovery plan")
            action_plan = await self._get_ai_recovery_plan
            print(f"Action plan: {action_plan}")

            print("Executing AI recovery actions")
            self.current_mode = NavigationMode.AI_EXECUTING
            execution_success = await self._execute_ai_actions(action_plan)

            if execution_success:
                print("AI intervention completed successfully")
                self.stuck_start_time = None
                self.last_position = None
                return True
            
            else:
                print("AI intervention execution failed")
                return False
        except Exception as e:
            logger.error(f"AI intervention error: {e}")
            return False
        
    async def _get_ai_situation_analysis(self):
        try:
            status = self.robot.get_status()
            analysis_prompt = f"""
                    I'm a robot that got stuck during autonomous navigation. Please analyze what might be wrong.

                    NAVIGATION CONTEXT:
                    - Goal: Trying to reach {self.current_goal}
                    - Method: Using Nav2 autonomous navigation
                    - Problem: Haven't moved significantly in {self.stuck_time_threshold} seconds
                    - This is stuck incident #{self.stuck_count} during this navigation

                    CURRENT ROBOT STATE:
                    - Position: ({status.get('current_position', {}).get('x', 0):.2f}, {status.get('current_position', {}).get('y', 0):.2f})
                    - System status: {status.get('status', 'unknown')}
                    - System obstacle detection: {status.get('obstacle_detected', False)}

                    SENSOR READINGS:
                    - Front sensor: {status.get('sensor_distances', {}).get('front', 0)}cm
                    - Left sensor: {status.get('sensor_distances', {}).get('left', 0)}cm
                    - Right sensor: {status.get('sensor_distances', {}).get('right', 0)}cm

                    ROBOT CAPABILITIES:
                    - Omni-wheel robot (can move in any direction)
                    - Ultrasonic sensors with ~4m range
                    - Can turn, move forward/backward/left/right

                    ANALYSIS REQUEST:
                    What type of stuck situation is this? Consider:
                    1. Physical obstacle (wall, furniture, person)
                    2. Navigation confusion (sensors vs map mismatch)
                    3. Narrow passage (robot too wide)
                    4. Sensor interference (bad readings)
                    5. Mechanical issue (wheel stuck)

                    Give me your analysis of the most likely cause.
                    """
            analysis = await self.robot.chat(analysis_prompt)
            return analysis
        except Exception as e:
            logger.error(f"AI analysis error:{e}")
            return "Unable to analyze situation"

    async def _get_ai_recovery_plan(self, analysis):
        try:
            recovery_prompt = f"""
                            Based on my analysis: "{analysis}"

                            I need a specific recovery plan to get unstuck and continue navigation to {self.current_goal}.

                            REQUIREMENTS:
                            - Give me 2-4 specific, safe actions
                            - Be precise about directions and distances
                            - Consider that I'll resume Nav2 navigation after this
                            - Actions should be conservative to avoid making things worse

                            GOOD ACTION EXAMPLES:
                            - "Turn left 45 degrees"
                            - "Move backward 0.8 meters"  
                            - "Rotate right 90 degrees to scan area"
                            - "Move forward slowly 0.3 meters"

                            BAD EXAMPLES:
                            - "Navigate around obstacle" (too vague)
                            - "Move forward 5 meters" (too far, might hit something)
                            - "Turn around completely" (might lose progress)

                            Please provide a numbered action plan:
                            """
            plan = await self.robot.chat(recovery_prompt)  
            return plan
        except Exception as e:
            logger.error(f"AI planning error: {e}")
            return "Unable to create recovery plan"
    
    async def _execute_ai_actions(self,action_plan):
        try:
            actions = self._parse_ai_action_text(action_plan)

            if not actions:
                print("Couldnt parse AI action plan")
                return False
            
            print(f"Executing {len(actions)} AI actions")

            for i, action in enumerate(actions):
                print(f"Action{i+1}/{len(actions)}: {action}")

                success = await self._execute_single_action(action)
                if not success:
                    print(f"Action {i+1} failed")
                    return False
                
                print(f"Action {i+1} completed")

                await asyncio.sleep(1.0)

            print("All AI actions completed")

            return True
        except Exception as e:
            logger.error(f"AI execution error: {e}")
            return False
        
    def _parse_ai_action_text(self, action_text):
        actions = []
        lines = action_text.lower().split('\n')

        for line in lines:
            line=line.strip()
            if len(line)<5:
                continue

            if 'turn left' in line or 'rotate left' in line:
                angle_match = re.search (r'(\d+)\s*degrees?', line)
                angle = int(angle_match.group(1)) if angle_match else 45
                actions.append(('turn_left', angle))

            elif 'turn right' in line or 'rotate right' in line:
                angle_match = re.search(r'(\d+)\s*degrees?', line)
                angle = int(angle_match.group(1)) if angle_match else 45
                actions.append(('turn_right', angle))

            
            elif 'move backward' in line or 'back up' in line:
                distance_match = re.search(r'(\d+\.?\d*)\s*meters?', line)
                distance = float(distance_match.group(1)) if distance_match else 0.5
                actions.append(('move_backward', distance))

            elif 'move backward' in line:
                distance_match = re.search(r'(\d+\.?\d*)\s*meters?', line)
                distance = float(distance_match.group(1)) if distance_match else 0.5
                actions.append(('move_forward', distance))

            elif 'rotate' in line:
                angle_match = re.search(r'(\d+)\s*degrees?', line)
                angle = int(angle_match.group(1)) if angle_match else 90
                actions.append(('rotate', angle))
        return actions
    
    async def _execute_single_action(self, action): 
        action_type, value = action

        try:
            if action_type == 'turn_left':
                duration_ms = int(value*50)
                return self.robot.move('turnleft', duration_ms=duration_ms)

            elif action_type == 'turn_right':
                duration_ms = int(value*50)
                return self.robot.move('turnright', duration_ms=duration_ms)
        
            elif action_type == 'move_forward':
                duration_ms = int(value*2000)
                return self.robot.move('forward', duration_ms = duration_ms)
            
            elif action_type == 'move_backward':
                duration_ms = int(value*2000)
                return self.robot.move('backward', duration_ms=duration_ms)

            elif action_type == 'rotate' :
                duration_ms = int(value*50)
                return self.robot.move('turnleft', duration_ms = duration_ms)
            
            else:
                print(f"Unknown action type: {action_type}")
                return False
            
        except Exception as e:
            logger.error(f"Error executing action {action}: {e}")
            return False
        
    async def _navigate_with_ai(self):
        print("starting AI only navigation mode")
        print("This mode asks AI to plan each navigation step")

        self.current_mode = NavigationMode.AI_ANALYZING
        max_steps = 20

        for step in range(max_steps):
            print(f"\n AI navigation step {step+1}/{max_steps}")

            if self._is_goal_reached():
                print("AI navigation successful - goal reached")
                self.current_mode = NavigationMode.SUCCEEDED
                return True
            
            self.current_mode = NavigationMode.AI_ANALYZING
            next_instruction = await self._get_ai_navigation_step()
            
            if not next_instruction:
                print("AI couldn't determine next step")
                break

            self.current_mode = NavigationMode.AI_EXECUTING
            step_success = await self._execute_ai_actions(next_instruction)
            if not step_success:
                print("Failed to execute AI navigation step")
                break

            await asyncio.sleep(2)

        print("AI navigation failed or exceeded maximum steps")
        self.current_mode = NavigationMode.FAILED
        return False
    
    async def _get_ai_navigation_step(self):
        try:
            status = self.robot.get_status()
            current_x = status.get('current_position', {}).get('x', 0)
            current_y = status.get('current_position', {}).get('y', 0)
            goal_x, goal_y, goal_theta = self.current_goal

            distance_to_goal = self._calculate_distance_to_goal()
            bearing_to_goal = self._calculate_bearing_to_goal()

            navigation_prompt = f"""
                                I'm a robot navigating step-by-step with AI guidance (no autonomous navigation system).

                                CURRENT SITUATION:
                                - My position: ({current_x:.2f}, {current_y:.2f})
                                - My goal: ({goal_x:.2f}, {goal_y:.2f})
                                - Distance to goal: {distance_to_goal:.2f} meters
                                - Direction to goal: {bearing_to_goal:.1f} degrees

                                SENSOR READINGS:
                                - Front: {status.get('sensor_distances', {}).get('front', 0)}cm
                                - Left: {status.get('sensor_distances', {}).get('left', 0)}cm
                                - Right: {status.get('sensor_distances', {}).get('right', 0)}cm

                                INSTRUCTIONS:
                                Give me 1-2 specific actions to get closer to my goal safely.
                                Consider obstacles and choose movements that make clear progress.

                                Examples of good responses:
                                - "Turn toward goal direction and move forward 1 meter"
                                - "Turn left 30 degrees to avoid obstacle, then move forward 0.8 meters"
                                - "Move backward 0.5 meters to get better positioning"

                                Be specific about distances and angles. Think step by step.
                                """
            instruction = await self.robot.chat(navigation_prompt)
            return instruction
        except Exception as e:
            logger.error(f"Error getting AI navigation step: {e}")
            return None
        
    
    def _is_goal_reached(self):
        distance = self._calculate_distance_to_goal()

        goal_tolerance = 1
        
        reached = distance < goal_tolerance
        if reached:
            print(f"Goal reached Distance: {distance: .2f}m (tolerance: {goal_tolerance}m)")
        return reached
    
     
    def _calculate_distance_to_goal(self):
        if not self.current_goal:
            return float('inf')
        
        try:
            status = self.robot.get_status()
            current_x = status.get('current_position', {}).get('x',0)
            current_y = status.get('current_position', {}).get('y',0)

            goal_x, goal_y, goal_theta = self.current_goal

            distance = math.sqrt((current_x-goal_x)**2 + (current_y - goal_y)**2)
            return distance
        
        except Exception as e:
            logger.error(f"Error calculating distance: {e}")
            return float('inf')
        

    def _calculate_bearing_to_goal(self):#it is used to figuring out the direction(angle) from the robot's current position to the goal
        if not self.current_goal:
            return 0.0
        #from where i am right now, in which direciton(angle) is my goal\

        try:
            status = self.robot.get_status()
            current_x = status.get('current_position', {}).get('x')
            current_y = status.get('current_position', {}).get('y')
            goal_x, goal_y, goal_theta = self.current_goal

            dx = goal_x - current_x
            dy= goal_y - current_y

            bearing_rad = math.atan2(dy,dx)
            bearing_deg = math.degrees(bearing_rad)

            if bearing_deg < 0:
                bearing_deg += 360
            return bearing_deg
        
        except Exception as e:
            logger.error(f"Error calculting bearing: {e}")
            return 0.0
        
    def _cleanup_navigation(self):
        print("Cleaning up navigation state")

        self.navigation_active = False
        self.current_mode = NavigationMode.IDLE
        self.current_goal = None

        self.stuck_start_time = None
        self.last_position = None
        self.stuck_count = 0
        self.ai_attempt_count = 0

        print("Navigation cleanup complete")s