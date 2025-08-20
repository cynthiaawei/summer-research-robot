# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion, TransformStamped
# from tf2_ros import TransformBroadcaster
# import socket
# import time
# import threading
# import math
# from rclpy.qos import QoSProfile

# class NetworkOdometryNode(Node):
#     def __init__(self):
#         super().__init__('network_odom_node')
        
#         # TF broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         # Publisher with reliable QoS
#         qos_profile = QoSProfile(depth=10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
#         # Robot state - START AT ORIGIN
#         self.x = 0.0
#         self.y = 0.0 
#         self.theta = 0.0
#         self.vx = 0.0
#         self.vy = 0.0
#         self.vth = 0.0
        
#         # Previous values for change detection
#         self.last_x = 0.0
#         self.last_y = 0.0
#         self.last_theta = 0.0
#         self.position_threshold = 0.02  # 2cm threshold
#         self.angle_threshold = 0.05     # ~3 degree threshold
        
#         # Angular velocity filtering
#         self.angular_velocity_threshold = 0.05  # rad/s - ignore below this
#         self.angular_velocity_history = []
#         self.history_size = 5
        
#         # Network connection
#         self.socket = None
#         self.connected = False
#         self.windows_ip = "172.31.176.1"
#         self.tcp_port = 8888
#         self.arduino_reset_sent = False  # Track if we've reset Arduino
        
#         # TF publishing timer - CRITICAL FOR NAVIGATION
#         self.tf_timer = self.create_timer(0.1, self.publish_tf_continuously)  # 10Hz TF
        
#         self.get_logger().info("🤖 Network Arduino Odometry node started")
#         self.get_logger().info(f"🌐 Connecting to Windows bridge at {self.windows_ip}:{self.tcp_port}")
        
#         # Start network connection
#         self.connect_thread = threading.Thread(target=self.connect_to_bridge, daemon=True)
#         self.connect_thread.start()
        
#     def filter_angular_velocity(self, vel_angular):
#         """Filter out small angular velocities and smooth the signal"""
#         # Add to history
#         self.angular_velocity_history.append(vel_angular)
#         if len(self.angular_velocity_history) > self.history_size:
#             self.angular_velocity_history.pop(0)
        
#         # Calculate moving average
#         avg_angular = sum(self.angular_velocity_history) / len(self.angular_velocity_history)
        
#         # Apply threshold - ignore small movements
#         if abs(avg_angular) < self.angular_velocity_threshold:
#             return 0.0
        
#         return avg_angular
        
#     def reset_arduino_position(self):
#         """Reset Arduino position to (0,0) when ROS starts"""
#         if self.connected and not self.arduino_reset_sent:
#             try:
#                 self.socket.send(b'RESET\n')
#                 self.arduino_reset_sent = True
#                 self.get_logger().info("🔄 Sent RESET command to Arduino - position should be (0,0)")
#                 time.sleep(0.1)  # Give Arduino time to process
#             except Exception as e:
#                 self.get_logger().warn(f"Failed to reset Arduino: {e}")
        
#     def connect_to_bridge(self):
#         """Connect to Arduino bridge on Windows"""
#         retry_count = 0
#         while rclpy.ok() and retry_count < 10:
#             try:
#                 self.get_logger().info(f"🔍 Attempt {retry_count + 1}/10: Connecting to {self.windows_ip}:{self.tcp_port}")
#                 self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 self.socket.settimeout(5)
#                 self.socket.connect((self.windows_ip, self.tcp_port))
                
#                 self.connected = True
#                 self.get_logger().info("✅ Connected to Arduino bridge!")
                
#                 # Send PING to test
#                 self.socket.send(b'PING\n')
#                 time.sleep(0.5)
                
#                 # Reset Arduino position to (0,0)
#                 self.reset_arduino_position()
                
#                 # Start reading data
#                 self.read_network_data()
#                 break
                
#             except Exception as e:
#                 self.get_logger().warn(f"❌ Connection failed: {e}")
#                 if self.socket:
#                     self.socket.close()
#                 retry_count += 1
#                 time.sleep(2)
                
#         if not self.connected:
#             self.get_logger().error("❌ Could not connect to Arduino bridge after 10 attempts!")
#             self.get_logger().info("📝 Make sure arduino_bridge.py is running on Windows")
                
#     def read_network_data(self):
#         """Read Arduino data via network"""
#         buffer = ""
#         while rclpy.ok() and self.connected:
#             try:
#                 self.socket.settimeout(1.0)
#                 data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                
#                 if data:
#                     buffer += data
                    
#                     # Process complete lines
#                     while '\n' in buffer:
#                         line, buffer = buffer.split('\n', 1)
#                         line = line.strip()
                        
#                         if line.startswith('ODOM:'):
#                             # Parse: ODOM:pos_x,pos_y,heading,vel_x,vel_y,vel_angular
#                             parts = line.replace('ODOM:', '').split(',')
#                             if len(parts) >= 6:
#                                 try:
#                                     # Parse Arduino data (assuming meters)
#                                     pos_x = float(parts[0])
#                                     pos_y = float(parts[1])
#                                     heading = float(parts[2])
#                                     vel_x = float(parts[3])
#                                     vel_y = float(parts[4])
#                                     vel_angular = float(parts[5])
                                    
#                                     # Filter angular velocity to reduce drift
#                                     filtered_vel_angular = self.filter_angular_velocity(vel_angular)
                                    
#                                     # ALWAYS update robot state (even if no significant change)
#                                     self.x = pos_x
#                                     self.y = pos_y
#                                     self.theta = heading
#                                     self.vx = vel_x  # Use Arduino velocity directly
#                                     self.vy = vel_y  # Use Arduino velocity directly
#                                     self.vth = filtered_vel_angular
                                    
#                                     # Check if data has changed significantly for message publishing
#                                     pos_changed = (abs(pos_x - self.last_x) > self.position_threshold or 
#                                                  abs(pos_y - self.last_y) > self.position_threshold)
#                                     angle_changed = abs(heading - self.last_theta) > self.angle_threshold
#                                     vel_significant = (abs(vel_x) > 0.01 or abs(vel_y) > 0.01 or 
#                                                      abs(filtered_vel_angular) > 0.01)
                                    
#                                     # Only publish odom MESSAGE if significant change (saves bandwidth)
#                                     if pos_changed or angle_changed or vel_significant:
#                                         self.publish_odometry_message()
                                        
#                                         # Update last values
#                                         self.last_x = pos_x
#                                         self.last_y = pos_y
#                                         self.last_theta = heading
                                        
#                                         # Log occasionally
#                                         if hasattr(self, '_msg_count'):
#                                             self._msg_count += 1
#                                         else:
#                                             self._msg_count = 1
                                        
#                                         if self._msg_count % 20 == 0:
#                                             self.get_logger().info(f"📊 Odom: x={pos_x:.3f}, y={pos_y:.3f}, θ={heading:.3f}, vx={vel_x:.3f}, vy={vel_y:.3f}, ω={filtered_vel_angular:.3f}")
                                    
#                                     # NOTE: TF is published continuously by timer, not here!
                                    
#                                 except ValueError as e:
#                                     self.get_logger().warn(f"⚠️ Parse error: {e}")
                        
#                         elif line == 'PONG':
#                             self.get_logger().info("🏓 Arduino bridge responding")
#                         elif line == 'RESET_OK':
#                             self.get_logger().info("✅ Arduino position reset confirmed")
                            
#             except socket.timeout:
#                 continue
#             except Exception as e:
#                 self.get_logger().error(f"❌ Network error: {e}")
#                 self.connected = False
#                 break
                
#     def euler_to_quaternion(self, yaw):
#         """Convert yaw to quaternion"""
#         qz = math.sin(yaw / 2.0)
#         qw = math.cos(yaw / 2.0)
#         return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    
#     def publish_tf_continuously(self):
#         """Publish TF transform continuously - CRITICAL FOR NAVIGATION!"""
#         current_time = self.get_clock().now()
#         tf_msg = TransformStamped()
#         tf_msg.header.stamp = current_time.to_msg()
#         tf_msg.header.frame_id = 'odom'
#         tf_msg.child_frame_id = 'base_footprint'
#         tf_msg.transform.translation.x = self.x
#         tf_msg.transform.translation.y = self.y
#         tf_msg.transform.translation.z = 0.0
#         tf_msg.transform.rotation = self.euler_to_quaternion(self.theta)
#         self.tf_broadcaster.sendTransform(tf_msg)
        
#     def publish_odometry_message(self):
#         """Publish odometry message (only when needed to save bandwidth)"""
#         current_time = self.get_clock().now()
        
#         # Create odometry message
#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_footprint'
        
#         # Position (directly from Arduino)
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        
#         # Velocity (directly from Arduino)
#         odom.twist.twist.linear.x = self.vx
#         odom.twist.twist.linear.y = self.vy  # For omni drive
#         odom.twist.twist.angular.z = self.vth
        
#         # Add covariance
#         odom.pose.covariance = [0.0] * 36
#         odom.twist.covariance = [0.0] * 36
#         odom.pose.covariance[0] = 0.1   # x
#         odom.pose.covariance[7] = 0.1   # y  
#         odom.pose.covariance[35] = 0.2  # yaw
#         odom.twist.covariance[0] = 0.1   # vx
#         odom.twist.covariance[7] = 0.1   # vy
#         odom.twist.covariance[35] = 0.2  # vyaw
        
#         # Publish
#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = NetworkOdometryNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import socket
import time
import threading
import math
from rclpy.qos import QoSProfile

class NetworkOdometryNode(Node):
    def __init__(self):
        super().__init__('network_odom_node')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher with reliable QoS
        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
        # Robot state - START AT ORIGIN
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Previous values for change detection
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.last_angular_vel = 0.0  # Track last angular velocity for change detection
        
        # UPDATED THRESHOLDS - more responsive
        self.position_threshold = 0.02      # Keep this (2cm)
        self.angle_threshold = 0.01         # REDUCED from 0.05 to 0.01 for responsiveness
        
        # Angular velocity filtering
        self.angular_velocity_history = []
        self.history_size = 5
        
        # Network connection
        self.socket = None
        self.connected = False
        self.windows_ip = "172.27.144.1"
        self.tcp_port = 8888
        self.arduino_reset_sent = False  # Track if we've reset Arduino
        
        # TF publishing timer - CRITICAL FOR NAVIGATION
        self.tf_timer = self.create_timer(0.1, self.publish_tf_continuously)  # 10Hz TF
        
        self.get_logger().info("🤖 Network Arduino Odometry node started")
        self.get_logger().info(f"🌐 Connecting to Windows bridge at {self.windows_ip}:{self.tcp_port}")
        
        # Start network connection
        self.connect_thread = threading.Thread(target=self.connect_to_bridge, daemon=True)
        self.connect_thread.start()
        
    def filter_angular_velocity(self, vel_angular):
        """Apply consistent deadzone and smooth the signal"""
        
        # FIRST: Apply same deadzone as Arduino (consistency)
        if abs(vel_angular) < 0.035:  # Match Arduino threshold
            vel_angular = 0.0
        
        # THEN: Apply smoothing
        self.angular_velocity_history.append(vel_angular)
        if len(self.angular_velocity_history) > self.history_size:
            self.angular_velocity_history.pop(0)
        
        # Calculate moving average
        avg_angular = sum(self.angular_velocity_history) / len(self.angular_velocity_history)
        
        return avg_angular
        
    def reset_arduino_position(self):
        """Reset Arduino position to (0,0) when ROS starts"""
        if self.connected and not self.arduino_reset_sent:
            try:
                self.socket.send(b'RESET\n')
                self.arduino_reset_sent = True
                self.get_logger().info("🔄 Sent RESET command to Arduino - position should be (0,0)")
                time.sleep(0.1)  # Give Arduino time to process
            except Exception as e:
                self.get_logger().warn(f"Failed to reset Arduino: {e}")
        
    def connect_to_bridge(self):
        """Connect to Arduino bridge on Windows"""
        retry_count = 0
        while rclpy.ok() and retry_count < 10:
            try:
                self.get_logger().info(f"🔍 Attempt {retry_count + 1}/10: Connecting to {self.windows_ip}:{self.tcp_port}")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5)
                self.socket.connect((self.windows_ip, self.tcp_port))
                
                self.connected = True
                self.get_logger().info("✅ Connected to Arduino bridge!")
                
                # Send PING to test
                self.socket.send(b'PING\n')
                time.sleep(0.5)
                
                # Reset Arduino position to (0,0)
                self.reset_arduino_position()
                
                # Start reading data
                self.read_network_data()
                break
                
            except Exception as e:
                self.get_logger().warn(f"❌ Connection failed: {e}")
                if self.socket:
                    self.socket.close()
                retry_count += 1
                time.sleep(2)
                
        if not self.connected:
            self.get_logger().error("❌ Could not connect to Arduino bridge after 10 attempts!")
            self.get_logger().info("📝 Make sure arduino_bridge.py is running on Windows")
                
    def read_network_data(self):
        """Read Arduino data via network"""
        buffer = ""
        while rclpy.ok() and self.connected:
            try:
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                
                if data:
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line.startswith('ODOM:'):
                            # Parse: ODOM:pos_x,pos_y,heading,vel_x,vel_y,vel_angular
                            parts = line.replace('ODOM:', '').split(',')
                            if len(parts) >= 6:
                                try:
                                    # Parse Arduino data (assuming meters)
                                    pos_x = float(parts[0])
                                    pos_y = float(parts[1])
                                    heading = float(parts[2])
                                    vel_x = float(parts[3])
                                    vel_y = float(parts[4])
                                    vel_angular = float(parts[5])
                                    
                                    # Filter angular velocity to reduce drift
                                    filtered_vel_angular = self.filter_angular_velocity(vel_angular)
                                    
                                    # ALWAYS update robot state (even if no significant change)
                                    self.x = pos_x
                                    self.y = pos_y
                                    self.theta = heading
                                    self.vx = vel_x  # Use Arduino velocity directly
                                    self.vy = vel_y  # Use Arduino velocity directly
                                    self.vth = filtered_vel_angular
                                    
                                    # IMPROVED publishing logic - better change detection
                                    pos_changed = (abs(pos_x - self.last_x) > self.position_threshold or 
                                                 abs(pos_y - self.last_y) > self.position_threshold)
                                    
                                    angle_changed = abs(heading - self.last_theta) > self.angle_threshold
                                    
                                    # Check for angular velocity changes (important for deceleration)
                                    angular_change = abs(filtered_vel_angular - self.last_angular_vel)
                                    angular_changed = angular_change > 0.015  # Small threshold for angular changes
                                    
                                    # Special case: robot stopping (angular velocity goes to zero)
                                    angular_stopping = (abs(filtered_vel_angular) < 0.01 and 
                                                      abs(self.last_angular_vel) > 0.05)
                                    
                                    # Any significant velocity
                                    vel_significant = (abs(vel_x) > 0.01 or abs(vel_y) > 0.01 or 
                                                     abs(filtered_vel_angular) > 0.01)
                                    
                                    # Publish if ANY condition is met
                                    should_publish = (pos_changed or angle_changed or angular_changed or 
                                                    angular_stopping or vel_significant)
                                    
                                    # Only publish odom MESSAGE if significant change (saves bandwidth)
                                    if should_publish:
                                        self.publish_odometry_message()
                                        
                                        # Update ALL last values
                                        self.last_x = pos_x
                                        self.last_y = pos_y
                                        self.last_theta = heading
                                        self.last_angular_vel = filtered_vel_angular  # Important!
                                        
                                        # Log occasionally
                                        if hasattr(self, '_msg_count'):
                                            self._msg_count += 1
                                        else:
                                            self._msg_count = 1
                                        
                                        if self._msg_count % 20 == 0:
                                            self.get_logger().info(f"📊 Odom: x={pos_x:.3f}, y={pos_y:.3f}, θ={heading:.3f}, vx={vel_x:.3f}, vy={vel_y:.3f}, ω={filtered_vel_angular:.3f}")
                                    
                                    # NOTE: TF is published continuously by timer, not here!
                                    
                                except ValueError as e:
                                    self.get_logger().warn(f"⚠️ Parse error: {e}")
                        
                        elif line == 'PONG':
                            self.get_logger().info("🏓 Arduino bridge responding")
                        elif line == 'RESET_OK':
                            self.get_logger().info("✅ Arduino position reset confirmed")
                            
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ Network error: {e}")
                self.connected = False
                break
                
    def euler_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    
    def publish_tf_continuously(self):
        """Publish TF transform continuously - CRITICAL FOR NAVIGATION!"""
        current_time = self.get_clock().now()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self.euler_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(tf_msg)
        
    def publish_odometry_message(self):
        """Publish odometry message (only when needed to save bandwidth)"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position (directly from Arduino)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        
        # Velocity (directly from Arduino)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy  # For omni drive
        odom.twist.twist.angular.z = self.vth
        
        # Add covariance
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y  
        odom.pose.covariance[35] = 0.2  # yaw
        odom.twist.covariance[0] = 0.1   # vx
        odom.twist.covariance[7] = 0.1   # vy
        odom.twist.covariance[35] = 0.2  # vyaw
        
        # Publish
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NetworkOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()