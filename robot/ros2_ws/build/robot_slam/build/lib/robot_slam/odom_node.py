#!/usr/bin/env python3
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
        
        # Robot state
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
        self.position_threshold = 0.02  # 1cm threshold
        self.angle_threshold = 0.05     # ~3 degree threshold
        
        # Angular velocity filtering
        self.angular_velocity_threshold = 0.05  # rad/s - ignore below this
        self.angular_velocity_history = []
        self.history_size = 5
        
        # Network connection - HARDCODED IP
        self.socket = None
        self.connected = False
        self.windows_ip = "172.27.144.1"  # Hardcoded since nc works with this
        self.tcp_port = 8888
        
        self.get_logger().info("🤖 Network Arduino Odometry node started")
        self.get_logger().info(f"🌐 Connecting to Windows bridge at {self.windows_ip}:{self.tcp_port}")
        
        # Start network connection
        self.connect_thread = threading.Thread(target=self.connect_to_bridge, daemon=True)
        self.connect_thread.start()
        
    def filter_angular_velocity(self, vel_angular):
        """Filter out small angular velocities and smooth the signal"""
        self.angular_velocity_history.append(vel_angular)
        if len(self.angular_velocity_history) > self.history_size:
            self.angular_velocity_history.pop(0)
        
        avg_angular = sum(self.angular_velocity_history) / len(self.angular_velocity_history)
        
        if abs(avg_angular) < self.angular_velocity_threshold:
            return 0.0
        
        return avg_angular
        
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
                self.socket.send(b'PING\n')
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
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line.startswith('ODOM:'):
                            parts = line.replace('ODOM:', '').split(',')
                            if len(parts) >= 6:
                                try:
                                    pos_x = float(parts[0])
                                    pos_y = float(parts[1])
                                    heading = float(parts[2])
                                    vel_x = float(parts[3])
                                    vel_y = float(parts[4])
                                    vel_angular = float(parts[5])
                                    
                                    filtered_vel_angular = self.filter_angular_velocity(vel_angular)
                                    
                                    pos_changed = (abs(pos_x - self.last_x) > self.position_threshold or 
                                                 abs(pos_y - self.last_y) > self.position_threshold)
                                    angle_changed = abs(heading - self.last_theta) > self.angle_threshold
                                    
                                    if pos_changed or angle_changed or abs(filtered_vel_angular) > 0.01:
                                        self.x = pos_x
                                        self.y = pos_y
                                        self.theta = heading
                                        self.vx = vel_x
                                        self.vy = vel_y
                                        self.vth = filtered_vel_angular
                                        self.publish_odometry()
                                        self.last_x = pos_x
                                        self.last_y = pos_y
                                        self.last_theta = heading
                                        
                                        if hasattr(self, '_msg_count'):
                                            self._msg_count += 1
                                        else:
                                            self._msg_count = 1
                                        
                                        if self._msg_count % 20 == 0:
                                            self.get_logger().info(f"📊 Odom: x={pos_x:.3f}, y={pos_y:.3f}, θ={heading:.3f}, ω={filtered_vel_angular:.3f}")
                                    
                                except ValueError as e:
                                    self.get_logger().warn(f"⚠️ Parse error: {e}")
                        
                        elif line == 'PONG':
                            self.get_logger().info("🏓 Arduino bridge responding")
                            
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ Network error: {e}")
                self.connected = False
                break
                
    def euler_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[35] = 0.2
        
        self.odom_pub.publish(odom)
        
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self.euler_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()